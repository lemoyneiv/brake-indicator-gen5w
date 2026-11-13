package dev.lemoyne.brakeindicator;

/**
 * Evaluates live vehicle readings and Option 1 OEM CAN signals to determine
 * whether the vehicle's brake lights are illuminated.
 */
public final class BrakeEvaluator {

    /** HKMC HSCAN getCylPrsrVal invalid/init sentinel is 0xFFF * 0.1 = 409.5. */
    public static final float MAX_VALID_PRESSURE = 409.4f;
    public static final float MAX_VALID_SPEED_KMH = 350.0f;
    public static final float MAX_VALID_ACCEL_PEDAL = 250.0f;

    private static final float PRESSURE_RELEASE_RATIO = 0.70f;
    private static final float DECEL_RELEASE_RATIO = 0.62f;
    private static final float DECEL_EMA_ALPHA = 0.45f;
    private static final long DECEL_HOLD_MS = 250L;

    public static final class Input {
        public final long nowMs;
        public final Boolean nativeStopLamp;
        public final String nativeStopLampName;
        public final Float brakePressure;
        public final Float speedKmh;
        public final Float accelPedal;
        public final Boolean gearPark;
        public final Boolean parkingBrake;
        public final long testUntilMs;

        public Input(
                long nowMs,
                Boolean nativeStopLamp,
                String nativeStopLampName,
                Float brakePressure,
                Float speedKmh,
                Float accelPedal,
                Boolean gearPark,
                Boolean parkingBrake,
                long testUntilMs) {
            this.nowMs = nowMs;
            this.nativeStopLamp = nativeStopLamp;
            this.nativeStopLampName = nativeStopLampName == null ? "" : nativeStopLampName;
            this.brakePressure = sanitizeRange(brakePressure, 0.0f, MAX_VALID_PRESSURE);
            this.speedKmh = sanitizeRange(speedKmh, 0.0f, MAX_VALID_SPEED_KMH);
            this.accelPedal = sanitizeRange(accelPedal, 0.0f, MAX_VALID_ACCEL_PEDAL);
            this.gearPark = gearPark;
            this.parkingBrake = parkingBrake;
            this.testUntilMs = testUntilMs;
        }

        private static Float sanitizeRange(Float v, float min, float max) {
            if (v == null || Float.isNaN(v) || Float.isInfinite(v)) {
                return null;
            }
            if (v < min || v > max) {
                return null;
            }
            return v;
        }
    }

    public static final class Snapshot {
        public final long timestampMs;
        public final boolean illuminated;
        public final String source;
        public final Boolean nativeStopLamp;
        public final String nativeStopLampName;
        public final Float brakePressure;
        public final Float speedKmh;
        public final Float accelPedal;
        public final float decelMps2;
        public final Boolean gearPark;
        public final Boolean parkingBrake;

        public Snapshot(
                long timestampMs,
                boolean illuminated,
                String source,
                Boolean nativeStopLamp,
                String nativeStopLampName,
                Float brakePressure,
                Float speedKmh,
                Float accelPedal,
                float decelMps2,
                Boolean gearPark,
                Boolean parkingBrake) {
            this.timestampMs = timestampMs;
            this.illuminated = illuminated;
            this.source = source == null ? "OFF" : source;
            this.nativeStopLamp = nativeStopLamp;
            this.nativeStopLampName = nativeStopLampName == null ? "" : nativeStopLampName;
            this.brakePressure = brakePressure;
            this.speedKmh = speedKmh;
            this.accelPedal = accelPedal;
            this.decelMps2 = decelMps2;
            this.gearPark = gearPark;
            this.parkingBrake = parkingBrake;
        }
    }

    private boolean pressureLatched;
    private boolean decelLatched;
    private long decelHoldUntilMs;
    private Float prevSpeedKmh;
    private long prevSpeedTimeMs;
    private float smoothedDecelMps2;
    private boolean hasDecelSample;

    public Snapshot evaluate(IndicatorConfig cfg, Input in) {
        IndicatorConfig config = cfg == null ? IndicatorConfig.defaults() : cfg;

        // 1. Update longitudinal deceleration estimate (-dv/dt) from CAN speed
        updateDecel(in.nowMs, in.speedKmh);

        // 2. Update hydraulic/IEB cylinder pressure latch with hysteresis
        if (in.brakePressure == null) {
            pressureLatched = false;
        } else if (pressureLatched) {
            if (in.brakePressure < config.pressureThreshold * PRESSURE_RELEASE_RATIO) {
                pressureLatched = false;
            }
        } else if (in.brakePressure >= config.pressureThreshold) {
            pressureLatched = true;
        }

        // 3. Update regenerative deceleration latch (used when NATIVE_AND_DECEL enabled)
        boolean canRegenBrake = !Boolean.TRUE.equals(in.gearPark)
                && in.speedKmh != null
                && in.speedKmh >= 2.0f
                && (in.accelPedal == null || in.accelPedal < 3.0f);

        if (!canRegenBrake) {
            decelLatched = false;
            decelHoldUntilMs = 0L;
        } else if (smoothedDecelMps2 >= config.decelThresholdMps2) {
            decelLatched = true;
            decelHoldUntilMs = in.nowMs + DECEL_HOLD_MS;
        } else if (decelLatched) {
            if (smoothedDecelMps2 >= config.decelThresholdMps2 * DECEL_RELEASE_RATIO) {
                decelHoldUntilMs = in.nowMs + DECEL_HOLD_MS;
            } else if (in.nowMs >= decelHoldUntilMs) {
                decelLatched = false;
            }
        }

        // 4. Priority resolution
        boolean lit = false;
        String src = "OFF";

        if (in.nowMs < in.testUntilMs) {
            lit = true;
            src = "MANUAL_TEST";
        } else if (Boolean.TRUE.equals(in.nativeStopLamp)) {
            lit = true;
            src = in.nativeStopLampName.isEmpty() ? "OEM_LAMP" : "OEM_LAMP:" + in.nativeStopLampName;
        } else if (pressureLatched) {
            lit = true;
            src = "BRAKE_PRESSURE";
        } else if (config.signalMode == IndicatorConfig.SignalMode.NATIVE_AND_DECEL && decelLatched) {
            lit = true;
            src = "REGEN_DECEL";
        }

        return new Snapshot(
                in.nowMs,
                lit,
                src,
                in.nativeStopLamp,
                in.nativeStopLampName,
                in.brakePressure,
                in.speedKmh,
                in.accelPedal,
                smoothedDecelMps2,
                in.gearPark,
                in.parkingBrake);
    }

    public static Float parseReadingFloat(String text) {
        if (text == null) {
            return null;
        }
        String t = text.trim();
        if (t.isEmpty() || "-".equals(t)) {
            return null;
        }
        try {
            return Float.parseFloat(t);
        } catch (NumberFormatException e) {
            return null;
        }
    }

    public static Boolean parseReadingFlag(String text) {
        if (text == null) {
            return null;
        }
        String t = text.trim();
        if ("1".equals(t) || "true".equalsIgnoreCase(t)) {
            return Boolean.TRUE;
        }
        if ("0".equals(t) || "false".equalsIgnoreCase(t)) {
            return Boolean.FALSE;
        }
        return null;
    }

    private void updateDecel(long nowMs, Float speedKmh) {
        if (speedKmh == null) {
            prevSpeedKmh = null;
            smoothedDecelMps2 = 0.0f;
            hasDecelSample = false;
            return;
        }
        if (prevSpeedKmh != null) {
            long dtMs = nowMs - prevSpeedTimeMs;
            if (dtMs >= 50L && dtMs <= 1500L) {
                float dvMps = (speedKmh - prevSpeedKmh) / 3.6f;
                float rawDecel = -dvMps / (dtMs / 1000.0f);
                if (!hasDecelSample) {
                    smoothedDecelMps2 = rawDecel;
                    hasDecelSample = true;
                } else {
                    smoothedDecelMps2 = DECEL_EMA_ALPHA * rawDecel
                            + (1.0f - DECEL_EMA_ALPHA) * smoothedDecelMps2;
                }
                prevSpeedKmh = speedKmh;
                prevSpeedTimeMs = nowMs;
            } else if (dtMs > 1500L || dtMs < 0L) {
                prevSpeedKmh = speedKmh;
                prevSpeedTimeMs = nowMs;
                smoothedDecelMps2 = 0.0f;
                hasDecelSample = false;
            }
            // If 0 <= dtMs < 50ms, keep previous sample anchor until >= 50ms elapses
        } else {
            prevSpeedKmh = speedKmh;
            prevSpeedTimeMs = nowMs;
            smoothedDecelMps2 = 0.0f;
            hasDecelSample = false;
        }
    }
}
