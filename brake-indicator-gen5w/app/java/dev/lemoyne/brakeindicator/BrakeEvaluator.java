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
    private static final float DECEL_RELEASE_RATIO = 0.60f;
    private static final float DECEL_EMA_ALPHA = 0.60f;
    private static final long DECEL_HOLD_MS = 180L;

    private static final long WINDOW_TARGET_MS = 240L;
    private static final long MIN_WINDOW_MS = 75L;
    private static final long MAX_WINDOW_MS = 1200L;
    private static final int SPEED_HISTORY_CAP = 20;

    /**
     * In i-Pedal / Level 3 regen, drivers often modulate deceleration between 0% and ~20%
     * accelerator travel while still commanding strong regenerative braking.
     */
    private static final float MAX_REGEN_ACCEL_PEDAL = 25.0f;
    /** Any deliberate throttle press (>= 2%) from standstill immediately releases hold. */
    private static final float STANDSTILL_RELEASE_ACCEL = 2.0f;
    private static final float STANDSTILL_SPEED_MAX_KMH = 1.5f;
    private static final long STANDSTILL_ARM_WINDOW_MS = 2500L;

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
    private boolean standstillLatched;
    private long decelHoldUntilMs;
    private long lastBrakingNearStopMs;

    private final long[] histTimeMs = new long[SPEED_HISTORY_CAP];
    private final float[] histSpeedKmh = new float[SPEED_HISTORY_CAP];
    private int histCount;
    private float smoothedDecelMps2;
    private boolean hasDecelSample;

    public Snapshot evaluate(IndicatorConfig cfg, Input in) {
        IndicatorConfig config = cfg == null ? IndicatorConfig.defaults() : cfg;

        // 1. Update longitudinal deceleration estimate (-dv/dt) over a ~240 ms sliding window
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

        // 3. Update regenerative / Smart Cruise Control deceleration latch
        boolean canRegenBrake = !Boolean.TRUE.equals(in.gearPark)
                && in.speedKmh != null
                && in.speedKmh >= STANDSTILL_SPEED_MAX_KMH
                && (in.accelPedal == null || in.accelPedal < MAX_REGEN_ACCEL_PEDAL);

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

        // 4. Arm & maintain i-Pedal / Smart Cruise Control standstill hold at 0 km/h
        boolean activeBraking = decelLatched || pressureLatched || Boolean.TRUE.equals(in.nativeStopLamp);
        if (!Boolean.TRUE.equals(in.gearPark)
                && in.speedKmh != null
                && in.speedKmh >= 0.5f
                && in.speedKmh <= 12.0f
                && activeBraking) {
            lastBrakingNearStopMs = in.nowMs;
        }

        if (config.signalMode != IndicatorConfig.SignalMode.NATIVE_AND_DECEL || !config.standstillHold) {
            standstillLatched = false;
        } else if (Boolean.TRUE.equals(in.gearPark)
                || in.speedKmh == null
                || in.speedKmh >= STANDSTILL_SPEED_MAX_KMH
                || in.accelPedal == null
                || in.accelPedal >= STANDSTILL_RELEASE_ACCEL) {
            standstillLatched = false;
            if (Boolean.TRUE.equals(in.gearPark)
                    || (in.speedKmh != null && in.speedKmh >= 3.0f)
                    || (in.accelPedal != null && in.accelPedal >= STANDSTILL_RELEASE_ACCEL)) {
                lastBrakingNearStopMs = 0L;
            }
        } else if (standstillLatched
                || (lastBrakingNearStopMs > 0L && in.nowMs - lastBrakingNearStopMs <= STANDSTILL_ARM_WINDOW_MS)) {
            standstillLatched = true;
        }

        // 5. Priority resolution
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
        } else if (config.signalMode == IndicatorConfig.SignalMode.NATIVE_AND_DECEL && standstillLatched) {
            lit = true;
            src = "STANDSTILL_HOLD";
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
            histCount = 0;
            smoothedDecelMps2 = 0.0f;
            hasDecelSample = false;
            return;
        }
        if (histCount > 0) {
            long gap = nowMs - histTimeMs[histCount - 1];
            if (gap < 0L || gap > MAX_WINDOW_MS) {
                histCount = 0;
                smoothedDecelMps2 = 0.0f;
                hasDecelSample = false;
            }
        }

        // Prune samples older than MAX_WINDOW_MS
        int expired = 0;
        while (expired < histCount && (nowMs - histTimeMs[expired]) > MAX_WINDOW_MS) {
            expired++;
        }
        if (expired > 0) {
            int remain = histCount - expired;
            for (int i = 0; i < remain; i++) {
                histTimeMs[i] = histTimeMs[i + expired];
                histSpeedKmh[i] = histSpeedKmh[i + expired];
            }
            histCount = remain;
        }

        // Choose newest sample at least WINDOW_TARGET_MS old, or oldest sample once MIN_WINDOW_MS has elapsed
        int anchorIdx = -1;
        for (int i = histCount - 1; i >= 0; i--) {
            if (nowMs - histTimeMs[i] >= WINDOW_TARGET_MS) {
                anchorIdx = i;
                break;
            }
        }
        if (anchorIdx < 0 && histCount > 0 && (nowMs - histTimeMs[0]) >= MIN_WINDOW_MS) {
            anchorIdx = 0;
        }

        if (anchorIdx >= 0) {
            long dtMs = nowMs - histTimeMs[anchorIdx];
            float dvMps = (speedKmh - histSpeedKmh[anchorIdx]) / 3.6f;
            float rawDecel = -dvMps / (dtMs / 1000.0f);
            if (!hasDecelSample) {
                smoothedDecelMps2 = rawDecel;
                hasDecelSample = true;
            } else {
                smoothedDecelMps2 = DECEL_EMA_ALPHA * rawDecel
                        + (1.0f - DECEL_EMA_ALPHA) * smoothedDecelMps2;
            }
        }

        // Record sample if at least 20 ms elapsed since last stored sample
        if (histCount == 0 || (nowMs - histTimeMs[histCount - 1]) >= 20L) {
            if (histCount == SPEED_HISTORY_CAP) {
                for (int i = 0; i < SPEED_HISTORY_CAP - 1; i++) {
                    histTimeMs[i] = histTimeMs[i + 1];
                    histSpeedKmh[i] = histSpeedKmh[i + 1];
                }
                histCount = SPEED_HISTORY_CAP - 1;
            }
            histTimeMs[histCount] = nowMs;
            histSpeedKmh[histCount] = speedKmh;
            histCount++;
        }
    }
}
