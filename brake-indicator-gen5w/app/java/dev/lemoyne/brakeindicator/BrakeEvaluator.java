package dev.lemoyne.brakeindicator;

/**
 * Evaluates built-in HMG-API vehicle readings (`brake.pressure`, `speed`, `pedal.accel`,
 * `gear.park`, `parking_brake`, `moving`, `steering.angle`, `hv.soc`) alongside optional
 * real-time OBD-II IGMP (`770` -> `778`, UDS `22 BC06`) brake-lamp ground truth and
 * off-throttle coasting regenerative level inference (`L0 / L1 / L2 / L3`).
 *
 * Tuned for HMG-API's reading freshness windows (`100 ms` on updated HMG-API, `250 ms`
 * on older builds): unchanged cached `speed` reads within `< 280 ms` do not inject
 * artificial `0.0 m/s^2` steps between `Hub` refreshes.
 */
public final class BrakeEvaluator {

    /** HKMC HSCAN getCylPrsrVal invalid/init sentinel is 0xFFF * 0.1 = 409.5. */
    public static final float MAX_VALID_PRESSURE = 409.4f;
    public static final float MAX_VALID_SPEED_KMH = 350.0f;
    public static final float MAX_VALID_ACCEL_PEDAL = 250.0f;
    public static final float MAX_VALID_STEERING_DEG = 1080.0f;
    public static final float MAX_VALID_SOC_PCT = 105.0f;

    /** Maximum age of an OBD 22BC06 sample before falling back to the HMG-API model. */
    public static final long MAX_OBD_FRESH_MS = 900L;

    private static final float PRESSURE_RELEASE_RATIO = 0.70f;
    private static final float DECEL_RELEASE_RATIO = 0.60f;
    private static final float DECEL_EMA_ALPHA = 0.65f;
    private static final long DECEL_HOLD_MS = 320L;

    /**
     * HMG-API caches `speed` for 100 ms (or 250 ms on older builds). When polled at 20 Hz (50 ms),
     * consecutive reads inside that freshness window return the exact same float.
     * Treating unchanged values within `< 280 ms` as cached repeats prevents sawtooth dropouts
     * on both 100 ms and 250 ms `Hub` cache windows while reacting immediately whenever speed steps.
     */
    private static final long SPEED_CACHE_REPEAT_MS = 280L;
    private static final long WINDOW_TARGET_MS = 250L;
    private static final long MIN_WINDOW_MS = 100L;
    private static final long MAX_WINDOW_MS = 1500L;
    private static final int SPEED_HISTORY_CAP = 24;

    /**
     * In i-Pedal / Level 3 regen, drivers often modulate deceleration between 0% and ~20%
     * accelerator travel while still commanding strong regenerative braking.
     */
    private static final float MAX_REGEN_ACCEL_PEDAL = 25.0f;
    /** Any deliberate throttle press (>= 2%) from standstill immediately releases hold. */
    private static final float STANDSTILL_RELEASE_ACCEL = 2.0f;
    private static final float STANDSTILL_SPEED_MAX_KMH = 1.5f;
    private static final long STANDSTILL_ARM_WINDOW_MS = 2500L;

    // Coasting-deceleration Regen Level (0..3) estimator parameters:
    // Sampled only when foot is completely off both pedals (pedal.accel < 1%, brake.pressure < 0.25)
    // above 18 km/h for >= 450 ms so drivetrain tip-out transients settle cleanly.
    private static final float COAST_MIN_SPEED_KMH = 18.0f;
    private static final float COAST_MAX_ACCEL_PCT = 1.0f;
    private static final float COAST_MAX_PRESSURE = 0.25f;
    private static final long COAST_SETTLE_MS = 450L;
    private static final float COAST_EMA_ALPHA = 0.45f;
    private static final float REGEN_L0_L1_MPS2 = 0.30f;
    private static final float REGEN_L1_L2_MPS2 = 0.85f;
    private static final float REGEN_L2_L3_MPS2 = 1.42f;
    private static final float REGEN_HYST_MPS2 = 0.08f;

    public static final class ObdSample {
        public final boolean connected;
        public final String transport; // "BLE", "BRIDGE", or ""
        public final long sampleElapsedMs;
        public final int rawCode;      // e.g. 0, 4, 36, 37, 40, 41, 44, 45, or -1 if none yet
        public final boolean brakeLampLit;
        public final boolean pedalBit;

        public ObdSample(
                boolean connected,
                String transport,
                long sampleElapsedMs,
                int rawCode,
                boolean brakeLampLit,
                boolean pedalBit) {
            this.connected = connected;
            this.transport = transport == null ? "" : transport;
            this.sampleElapsedMs = sampleElapsedMs;
            this.rawCode = rawCode;
            this.brakeLampLit = brakeLampLit;
            this.pedalBit = pedalBit;
        }

        public static ObdSample disconnected() {
            return new ObdSample(false, "", 0L, -1, false, false);
        }

        public boolean isFreshAt(long nowMs) {
            return connected
                    && rawCode >= 0
                    && sampleElapsedMs > 0L
                    && (nowMs - sampleElapsedMs) >= 0L
                    && (nowMs - sampleElapsedMs) <= MAX_OBD_FRESH_MS;
        }

        public long ageMsAt(long nowMs) {
            if (!connected || rawCode < 0 || sampleElapsedMs <= 0L) {
                return -1L;
            }
            return Math.max(0L, nowMs - sampleElapsedMs);
        }
    }

    public static final class Input {
        public final long nowMs;
        public final Float brakePressure;
        public final Float speedKmh;
        public final Float accelPedal;
        public final Float steeringAngleDeg;
        public final Boolean gearPark;
        public final Boolean parkingBrake;
        public final Boolean moving;
        public final Float hvSocPct;
        public final ObdSample obd;
        public final long testUntilMs;
        public final int hevRegenRaw;

        public Input(
                long nowMs,
                Float brakePressure,
                Float speedKmh,
                Float accelPedal,
                Float steeringAngleDeg,
                Boolean gearPark,
                Boolean parkingBrake,
                Boolean moving,
                Float hvSocPct,
                ObdSample obd,
                long testUntilMs) {
            this(nowMs, brakePressure, speedKmh, accelPedal, steeringAngleDeg,
                    gearPark, parkingBrake, moving, hvSocPct, obd, testUntilMs, -1);
        }

        public Input(
                long nowMs,
                Float brakePressure,
                Float speedKmh,
                Float accelPedal,
                Float steeringAngleDeg,
                Boolean gearPark,
                Boolean parkingBrake,
                Boolean moving,
                Float hvSocPct,
                ObdSample obd,
                long testUntilMs,
                int hevRegenRaw) {
            this.nowMs = nowMs;
            this.brakePressure = sanitizeRange(brakePressure, 0.0f, MAX_VALID_PRESSURE);
            this.speedKmh = sanitizeRange(speedKmh, 0.0f, MAX_VALID_SPEED_KMH);
            this.accelPedal = sanitizeRange(accelPedal, 0.0f, MAX_VALID_ACCEL_PEDAL);
            this.steeringAngleDeg = sanitizeRange(steeringAngleDeg, -MAX_VALID_STEERING_DEG, MAX_VALID_STEERING_DEG);
            this.gearPark = gearPark;
            this.parkingBrake = parkingBrake;
            this.moving = moving;
            this.hvSocPct = sanitizeRange(hvSocPct, 0.0f, MAX_VALID_SOC_PCT);
            this.obd = obd == null ? ObdSample.disconnected() : obd;
            this.testUntilMs = testUntilMs;
            this.hevRegenRaw = hevRegenRaw;
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
        /** Final illumination state rendered on the top-bar icon (respecting ObdMode & manual test). */
        public final boolean illuminated;
        public final String source;
        /** Pure HMG-API built-in readings model decision (always evaluated for live accuracy scoring). */
        public final boolean modelIlluminated;
        public final String modelSource;

        public final Float brakePressure;
        public final Float speedKmh;
        public final Float accelPedal;
        public final Float steeringAngleDeg;
        public final float decelMps2;
        public final Boolean gearPark;
        public final Boolean parkingBrake;
        public final Boolean moving;
        public final Float hvSocPct;
        public final ObdSample obd;
        public final boolean obdFresh;
        public final boolean manualTestActive;
        /** Inferred regen level from off-throttle coasting decel (-1 unknown, 0..3 = L0..L3/i-Pedal). */
        public final int inferredRegenLevel;
        /** Direct HEVFlag/USM regen raw candidate value, or -1 if none detected. */
        public final int hevRegenRaw;

        public Snapshot(
                long timestampMs,
                boolean illuminated,
                String source,
                boolean modelIlluminated,
                String modelSource,
                Float brakePressure,
                Float speedKmh,
                Float accelPedal,
                Float steeringAngleDeg,
                float decelMps2,
                Boolean gearPark,
                Boolean parkingBrake,
                Boolean moving,
                Float hvSocPct,
                ObdSample obd,
                boolean obdFresh,
                boolean manualTestActive) {
            this(timestampMs, illuminated, source, modelIlluminated, modelSource,
                    brakePressure, speedKmh, accelPedal, steeringAngleDeg, decelMps2,
                    gearPark, parkingBrake, moving, hvSocPct, obd, obdFresh, manualTestActive, -1, -1);
        }

        public Snapshot(
                long timestampMs,
                boolean illuminated,
                String source,
                boolean modelIlluminated,
                String modelSource,
                Float brakePressure,
                Float speedKmh,
                Float accelPedal,
                Float steeringAngleDeg,
                float decelMps2,
                Boolean gearPark,
                Boolean parkingBrake,
                Boolean moving,
                Float hvSocPct,
                ObdSample obd,
                boolean obdFresh,
                boolean manualTestActive,
                int inferredRegenLevel,
                int hevRegenRaw) {
            this.timestampMs = timestampMs;
            this.illuminated = illuminated;
            this.source = source == null ? "OFF" : source;
            this.modelIlluminated = modelIlluminated;
            this.modelSource = modelSource == null ? "OFF" : modelSource;
            this.brakePressure = brakePressure;
            this.speedKmh = speedKmh;
            this.accelPedal = accelPedal;
            this.steeringAngleDeg = steeringAngleDeg;
            this.decelMps2 = decelMps2;
            this.gearPark = gearPark;
            this.parkingBrake = parkingBrake;
            this.moving = moving;
            this.hvSocPct = hvSocPct;
            this.obd = obd == null ? ObdSample.disconnected() : obd;
            this.obdFresh = obdFresh;
            this.manualTestActive = manualTestActive;
            this.inferredRegenLevel = inferredRegenLevel;
            this.hevRegenRaw = hevRegenRaw;
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

    private long coastSinceMs = -1L;
    private float coastDecelEma;
    private boolean hasCoastEma;
    private int inferredRegenLevel = -1;

    public Snapshot evaluate(IndicatorConfig cfg, Input in) {
        IndicatorConfig config = cfg == null ? IndicatorConfig.defaults() : cfg;

        // 1. Update longitudinal deceleration estimate (-dv/dt)
        updateDecel(in.nowMs, in.speedKmh);

        // 2. Update off-throttle coasting Regen Level estimator (L0 / L1 / L2 / L3)
        boolean obdFresh = (config.obdMode != IndicatorConfig.ObdMode.DISABLED) && in.obd.isFreshAt(in.nowMs);
        updateCoastingRegenEstimate(in, obdFresh);

        // 3. Update hydraulic/IEB cylinder pressure latch (`brake.pressure`) with hysteresis
        if (in.brakePressure == null) {
            pressureLatched = false;
        } else if (pressureLatched) {
            if (in.brakePressure < config.pressureThreshold * PRESSURE_RELEASE_RATIO) {
                pressureLatched = false;
            }
        } else if (in.brakePressure >= config.pressureThreshold) {
            pressureLatched = true;
        }

        // 4. Update regenerative / Smart Cruise Control deceleration latch
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

        // 5. Arm & maintain i-Pedal / Smart Cruise Control standstill hold at 0 km/h
        boolean activeModelBraking = decelLatched || pressureLatched;
        if (!Boolean.TRUE.equals(in.gearPark)
                && in.speedKmh != null
                && in.speedKmh >= 0.5f
                && in.speedKmh <= 14.0f
                && activeModelBraking) {
            lastBrakingNearStopMs = in.nowMs;
        }

        boolean decelModeEnabled = (config.signalMode == IndicatorConfig.SignalMode.PRESSURE_AND_DECEL);
        if (!decelModeEnabled || !config.standstillHold) {
            standstillLatched = false;
        } else if (Boolean.TRUE.equals(in.gearPark)
                || in.speedKmh == null
                || in.speedKmh >= STANDSTILL_SPEED_MAX_KMH
                || Boolean.TRUE.equals(in.moving) && in.speedKmh >= 2.0f
                || (in.accelPedal != null && in.accelPedal >= STANDSTILL_RELEASE_ACCEL)) {
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

        // 6. Pure HMG-API model resolution (always computed so accuracy vs OBD is scored continuously)
        boolean modelLit = false;
        String modelSrc = "OFF";
        if (pressureLatched) {
            modelLit = true;
            modelSrc = "BRAKE_PRESSURE";
        } else if (decelModeEnabled && decelLatched) {
            modelLit = true;
            modelSrc = "REGEN_DECEL";
        } else if (decelModeEnabled && standstillLatched) {
            modelLit = true;
            modelSrc = "STANDSTILL_HOLD";
        }

        // 7. Top-bar icon resolution based on manual test + ObdMode + live OBD freshness
        boolean manualTest = in.nowMs < in.testUntilMs;
        boolean iconLit;
        String iconSrc;

        if (manualTest) {
            iconLit = true;
            iconSrc = "MANUAL_TEST";
        } else if (config.obdMode == IndicatorConfig.ObdMode.OBD_WHEN_CONNECTED && obdFresh) {
            iconLit = in.obd.brakeLampLit;
            if (iconLit) {
                iconSrc = in.obd.pedalBit
                        ? ("OBD_PEDAL[" + in.obd.rawCode + "]")
                        : ("OBD_REGEN[" + in.obd.rawCode + "]");
            } else {
                iconSrc = "OBD_OFF";
            }
        } else {
            iconLit = modelLit;
            iconSrc = modelSrc;
        }

        return new Snapshot(
                in.nowMs,
                iconLit,
                iconSrc,
                modelLit,
                modelSrc,
                in.brakePressure,
                in.speedKmh,
                in.accelPedal,
                in.steeringAngleDeg,
                smoothedDecelMps2,
                in.gearPark,
                in.parkingBrake,
                in.moving,
                in.hvSocPct,
                in.obd,
                obdFresh,
                manualTest,
                inferredRegenLevel,
                in.hevRegenRaw);
    }

    private void updateCoastingRegenEstimate(Input in, boolean obdFresh) {
        if (Boolean.TRUE.equals(in.gearPark)) {
            coastSinceMs = -1L;
            hasCoastEma = false;
            inferredRegenLevel = -1;
            return;
        }
        boolean pureZeroPedalCoast = in.speedKmh != null
                && in.speedKmh >= COAST_MIN_SPEED_KMH
                && in.accelPedal != null
                && in.accelPedal < COAST_MAX_ACCEL_PCT
                && in.brakePressure != null
                && in.brakePressure < COAST_MAX_PRESSURE
                && !(obdFresh && in.obd.pedalBit)
                && hasDecelSample
                && smoothedDecelMps2 >= -0.40f
                && smoothedDecelMps2 <= 4.0f;

        if (!pureZeroPedalCoast) {
            coastSinceMs = -1L;
            hasCoastEma = false;
            return;
        }

        if (coastSinceMs < 0L) {
            coastSinceMs = in.nowMs;
            return;
        }
        if ((in.nowMs - coastSinceMs) < COAST_SETTLE_MS) {
            return;
        }

        if (!hasCoastEma) {
            coastDecelEma = smoothedDecelMps2;
            hasCoastEma = true;
        } else {
            coastDecelEma = COAST_EMA_ALPHA * smoothedDecelMps2 + (1.0f - COAST_EMA_ALPHA) * coastDecelEma;
        }
        inferredRegenLevel = classifyCoastingRegenLevel(coastDecelEma, inferredRegenLevel);
    }

    public static int classifyCoastingRegenLevel(float decelMps2, int prevLevel) {
        if (prevLevel == 0) {
            if (decelMps2 < REGEN_L0_L1_MPS2 + REGEN_HYST_MPS2) return 0;
        } else if (prevLevel == 1) {
            if (decelMps2 >= REGEN_L0_L1_MPS2 - REGEN_HYST_MPS2
                    && decelMps2 < REGEN_L1_L2_MPS2 + REGEN_HYST_MPS2) return 1;
        } else if (prevLevel == 2) {
            if (decelMps2 >= REGEN_L1_L2_MPS2 - REGEN_HYST_MPS2
                    && decelMps2 < REGEN_L2_L3_MPS2 + REGEN_HYST_MPS2) return 2;
        } else if (prevLevel == 3) {
            if (decelMps2 >= REGEN_L2_L3_MPS2 - REGEN_HYST_MPS2) return 3;
        }

        if (decelMps2 < REGEN_L0_L1_MPS2) return 0;
        if (decelMps2 < REGEN_L1_L2_MPS2) return 1;
        if (decelMps2 < REGEN_L2_L3_MPS2) return 2;
        return 3;
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

        if (speedKmh < 0.3f) {
            smoothedDecelMps2 = 0.0f;
            hasDecelSample = true;
            pushHistorySample(nowMs, speedKmh);
            return;
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

        // Detect cached duplicate speed reads inside HMG-API's freshness window:
        // hold current smoothedDecelMps2 until either speed steps OR 280 ms elapses unchanged.
        if (histCount > 0) {
            float prevSpeed = histSpeedKmh[histCount - 1];
            long sincePrevMs = nowMs - histTimeMs[histCount - 1];
            if (Math.abs(speedKmh - prevSpeed) < 0.04f && sincePrevMs < SPEED_CACHE_REPEAT_MS) {
                return;
            }
        }

        // Choose newest distinct sample at least WINDOW_TARGET_MS old, or oldest once MIN_WINDOW_MS elapsed
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

        pushHistorySample(nowMs, speedKmh);
    }

    private void pushHistorySample(long nowMs, float speedKmh) {
        if (histCount > 0 && (nowMs - histTimeMs[histCount - 1]) < 20L) {
            return;
        }
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
