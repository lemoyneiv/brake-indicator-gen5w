package dev.lemoyne.brakeindicator;

/**
 * Evaluates built-in HMG-API vehicle readings (`brake.pressure`, `speed`, `pedal.accel`,
 * `gear.park`, `parking_brake`, `moving`, `steering.angle`, `hv.soc`) alongside optional
 * real-time OBD-II IGMP (`770` -> `778`, UDS `22 BC06`) brake-lamp ground truth and
 * poll-rate-independent regenerative mode (`L0 / L1 / L2 / L3 / i-Pedal`) + Smart Cruise
 * Control (`SCC`) auto-detection.
 *
 * Calibrated directly against multi-mode city, arterial, Auto-Hold, Smart Cruise Control
 * stop-and-go, and `70 mph` (`112.6 km/h`) highway E-GMP drive logs on both `50-100 ms`
 * patched HMG-API `Hub`s and `650 ms` legacy KitKat `Hub`s:
 * - Aligns `-dv/dt` updates to actual HMG-API poll timestamps (`hmgSampleElapsedMs`) and
 *   suppresses `< 560 ms` `0.8 km/h` cluster speed-quantization repeats.
 * - Compensates passive aerodynamic drag excess above `36 km/h` (`10 m/s`) so highway wind
 *   drag neither inflates `L0..L3` coasting classification nor false-triggers UN ECE R13-H
 *   motor-torque brake-lamp estimation.
 * - Uses a settled `>= 21 km/h` history-ring coasting chord (`1.05-1.95 s` preferred) to
 *   separate `L0`, `L1`, `L2`, and `L3+`, and distinguishes `i-Pedal` (`4`) from `L3` (`3`)
 *   via low-speed full-stop regen and urban neutral-pedal signatures.
 * - Automatically detects Smart Cruise Control (`sccActive`) from settled zero-pedal
 *   acceleration or sustained zero-pedal cruise, freezing driver paddle regen inference and
 *   applying SCC braking thresholds until cancelled by brake, Park/EPB, or sustained throttle.
 * - Preserves standstill hold arming across `<= 14 km/h` -> `0 km/h` stops (including
 *   extended foot-brake holds into Auto-Hold and Park-carryover EPB auto-release), releasing
 *   immediately on manual EPB engagement, Park, `0.7 km/h` creep, `moving`, or `>= 1.5%` throttle.
 */
public final class BrakeEvaluator {

    public static final int REGEN_LEVEL_UNKNOWN = -1;
    public static final int REGEN_LEVEL_0       = 0;
    public static final int REGEN_LEVEL_1       = 1;
    public static final int REGEN_LEVEL_2       = 2;
    public static final int REGEN_LEVEL_3       = 3;
    public static final int REGEN_LEVEL_IPEDAL  = 4;

    /** HKMC HSCAN getCylPrsrVal invalid/init sentinel is 0xFFF * 0.1 = 409.5. */
    public static final float MAX_VALID_PRESSURE = 409.4f;
    public static final float MAX_VALID_SPEED_KMH = 350.0f;
    public static final float MAX_VALID_ACCEL_PEDAL = 250.0f;
    public static final float MAX_VALID_STEERING_DEG = 1080.0f;
    public static final float MAX_VALID_SOC_PCT = 105.0f;

    /** Maximum age of an OBD 22BC06 sample before falling back to the HMG-API model. */
    public static final long MAX_OBD_FRESH_MS = 900L;

    private static final float PRESSURE_RELEASE_RATIO = 0.70f;
    private static final float DEFAULT_DECEL_THRESHOLD_MPS2 = 1.20f;
    private static final float BASE_ONSET_STANDARD_MPS2 = 1.15f;
    private static final float BASE_ONSET_HIGH_REGEN_MPS2 = 0.92f;
    private static final float BASE_ONSET_SCC_MPS2 = 0.48f;
    private static final float BASE_SUSTAINED_SCC_MPS2 = 0.28f;
    private static final long SUSTAINED_SCC_CONFIRM_MS = 550L;
    private static final float BASE_RELEASE_HYST_MPS2 = 0.55f;
    private static final float BASE_RELEASE_LOW_REGEN_MPS2 = 0.85f;
    private static final float BASE_RELEASE_SCC_MPS2 = 0.26f;

    // Mode-specific neutral-pedal release thresholds calibrated from 6-drive E-GMP logs:
    private static final float RELEASE_ACCEL_LOW_PCT = 4.0f;
    private static final float RELEASE_ACCEL_L2_PCT = 4.8f;
    private static final float RELEASE_ACCEL_L3_PCT = 7.5f;
    private static final float RELEASE_ACCEL_IPEDAL_PCT = 11.5f;
    private static final float STRONG_DECEL_RELEASE_ACCEL_PADDLE_PCT = 13.0f;
    private static final float ZERO_PEDAL_HIGH_REGEN_RELEASE_MPS2 = 0.42f;
    private static final float CREEP_TAPER_RELEASE_SPEED_KMH = 14.2f;

    private static final float DECEL_EMA_ALPHA = 0.72f;
    private static final long DECEL_HOLD_MS = 140L;
    private static final long LIFTOFF_CRUISE_ANTICIPATION_MAX_MS = 900L;
    private static final long LIFTOFF_ACCEL_RAMP_DELAY_MS = 550L;
    private static final long LIFTOFF_ACCEL_ANTICIPATION_MAX_MS = 1350L;
    private static final long POST_BRAKE_REGEN_SETTLE_MS = 650L;

    /**
     * Passive aerodynamic drag excess above 36 km/h (10 m/s):
     * a_aero_excess = K_AERO * max(0, v_mps^2 - 10^2).
     * E-GMP triggers UN ECE R13-H regen lamps from motor regen torque rather than passive wind drag.
     */
    private static final float AERO_REF_SPEED_MPS = 10.0f;
    private static final float AERO_DRAG_COEFF_PER_M = 0.00024f;

    /**
     * Suppresses duplicate speed samples for `< 560 ms` so neither HMG-API's `100/250 ms`
     * `Hub` cache nor `0.8 km/h` (`0.5 mph`) cluster quantization steps inject artificial
     * `0.0 m/s^2` dropouts or late-anchor `2.5x` spikes (`0.8 km/h` at `0.40 m/s^2` takes `555 ms`,
     * while `650 ms` legacy `Hub` polls still pass cleanly).
     */
    private static final long SPEED_CACHE_REPEAT_MS = 560L;
    private static final long STALE_HMG_POLL_DECAY_MS = 1200L;
    private static final long WINDOW_TARGET_MS = 380L;
    private static final long MIN_WINDOW_MS = 95L;
    private static final long MAX_WINDOW_MS = 2000L;
    private static final int SPEED_HISTORY_CAP = 24;

    private static final float MAX_REGEN_ACCEL_PEDAL = 25.0f;
    /** Any deliberate throttle press (>= 1.5%) or creep (>= 0.7 km/h) releases standstill hold. */
    private static final float STANDSTILL_RELEASE_ACCEL = 1.5f;
    private static final float STANDSTILL_SPEED_MAX_KMH = 0.7f;
    private static final long STANDSTILL_ARM_WINDOW_MS = 3000L;

    // History-ring settled coasting-chord Regen Level (L0..L3) estimator parameters:
    // Sampled above 21 km/h (above E-GMP's < 20 km/h creep taper) when off both pedals.
    private static final float COAST_MIN_SPEED_KMH = 21.0f;
    private static final float LIFTOFF_MIN_SPEED_L2_L3_KMH = 20.5f;
    private static final float COAST_MAX_ACCEL_PCT = 1.0f;
    private static final float COAST_MAX_PRESSURE = 0.25f;
    private static final long COAST_SETTLE_MS = 520L;
    private static final long COAST_CHORD_PREF_MS = 1050L;
    private static final long COAST_CHORD_MIN_MS = 580L;
    private static final long COAST_CHORD_MAX_MS = 1950L;
    private static final float REGEN_L0_L1_MPS2 = 0.44f;
    private static final float REGEN_L1_L2_MPS2 = 0.82f;
    private static final float REGEN_L2_L3_MPS2 = 1.22f;
    private static final float REGEN_HYST_MPS2 = 0.08f;

    // i-Pedal (4) vs Level 3 (3) discrimination thresholds:
    private static final float IPEDAL_LOW_SPD_MIN_KMH = 2.0f;
    private static final float IPEDAL_LOW_SPD_MAX_KMH = 14.0f;
    private static final float IPEDAL_LOW_SPD_DECEL_MPS2 = 1.35f;
    private static final long IPEDAL_LOW_SPD_CONFIRM_MS = 250L;
    private static final float IPEDAL_CRUISE_MAX_SPD_KMH = 56.0f;
    private static final long IPEDAL_CRUISE_CONFIRM_MS = 1100L;
    private static final long CREEP_TAPER_CONFIRM_MS = 450L;
    private static final long IPEDAL_LOW_PEDAL_CRUISE_CONFIRM_MS = 1200L;

    // Smart Cruise Control (SCC / HDA) zero-pedal travel detection:
    private static final long SCC_SETTLE_MS = 700L;
    private static final long SCC_ACCEL_CHORD_MS = 1100L;
    private static final long SCC_CRUISE_CHORD_MS = 2600L;
    private static final long SCC_MANUAL_OVERRIDE_EXIT_MS = 2500L;

    public static String regenLevelLabel(int level) {
        if (level < 0) {
            return "?";
        }
        if (level == REGEN_LEVEL_IPEDAL) {
            return "i-Pedal";
        }
        return "L" + level;
    }

    public static String regenLevelLabel(int level, boolean sccActive) {
        String base = regenLevelLabel(level);
        if (!sccActive) {
            return base;
        }
        return "?".equals(base) ? "SCC" : (base + " (SCC)");
    }

    public static float aeroDragExcessMps2(Float speedKmh) {
        if (speedKmh == null) {
            return 0.0f;
        }
        float vMps = speedKmh.floatValue() / 3.6f;
        float excessSq = (vMps * vMps) - (AERO_REF_SPEED_MPS * AERO_REF_SPEED_MPS);
        if (excessSq <= 0.0f) {
            return 0.0f;
        }
        return AERO_DRAG_COEFF_PER_M * excessSq;
    }

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
        public final long hmgSampleElapsedMs;

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
                    gearPark, parkingBrake, moving, hvSocPct, obd, testUntilMs, -1, -1L);
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
            this(nowMs, brakePressure, speedKmh, accelPedal, steeringAngleDeg,
                    gearPark, parkingBrake, moving, hvSocPct, obd, testUntilMs, hevRegenRaw, -1L);
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
                int hevRegenRaw,
                long hmgSampleElapsedMs) {
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
            this.hmgSampleElapsedMs = hmgSampleElapsedMs;
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
        /** Auto-detected regen mode (-1 unknown, 0..3 = L0..L3, 4 = i-Pedal). */
        public final int inferredRegenLevel;
        /** Optional direct OEM regen raw candidate value, or -1 if none. */
        public final int hevRegenRaw;
        /** True when Smart Cruise Control / HDA zero-pedal travel is active. */
        public final boolean sccActive;

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
                    gearPark, parkingBrake, moving, hvSocPct, obd, obdFresh, manualTestActive, -1, -1, false);
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
            this(timestampMs, illuminated, source, modelIlluminated, modelSource,
                    brakePressure, speedKmh, accelPedal, steeringAngleDeg, decelMps2,
                    gearPark, parkingBrake, moving, hvSocPct, obd, obdFresh, manualTestActive,
                    inferredRegenLevel, hevRegenRaw, false);
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
                int hevRegenRaw,
                boolean sccActive) {
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
            this.sccActive = sccActive;
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
    private long lastProcessedHmgPollMs = -1L;
    private float smoothedPollIntervalMs = 100.0f;
    private float lastRawDecelMps2;
    private float smoothedDecelMps2;
    private boolean hasDecelSample;

    private long coastStartMs = -1L;
    private long lastFootBrakeMs = -1L;
    private long zeroPedalLiftStartMs = -1L;
    private float zeroPedalLiftInitDecelMps2;
    private long ipedalLowSpdSinceMs = -1L;
    private long ipedalCruiseSinceMs = -1L;
    private long creepTaperSinceMs = -1L;
    private long lowPedalCruiseSinceMs = -1L;
    private int inferredRegenLevel = REGEN_LEVEL_UNKNOWN;

    private boolean sccActive;
    private boolean prevParkingBrake;
    private boolean epbEngagedInDrive;
    private long sccZeroPedalStartMs = -1L;
    private long sccAnchorMs = -1L;
    private float sccAnchorSpeedKmh;
    private long sccManualThrottleSinceMs = -1L;
    private long sccSustainedDecelSinceMs = -1L;

    public Snapshot evaluate(IndicatorConfig cfg, Input in) {
        IndicatorConfig config = cfg == null ? IndicatorConfig.defaults() : cfg;
        long effPollMs = in.hmgSampleElapsedMs > 0L ? in.hmgSampleElapsedMs : in.nowMs;

        // 1. Update poll-aligned longitudinal deceleration (-dv/dt)
        boolean decelUpdated = updateDecel(in.nowMs, in.hmgSampleElapsedMs, in.speedKmh);

        // 2. Update hydraulic/IEB cylinder pressure latch (`brake.pressure`) with hysteresis.
        //    When rolling >= 2 km/h off-throttle and actively decelerating, E-GMP's physical
        //    brake switch (`22BC06` 37/41/45) closes at ~0.20 bar (0.5x default 0.40 threshold).
        boolean rollingBrakeBrush = in.speedKmh != null
                && in.speedKmh >= 2.0f
                && smoothedDecelMps2 >= 0.25f
                && (in.accelPedal == null || in.accelPedal < 1.0f);
        float prsrOnset = rollingBrakeBrush
                ? (config.pressureThreshold * 0.50f) : config.pressureThreshold;
        float prsrRelease = rollingBrakeBrush
                ? (config.pressureThreshold * 0.38f) : (config.pressureThreshold * PRESSURE_RELEASE_RATIO);
        if (in.brakePressure == null) {
            pressureLatched = false;
        } else if (pressureLatched) {
            if (in.brakePressure < prsrRelease) {
                pressureLatched = false;
            }
        } else if (in.brakePressure >= prsrOnset) {
            pressureLatched = true;
        }

        // 3. Update Smart Cruise Control (`sccActive`) & Regen Mode (`L0..i-Pedal`) estimators
        boolean obdFresh = (config.obdMode != IndicatorConfig.ObdMode.DISABLED) && in.obd.isFreshAt(in.nowMs);
        if (Boolean.TRUE.equals(in.gearPark)) {
            lastFootBrakeMs = -1L;
        } else if (pressureLatched || (in.brakePressure != null && in.brakePressure >= 0.15f)) {
            lastFootBrakeMs = effPollMs;
        }
        updateSccState(in, effPollMs);
        updateRegenModeEstimate(in, decelUpdated, effPollMs);

        // 4. Update E-GMP regenerative / Smart Cruise Control deceleration latch
        float threshScale = config.decelThresholdMps2 / DEFAULT_DECEL_THRESHOLD_MPS2;
        float aeroExcess = aeroDragExcessMps2(in.speedKmh);
        float motorDecelMps2 = smoothedDecelMps2 - aeroExcess;

        float liftoffMinSpd = (inferredRegenLevel == REGEN_LEVEL_IPEDAL)
                ? 2.0f : LIFTOFF_MIN_SPEED_L2_L3_KMH;
        boolean zeroPedalAboveLiftMin = !sccActive
                && in.speedKmh != null
                && in.speedKmh >= liftoffMinSpd
                && in.accelPedal != null
                && in.accelPedal < COAST_MAX_ACCEL_PCT;

        float onsetThresh;
        float releaseThresh;
        if (sccActive) {
            onsetThresh = BASE_ONSET_SCC_MPS2 * threshScale;
            releaseThresh = BASE_RELEASE_SCC_MPS2 * threshScale;
        } else if (inferredRegenLevel >= REGEN_LEVEL_2) {
            onsetThresh = BASE_ONSET_HIGH_REGEN_MPS2 * threshScale;
            if (inferredRegenLevel != REGEN_LEVEL_IPEDAL
                    && smoothedPollIntervalMs < 350.0f
                    && in.speedKmh != null
                    && in.speedKmh < CREEP_TAPER_RELEASE_SPEED_KMH) {
                // E-GMP tapers L2/L3 paddle regen down to idle creep below ~14.2 km/h (22BC06 44 -> 0)
                releaseThresh = BASE_RELEASE_LOW_REGEN_MPS2 * threshScale;
            } else if (zeroPedalAboveLiftMin) {
                releaseThresh = ZERO_PEDAL_HIGH_REGEN_RELEASE_MPS2 * threshScale;
            } else {
                releaseThresh = BASE_RELEASE_HYST_MPS2 * threshScale;
            }
        } else {
            onsetThresh = BASE_ONSET_STANDARD_MPS2 * threshScale;
            releaseThresh = BASE_RELEASE_LOW_REGEN_MPS2 * threshScale;
        }

        float evalDecelMps2 = sccActive ? smoothedDecelMps2 : motorDecelMps2;
        float releaseAccelPct = regenReleaseAccelPct(inferredRegenLevel, sccActive);

        boolean canRegenBrake = !Boolean.TRUE.equals(in.gearPark)
                && !Boolean.TRUE.equals(in.parkingBrake)
                && in.speedKmh != null
                && in.speedKmh >= 1.2f
                && (in.accelPedal == null || in.accelPedal < MAX_REGEN_ACCEL_PEDAL);

        // Sustained mild SCC active braking (e.g. >= 0.28 m/s^2 above passive drag for >= 550 ms)
        boolean sccSustainedBraking = false;
        if (sccActive && canRegenBrake && (in.accelPedal == null || in.accelPedal < 0.8f)
                && motorDecelMps2 >= BASE_SUSTAINED_SCC_MPS2 * threshScale) {
            if (sccSustainedDecelSinceMs < 0L) {
                sccSustainedDecelSinceMs = effPollMs;
            } else if ((effPollMs - sccSustainedDecelSinceMs) >= SUSTAINED_SCC_CONFIRM_MS) {
                sccSustainedBraking = true;
            }
        } else {
            sccSustainedDecelSinceMs = -1L;
        }

        // In manual L2/L3/i-Pedal, E-GMP illuminates 22BC06 (44) on full throttle lift-off above
        // the < 20 km/h creep-taper zone (and down to standstill in i-Pedal). When lifting off from
        // active forward acceleration (smoothedDecelMps2 < -0.25 m/s^2), VCU motor-torque slew-rate
        // limiting takes ~550 ms to ramp +drive -> 0 -> -regen before 22BC06 lights.
        if (!zeroPedalAboveLiftMin) {
            zeroPedalLiftStartMs = -1L;
            zeroPedalLiftInitDecelMps2 = 0.0f;
        } else if (zeroPedalLiftStartMs < 0L) {
            zeroPedalLiftStartMs = effPollMs;
            zeroPedalLiftInitDecelMps2 = hasDecelSample ? smoothedDecelMps2 : 0.0f;
        }
        boolean liftoffAnticipation = false;
        if (zeroPedalAboveLiftMin && inferredRegenLevel >= REGEN_LEVEL_2) {
            long liftElapsedMs = effPollMs - zeroPedalLiftStartMs;
            boolean coarsePollHub = smoothedPollIntervalMs >= 350.0f;
            if (!coarsePollHub && zeroPedalLiftInitDecelMps2 < -0.25f) {
                liftoffAnticipation = liftElapsedMs >= LIFTOFF_ACCEL_RAMP_DELAY_MS
                        && liftElapsedMs < LIFTOFF_ACCEL_ANTICIPATION_MAX_MS
                        && (!hasDecelSample || smoothedDecelMps2 >= -1.20f);
            } else {
                liftoffAnticipation = liftElapsedMs < LIFTOFF_CRUISE_ANTICIPATION_MAX_MS;
            }
        }

        boolean featheredIpedal = !sccActive
                && inferredRegenLevel == REGEN_LEVEL_IPEDAL
                && in.speedKmh != null
                && in.speedKmh >= STANDSTILL_SPEED_MAX_KMH
                && in.accelPedal != null
                && in.accelPedal < RELEASE_ACCEL_IPEDAL_PCT
                && motorDecelMps2 >= releaseThresh;

        boolean throttleCancelsPaddleRegen = !sccActive
                && inferredRegenLevel != REGEN_LEVEL_IPEDAL
                && in.accelPedal != null
                && in.accelPedal >= STRONG_DECEL_RELEASE_ACCEL_PADDLE_PCT;
        boolean throttleSuppressesMildRegen = in.accelPedal != null
                && in.accelPedal >= releaseAccelPct
                && evalDecelMps2 < config.decelThresholdMps2;

        // Immediately after releasing the hydraulic foot brake, smoothedDecelMps2 still reflects
        // the prior hydraulic braking chord for ~650 ms. Unless SCC or high paddle regen (L2/L3 >= 20.5 km/h
        // or i-Pedal) is active, do not carry that hydraulic -dv/dt tail over as REGEN_DECEL.
        boolean strongZeroPedalRegenMode = sccActive
                || inferredRegenLevel == REGEN_LEVEL_IPEDAL
                || (inferredRegenLevel >= REGEN_LEVEL_2 && zeroPedalAboveLiftMin);
        boolean postFootBrakeDecelBleed = !pressureLatched
                && !strongZeroPedalRegenMode
                && lastFootBrakeMs > 0L
                && (effPollMs - lastFootBrakeMs) < POST_BRAKE_REGEN_SETTLE_MS;

        if (!canRegenBrake || throttleCancelsPaddleRegen || throttleSuppressesMildRegen || postFootBrakeDecelBleed) {
            decelLatched = false;
            decelHoldUntilMs = 0L;
        } else if (evalDecelMps2 >= onsetThresh || sccSustainedBraking || liftoffAnticipation || featheredIpedal) {
            decelLatched = true;
            decelHoldUntilMs = in.nowMs + DECEL_HOLD_MS;
        } else if (decelLatched) {
            if (evalDecelMps2 >= releaseThresh
                    && (in.accelPedal == null || in.accelPedal < releaseAccelPct)) {
                decelHoldUntilMs = in.nowMs + DECEL_HOLD_MS;
            } else if (in.nowMs >= decelHoldUntilMs) {
                decelLatched = false;
            }
        }

        // 5. Arm & maintain Auto-Hold / EPB auto-release / i-Pedal / SCC standstill hold at 0 km/h
        boolean curEpb = Boolean.TRUE.equals(in.parkingBrake);
        if (Boolean.TRUE.equals(in.gearPark) || !curEpb) {
            epbEngagedInDrive = false;
        } else if (!prevParkingBrake && curEpb) {
            epbEngagedInDrive = true;
        }
        prevParkingBrake = curEpb;

        boolean activeModelBraking = decelLatched || pressureLatched;
        if (Boolean.TRUE.equals(in.gearPark) || epbEngagedInDrive) {
            lastBrakingNearStopMs = 0L;
        } else if (in.speedKmh != null && in.speedKmh <= 14.0f && activeModelBraking) {
            lastBrakingNearStopMs = in.nowMs;
        } else if ((in.speedKmh != null && in.speedKmh >= 3.0f)
                || (in.accelPedal != null && in.accelPedal >= STANDSTILL_RELEASE_ACCEL)) {
            lastBrakingNearStopMs = 0L;
        }

        boolean decelModeEnabled = (config.signalMode == IndicatorConfig.SignalMode.PRESSURE_AND_DECEL);
        if (!decelModeEnabled || !config.standstillHold) {
            standstillLatched = false;
        } else if (Boolean.TRUE.equals(in.gearPark)
                || epbEngagedInDrive
                || in.speedKmh == null
                || in.speedKmh >= STANDSTILL_SPEED_MAX_KMH
                || Boolean.TRUE.equals(in.moving)
                || (in.accelPedal != null && in.accelPedal >= STANDSTILL_RELEASE_ACCEL)) {
            standstillLatched = false;
        } else if (standstillLatched
                || inferredRegenLevel == REGEN_LEVEL_IPEDAL
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
                in.hevRegenRaw,
                sccActive);
    }

    private static float regenReleaseAccelPct(int regenLevel, boolean sccActive) {
        if (sccActive) {
            return RELEASE_ACCEL_LOW_PCT;
        }
        if (regenLevel == REGEN_LEVEL_IPEDAL) {
            return RELEASE_ACCEL_IPEDAL_PCT;
        }
        if (regenLevel == REGEN_LEVEL_3) {
            return RELEASE_ACCEL_L3_PCT;
        }
        if (regenLevel == REGEN_LEVEL_2) {
            return RELEASE_ACCEL_L2_PCT;
        }
        return RELEASE_ACCEL_LOW_PCT;
    }

    private void updateSccState(Input in, long effPollMs) {
        boolean footBrakePressed = pressureLatched
                || (in.brakePressure != null && in.brakePressure >= COAST_MAX_PRESSURE);
        if (Boolean.TRUE.equals(in.gearPark)
                || Boolean.TRUE.equals(in.parkingBrake)
                || footBrakePressed) {
            sccActive = false;
            sccZeroPedalStartMs = -1L;
            sccAnchorMs = -1L;
            sccManualThrottleSinceMs = -1L;
            sccSustainedDecelSinceMs = -1L;
            return;
        }

        if (in.accelPedal != null && in.accelPedal >= 2.0f) {
            sccZeroPedalStartMs = -1L;
            sccAnchorMs = -1L;
            if (sccActive) {
                if (sccManualThrottleSinceMs < 0L) {
                    sccManualThrottleSinceMs = effPollMs;
                } else if ((effPollMs - sccManualThrottleSinceMs) >= SCC_MANUAL_OVERRIDE_EXIT_MS) {
                    sccActive = false;
                    sccManualThrottleSinceMs = -1L;
                }
            }
            return;
        }
        sccManualThrottleSinceMs = -1L;

        if (in.speedKmh == null || in.speedKmh < 10.0f || in.accelPedal == null || in.accelPedal >= 0.5f) {
            sccZeroPedalStartMs = -1L;
            sccAnchorMs = -1L;
            return;
        }

        if (sccZeroPedalStartMs < 0L) {
            sccZeroPedalStartMs = effPollMs;
            sccAnchorMs = -1L;
            return;
        }

        if (sccAnchorMs < 0L) {
            if ((effPollMs - sccZeroPedalStartMs) >= SCC_SETTLE_MS) {
                sccAnchorMs = effPollMs;
                sccAnchorSpeedKmh = in.speedKmh.floatValue();
            }
            return;
        }

        long dtMs = effPollMs - sccAnchorMs;
        if (dtMs >= SCC_ACCEL_CHORD_MS) {
            float dvKmh = in.speedKmh.floatValue() - sccAnchorSpeedKmh;
            float chordDecel = -(dvKmh / 3.6f) / (dtMs / 1000.0f);
            if (dvKmh >= 1.4f && chordDecel <= -0.18f) {
                sccActive = true;
                sccAnchorMs = effPollMs;
                sccAnchorSpeedKmh = in.speedKmh.floatValue();
            } else if (dtMs >= SCC_CRUISE_CHORD_MS && in.speedKmh >= 20.0f && chordDecel <= 0.06f) {
                sccActive = true;
                sccAnchorMs = effPollMs;
                sccAnchorSpeedKmh = in.speedKmh.floatValue();
            } else if (chordDecel > 0.16f && dtMs >= 1500L) {
                // Re-anchor if vehicle is gently slowing down so subsequent SCC acceleration is caught
                sccAnchorMs = effPollMs;
                sccAnchorSpeedKmh = in.speedKmh.floatValue();
            }
        }
    }

    private void updateRegenModeEstimate(Input in, boolean decelUpdated, long effPollMs) {
        if (Boolean.TRUE.equals(in.gearPark)) {
            coastStartMs = -1L;
            ipedalLowSpdSinceMs = -1L;
            ipedalCruiseSinceMs = -1L;
            creepTaperSinceMs = -1L;
            lowPedalCruiseSinceMs = -1L;
            inferredRegenLevel = REGEN_LEVEL_UNKNOWN;
            return;
        }

        // Freeze driver paddle regen-mode inference while Smart Cruise Control owns longitudinal control
        if (sccActive) {
            coastStartMs = -1L;
            ipedalLowSpdSinceMs = -1L;
            ipedalCruiseSinceMs = -1L;
            creepTaperSinceMs = -1L;
            lowPedalCruiseSinceMs = -1L;
            return;
        }

        boolean footBrakeSettled = lastFootBrakeMs < 0L
                || (effPollMs - lastFootBrakeMs) >= POST_BRAKE_REGEN_SETTLE_MS;
        boolean noFootBrake = footBrakeSettled
                && in.brakePressure != null
                && in.brakePressure < COAST_MAX_PRESSURE;

        // Signature A: History-ring settled coasting chord above 21 km/h (L0 / L1 / L2 / L3+)
        boolean zeroPedalHighSpeed = in.speedKmh != null
                && in.speedKmh >= COAST_MIN_SPEED_KMH
                && in.accelPedal != null
                && in.accelPedal < COAST_MAX_ACCEL_PCT
                && noFootBrake;

        if (zeroPedalHighSpeed) {
            if (coastStartMs < 0L) {
                if (!hasDecelSample || smoothedDecelMps2 >= -0.55f) {
                    coastStartMs = effPollMs;
                }
            } else if (decelUpdated && (effPollMs - coastStartMs) >= (COAST_SETTLE_MS + COAST_CHORD_MIN_MS)) {
                long earliestValidMs = coastStartMs + COAST_SETTLE_MS;
                int anchorIdx = -1;
                // Prefer newest chord >= COAST_CHORD_PREF_MS (~1.05s on 20Hz Hub, ~1.30s = 2 polls on 650ms Hub)
                for (int i = histCount - 1; i >= 0; i--) {
                    long t = histTimeMs[i];
                    long age = effPollMs - t;
                    if (t >= earliestValidMs && age >= COAST_CHORD_PREF_MS && age <= COAST_CHORD_MAX_MS) {
                        anchorIdx = i;
                        break;
                    }
                }
                if (anchorIdx < 0) {
                    for (int i = 0; i < histCount; i++) {
                        long t = histTimeMs[i];
                        long age = effPollMs - t;
                        if (t >= earliestValidMs && age >= COAST_CHORD_MIN_MS && age <= COAST_CHORD_MAX_MS) {
                            anchorIdx = i;
                            break;
                        }
                    }
                }
                if (anchorIdx >= 0) {
                    long chordDtMs = effPollMs - histTimeMs[anchorIdx];
                    float curSpd = in.speedKmh.floatValue();
                    float anchorSpd = histSpeedKmh[anchorIdx];
                    float midSpd = 0.5f * (curSpd + anchorSpd);
                    float dvMps = (curSpd - anchorSpd) / 3.6f;
                    float chordDecel = (-dvMps / (chordDtMs / 1000.0f)) - aeroDragExcessMps2(Float.valueOf(midSpd));
                    if (chordDecel >= 0.08f && chordDecel <= 4.0f) {
                        int prevForChord = (inferredRegenLevel == REGEN_LEVEL_IPEDAL)
                                ? REGEN_LEVEL_3 : inferredRegenLevel;
                        int classified = classifyCoastingRegenLevel(chordDecel, prevForChord);
                        if (!(classified == REGEN_LEVEL_3 && inferredRegenLevel == REGEN_LEVEL_IPEDAL)) {
                            inferredRegenLevel = classified;
                        }
                    }
                }
            }
        } else {
            coastStartMs = -1L;
        }

        // Signature B: Distinguish i-Pedal (4) from Level 3 (3)
        if (in.speedKmh != null && in.accelPedal != null && noFootBrake && hasDecelSample) {
            float spd = in.speedKmh.floatValue();
            float acc = in.accelPedal.floatValue();

            // 1) Low-speed full-stop regen (2..14 km/h): L0..L3 creep-taper to <= 0.75 m/s^2,
            //    whereas i-Pedal pulls >= 1.35 m/s^2 all the way to standstill.
            if (spd >= IPEDAL_LOW_SPD_MIN_KMH
                    && spd <= IPEDAL_LOW_SPD_MAX_KMH
                    && acc < 5.0f
                    && smoothedDecelMps2 >= IPEDAL_LOW_SPD_DECEL_MPS2) {
                if (ipedalLowSpdSinceMs < 0L) {
                    ipedalLowSpdSinceMs = effPollMs;
                    boolean legacyCoarsePoll = histCount >= 2
                            && (histTimeMs[histCount - 1] - histTimeMs[histCount - 2]) >= 500L;
                    if (legacyCoarsePoll && decelUpdated && lastRawDecelMps2 >= 1.50f) {
                        inferredRegenLevel = REGEN_LEVEL_IPEDAL;
                    }
                } else if ((effPollMs - ipedalLowSpdSinceMs) >= IPEDAL_LOW_SPD_CONFIRM_MS) {
                    inferredRegenLevel = REGEN_LEVEL_IPEDAL;
                }
            } else {
                ipedalLowSpdSinceMs = -1L;
            }

            // 2) High neutral cruise throttle in urban speeds (16..56 km/h) at 23..30% pedal
            if (spd >= 16.0f && spd <= IPEDAL_CRUISE_MAX_SPD_KMH
                    && acc >= 23.0f && acc <= 30.0f
                    && Math.abs(smoothedDecelMps2) <= 0.15f) {
                if (ipedalCruiseSinceMs < 0L) {
                    ipedalCruiseSinceMs = effPollMs;
                } else if ((effPollMs - ipedalCruiseSinceMs) >= IPEDAL_CRUISE_CONFIRM_MS) {
                    inferredRegenLevel = REGEN_LEVEL_IPEDAL;
                }
            } else {
                ipedalCruiseSinceMs = -1L;
            }

            // 3) Step down i-Pedal -> L3 if low-speed zero-pedal coast exhibits creep taper (0.15..0.80 m/s^2)
            if (inferredRegenLevel == REGEN_LEVEL_IPEDAL
                    && spd >= 9.0f && spd <= 17.0f
                    && acc < COAST_MAX_ACCEL_PCT
                    && smoothedDecelMps2 >= 0.15f && smoothedDecelMps2 <= 0.80f) {
                if (creepTaperSinceMs < 0L) {
                    creepTaperSinceMs = effPollMs;
                } else if ((effPollMs - creepTaperSinceMs) >= CREEP_TAPER_CONFIRM_MS) {
                    inferredRegenLevel = REGEN_LEVEL_3;
                }
            } else {
                creepTaperSinceMs = -1L;
            }

            // 4) Step down i-Pedal -> L3 if driver cruises/accelerates (>= 20 km/h) at 3..17.5% pedal
            if (inferredRegenLevel == REGEN_LEVEL_IPEDAL
                    && spd >= 20.0f
                    && acc >= 3.0f && acc <= 17.5f
                    && smoothedDecelMps2 <= 0.15f) {
                if (lowPedalCruiseSinceMs < 0L) {
                    lowPedalCruiseSinceMs = effPollMs;
                } else if ((effPollMs - lowPedalCruiseSinceMs) >= IPEDAL_LOW_PEDAL_CRUISE_CONFIRM_MS) {
                    inferredRegenLevel = REGEN_LEVEL_3;
                }
            } else {
                lowPedalCruiseSinceMs = -1L;
            }
        } else {
            ipedalLowSpdSinceMs = -1L;
            ipedalCruiseSinceMs = -1L;
            creepTaperSinceMs = -1L;
            lowPedalCruiseSinceMs = -1L;
        }
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

    private boolean updateDecel(long nowMs, long hmgSampleElapsedMs, Float speedKmh) {
        if (speedKmh == null) {
            histCount = 0;
            lastProcessedHmgPollMs = -1L;
            lastRawDecelMps2 = 0.0f;
            smoothedDecelMps2 = 0.0f;
            hasDecelSample = false;
            return false;
        }

        long effTimeMs = hmgSampleElapsedMs > 0L ? hmgSampleElapsedMs : nowMs;
        if (hmgSampleElapsedMs > 0L) {
            if (hmgSampleElapsedMs == lastProcessedHmgPollMs) {
                if ((nowMs - hmgSampleElapsedMs) > STALE_HMG_POLL_DECAY_MS) {
                    smoothedDecelMps2 *= 0.85f;
                    if (Math.abs(smoothedDecelMps2) < 0.05f) {
                        smoothedDecelMps2 = 0.0f;
                    }
                }
                return false;
            }
            if (lastProcessedHmgPollMs > 0L) {
                long pollDt = hmgSampleElapsedMs - lastProcessedHmgPollMs;
                if (pollDt >= 20L && pollDt <= 850L) {
                    smoothedPollIntervalMs = 0.35f * pollDt + 0.65f * smoothedPollIntervalMs;
                }
            }
            lastProcessedHmgPollMs = hmgSampleElapsedMs;
        }

        if (histCount > 0) {
            long gap = effTimeMs - histTimeMs[histCount - 1];
            if (gap < 0L || gap > MAX_WINDOW_MS) {
                histCount = 0;
                lastRawDecelMps2 = 0.0f;
                smoothedDecelMps2 = 0.0f;
                hasDecelSample = false;
            }
        }

        if (speedKmh < 0.3f) {
            lastRawDecelMps2 = 0.0f;
            smoothedDecelMps2 = 0.0f;
            hasDecelSample = true;
            pushHistorySample(effTimeMs, speedKmh);
            return true;
        }

        // Prune samples older than MAX_WINDOW_MS
        int expired = 0;
        while (expired < histCount && (effTimeMs - histTimeMs[expired]) > MAX_WINDOW_MS) {
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

        // Suppress cached / 0.8 km/h quantized repeats inside SPEED_CACHE_REPEAT_MS (380 ms)
        if (histCount > 0) {
            float prevSpeed = histSpeedKmh[histCount - 1];
            long sincePrevMs = effTimeMs - histTimeMs[histCount - 1];
            if (Math.abs(speedKmh - prevSpeed) < 0.04f && sincePrevMs < SPEED_CACHE_REPEAT_MS) {
                return false;
            }
        }

        // Choose newest distinct sample at least WINDOW_TARGET_MS old, or oldest once MIN_WINDOW_MS elapsed
        int anchorIdx = -1;
        for (int i = histCount - 1; i >= 0; i--) {
            if (effTimeMs - histTimeMs[i] >= WINDOW_TARGET_MS) {
                anchorIdx = i;
                break;
            }
        }
        if (anchorIdx < 0 && histCount > 0 && (effTimeMs - histTimeMs[0]) >= MIN_WINDOW_MS) {
            anchorIdx = 0;
        }

        if (anchorIdx >= 0) {
            long dtMs = effTimeMs - histTimeMs[anchorIdx];
            float dvMps = (speedKmh - histSpeedKmh[anchorIdx]) / 3.6f;
            float rawDecel = -dvMps / (dtMs / 1000.0f);
            if (rawDecel < -10.0f || rawDecel > 14.0f) {
                histCount = 0;
                lastRawDecelMps2 = 0.0f;
                smoothedDecelMps2 = 0.0f;
                hasDecelSample = false;
                pushHistorySample(effTimeMs, speedKmh);
                return true;
            }
            rawDecel = Math.max(-8.0f, Math.min(12.0f, rawDecel));
            lastRawDecelMps2 = rawDecel;
            if (!hasDecelSample) {
                smoothedDecelMps2 = rawDecel;
                hasDecelSample = true;
            } else if (smoothedDecelMps2 < 0.0f && rawDecel >= 0.0f) {
                smoothedDecelMps2 = rawDecel;
            } else {
                smoothedDecelMps2 = DECEL_EMA_ALPHA * rawDecel
                        + (1.0f - DECEL_EMA_ALPHA) * smoothedDecelMps2;
            }
        }

        pushHistorySample(effTimeMs, speedKmh);
        return true;
    }

    private void pushHistorySample(long timeMs, float speedKmh) {
        if (histCount > 0 && (timeMs - histTimeMs[histCount - 1]) < 20L) {
            return;
        }
        if (histCount == SPEED_HISTORY_CAP) {
            for (int i = 0; i < SPEED_HISTORY_CAP - 1; i++) {
                histTimeMs[i] = histTimeMs[i + 1];
                histSpeedKmh[i] = histSpeedKmh[i + 1];
            }
            histCount = SPEED_HISTORY_CAP - 1;
        }
        histTimeMs[histCount] = timeMs;
        histSpeedKmh[histCount] = speedKmh;
        histCount++;
    }
}
