package dev.lemoyne.brakeindicator;

import java.util.Map;

/**
 * Evaluates built-in HMG-API vehicle readings (`brake.pressure`, `speed`, `pedal.accel`,
 * `gear.park`, `parking_brake`, `moving`, `steering.angle`, `hv.soc`) alongside optional
 * real-time OBD-II IGMP (`770` -> `778`, UDS `22 BC06`) brake-lamp ground truth and
 * poll-rate-independent regenerative mode (`L0 / L1 / L2 / L3 / i-Pedal`) + Smart Cruise
 * Control (`SCC`) auto-detection.
 *
 * Calibrated directly against multi-mode city, arterial, Auto-Hold ON/OFF, Auto Regen
 * (`Smart Recuperation`), drive-through stop-and-go, Smart Cruise Control, mid-drive
 * `Auto -> i-Pedal` transitions, and `84 mph` (`135.2 km/h`) highway E-GMP drive logs on both
 * `50-100 ms` patched HMG-API `Hub`s and `650 ms` legacy KitKat `Hub`s:
 * - Aligns `-dv/dt` updates to actual HMG-API poll timestamps (`hmgSampleElapsedMs`) and
 *   suppresses `< 560 ms` `0.8 km/h` cluster speed-quantization repeats.
 * - Compensates passive aerodynamic drag excess above `36 km/h` (`10 m/s`) and low-speed
 *   full-lock Ackermann tire-scrub drag (`|steering.angle| > 220 deg` below `18.5 km/h` in
 *   `L0 / L1`) so wind drag and sharp parking-lot turns neither inflate regen mode inference
 *   nor false-trigger UN ECE R13-H motor-torque brake-lamp estimation.
 * - Uses a settled `>= 21 km/h` history-ring coasting chord (`1.05-1.95 s` preferred) to
 *   separate `L0`, `L1`, `L2`, and `L3+`, steps down stale Auto-Regen `L2/L3 -> L0` on
 *   near-zero `< 2%` feather coasting, and distinguishes `i-Pedal` (`4`) from `L3` (`3`) /
 *   `Auto Regen` via low-speed full-stop regen, `16-125 km/h` neutral-pedal cruise, and
 *   sustained `6.8-22.5%` feathered one-pedal braking signatures.
 * - Automatically detects Smart Cruise Control (`sccActive`) from settled zero-pedal
 *   acceleration or sustained zero-pedal cruise (with an extended `L0` downgrade guard),
 *   freezing driver paddle regen inference and applying SCC braking thresholds until cancelled.
 * - Distinguishes console `AUTO HOLD` ON vs OFF from observed `0%`-pedal idle creep and
 *   `parking_brake` actuator release transitions so foot-brake stops with `AUTO HOLD` OFF
 *   release cleanly when the brake pedal is lifted while preserving true Auto-Hold, `i-Pedal`,
 *   and SCC standstill holds.
 */
public final class BrakeEvaluator implements BrakeModel {

    public static final String BUILTIN_MODEL_VERSION = "2026.11.19.1";
    public static final long BUILTIN_MODEL_SEQ = 2026111901L;

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
    private static final float STRONG_DECEL_RELEASE_ACCEL_PADDLE_PCT = 13.0f;
    private static final float ZERO_PEDAL_HIGH_REGEN_RELEASE_MPS2 = 0.42f;
    private static final float CREEP_TAPER_RELEASE_SPEED_KMH = 14.2f;

    // Speed-dependent i-Pedal one-pedal braking map (1.8% <= 1.5 km/h -> 5.5% @ 5 km/h -> 16.0% @ 55 km/h -> 22.6% >= 125 km/h onset;
    // 2.1% <= 1.5 km/h -> 9.6% @ 8 km/h -> 18.6% @ 55 km/h -> 24.8% >= 92 km/h release; 12.2% / 11.2% coarse-poll legacy Hub floors):
    private static final float IPEDAL_ONSET_MIN_PCT = 12.2f;
    private static final float IPEDAL_ONSET_MAX_PCT = 16.0f;
    private static final float IPEDAL_ONSET_HWY_85_PCT = 21.0f;
    private static final float IPEDAL_ONSET_HWY_MAX_PCT = 22.6f;
    private static final float IPEDAL_RELEASE_MIN_PCT = 16.8f;
    private static final float IPEDAL_RELEASE_MAX_PCT = 18.6f;
    private static final float IPEDAL_RELEASE_HWY_MAX_PCT = 24.8f;
    private static final float IPEDAL_FEATHER_IMMEDIATE_DECEL_MPS2 = 0.88f;
    private static final float IPEDAL_FEATHER_MIN_DECEL_MPS2 = 0.22f;
    private static final float IPEDAL_FEATHER_HWY_MIN_DECEL_MPS2 = 0.58f;
    private static final float IPEDAL_FEATHER_HOLD_DECEL_MPS2 = 0.20f;
    private static final long IPEDAL_FEATHER_CONFIRM_MS = 220L;
    private static final long IPEDAL_FEATHER_SETTLED_CONFIRM_MS = 450L;

    private static final float DECEL_EMA_ALPHA = 0.72f;
    private static final long DECEL_HOLD_MS = 140L;
    private static final long LIFTOFF_IPEDAL_CRUISE_RAMP_DELAY_MS = 220L;
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

    private static final float MAX_REGEN_ACCEL_PEDAL = 27.5f;
    /** Any deliberate throttle press (>= 1.0%) or creep (>= 0.7 km/h) releases standstill hold. */
    private static final float STANDSTILL_RELEASE_ACCEL = 1.0f;
    /** At 0 km/h in i-Pedal, VCU engages standstill lamp hold once pedal drops below 0.6% (0.0-0.4%). */
    private static final float STANDSTILL_IPEDAL_LATCH_ACCEL = 0.6f;
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
    private static final float IPEDAL_HWY_CRUISE_MAX_SPD_KMH = 105.0f;
    private static final long IPEDAL_CRUISE_CONFIRM_MS = 1100L;
    private static final long IPEDAL_HWY_CRUISE_CONFIRM_MS = 1650L;
    private static final long IPEDAL_FEATHER_PROMOTE_CONFIRM_MS = 1150L;
    private static final long IPEDAL_LAUNCH_CONFIRM_MS = 1350L;
    private static final long CREEP_TAPER_CONFIRM_MS = 780L;
    private static final long IPEDAL_LOW_PEDAL_CRUISE_CONFIRM_MS = 1200L;

    // Smart Cruise Control (SCC / HDA) zero-pedal travel detection:
    private static final long SCC_SETTLE_MS = 700L;
    private static final long SCC_ACCEL_CHORD_MS = 1100L;
    private static final long SCC_CRUISE_CHORD_MS = 2600L;
    private static final long SCC_CRUISE_L0_CHORD_MS = 9000L;
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
        return regenLevelLabel(level, sccActive, false);
    }

    public static String regenLevelLabel(int level, boolean sccActive, boolean autoRegenActive) {
        String base = regenLevelLabel(level);
        if (sccActive) {
            return "?".equals(base) ? "SCC" : (base + " (SCC)");
        }
        if (autoRegenActive && level != REGEN_LEVEL_IPEDAL) {
            return "?".equals(base) ? "Auto" : (base + " (Auto)");
        }
        return base;
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

    /**
     * Passive low-speed Ackermann / kinematic tire-scrub deceleration excess during sharp
     * parking-lot turns (`|steeringAngleDeg| > 220 deg` at `<= 18.5 km/h`). Prevents near-full-lock
     * steering drag (`~1.0-1.9 m/s^2` with `22BC06 = 0`) in `L0 / L1` from false-triggering
     * `REGEN_DECEL` or corrupting `RegenModeEst` to `i-Pedal`.
     */
    public static float steeringScrubExcessMps2(Float speedKmh, Float steeringAngleDeg) {
        if (speedKmh == null || steeringAngleDeg == null) {
            return 0.0f;
        }
        float spd = speedKmh.floatValue();
        float absSteer = Math.abs(steeringAngleDeg.floatValue());
        if (spd < 1.2f || spd > 18.5f || absSteer <= 220.0f) {
            return 0.0f;
        }
        float norm = clamp01((absSteer - 220.0f) / 240.0f);
        return 1.05f * norm * norm;
    }

    // === HOST_ABI_DTOS_BEGIN ===
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
        /** True when E-GMP Auto Regen (Smart Recuperation) dynamic radar modulation is detected. */
        public final boolean autoRegenActive;

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
                    gearPark, parkingBrake, moving, hvSocPct, obd, obdFresh, manualTestActive, -1, -1, false, false);
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
                    inferredRegenLevel, hevRegenRaw, false, false);
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
            this(timestampMs, illuminated, source, modelIlluminated, modelSource,
                    brakePressure, speedKmh, accelPedal, steeringAngleDeg, decelMps2,
                    gearPark, parkingBrake, moving, hvSocPct, obd, obdFresh, manualTestActive,
                    inferredRegenLevel, hevRegenRaw, sccActive, false);
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
                boolean sccActive,
                boolean autoRegenActive) {
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
            this.autoRegenActive = autoRegenActive;
        }
    }
    // === HOST_ABI_DTOS_END ===

    @Override
    public String modelVersion() {
        return BUILTIN_MODEL_VERSION;
    }

    @Override
    public long modelSeq() {
        return BUILTIN_MODEL_SEQ;
    }

    @Override
    public int modelApiVersion() {
        return BrakeModel.HOST_MODEL_API_VERSION;
    }

    @Override
    public Snapshot evaluate(
            IndicatorConfig config,
            Input input,
            Map<String, String> rawHmgReadings) {
        return evaluate(config, input);
    }

    @Override
    public void reset() {
        pressureLatched = false;
        decelLatched = false;
        standstillLatched = false;
        decelHoldUntilMs = 0L;
        lastBrakingNearStopMs = 0L;
        for (int i = 0; i < SPEED_HISTORY_CAP; i++) {
            histTimeMs[i] = 0L;
            histSpeedKmh[i] = 0.0f;
        }
        histCount = 0;
        lastProcessedHmgPollMs = -1L;
        smoothedPollIntervalMs = 100.0f;
        lastRawDecelMps2 = 0.0f;
        smoothedDecelMps2 = 0.0f;
        hasDecelSample = false;
        coastStartMs = -1L;
        lastFootBrakeMs = -1L;
        zeroPedalLiftStartMs = -1L;
        zeroPedalLiftInitDecelMps2 = 0.0f;
        featheredIpedalSinceMs = -1L;
        featheredL3SinceMs = -1L;
        ipedalLowSpdSinceMs = -1L;
        ipedalCruiseSinceMs = -1L;
        ipedalLaunchSinceMs = -1L;
        ipedalFeatherPromoteSinceMs = -1L;
        creepTaperSinceMs = -1L;
        lowPedalCruiseSinceMs = -1L;
        inferredRegenLevel = REGEN_LEVEL_UNKNOWN;
        sccActive = false;
        autoRegenActive = false;
        confirmedZeroRegenCoast = false;
        autoRegenBaseLevel = REGEN_LEVEL_0;
        coastMinChordDecel = Float.MAX_VALUE;
        coastMaxChordDecel = -Float.MAX_VALUE;
        coastRecordedHigh = false;
        consecutiveFixedHighCoasts = 0;
        autoRegenSustainedSinceMs = -1L;
        autoLightThrottleSinceMs = -1L;
        postBrakeStepConfirmed = false;
        pressureLatchPeakBar = 0.0f;
        prevParkingBrake = false;
        epbEngagedInDrive = false;
        epbAutoReleasing = false;
        autoHoldConfirmedOff = false;
        recentZeroSpeedMs = -1L;
        zeroPedalStationaryStartMs = -1L;
        lowDecelFeatherCoastSinceMs = -1L;
        sccZeroPedalStartMs = -1L;
        sccAnchorMs = -1L;
        sccAnchorSpeedKmh = 0.0f;
        sccManualThrottleSinceMs = -1L;
        sccSustainedDecelSinceMs = -1L;
        preSccRegenLevel = REGEN_LEVEL_UNKNOWN;
        preSccAutoRegen = false;
        preSccZeroCoast = false;
        subIpedalHwyVerifySinceMs = -1L;
        lastVerifiedNonIpedalMs = -1L;
        confirmedForwardDrive = false;
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
    private long featheredIpedalSinceMs = -1L;
    private long featheredL3SinceMs = -1L;
    private long ipedalLowSpdSinceMs = -1L;
    private long ipedalCruiseSinceMs = -1L;
    private long ipedalLaunchSinceMs = -1L;
    private long ipedalFeatherPromoteSinceMs = -1L;
    private long creepTaperSinceMs = -1L;
    private long lowPedalCruiseSinceMs = -1L;
    private int inferredRegenLevel = REGEN_LEVEL_UNKNOWN;

    private boolean sccActive;
    private boolean autoRegenActive;
    private boolean confirmedZeroRegenCoast;
    private int autoRegenBaseLevel = REGEN_LEVEL_0;
    private float coastMinChordDecel = Float.MAX_VALUE;
    private float coastMaxChordDecel = -Float.MAX_VALUE;
    private boolean coastRecordedHigh;
    private int consecutiveFixedHighCoasts;
    private long autoRegenSustainedSinceMs = -1L;
    private long autoLightThrottleSinceMs = -1L;
    private boolean postBrakeStepConfirmed;
    private float pressureLatchPeakBar;
    private boolean prevParkingBrake;
    private boolean epbEngagedInDrive;
    private boolean epbAutoReleasing;
    private boolean autoHoldConfirmedOff;
    private long recentZeroSpeedMs = -1L;
    private long zeroPedalStationaryStartMs = -1L;
    private long lowDecelFeatherCoastSinceMs = -1L;
    private long sccZeroPedalStartMs = -1L;
    private long sccAnchorMs = -1L;
    private float sccAnchorSpeedKmh;
    private long sccManualThrottleSinceMs = -1L;
    private long sccSustainedDecelSinceMs = -1L;
    private int preSccRegenLevel = REGEN_LEVEL_UNKNOWN;
    private boolean preSccAutoRegen;
    private boolean preSccZeroCoast;
    private long subIpedalHwyVerifySinceMs = -1L;
    private long lastVerifiedNonIpedalMs = -1L;
    private boolean confirmedForwardDrive;

    public Snapshot evaluate(IndicatorConfig cfg, Input rawIn) {
        IndicatorConfig config = cfg == null ? IndicatorConfig.defaults() : cfg;
        Input in = rawIn == null
                ? new Input(0L, null, null, null, null, null, null, null, null,
                        ObdSample.disconnected(), 0L, -1, -1L)
                : rawIn;
        long effPollMs = in.hmgSampleElapsedMs > 0L ? in.hmgSampleElapsedMs : in.nowMs;

        // 1. Update poll-aligned longitudinal deceleration (-dv/dt)
        boolean decelUpdated = updateDecel(in.nowMs, in.hmgSampleElapsedMs, in.speedKmh);

        // 2. Update hydraulic/IEB cylinder pressure latch (`brake.pressure`) with hysteresis.
        //    When rolling >= 2 km/h off-throttle and actively decelerating, E-GMP's physical
        //    brake switch (`22BC06` 37/41/45) closes at ~0.30 bar on 20 Hz Hubs (~0.20 bar on 650 ms Hubs).
        boolean fastPollHub = smoothedPollIntervalMs < 350.0f;
        boolean rollingBrakeBrush = in.speedKmh != null
                && in.speedKmh >= 2.0f
                && smoothedDecelMps2 >= (fastPollHub ? 0.20f : 0.25f)
                && (in.accelPedal == null || in.accelPedal < 1.0f);
        boolean gentleDriveFootHold = fastPollHub
                && !Boolean.TRUE.equals(in.gearPark)
                && in.speedKmh != null
                && in.speedKmh < 2.0f
                && (in.accelPedal == null || in.accelPedal < 0.8f)
                && ((in.speedKmh >= 0.3f && smoothedDecelMps2 >= 0.35f)
                        || (pressureLatched && pressureLatchPeakBar > 0.0f && pressureLatchPeakBar <= 1.55f));
        float prsrOnset = rollingBrakeBrush
                ? (config.pressureThreshold * (fastPollHub ? 0.65f : 0.50f))
                : config.pressureThreshold;
        float prsrRelease;
        if (rollingBrakeBrush || gentleDriveFootHold) {
            prsrRelease = config.pressureThreshold * (fastPollHub ? 0.45f : 0.38f);
        } else {
            prsrRelease = config.pressureThreshold * PRESSURE_RELEASE_RATIO;
        }
        if (in.brakePressure == null) {
            pressureLatched = false;
            pressureLatchPeakBar = 0.0f;
        } else if (pressureLatched) {
            if (in.brakePressure > pressureLatchPeakBar) {
                pressureLatchPeakBar = in.brakePressure;
            }
            if (in.brakePressure < prsrRelease) {
                pressureLatched = false;
                pressureLatchPeakBar = 0.0f;
            }
        } else if (in.brakePressure >= prsrOnset) {
            pressureLatched = true;
            pressureLatchPeakBar = in.brakePressure;
        }

        // 3. Update Smart Cruise Control (`sccActive`), Auto Regen, & Regen Mode (`L0..i-Pedal`) estimators
        boolean obdFresh = (config.obdMode != IndicatorConfig.ObdMode.DISABLED) && in.obd.isFreshAt(in.nowMs);
        if (Boolean.TRUE.equals(in.gearPark)) {
            lastFootBrakeMs = -1L;
            postBrakeStepConfirmed = false;
            confirmedForwardDrive = false;
        } else if (pressureLatched) {
            lastFootBrakeMs = effPollMs;
            postBrakeStepConfirmed = false;
            if (in.speedKmh == null || in.speedKmh < 4.5f) {
                confirmedForwardDrive = false;
            }
        } else {
            if (in.brakePressure != null && in.brakePressure >= 0.35f && (in.speedKmh == null || in.speedKmh < 4.5f)) {
                confirmedForwardDrive = false;
            } else if (in.speedKmh != null && in.speedKmh >= 18.0f) {
                confirmedForwardDrive = true;
            }
            if (decelUpdated && in.speedKmh != null && in.speedKmh >= 1.5f && lastRawDecelMps2 >= 0.95f) {
                postBrakeStepConfirmed = true;
            }
        }
        updateSccState(in, effPollMs);
        updateRegenModeEstimate(in, decelUpdated, effPollMs);

        // 4. Update E-GMP regenerative / Auto Regen / Smart Cruise Control deceleration latch
        float threshScale = config.decelThresholdMps2 / DEFAULT_DECEL_THRESHOLD_MPS2;
        float aeroExcess = aeroDragExcessMps2(in.speedKmh);
        float steerScrubExcess = (!sccActive && (autoRegenActive || inferredRegenLevel < REGEN_LEVEL_2))
                ? steeringScrubExcessMps2(in.speedKmh, in.steeringAngleDeg) : 0.0f;
        float motorDecelMps2 = smoothedDecelMps2 - aeroExcess - steerScrubExcess;

        float liftoffMinSpd = (inferredRegenLevel == REGEN_LEVEL_IPEDAL)
                ? 2.0f : LIFTOFF_MIN_SPEED_L2_L3_KMH;
        boolean zeroPedalAboveLiftMin = !sccActive
                && in.speedKmh != null
                && in.speedKmh >= liftoffMinSpd
                && in.accelPedal != null
                && in.accelPedal < COAST_MAX_ACCEL_PCT;

        float evalAbsSteer = in.steeringAngleDeg != null ? Math.abs(in.steeringAngleDeg.floatValue()) : 0.0f;
        boolean coarseFeatherHub = smoothedPollIntervalMs >= 350.0f;
        float sharpIpedalTurnBoost = 0.0f;
        if (!coarseFeatherHub && inferredRegenLevel == REGEN_LEVEL_IPEDAL && in.speedKmh != null) {
            if (in.speedKmh >= 9.5f && in.speedKmh < 24.0f && evalAbsSteer >= 260.0f && motorDecelMps2 >= 0.55f) {
                sharpIpedalTurnBoost = 4.6f;
            } else if (in.speedKmh >= 9.5f && in.speedKmh < 19.5f && evalAbsSteer >= 120.0f) {
                sharpIpedalTurnBoost = 2.8f;
            }
        }
        float ipedalOnsetPct = ipedalOnsetPedalPct(in.speedKmh) + sharpIpedalTurnBoost;
        float onsetThresh;
        float releaseThresh;
        if (sccActive || !autoRegenActive || inferredRegenLevel == REGEN_LEVEL_IPEDAL) {
            autoLightThrottleSinceMs = -1L;
        }
        if (sccActive) {
            onsetThresh = BASE_ONSET_SCC_MPS2 * threshScale;
            releaseThresh = BASE_RELEASE_SCC_MPS2 * threshScale;
        } else if (autoRegenActive && inferredRegenLevel != REGEN_LEVEL_IPEDAL) {
            float spdVal = in.speedKmh != null ? in.speedKmh.floatValue() : 0.0f;
            float accVal = in.accelPedal != null ? in.accelPedal.floatValue() : 0.0f;
            if (accVal >= 2.2f) {
                if (autoLightThrottleSinceMs < 0L) autoLightThrottleSinceMs = effPollMs;
            } else {
                autoLightThrottleSinceMs = -1L;
            }
            boolean sustainedAutoThrottle = smoothedPollIntervalMs < 350.0f
                    && autoLightThrottleSinceMs > 0L
                    && (effPollMs - autoLightThrottleSinceMs) >= 350L;
            boolean noRecentFootBrake = lastFootBrakeMs < 0L || (effPollMs - lastFootBrakeMs) >= 1200L;
            boolean recentFootBrakeMid = lastFootBrakeMs > 0L && (effPollMs - lastFootBrakeMs) < 1600L;
            float deadzoneAccMax = spdVal >= 56.0f ? 1.7f : 0.8f;
            if (spdVal < CREEP_TAPER_RELEASE_SPEED_KMH) {
                onsetThresh = BASE_ONSET_STANDARD_MPS2 * threshScale;
                releaseThresh = 1.08f * threshScale;
            } else if (accVal < deadzoneAccMax) {
                if (spdVal >= 39.5f) {
                    boolean settledL1OrHwyAuto = inferredRegenLevel == REGEN_LEVEL_1 || spdVal >= 60.0f;
                    onsetThresh = (settledL1OrHwyAuto ? 0.88f : 0.68f) * threshScale;
                    releaseThresh = (settledL1OrHwyAuto ? 0.78f : 0.68f) * threshScale;
                } else {
                    boolean settledAutoMidTaper = smoothedPollIntervalMs < 350.0f
                            && (recentFootBrakeMid
                                    || (zeroPedalLiftStartMs > 0L && (effPollMs - zeroPedalLiftStartMs) >= 2050L));
                    onsetThresh = 0.88f * threshScale;
                    releaseThresh = (settledAutoMidTaper ? 0.88f : 0.64f) * threshScale;
                }
            } else if (noRecentFootBrake && accVal < 4.4f) {
                onsetThresh = (!decelLatched && accVal >= 1.8f)
                        ? ((accVal >= 3.2f ? 1.25f : BASE_ONSET_STANDARD_MPS2) * threshScale)
                        : ((sustainedAutoThrottle ? 1.02f : (spdVal >= 48.0f ? 0.86f : 0.84f)) * threshScale);
                releaseThresh = (sustainedAutoThrottle ? 1.02f : 0.78f) * threshScale;
            } else if (noRecentFootBrake && spdVal >= 48.0f && accVal < 9.6f) {
                onsetThresh = (!decelLatched ? 1.25f : (sustainedAutoThrottle ? 1.08f : 1.02f)) * threshScale;
                releaseThresh = (sustainedAutoThrottle ? 1.08f : 0.88f) * threshScale;
            } else {
                onsetThresh = (accVal >= 3.2f ? 1.25f : BASE_ONSET_STANDARD_MPS2) * threshScale;
                releaseThresh = 0.75f * threshScale;
            }
        } else if (inferredRegenLevel >= REGEN_LEVEL_2) {
            boolean aboveIpedalOnset = inferredRegenLevel == REGEN_LEVEL_IPEDAL
                    && smoothedPollIntervalMs < 350.0f
                    && in.speedKmh != null
                    && (in.speedKmh < 20.0f || (!decelLatched && in.speedKmh >= 25.0f))
                    && in.accelPedal != null
                    && in.accelPedal > (ipedalOnsetPct + 0.8f);
            boolean aboveL3Onset = inferredRegenLevel == REGEN_LEVEL_3
                    && !autoRegenActive
                    && smoothedPollIntervalMs < 350.0f
                    && !decelLatched
                    && in.speedKmh != null
                    && in.speedKmh >= 42.0f
                    && in.accelPedal != null
                    && in.accelPedal > l3OnsetPedalPct(in.speedKmh);
            onsetThresh = ((aboveIpedalOnset || aboveL3Onset) ? BASE_ONSET_STANDARD_MPS2 : BASE_ONSET_HIGH_REGEN_MPS2) * threshScale;
            if (inferredRegenLevel != REGEN_LEVEL_IPEDAL
                    && smoothedPollIntervalMs < 350.0f
                    && in.speedKmh != null
                    && in.speedKmh < 18.6f) {
                releaseThresh = (in.speedKmh < CREEP_TAPER_RELEASE_SPEED_KMH
                        ? BASE_RELEASE_LOW_REGEN_MPS2 : 0.75f) * threshScale;
            } else if (zeroPedalAboveLiftMin) {
                boolean settledL2Taper = inferredRegenLevel == REGEN_LEVEL_2
                        && smoothedPollIntervalMs < 350.0f
                        && zeroPedalLiftStartMs > 0L
                        && (effPollMs - zeroPedalLiftStartMs) >= 1500L;
                releaseThresh = (settledL2Taper ? REGEN_L1_L2_MPS2 : ZERO_PEDAL_HIGH_REGEN_RELEASE_MPS2) * threshScale;
            } else if (inferredRegenLevel == REGEN_LEVEL_IPEDAL) {
                boolean lowSpdFeatherCrawl = smoothedPollIntervalMs < 350.0f
                        && in.speedKmh != null
                        && in.speedKmh < 10.5f
                        && in.accelPedal != null
                        && in.accelPedal >= 4.5f;
                boolean aboveOnsetReapply = smoothedPollIntervalMs < 350.0f
                        && in.speedKmh != null
                        && (in.speedKmh >= 32.0f || (in.speedKmh >= 20.5f && evalAbsSteer < 45.0f))
                        && in.accelPedal != null
                        && in.accelPedal > (ipedalOnsetPct + 1.2f);
                boolean highAboveOnsetReapply = aboveOnsetReapply
                        && in.speedKmh < 58.0f
                        && evalAbsSteer < 45.0f
                        && in.accelPedal > (ipedalOnsetPct + 2.2f);
                float ipedalRelBase = highAboveOnsetReapply ? 0.78f
                        : ((lowSpdFeatherCrawl || aboveOnsetReapply) ? 0.46f : IPEDAL_FEATHER_HOLD_DECEL_MPS2);
                releaseThresh = ipedalRelBase * threshScale;
            } else {
                releaseThresh = BASE_RELEASE_HYST_MPS2 * threshScale;
            }
        } else {
            boolean l0RadarJump = inferredRegenLevel == REGEN_LEVEL_0
                    && in.speedKmh != null
                    && in.speedKmh >= 24.0f
                    && in.accelPedal != null
                    && in.accelPedal < 0.8f;
            onsetThresh = (l0RadarJump ? 0.82f : BASE_ONSET_STANDARD_MPS2) * threshScale;
            releaseThresh = (l0RadarJump ? 0.70f : BASE_RELEASE_LOW_REGEN_MPS2) * threshScale;
        }

        float evalDecelMps2 = sccActive ? smoothedDecelMps2 : motorDecelMps2;
        float releaseAccelPct;
        if (autoRegenActive && inferredRegenLevel != REGEN_LEVEL_IPEDAL) {
            boolean noRecentFootBrake = lastFootBrakeMs < 0L || (effPollMs - lastFootBrakeMs) >= 1200L;
            if (decelLatched && in.speedKmh != null && in.speedKmh >= 22.0f && evalAbsSteer < 85.0f && evalDecelMps2 >= 1.18f * threshScale) {
                releaseAccelPct = 12.8f;
            } else if (noRecentFootBrake && in.speedKmh != null && in.speedKmh >= 48.0f && evalDecelMps2 >= 0.88f * threshScale) {
                releaseAccelPct = 9.6f;
            } else if (noRecentFootBrake && in.speedKmh != null && in.speedKmh >= 15.0f && evalDecelMps2 >= 0.78f * threshScale) {
                releaseAccelPct = 4.4f;
            } else {
                releaseAccelPct = (in.speedKmh != null && in.speedKmh >= 56.0f) ? 1.7f : 1.2f;
            }
        } else if (!sccActive && inferredRegenLevel == REGEN_LEVEL_IPEDAL) {
            float baseIpedalRel = ipedalReleasePedalPct(in.speedKmh) + sharpIpedalTurnBoost;
            releaseAccelPct = coarseFeatherHub ? Math.max(baseIpedalRel, 11.2f) : baseIpedalRel;
        } else {
            releaseAccelPct = regenReleaseAccelPct(inferredRegenLevel, sccActive, in.speedKmh);
        }
        float minRegenSpdKmh = (inferredRegenLevel == REGEN_LEVEL_IPEDAL)
                ? STANDSTILL_SPEED_MAX_KMH : 1.2f;

        boolean canRegenBrake = !Boolean.TRUE.equals(in.gearPark)
                && !Boolean.TRUE.equals(in.parkingBrake)
                && in.speedKmh != null
                && in.speedKmh >= minRegenSpdKmh
                && (in.accelPedal == null || in.accelPedal < MAX_REGEN_ACCEL_PEDAL);

        // Sustained mild SCC active braking (>= 0.28 m/s^2 above drag for >= 550 ms, or 20Hz highway chord drop)
        boolean sccSustainedBraking = false;
        if (sccActive && canRegenBrake && (in.accelPedal == null || in.accelPedal < 0.8f)) {
            if (motorDecelMps2 >= BASE_SUSTAINED_SCC_MPS2 * threshScale) {
                if (sccSustainedDecelSinceMs < 0L) {
                    sccSustainedDecelSinceMs = effPollMs;
                } else if ((effPollMs - sccSustainedDecelSinceMs) >= SUSTAINED_SCC_CONFIRM_MS) {
                    sccSustainedBraking = true;
                }
            } else {
                sccSustainedDecelSinceMs = -1L;
            }
            if (!sccSustainedBraking && fastPollHub && in.speedKmh >= 55.0f
                    && smoothedDecelMps2 >= 0.21f * threshScale) {
                for (int i = histCount - 1; i >= 0; i--) {
                    long age = effPollMs - histTimeMs[i];
                    if (age >= 1500L && age <= 2300L) {
                        float dropKmh = histSpeedKmh[i] - in.speedKmh.floatValue();
                        float chordDec = (dropKmh / 3.6f) / (age / 1000.0f);
                        if (dropKmh >= 1.7f && chordDec >= 0.26f * threshScale) {
                            sccSustainedBraking = true;
                        }
                        break;
                    }
                }
            }
        } else {
            sccSustainedDecelSinceMs = -1L;
        }

        // Sustained arterial/highway Auto-Regen radar braking (>= 0.60 m/s^2 off-pedal above 39.5 km/h for >= 200 ms)
        boolean autoRegenSustainedBraking = false;
        if (!sccActive && autoRegenActive && inferredRegenLevel != REGEN_LEVEL_IPEDAL && canRegenBrake
                && in.speedKmh >= 39.5f
                && (in.accelPedal == null || in.accelPedal < (in.speedKmh >= 56.0f ? 1.7f : 0.8f))
                && motorDecelMps2 >= ((inferredRegenLevel == REGEN_LEVEL_1 || in.speedKmh >= 60.0f) ? 0.85f : 0.60f) * threshScale) {
            if (autoRegenSustainedSinceMs < 0L) {
                autoRegenSustainedSinceMs = effPollMs;
            } else if ((effPollMs - autoRegenSustainedSinceMs) >= 200L) {
                autoRegenSustainedBraking = true;
            }
        } else {
            autoRegenSustainedSinceMs = -1L;
        }

        if (!zeroPedalAboveLiftMin) {
            zeroPedalLiftStartMs = -1L;
            zeroPedalLiftInitDecelMps2 = 0.0f;
        } else if (zeroPedalLiftStartMs < 0L) {
            zeroPedalLiftStartMs = effPollMs;
            zeroPedalLiftInitDecelMps2 = hasDecelSample ? smoothedDecelMps2 : 0.0f;
        }
        boolean liftoffAnticipation = false;
        if (!autoRegenActive && zeroPedalAboveLiftMin && inferredRegenLevel >= REGEN_LEVEL_2) {
            long liftElapsedMs = effPollMs - zeroPedalLiftStartMs;
            boolean coarsePollHub = smoothedPollIntervalMs >= 350.0f;
            boolean confirmedForwardLowSpdIpedal = confirmedForwardDrive
                    && inferredRegenLevel == REGEN_LEVEL_IPEDAL
                    && in.accelPedal < STANDSTILL_IPEDAL_LATCH_ACCEL
                    && in.speedKmh <= 7.0f
                    && evalAbsSteer < 180.0f
                    && (!hasDecelSample || smoothedDecelMps2 >= 0.0f);
            boolean lowSpeedLift = !coarsePollHub && in.speedKmh < 18.0f && !confirmedForwardLowSpdIpedal;
            boolean decelAllowsAnticipation = lowSpeedLift
                    ? (hasDecelSample && smoothedDecelMps2 >= 0.28f)
                    : (!hasDecelSample || smoothedDecelMps2 >= -0.20f);
            if (confirmedForwardLowSpdIpedal) {
                long reqDelayMs = (!coarsePollHub && zeroPedalLiftInitDecelMps2 < -0.25f)
                        ? LIFTOFF_ACCEL_RAMP_DELAY_MS
                        : ((!coarsePollHub && zeroPedalLiftInitDecelMps2 < 0.65f) ? LIFTOFF_IPEDAL_CRUISE_RAMP_DELAY_MS : 0L);
                liftoffAnticipation = liftElapsedMs >= reqDelayMs
                        && liftElapsedMs < LIFTOFF_CRUISE_ANTICIPATION_MAX_MS;
            } else if (!coarsePollHub && zeroPedalLiftInitDecelMps2 < -0.25f) {
                liftoffAnticipation = liftElapsedMs >= LIFTOFF_ACCEL_RAMP_DELAY_MS
                        && liftElapsedMs < LIFTOFF_ACCEL_ANTICIPATION_MAX_MS
                        && decelAllowsAnticipation;
            } else if (!coarsePollHub
                    && inferredRegenLevel == REGEN_LEVEL_IPEDAL
                    && zeroPedalLiftInitDecelMps2 < 0.65f) {
                liftoffAnticipation = liftElapsedMs >= LIFTOFF_IPEDAL_CRUISE_RAMP_DELAY_MS
                        && liftElapsedMs < LIFTOFF_CRUISE_ANTICIPATION_MAX_MS
                        && decelAllowsAnticipation;
            } else {
                boolean flatFastHubCoast = !coarsePollHub
                        && liftElapsedMs >= 350L
                        && hasDecelSample
                        && smoothedDecelMps2 < 0.30f;
                liftoffAnticipation = !flatFastHubCoast
                        && liftElapsedMs < LIFTOFF_CRUISE_ANTICIPATION_MAX_MS
                        && (lowSpeedLift
                                ? (hasDecelSample && smoothedDecelMps2 >= 0.28f)
                                : (!hasDecelSample || smoothedDecelMps2 >= -0.50f));
            }
        }

        boolean featheredIpedal = false;
        float effIpedalOnsetPct = coarseFeatherHub ? Math.max(ipedalOnsetPct, IPEDAL_ONSET_MIN_PCT) : ipedalOnsetPct;
        if (!sccActive
                && inferredRegenLevel == REGEN_LEVEL_IPEDAL
                && canRegenBrake
                && in.accelPedal != null
                && in.accelPedal <= effIpedalOnsetPct
                && (!hasDecelSample || smoothedDecelMps2 >= -0.05f)) {
            if (featheredIpedalSinceMs < 0L) {
                featheredIpedalSinceMs = effPollMs;
            }
            long featherMs = effPollMs - featheredIpedalSinceMs;
            long reqFeatherConfirmMs = coarseFeatherHub ? 0L : IPEDAL_FEATHER_CONFIRM_MS;
            float minFeatherDecel = IPEDAL_FEATHER_MIN_DECEL_MPS2;
            if (!coarseFeatherHub && in.speedKmh >= 50.0f && in.accelPedal > effIpedalOnsetPct - 0.8f) {
                minFeatherDecel = in.speedKmh >= 70.0f ? 0.36f : IPEDAL_FEATHER_HWY_MIN_DECEL_MPS2;
            } else if (!coarseFeatherHub && in.accelPedal > IPEDAL_ONSET_MAX_PCT) {
                minFeatherDecel = 0.28f;
            } else if (!coarseFeatherHub && in.speedKmh < 4.5f && in.accelPedal >= 3.2f) {
                minFeatherDecel = 0.42f;
            } else if (!coarseFeatherHub && !decelLatched && in.speedKmh < 4.5f && lastFootBrakeMs > 0L
                    && (effPollMs - lastFootBrakeMs) < 1200L) {
                minFeatherDecel = 0.95f;
            }
            if (in.accelPedal < COAST_MAX_ACCEL_PCT && in.speedKmh < 2.0f) {
                featheredIpedal = motorDecelMps2 >= IPEDAL_FEATHER_HOLD_DECEL_MPS2 * threshScale;
            } else if (motorDecelMps2 >= IPEDAL_FEATHER_IMMEDIATE_DECEL_MPS2 * threshScale) {
                featheredIpedal = true;
            } else if (featherMs >= reqFeatherConfirmMs
                    && motorDecelMps2 >= minFeatherDecel * threshScale) {
                featheredIpedal = true;
            } else if ((coarseFeatherHub || (in.speedKmh >= 9.0f && (in.speedKmh >= 14.5f || in.accelPedal >= 1.0f) && featherMs <= 1200L))
                    && featherMs >= IPEDAL_FEATHER_SETTLED_CONFIRM_MS
                    && (in.speedKmh < 55.0f ? smoothedDecelMps2 >= 0.0f : motorDecelMps2 >= 0.16f)
                    && in.accelPedal <= effIpedalOnsetPct - (in.speedKmh < 18.0f ? 0.1f : (in.speedKmh >= 55.0f ? 1.5f : 1.2f))) {
                featheredIpedal = true;
            }
        } else {
            featheredIpedalSinceMs = -1L;
        }

        boolean featheredL3 = false;
        float l3OnsetPct = l3OnsetPedalPct(in.speedKmh);
        if (!sccActive
                && !autoRegenActive
                && inferredRegenLevel == REGEN_LEVEL_3
                && smoothedPollIntervalMs < 350.0f
                && canRegenBrake
                && in.speedKmh >= 38.0f
                && l3OnsetPct > 0.0f
                && in.accelPedal != null
                && in.accelPedal <= l3OnsetPct
                && (!hasDecelSample || smoothedDecelMps2 >= 0.05f)) {
            if (featheredL3SinceMs < 0L) {
                featheredL3SinceMs = effPollMs;
            }
            long l3FeatherMs = effPollMs - featheredL3SinceMs;
            float minL3FeatherDecel = in.speedKmh >= 75.0f ? 0.52f : 0.36f;
            if (motorDecelMps2 >= 0.95f * threshScale) {
                featheredL3 = true;
            } else if (l3FeatherMs >= 100L && motorDecelMps2 >= minL3FeatherDecel * threshScale) {
                featheredL3 = true;
            }
        } else {
            featheredL3SinceMs = -1L;
        }

        float paddleStrongCancelPct;
        if (autoRegenActive && inferredRegenLevel != REGEN_LEVEL_IPEDAL) {
            boolean noRecentFootBrake = lastFootBrakeMs < 0L || (effPollMs - lastFootBrakeMs) >= 1200L;
            if (evalDecelMps2 >= 2.2f * threshScale
                    || (decelLatched && in.speedKmh != null && in.speedKmh >= 22.0f && evalAbsSteer < 85.0f && evalDecelMps2 >= 1.18f * threshScale)) {
                paddleStrongCancelPct = 12.8f;
            } else if (noRecentFootBrake && in.speedKmh != null && in.speedKmh >= 48.0f && evalDecelMps2 >= 0.88f * threshScale) {
                paddleStrongCancelPct = 9.6f;
            } else if (noRecentFootBrake && in.speedKmh != null && in.speedKmh >= 15.0f && evalDecelMps2 >= 0.78f * threshScale) {
                paddleStrongCancelPct = 4.4f;
            } else {
                paddleStrongCancelPct = 2.0f;
            }
        } else if (inferredRegenLevel == REGEN_LEVEL_0 || inferredRegenLevel == REGEN_LEVEL_1) {
            paddleStrongCancelPct = (lastFootBrakeMs > 0L && (effPollMs - lastFootBrakeMs) < 1200L) ? 2.0f : RELEASE_ACCEL_LOW_PCT;
        } else if (inferredRegenLevel == REGEN_LEVEL_3) {
            paddleStrongCancelPct = l3StrongCancelPedalPct(in.speedKmh);
        } else {
            paddleStrongCancelPct = STRONG_DECEL_RELEASE_ACCEL_PADDLE_PCT;
        }
        boolean throttleCancelsPaddleRegen = !sccActive
                && inferredRegenLevel != REGEN_LEVEL_IPEDAL
                && in.accelPedal != null
                && in.accelPedal >= paddleStrongCancelPct;
        float baseStrongCancel = ipedalStrongCancelPedalPct(in.speedKmh);
        if (!coarseFeatherHub && evalAbsSteer < 45.0f && in.speedKmh != null && in.speedKmh >= 19.0f && in.speedKmh < 48.0f) {
            baseStrongCancel -= 0.8f;
        }
        float ipedalStrongCancelPct = (coarseFeatherHub
                ? Math.max(baseStrongCancel, 11.4f)
                : baseStrongCancel) + sharpIpedalTurnBoost;
        boolean throttleCancelsIpedalRegen = !sccActive
                && inferredRegenLevel == REGEN_LEVEL_IPEDAL
                && in.accelPedal != null
                && in.accelPedal >= ipedalStrongCancelPct;
        float mildRegenCeiling;
        if (!sccActive && inferredRegenLevel == REGEN_LEVEL_IPEDAL
                && in.speedKmh != null && in.speedKmh >= 20.5f && in.speedKmh < 27.0f) {
            mildRegenCeiling = (evalAbsSteer < 45.0f ? 1.08f : 0.95f) * threshScale;
        } else if (!sccActive && autoRegenActive && inferredRegenLevel != REGEN_LEVEL_IPEDAL
                && in.speedKmh != null && in.speedKmh >= 15.0f) {
            mildRegenCeiling = 0.78f * threshScale;
        } else {
            mildRegenCeiling = config.decelThresholdMps2;
        }
        boolean throttleSuppressesMildRegen = in.accelPedal != null
                && in.accelPedal >= releaseAccelPct
                && evalDecelMps2 < mildRegenCeiling;

        boolean intersectionTurnScrub = in.speedKmh != null && in.speedKmh >= 18.0f && evalAbsSteer >= 80.0f;
        boolean highDecelAutoRegenContinuation = in.speedKmh != null
                && !intersectionTurnScrub
                && (in.accelPedal == null || in.accelPedal < 0.8f)
                && ((in.speedKmh >= 20.0f && motorDecelMps2 >= 1.98f * threshScale)
                        || (postBrakeStepConfirmed && in.speedKmh >= 12.0f && motorDecelMps2 >= 1.05f * threshScale));
        boolean strongZeroPedalRegenMode = sccActive
                || (inferredRegenLevel == REGEN_LEVEL_IPEDAL
                        && (coarseFeatherHub || in.speedKmh == null || in.speedKmh >= 6.0f || (confirmedForwardDrive && evalAbsSteer < 180.0f) || motorDecelMps2 >= 1.25f * threshScale))
                || (!autoRegenActive
                        && inferredRegenLevel != REGEN_LEVEL_IPEDAL
                        && inferredRegenLevel >= REGEN_LEVEL_2
                        && zeroPedalAboveLiftMin)
                || highDecelAutoRegenContinuation;
        long postBrakeSettleWindowMs = intersectionTurnScrub ? 950L : POST_BRAKE_REGEN_SETTLE_MS;
        boolean postFootBrakeDecelBleed = !pressureLatched
                && !strongZeroPedalRegenMode
                && lastFootBrakeMs > 0L
                && (effPollMs - lastFootBrakeMs) < postBrakeSettleWindowMs;

        if (!canRegenBrake
                || throttleCancelsPaddleRegen
                || throttleCancelsIpedalRegen
                || throttleSuppressesMildRegen
                || postFootBrakeDecelBleed) {
            decelLatched = false;
            decelHoldUntilMs = 0L;
        } else if (evalDecelMps2 >= onsetThresh || sccSustainedBraking || autoRegenSustainedBraking || liftoffAnticipation || featheredIpedal || featheredL3) {
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
        if (Boolean.TRUE.equals(in.gearPark)
                || curEpb
                || Boolean.FALSE.equals(in.parkingBrake)
                || (in.speedKmh != null && in.speedKmh >= STANDSTILL_SPEED_MAX_KMH)
                || (in.accelPedal != null && in.accelPedal >= STANDSTILL_RELEASE_ACCEL)) {
            epbAutoReleasing = false;
        } else if (prevParkingBrake && in.parkingBrake == null) {
            epbAutoReleasing = true;
        }
        if (Boolean.TRUE.equals(in.gearPark) || !curEpb) {
            epbEngagedInDrive = false;
        } else if (!prevParkingBrake && curEpb) {
            epbEngagedInDrive = true;
        }
        prevParkingBrake = curEpb;

        if (!Boolean.TRUE.equals(in.gearPark) && in.speedKmh != null && in.speedKmh < 0.3f) {
            recentZeroSpeedMs = effPollMs;
        }
        if (!sccActive
                && smoothedPollIntervalMs < 350.0f
                && inferredRegenLevel != REGEN_LEVEL_IPEDAL
                && !Boolean.TRUE.equals(in.gearPark)
                && in.speedKmh != null
                && in.speedKmh >= STANDSTILL_SPEED_MAX_KMH
                && in.speedKmh <= 5.0f
                && hasDecelSample
                && smoothedDecelMps2 <= -0.15f
                && (in.accelPedal == null || in.accelPedal < 0.8f)
                && !pressureLatched
                && (in.brakePressure == null || in.brakePressure < 0.20f)
                && recentZeroSpeedMs > 0L
                && (effPollMs - recentZeroSpeedMs) <= 2500L) {
            autoHoldConfirmedOff = true;
        }
        if (!Boolean.TRUE.equals(in.gearPark)
                && !Boolean.TRUE.equals(in.parkingBrake)
                && !epbAutoReleasing
                && !Boolean.TRUE.equals(in.moving)
                && in.speedKmh != null
                && in.speedKmh < 0.3f
                && !pressureLatched
                && (in.brakePressure == null || in.brakePressure < 0.20f)
                && (in.accelPedal == null || in.accelPedal < 0.8f)) {
            if (zeroPedalStationaryStartMs < 0L) {
                zeroPedalStationaryStartMs = effPollMs;
            } else if ((effPollMs - zeroPedalStationaryStartMs) >= 2200L) {
                autoHoldConfirmedOff = false;
            }
        } else {
            zeroPedalStationaryStartMs = -1L;
        }

        boolean footBrakeStopWithoutAutoHold = autoHoldConfirmedOff
                && !sccActive
                && inferredRegenLevel != REGEN_LEVEL_IPEDAL
                && !pressureLatched
                && lastFootBrakeMs > 0L
                && (effPollMs - lastFootBrakeMs) <= 5000L;

        boolean activeModelBraking = decelLatched || pressureLatched;
        if (Boolean.TRUE.equals(in.gearPark) || epbEngagedInDrive || epbAutoReleasing || footBrakeStopWithoutAutoHold) {
            lastBrakingNearStopMs = 0L;
        } else if (in.speedKmh != null && in.speedKmh <= 14.0f && activeModelBraking) {
            lastBrakingNearStopMs = in.nowMs;
        } else if ((in.speedKmh != null && in.speedKmh >= (fastPollHub ? 1.5f : 3.0f))
                || (in.accelPedal != null && in.accelPedal >= STANDSTILL_RELEASE_ACCEL)) {
            lastBrakingNearStopMs = 0L;
        }

        boolean decelModeEnabled = (config.signalMode == IndicatorConfig.SignalMode.PRESSURE_AND_DECEL);
        if (!decelModeEnabled || !config.standstillHold) {
            standstillLatched = false;
        } else if (Boolean.TRUE.equals(in.gearPark)
                || epbEngagedInDrive
                || epbAutoReleasing
                || footBrakeStopWithoutAutoHold
                || in.speedKmh == null
                || in.speedKmh >= STANDSTILL_SPEED_MAX_KMH
                || Boolean.TRUE.equals(in.moving)
                || (in.accelPedal != null && in.accelPedal >= STANDSTILL_RELEASE_ACCEL)) {
            standstillLatched = false;
        } else if (standstillLatched
                || (inferredRegenLevel == REGEN_LEVEL_IPEDAL
                        && (in.accelPedal == null || in.accelPedal < STANDSTILL_IPEDAL_LATCH_ACCEL))
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
                sccActive,
                autoRegenActive);
    }

    private static float clamp01(float x) {
        return x <= 0.0f ? 0.0f : (x >= 1.0f ? 1.0f : x);
    }

    public static float ipedalOnsetPedalPct(Float speedKmh) {
        float spd = speedKmh == null ? 25.0f : speedKmh.floatValue();
        if (spd < 5.0f) {
            return 1.8f + (5.5f - 1.8f) * clamp01((spd - 1.5f) / 3.5f);
        }
        if (spd < 8.0f) {
            return 5.5f + (7.5f - 5.5f) * clamp01((spd - 5.0f) / 3.0f);
        }
        if (spd < 17.0f) {
            return 7.5f + (11.0f - 7.5f) * clamp01((spd - 8.0f) / 9.0f);
        }
        if (spd < 24.0f) {
            return 11.0f + (13.4f - 11.0f) * clamp01((spd - 17.0f) / 7.0f);
        }
        if (spd < 55.0f) {
            return 13.4f + (IPEDAL_ONSET_MAX_PCT - 13.4f) * clamp01((spd - 24.0f) / 31.0f);
        }
        if (spd < 85.0f) {
            return IPEDAL_ONSET_MAX_PCT
                    + (IPEDAL_ONSET_HWY_85_PCT - IPEDAL_ONSET_MAX_PCT) * clamp01((spd - 55.0f) / 30.0f);
        }
        return IPEDAL_ONSET_HWY_85_PCT
                + (IPEDAL_ONSET_HWY_MAX_PCT - IPEDAL_ONSET_HWY_85_PCT) * clamp01((spd - 85.0f) / 40.0f);
    }

    public static float ipedalReleasePedalPct(Float speedKmh) {
        float spd = speedKmh == null ? 25.0f : speedKmh.floatValue();
        if (spd < 2.4f) {
            return 2.1f + (3.9f - 2.1f) * clamp01((spd - 1.5f) / 0.9f);
        }
        if (spd < 4.0f) {
            return 3.9f + (6.1f - 3.9f) * clamp01((spd - 2.4f) / 1.6f);
        }
        if (spd < 8.0f) {
            return 6.1f + (9.6f - 6.1f) * clamp01((spd - 4.0f) / 4.0f);
        }
        if (spd < 15.0f) {
            return 9.6f + (12.8f - 9.6f) * clamp01((spd - 8.0f) / 7.0f);
        }
        if (spd < 21.0f) {
            return 12.8f + (IPEDAL_RELEASE_MIN_PCT - 12.8f) * clamp01((spd - 15.0f) / 6.0f);
        }
        if (spd < 55.0f) {
            return IPEDAL_RELEASE_MIN_PCT
                    + (IPEDAL_RELEASE_MAX_PCT - IPEDAL_RELEASE_MIN_PCT) * clamp01((spd - 25.0f) / 30.0f);
        }
        if (spd < 60.0f) {
            return IPEDAL_RELEASE_MAX_PCT + (19.0f - IPEDAL_RELEASE_MAX_PCT) * clamp01((spd - 55.0f) / 5.0f);
        }
        return 19.0f + (IPEDAL_RELEASE_HWY_MAX_PCT - 19.0f) * clamp01((spd - 60.0f) / 32.0f);
    }

    private static float ipedalStrongCancelPedalPct(Float speedKmh) {
        float spd = speedKmh == null ? 25.0f : speedKmh.floatValue();
        float delta = spd < 19.0f ? 0.2f : (spd >= 32.0f ? 2.2f : 1.8f);
        return ipedalReleasePedalPct(speedKmh) + delta;
    }

    public static float l3OnsetPedalPct(Float speedKmh) {
        float spd = speedKmh == null ? 25.0f : speedKmh.floatValue();
        if (spd < 38.0f) {
            return 0.0f;
        }
        if (spd < 55.0f) {
            return 6.5f + (10.6f - 6.5f) * clamp01((spd - 38.0f) / 17.0f);
        }
        if (spd < 88.0f) {
            return 10.6f + (14.0f - 10.6f) * clamp01((spd - 55.0f) / 33.0f);
        }
        return 14.0f + (14.8f - 14.0f) * clamp01((spd - 88.0f) / 22.0f);
    }

    public static float l3ReleasePedalPct(Float speedKmh) {
        float spd = speedKmh == null ? 25.0f : speedKmh.floatValue();
        if (spd <= 28.0f) {
            return RELEASE_ACCEL_L3_PCT;
        }
        if (spd < 55.0f) {
            return RELEASE_ACCEL_L3_PCT + (11.8f - RELEASE_ACCEL_L3_PCT) * clamp01((spd - 28.0f) / 27.0f);
        }
        if (spd < 88.0f) {
            return 11.8f + (18.2f - 11.8f) * clamp01((spd - 55.0f) / 33.0f);
        }
        return 18.2f + (19.6f - 18.2f) * clamp01((spd - 88.0f) / 22.0f);
    }

    private static float l3StrongCancelPedalPct(Float speedKmh) {
        float spd = speedKmh == null ? 25.0f : speedKmh.floatValue();
        if (spd <= 45.0f) {
            return STRONG_DECEL_RELEASE_ACCEL_PADDLE_PCT;
        }
        if (spd < 88.0f) {
            return STRONG_DECEL_RELEASE_ACCEL_PADDLE_PCT
                    + (19.8f - STRONG_DECEL_RELEASE_ACCEL_PADDLE_PCT) * clamp01((spd - 45.0f) / 43.0f);
        }
        return 19.8f + (21.0f - 19.8f) * clamp01((spd - 88.0f) / 22.0f);
    }

    private static float regenReleaseAccelPct(int regenLevel, boolean sccActive, Float speedKmh) {
        if (sccActive) {
            return RELEASE_ACCEL_LOW_PCT;
        }
        if (regenLevel == REGEN_LEVEL_IPEDAL) {
            return ipedalReleasePedalPct(speedKmh);
        }
        if (regenLevel == REGEN_LEVEL_3) {
            return l3ReleasePedalPct(speedKmh);
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
            preSccRegenLevel = inferredRegenLevel;
            preSccAutoRegen = autoRegenActive;
            preSccZeroCoast = confirmedZeroRegenCoast;
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
            boolean preservePreSccFixedHigh = smoothedPollIntervalMs < 350.0f
                    && !sccActive
                    && preSccRegenLevel >= REGEN_LEVEL_2
                    && !preSccAutoRegen;
            int effSccGuardLevel = preservePreSccFixedHigh ? preSccRegenLevel : inferredRegenLevel;
            if (dvKmh >= 1.4f && chordDecel <= -0.18f) {
                if (preservePreSccFixedHigh) {
                    inferredRegenLevel = preSccRegenLevel;
                    autoRegenActive = false;
                    confirmedZeroRegenCoast = preSccZeroCoast;
                }
                sccActive = true;
                sccAnchorMs = effPollMs;
                sccAnchorSpeedKmh = in.speedKmh.floatValue();
            } else if (dtMs >= (effSccGuardLevel == REGEN_LEVEL_0 ? SCC_CRUISE_L0_CHORD_MS : SCC_CRUISE_CHORD_MS)
                    && in.speedKmh >= 20.0f && chordDecel <= 0.06f) {
                if (preservePreSccFixedHigh) {
                    inferredRegenLevel = preSccRegenLevel;
                    autoRegenActive = false;
                    confirmedZeroRegenCoast = preSccZeroCoast;
                }
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
            coastMinChordDecel = Float.MAX_VALUE;
            coastMaxChordDecel = -Float.MAX_VALUE;
            coastRecordedHigh = false;
            consecutiveFixedHighCoasts = 0;
            autoRegenActive = false;
            confirmedZeroRegenCoast = false;
            autoRegenBaseLevel = REGEN_LEVEL_0;
            autoRegenSustainedSinceMs = -1L;
            autoLightThrottleSinceMs = -1L;
            featheredIpedalSinceMs = -1L;
            featheredL3SinceMs = -1L;
            ipedalLowSpdSinceMs = -1L;
            ipedalCruiseSinceMs = -1L;
            ipedalLaunchSinceMs = -1L;
            ipedalFeatherPromoteSinceMs = -1L;
            creepTaperSinceMs = -1L;
            lowPedalCruiseSinceMs = -1L;
            lowDecelFeatherCoastSinceMs = -1L;
            subIpedalHwyVerifySinceMs = -1L;
            lastVerifiedNonIpedalMs = -1L;
            inferredRegenLevel = REGEN_LEVEL_UNKNOWN;
            return;
        }

        // Freeze driver paddle regen-mode inference while Smart Cruise Control owns longitudinal control
        if (sccActive) {
            coastStartMs = -1L;
            coastMinChordDecel = Float.MAX_VALUE;
            coastMaxChordDecel = -Float.MAX_VALUE;
            coastRecordedHigh = false;
            autoRegenSustainedSinceMs = -1L;
            autoLightThrottleSinceMs = -1L;
            featheredIpedalSinceMs = -1L;
            featheredL3SinceMs = -1L;
            ipedalLowSpdSinceMs = -1L;
            ipedalCruiseSinceMs = -1L;
            ipedalLaunchSinceMs = -1L;
            ipedalFeatherPromoteSinceMs = -1L;
            creepTaperSinceMs = -1L;
            lowPedalCruiseSinceMs = -1L;
            lowDecelFeatherCoastSinceMs = -1L;
            return;
        }

        // In Auto Regen, pressing throttle >= 3.5% ends temporary radar recuperation back to base coast
        if (autoRegenActive
                && inferredRegenLevel >= REGEN_LEVEL_2
                && inferredRegenLevel != REGEN_LEVEL_IPEDAL
                && in.accelPedal != null
                && in.accelPedal >= 3.5f) {
            inferredRegenLevel = autoRegenBaseLevel;
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
            if (hasDecelSample && smoothedDecelMps2 < -0.55f) {
                coastStartMs = -1L;
                coastMinChordDecel = Float.MAX_VALUE;
                coastMaxChordDecel = -Float.MAX_VALUE;
                coastRecordedHigh = false;
            } else if (coastStartMs < 0L) {
                coastStartMs = effPollMs;
                coastMinChordDecel = Float.MAX_VALUE;
                coastMaxChordDecel = -Float.MAX_VALUE;
                coastRecordedHigh = false;
            } else if (decelUpdated && (effPollMs - coastStartMs) >= (COAST_SETTLE_MS + COAST_CHORD_MIN_MS)) {
                long earliestValidMs = coastStartMs + COAST_SETTLE_MS;
                int anchorIdx = -1;
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
                        int prevLevel = inferredRegenLevel;
                        int prevForChord = (inferredRegenLevel == REGEN_LEVEL_IPEDAL)
                                ? REGEN_LEVEL_3 : inferredRegenLevel;
                        int classified = classifyCoastingRegenLevel(chordDecel, prevForChord);
                        boolean ipedalUnsettledRamp = prevLevel == REGEN_LEVEL_IPEDAL
                                && ((effPollMs - coastStartMs) < 1450L || chordDtMs < COAST_CHORD_PREF_MS);
                        boolean l3LowSpeedTaper = prevLevel == REGEN_LEVEL_3
                                && !autoRegenActive
                                && classified == REGEN_LEVEL_2
                                && (curSpd < 26.5f || (effPollMs - coastStartMs) < 1250L
                                        || (smoothedDecelMps2 - aeroDragExcessMps2(in.speedKmh)) >= 1.16f);
                        if (!ipedalUnsettledRamp && !l3LowSpeedTaper && !(classified == REGEN_LEVEL_3 && prevLevel == REGEN_LEVEL_IPEDAL)) {
                            inferredRegenLevel = classified;
                        }

                        // Detect Auto Regen (Smart Recuperation) from intra-coast radar modulation
                        // or cross-coast transitions between L0 open-road coasting and L2/L3 radar braking:
                        if (inferredRegenLevel != REGEN_LEVEL_IPEDAL && prevLevel != REGEN_LEVEL_IPEDAL) {
                            if (classified == REGEN_LEVEL_0) {
                                autoRegenBaseLevel = REGEN_LEVEL_0;
                                if (curSpd >= 24.5f && chordDtMs >= 950L) {
                                    consecutiveFixedHighCoasts = 0;
                                    if (chordDecel < 0.36f) {
                                        confirmedZeroRegenCoast = true;
                                    }
                                }
                                if (prevLevel >= REGEN_LEVEL_2) {
                                    autoRegenActive = true;
                                    consecutiveFixedHighCoasts = 0;
                                }
                            } else if (classified == REGEN_LEVEL_1 && curSpd >= 24.5f && chordDtMs >= 950L) {
                                consecutiveFixedHighCoasts = 0;
                            } else if (confirmedZeroRegenCoast && classified >= REGEN_LEVEL_2
                                    && smoothedPollIntervalMs < 350.0f && curSpd >= 24.5f
                                    && chordDtMs >= 950L && consecutiveFixedHighCoasts == 0) {
                                autoRegenActive = true;
                            }
                            if (curSpd >= 24.5f && chordDtMs >= 950L) {
                                if (chordDecel < coastMinChordDecel) coastMinChordDecel = chordDecel;
                                if (chordDecel > coastMaxChordDecel) coastMaxChordDecel = chordDecel;
                                if ((coastMinChordDecel < 0.52f && chordDecel >= 0.90f && (chordDecel - coastMinChordDecel) >= 0.45f)
                                        || (coastMaxChordDecel >= 1.08f && chordDecel <= (coastMaxChordDecel - 0.45f))) {
                                    autoRegenActive = true;
                                    consecutiveFixedHighCoasts = 0;
                                } else if (coastRecordedHigh && (coastMaxChordDecel - coastMinChordDecel) >= 0.36f) {
                                    consecutiveFixedHighCoasts = 0;
                                }
                                if (!coastRecordedHigh && (effPollMs - coastStartMs) >= 1600L && chordDecel >= 0.85f) {
                                    coastRecordedHigh = true;
                                    if (coastMinChordDecel >= 0.78f && (coastMaxChordDecel - coastMinChordDecel) < 0.36f) {
                                        consecutiveFixedHighCoasts++;
                                        if (consecutiveFixedHighCoasts >= 3) {
                                            autoRegenActive = false;
                                            confirmedZeroRegenCoast = false;
                                        }
                                    } else {
                                        consecutiveFixedHighCoasts = 0;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            coastStartMs = -1L;
            coastMinChordDecel = Float.MAX_VALUE;
            coastMaxChordDecel = -Float.MAX_VALUE;
            coastRecordedHigh = false;
        }

        // Signature B: Distinguish i-Pedal (4) from Level 3 (3) & clear stale Auto-Regen L2/L3
        if (in.speedKmh != null && in.accelPedal != null && noFootBrake && hasDecelSample) {
            float spd = in.speedKmh.floatValue();
            float acc = in.accelPedal.floatValue();

            // 1) Low-speed full-stop regen (2..14 km/h)
            float lowSpdScrub = (autoRegenActive || inferredRegenLevel < REGEN_LEVEL_2)
                    ? steeringScrubExcessMps2(in.speedKmh, in.steeringAngleDeg) : 0.0f;
            float lowSpdMotorDecel = smoothedDecelMps2 - lowSpdScrub;
            float absSteerDeg = in.steeringAngleDeg == null ? 0.0f : Math.abs(in.steeringAngleDeg.floatValue());
            boolean sharpParkingSteer = absSteerDeg >= 400.0f
                    || ((autoRegenActive || inferredRegenLevel == REGEN_LEVEL_0) && absSteerDeg >= 220.0f);
            boolean postBrakeFullySettledForIpedal = lastFootBrakeMs < 0L
                    || (effPollMs - lastFootBrakeMs) >= 1400L;
            if (!autoRegenActive
                    && postBrakeFullySettledForIpedal
                    && spd >= IPEDAL_LOW_SPD_MIN_KMH
                    && spd <= IPEDAL_LOW_SPD_MAX_KMH
                    && acc < 5.0f
                    && !sharpParkingSteer
                    && lowSpdMotorDecel >= IPEDAL_LOW_SPD_DECEL_MPS2) {
                if (ipedalLowSpdSinceMs < 0L) {
                    ipedalLowSpdSinceMs = effPollMs;
                    boolean legacyCoarsePoll = histCount >= 2
                            && (histTimeMs[histCount - 1] - histTimeMs[histCount - 2]) >= 500L;
                    if (legacyCoarsePoll && decelUpdated && lastRawDecelMps2 >= 1.50f) {
                        inferredRegenLevel = REGEN_LEVEL_IPEDAL;
                        autoRegenActive = false;
                        confirmedZeroRegenCoast = false;
                        subIpedalHwyVerifySinceMs = -1L;
                        lastVerifiedNonIpedalMs = -1L;
                    }
                } else if ((effPollMs - ipedalLowSpdSinceMs) >= IPEDAL_LOW_SPD_CONFIRM_MS) {
                    inferredRegenLevel = REGEN_LEVEL_IPEDAL;
                    autoRegenActive = false;
                    confirmedZeroRegenCoast = false;
                    subIpedalHwyVerifySinceMs = -1L;
                    lastVerifiedNonIpedalMs = -1L;
                }
            } else {
                ipedalLowSpdSinceMs = -1L;
            }

            // 2) High neutral cruise / gentle creep-up throttle (above L3's shifted neutral-pedal map:
            //    16..56 km/h urban at 20.8..25.4%+ pedal, or 56..125 km/h highway above L3's 24.6..39.5% neutral curve)
            float urbNeutralMinAcc = 20.8f + 4.6f * clamp01((spd - 28.0f) / 28.0f);
            boolean steadyNeutralCruise = spd >= 16.0f && spd <= IPEDAL_CRUISE_MAX_SPD_KMH
                    && absSteerDeg < 35.0f
                    && acc >= urbNeutralMinAcc && acc <= 31.0f
                    && Math.abs(smoothedDecelMps2) <= 0.15f;
            boolean gentleNeutralCreepUp = spd >= 18.0f && spd <= 38.0f
                    && absSteerDeg < 35.0f
                    && acc >= 23.5f && acc <= 30.5f
                    && smoothedDecelMps2 >= -0.42f && smoothedDecelMps2 <= 0.15f;
            float hwyNeutralMinAcc = (smoothedPollIntervalMs < 350.0f ? 24.6f : 24.0f)
                    + 5.8f * clamp01((spd - 65.5f) / 6.5f)
                    + 3.6f * clamp01((spd - 72.0f) / 16.0f)
                    + 0.8f * clamp01((spd - 88.0f) / 17.0f);
            float hwyNeutralMaxAcc = 30.5f
                    + 8.0f * clamp01((spd - 56.0f) / 28.0f)
                    + 11.5f * clamp01((spd - 84.0f) / 26.0f);
            float hwyNeutralMinDecel = smoothedPollIntervalMs < 350.0f ? -0.08f : -0.15f;
            float hwyNeutralMaxDecel = smoothedPollIntervalMs < 350.0f ? 0.48f : 0.15f;
            float subIpedalVerifyMaxAcc = spd > IPEDAL_CRUISE_MAX_SPD_KMH
                    ? Math.max(ipedalOnsetPedalPct(in.speedKmh) - 2.8f, hwyNeutralMinAcc - 4.2f)
                    : (urbNeutralMinAcc - 2.2f);
            float verifyMotorDecel = smoothedDecelMps2 - aeroDragExcessMps2(in.speedKmh);
            if (autoRegenActive || (inferredRegenLevel >= 0 && inferredRegenLevel < REGEN_LEVEL_2)) {
                subIpedalHwyVerifySinceMs = -1L;
                lastVerifiedNonIpedalMs = -1L;
            } else if (inferredRegenLevel >= REGEN_LEVEL_2
                    && inferredRegenLevel <= REGEN_LEVEL_3
                    && spd >= 24.0f && absSteerDeg < 30.0f
                    && acc >= 5.5f && acc <= subIpedalVerifyMaxAcc
                    && verifyMotorDecel >= -0.25f && verifyMotorDecel <= 0.22f) {
                if (subIpedalHwyVerifySinceMs < 0L) {
                    subIpedalHwyVerifySinceMs = effPollMs;
                } else if ((effPollMs - subIpedalHwyVerifySinceMs) >= 900L) {
                    lastVerifiedNonIpedalMs = effPollMs;
                }
            } else {
                subIpedalHwyVerifySinceMs = -1L;
            }
            boolean recentSubIpedalHwyCruise = !autoRegenActive
                    && inferredRegenLevel >= REGEN_LEVEL_2
                    && inferredRegenLevel <= REGEN_LEVEL_3
                    && lastVerifiedNonIpedalMs > 0L
                    && (effPollMs - lastVerifiedNonIpedalMs) <= 300000L;
            boolean highwayNeutralCruise = !recentSubIpedalHwyCruise
                    && spd > IPEDAL_CRUISE_MAX_SPD_KMH && spd <= 125.0f
                    && absSteerDeg < 25.0f
                    && acc >= hwyNeutralMinAcc && acc <= hwyNeutralMaxAcc
                    && smoothedDecelMps2 >= hwyNeutralMinDecel && smoothedDecelMps2 <= hwyNeutralMaxDecel;
            if (steadyNeutralCruise || gentleNeutralCreepUp || highwayNeutralCruise) {
                long reqCruiseConfirmMs = spd > IPEDAL_CRUISE_MAX_SPD_KMH
                        ? ((inferredRegenLevel == REGEN_LEVEL_3 && !autoRegenActive) ? 2450L : IPEDAL_HWY_CRUISE_CONFIRM_MS)
                        : IPEDAL_CRUISE_CONFIRM_MS;
                if (ipedalCruiseSinceMs < 0L) {
                    ipedalCruiseSinceMs = effPollMs;
                } else if ((effPollMs - ipedalCruiseSinceMs) >= reqCruiseConfirmMs) {
                    inferredRegenLevel = REGEN_LEVEL_IPEDAL;
                    autoRegenActive = false;
                    confirmedZeroRegenCoast = false;
                    subIpedalHwyVerifySinceMs = -1L;
                    lastVerifiedNonIpedalMs = -1L;
                }
            } else {
                ipedalCruiseSinceMs = -1L;
            }

            // 2c) Cold-start & mid-drive i-Pedal feathered one-pedal braking promotion
            //     (bounded above L3's 7..16.5% highway feathering envelope):
            float featherPromoteMotorDecel = smoothedDecelMps2 - aeroDragExcessMps2(in.speedKmh);
            float highFeatherMinAcc = 13.5f + 3.5f * clamp01((spd - 45.0f) / 30.0f);
            float highFeatherMinDecel = spd >= 56.0f ? 0.86f : 0.72f;
            boolean highFeatherPromote = spd >= 24.0f && spd <= 125.0f
                    && acc >= highFeatherMinAcc && acc <= 22.5f
                    && featherPromoteMotorDecel >= highFeatherMinDecel;
            boolean deepFeatherPromote = postBrakeFullySettledForIpedal
                    && spd >= 18.0f && spd <= 44.0f
                    && acc >= 6.8f && acc < 13.5f
                    && featherPromoteMotorDecel >= 1.05f;
            if (absSteerDeg < 35.0f && (highFeatherPromote || deepFeatherPromote)) {
                if (ipedalFeatherPromoteSinceMs < 0L) {
                    ipedalFeatherPromoteSinceMs = effPollMs;
                } else if ((effPollMs - ipedalFeatherPromoteSinceMs) >= IPEDAL_FEATHER_PROMOTE_CONFIRM_MS) {
                    inferredRegenLevel = REGEN_LEVEL_IPEDAL;
                    autoRegenActive = false;
                    confirmedZeroRegenCoast = false;
                    subIpedalHwyVerifySinceMs = -1L;
                    lastVerifiedNonIpedalMs = -1L;
                }
            } else {
                ipedalFeatherPromoteSinceMs = -1L;
            }

            // 2b) Post-Park / pull-away urban launch in i-Pedal (requires >= 29.5% pedal so brisk
            //     L3 turn-outs with high steering scrub do not false-promote before straightening)
            float minLaunchDecelMps2 = smoothedPollIntervalMs < 350.0f ? -1.72f : -1.65f;
            if (inferredRegenLevel == REGEN_LEVEL_UNKNOWN
                    && spd >= 8.0f && spd <= 36.0f
                    && acc >= 29.5f && acc <= 38.5f
                    && smoothedDecelMps2 >= minLaunchDecelMps2 && smoothedDecelMps2 <= -0.55f) {
                if (ipedalLaunchSinceMs < 0L) {
                    ipedalLaunchSinceMs = effPollMs;
                } else if ((effPollMs - ipedalLaunchSinceMs) >= IPEDAL_LAUNCH_CONFIRM_MS) {
                    inferredRegenLevel = REGEN_LEVEL_IPEDAL;
                    autoRegenActive = false;
                    confirmedZeroRegenCoast = false;
                    subIpedalHwyVerifySinceMs = -1L;
                    lastVerifiedNonIpedalMs = -1L;
                }
            } else {
                ipedalLaunchSinceMs = -1L;
            }

            // 3) Step down i-Pedal -> L3 (and verify non-i-Pedal L2/L3) if settled low-speed zero-pedal coast exhibits creep taper
            if (!autoRegenActive
                    && inferredRegenLevel >= REGEN_LEVEL_2
                    && spd >= 9.0f && spd <= 17.0f
                    && acc < COAST_MAX_ACCEL_PCT
                    && smoothedDecelMps2 >= 0.15f && smoothedDecelMps2 <= 0.80f) {
                if (creepTaperSinceMs < 0L) {
                    creepTaperSinceMs = effPollMs;
                } else if ((effPollMs - creepTaperSinceMs) >= CREEP_TAPER_CONFIRM_MS) {
                    if (inferredRegenLevel == REGEN_LEVEL_IPEDAL) {
                        inferredRegenLevel = REGEN_LEVEL_3;
                    }
                    lastVerifiedNonIpedalMs = effPollMs;
                }
            } else {
                creepTaperSinceMs = -1L;
            }

            // 4) Step down i-Pedal -> L3 if driver cruises/accelerates (>= 16 km/h) inside i-Pedal's
            //    braking zone (3..12% urban up to 24.0% at >= 90 km/h, where L3 cruises flat)
            float subIpedalCruiseMaxAcc = spd < 20.0f ? 12.0f
                    : (spd < 30.0f ? (13.0f + 1.5f * clamp01((spd - 20.0f) / 10.0f))
                    : (spd < 55.0f ? (14.5f + 4.0f * clamp01((spd - 30.0f) / 25.0f))
                    : (18.5f + 5.5f * clamp01((spd - 55.0f) / 35.0f))));
            if (inferredRegenLevel == REGEN_LEVEL_IPEDAL
                    && spd >= 16.0f
                    && absSteerDeg < 35.0f
                    && acc >= 3.0f && acc <= subIpedalCruiseMaxAcc
                    && (smoothedDecelMps2 - aeroDragExcessMps2(in.speedKmh))
                            <= (spd >= 55.0f && smoothedPollIntervalMs < 350.0f ? 0.24f : 0.08f)) {
                long reqLowPedalConfirmMs = acc <= 12.0f ? IPEDAL_LOW_PEDAL_CRUISE_CONFIRM_MS : 1450L;
                if (lowPedalCruiseSinceMs < 0L) {
                    lowPedalCruiseSinceMs = effPollMs;
                } else if ((effPollMs - lowPedalCruiseSinceMs) >= reqLowPedalConfirmMs) {
                    inferredRegenLevel = REGEN_LEVEL_3;
                }
            } else {
                lowPedalCruiseSinceMs = -1L;
            }

            // 5) Step down stale L2/L3 -> L0 when Auto Regen returns to near-zero coasting (< 0.30 m/s^2)
            float featherMotorDecel = smoothedDecelMps2 - aeroDragExcessMps2(in.speedKmh);
            if (inferredRegenLevel >= REGEN_LEVEL_2
                    && inferredRegenLevel != REGEN_LEVEL_IPEDAL
                    && spd >= 25.0f
                    && acc < 2.0f
                    && featherMotorDecel >= -0.15f
                    && featherMotorDecel < 0.30f) {
                if (lowDecelFeatherCoastSinceMs < 0L) {
                    lowDecelFeatherCoastSinceMs = effPollMs;
                } else if ((effPollMs - lowDecelFeatherCoastSinceMs) >= 900L) {
                    inferredRegenLevel = REGEN_LEVEL_0;
                    autoRegenBaseLevel = REGEN_LEVEL_0;
                    autoRegenActive = true;
                    consecutiveFixedHighCoasts = 0;
                    lowDecelFeatherCoastSinceMs = -1L;
                }
            } else {
                lowDecelFeatherCoastSinceMs = -1L;
            }
        } else {
            ipedalLowSpdSinceMs = -1L;
            ipedalCruiseSinceMs = -1L;
            ipedalLaunchSinceMs = -1L;
            ipedalFeatherPromoteSinceMs = -1L;
            creepTaperSinceMs = -1L;
            lowPedalCruiseSinceMs = -1L;
            lowDecelFeatherCoastSinceMs = -1L;
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

        // Suppress cached / 0.8 km/h quantized repeats inside SPEED_CACHE_REPEAT_MS (560 ms),
        // while bounding stale high deceleration on fast Hubs once flat-speed duration exceeds
        // the physical time required for a 0.8 km/h step.
        if (histCount > 0) {
            float prevSpeed = histSpeedKmh[histCount - 1];
            long sincePrevMs = effTimeMs - histTimeMs[histCount - 1];
            if (Math.abs(speedKmh - prevSpeed) < 0.04f && sincePrevMs < SPEED_CACHE_REPEAT_MS) {
                if (smoothedPollIntervalMs < 200.0f && sincePrevMs >= 220L && smoothedDecelMps2 > 0.0f) {
                    float quantUpperMps2 = 0.26f / (sincePrevMs / 1000.0f);
                    if (smoothedDecelMps2 > quantUpperMps2) {
                        smoothedDecelMps2 = quantUpperMps2;
                    }
                }
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
