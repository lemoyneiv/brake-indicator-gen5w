package dev.lemoyne.brakeindicator;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

/**
 * Bounded, zero-flash-write RAM dataset recorder and live Model-vs-OBD accuracy scorer.
 *
 * Stores up to {@link #CAPACITY} synchronized HMG-API + OBD ground-truth + Regen Level rows
 * (~30 minutes at 20 Hz, downsampled automatically to 1 Hz while parked idle) in compact
 * primitive arrays (~1.8 MB RAM) so Dalvik GC experiences zero object churn during drives.
 */
public final class TelemetryRecorder {

    public static final int CAPACITY = 36000;
    private static final int TRANSITION_LOG_CAP = 100;
    private static final long MAX_SCORING_OBD_AGE_MS = 450L;
    private static final long IDLE_PARK_DOWNSAMPLE_MS = 1000L;
    private static final long EDGE_MATCH_WINDOW_MS = 1400L;

    // Bitflags packed into flags[idx]
    private static final int FLAG_GEAR_PARK     = 1 << 0;
    private static final int FLAG_PARKING_BRAKE = 1 << 1;
    private static final int FLAG_MOVING        = 1 << 2;
    private static final int FLAG_MODEL_LIT     = 1 << 3;
    private static final int FLAG_ICON_LIT      = 1 << 4;
    private static final int FLAG_OBD_CONNECTED = 1 << 5;
    private static final int FLAG_OBD_LIT       = 1 << 6;
    private static final int FLAG_OBD_PEDAL     = 1 << 7;

    // Compact enum codes for modelSource
    private static final byte SRC_OFF             = 0;
    private static final byte SRC_BRAKE_PRESSURE  = 1;
    private static final byte SRC_REGEN_DECEL     = 2;
    private static final byte SRC_STANDSTILL_HOLD = 3;

    // Compact enum codes for OBD transport
    private static final byte TRANS_NONE   = 0;
    private static final byte TRANS_BLE    = 1;
    private static final byte TRANS_BRIDGE = 2;

    public static final class ScoreSnapshot {
        public final int bufferedRows;
        public final long totalRecordedRows;
        public final long scoredSamples;
        public final long tp;
        public final long fp;
        public final long tn;
        public final long fn;
        public final long pedalObdOnSamples;
        public final long pedalModelMatchedSamples;
        public final long regenObdOnSamples;
        public final long regenModelMatchedSamples;
        public final int onsetCount;
        public final long meanOnsetLagMs;
        public final int releaseCount;
        public final long meanReleaseLagMs;

        public ScoreSnapshot(
                int bufferedRows,
                long totalRecordedRows,
                long scoredSamples,
                long tp,
                long fp,
                long tn,
                long fn,
                long regenObdOnSamples,
                long regenModelMatchedSamples,
                int onsetCount,
                long meanOnsetLagMs,
                int releaseCount,
                long meanReleaseLagMs) {
            this(bufferedRows, totalRecordedRows, scoredSamples, tp, fp, tn, fn,
                    0L, 0L, regenObdOnSamples, regenModelMatchedSamples,
                    onsetCount, meanOnsetLagMs, releaseCount, meanReleaseLagMs);
        }

        public ScoreSnapshot(
                int bufferedRows,
                long totalRecordedRows,
                long scoredSamples,
                long tp,
                long fp,
                long tn,
                long fn,
                long pedalObdOnSamples,
                long pedalModelMatchedSamples,
                long regenObdOnSamples,
                long regenModelMatchedSamples,
                int onsetCount,
                long meanOnsetLagMs,
                int releaseCount,
                long meanReleaseLagMs) {
            this.bufferedRows = bufferedRows;
            this.totalRecordedRows = totalRecordedRows;
            this.scoredSamples = scoredSamples;
            this.tp = tp;
            this.fp = fp;
            this.tn = tn;
            this.fn = fn;
            this.pedalObdOnSamples = pedalObdOnSamples;
            this.pedalModelMatchedSamples = pedalModelMatchedSamples;
            this.regenObdOnSamples = regenObdOnSamples;
            this.regenModelMatchedSamples = regenModelMatchedSamples;
            this.onsetCount = onsetCount;
            this.meanOnsetLagMs = meanOnsetLagMs;
            this.releaseCount = releaseCount;
            this.meanReleaseLagMs = meanReleaseLagMs;
        }

        public float agreementPct() {
            if (scoredSamples <= 0L) return Float.NaN;
            return ((tp + tn) * 100.0f) / (float) scoredSamples;
        }

        public float precisionPct() {
            long denom = tp + fp;
            if (denom <= 0L) return Float.NaN;
            return (tp * 100.0f) / (float) denom;
        }

        public float recallPct() {
            long denom = tp + fn;
            if (denom <= 0L) return Float.NaN;
            return (tp * 100.0f) / (float) denom;
        }

        public float pedalRecallPct() {
            if (pedalObdOnSamples <= 0L) return Float.NaN;
            return (pedalModelMatchedSamples * 100.0f) / (float) pedalObdOnSamples;
        }

        public float regenRecallPct() {
            if (regenObdOnSamples <= 0L) return Float.NaN;
            return (regenModelMatchedSamples * 100.0f) / (float) regenObdOnSamples;
        }
    }

    private final long[] elapsedMs = new long[CAPACITY];
    private final long[] wallTimeMs = new long[CAPACITY];
    private final float[] speedKmh = new float[CAPACITY];
    private final float[] decelMps2 = new float[CAPACITY];
    private final float[] brakePressure = new float[CAPACITY];
    private final float[] accelPedal = new float[CAPACITY];
    private final float[] steeringDeg = new float[CAPACITY];
    private final float[] hvSoc = new float[CAPACITY];
    private final byte[] flags = new byte[CAPACITY];
    private final byte[] flagKnownMask = new byte[CAPACITY];
    private final byte[] modelSourceCode = new byte[CAPACITY];
    private final byte[] transportCode = new byte[CAPACITY];
    private final short[] obdAgeMs = new short[CAPACITY];
    private final short[] obdLampRaw = new short[CAPACITY];
    private final byte[] inferredRegen = new byte[CAPACITY];
    private final short[] hevRegenRaw = new short[CAPACITY];

    private int head;
    private int size;
    private long totalRecordedRows;
    private long startElapsedMs = -1L;
    private long lastStoredElapsedMs = -1L;
    private long idleParkedSinceMs = -1L;

    // Confusion matrix & lag tracking
    private long scoredSamples;
    private long tp;
    private long fp;
    private long tn;
    private long fn;
    private long pedalObdOnSamples;
    private long pedalModelMatchedSamples;
    private long regenObdOnSamples;
    private long regenModelMatchedSamples;

    private boolean prevStateValid;
    private boolean prevModelLit;
    private String prevModelSource = "OFF";
    private boolean prevObdFresh;
    private boolean prevObdLit;
    private int prevObdRaw = -1;
    private int prevInferredRegen = -1;
    private boolean prevSccActive;
    private boolean prevAutoRegenActive;

    private long lastModelRiseMs = -1L;
    private long lastModelFallMs = -1L;
    private long pendingObdRiseMs = -1L;
    private long pendingObdFallMs = -1L;
    private int onsetCount;
    private long onsetLagSumMs;
    private int releaseCount;
    private long releaseLagSumMs;

    private final Map<Integer, Long> rawCodeCounts = new TreeMap<Integer, Long>();
    private final String[] transitionRing = new String[TRANSITION_LOG_CAP];
    private int transitionHead;
    private int transitionSize;

    public static final class ExportPayload {
        public final String csv;
        public final String summary;
        public final int exportedRows;
        public final long exportedTotalRecordedRows;

        public ExportPayload(
                String csv,
                String summary,
                int exportedRows,
                long exportedTotalRecordedRows) {
            this.csv = csv;
            this.summary = summary;
            this.exportedRows = exportedRows;
            this.exportedTotalRecordedRows = exportedTotalRecordedRows;
        }
    }

    public synchronized int bufferedRows() {
        return size;
    }

    public synchronized void clear() {
        head = 0;
        size = 0;
        totalRecordedRows = 0L;
        startElapsedMs = -1L;
        lastStoredElapsedMs = -1L;
        idleParkedSinceMs = -1L;
        scoredSamples = 0L;
        tp = 0L;
        fp = 0L;
        tn = 0L;
        fn = 0L;
        pedalObdOnSamples = 0L;
        pedalModelMatchedSamples = 0L;
        regenObdOnSamples = 0L;
        regenModelMatchedSamples = 0L;
        prevStateValid = false;
        prevModelLit = false;
        prevModelSource = "OFF";
        prevObdFresh = false;
        prevObdLit = false;
        prevObdRaw = -1;
        prevInferredRegen = -1;
        prevSccActive = false;
        prevAutoRegenActive = false;
        lastModelRiseMs = -1L;
        lastModelFallMs = -1L;
        pendingObdRiseMs = -1L;
        pendingObdFallMs = -1L;
        onsetCount = 0;
        onsetLagSumMs = 0L;
        releaseCount = 0;
        releaseLagSumMs = 0L;
        rawCodeCounts.clear();
        transitionHead = 0;
        transitionSize = 0;
    }

    /**
     * Clears all rows included in a completed USB export while preserving and rebasing any
     * in-flight samples recorded after {@code exportedTotalRecordedRows} during the USB write.
     *
     * @return number of post-snapshot in-flight rows retained for the next clean log segment
     */
    public synchronized int clearAfterExport(long exportedTotalRecordedRows, String transitionNote) {
        long delta = totalRecordedRows - exportedTotalRecordedRows;
        int keep = (delta > 0L && size > 0) ? (int) Math.min((long) size, delta) : 0;
        if (keep <= 0) {
            clear();
            if (transitionNote != null && !transitionNote.isEmpty()) {
                pushTransitionLine("[   0.00s] " + transitionNote);
            }
            return 0;
        }

        int newHead = (head + (size - keep)) % CAPACITY;
        long baseRel = elapsedMs[newHead];
        for (int i = 0; i < keep; i++) {
            int idx = (newHead + i) % CAPACITY;
            elapsedMs[idx] = Math.max(0L, elapsedMs[idx] - baseRel);
        }
        head = newHead;
        size = keep;
        totalRecordedRows = keep;
        if (startElapsedMs >= 0L && baseRel > 0L) {
            startElapsedMs += baseRel;
        }
        lastStoredElapsedMs = elapsedMs[(head + keep - 1) % CAPACITY];
        idleParkedSinceMs = -1L;

        scoredSamples = 0L;
        tp = 0L;
        fp = 0L;
        tn = 0L;
        fn = 0L;
        pedalObdOnSamples = 0L;
        pedalModelMatchedSamples = 0L;
        regenObdOnSamples = 0L;
        regenModelMatchedSamples = 0L;
        prevStateValid = false;
        prevModelLit = false;
        prevModelSource = "OFF";
        prevObdFresh = false;
        prevObdLit = false;
        prevObdRaw = -1;
        prevInferredRegen = -1;
        prevSccActive = false;
        prevAutoRegenActive = false;
        lastModelRiseMs = -1L;
        lastModelFallMs = -1L;
        pendingObdRiseMs = -1L;
        pendingObdFallMs = -1L;
        onsetCount = 0;
        onsetLagSumMs = 0L;
        releaseCount = 0;
        releaseLagSumMs = 0L;
        rawCodeCounts.clear();
        transitionHead = 0;
        transitionSize = 0;

        if (transitionNote != null && !transitionNote.isEmpty()) {
            pushTransitionLine("[   0.00s] " + transitionNote);
        }

        for (int i = 0; i < keep; i++) {
            int idx = (head + i) % CAPACITY;
            int f = flags[idx] & 0xFF;
            short raw = obdLampRaw[idx];
            short age = obdAgeMs[idx];
            if (raw >= 0 && age >= 0 && age <= MAX_SCORING_OBD_AGE_MS) {
                boolean mLit = (f & FLAG_MODEL_LIT) != 0;
                boolean oLit = (f & FLAG_OBD_LIT) != 0;
                boolean oPed = (f & FLAG_OBD_PEDAL) != 0;
                scoredSamples++;
                if (mLit && oLit) tp++;
                else if (mLit) fp++;
                else if (!oLit) tn++;
                else fn++;
                if (oLit && oPed) {
                    pedalObdOnSamples++;
                    if (mLit) pedalModelMatchedSamples++;
                } else if (oLit) {
                    regenObdOnSamples++;
                    if (mLit) regenModelMatchedSamples++;
                }
                Integer key = Integer.valueOf(raw);
                Long prev = rawCodeCounts.get(key);
                rawCodeCounts.put(key, Long.valueOf(prev == null ? 1L : prev.longValue() + 1L));
            }
        }
        int lastIdx = (head + keep - 1) % CAPACITY;
        int lf = flags[lastIdx] & 0xFF;
        prevStateValid = true;
        prevModelLit = (lf & FLAG_MODEL_LIT) != 0;
        prevModelSource = decodeModelSource(modelSourceCode[lastIdx]);
        prevObdFresh = obdLampRaw[lastIdx] >= 0;
        prevObdLit = (lf & FLAG_OBD_LIT) != 0;
        prevObdRaw = obdLampRaw[lastIdx];
        prevInferredRegen = inferredRegen[lastIdx];
        return keep;
    }

    public synchronized void recordExternalEvent(long nowMonoMs, String eventText) {
        if (eventText == null || eventText.isEmpty()) return;
        if (startElapsedMs < 0L && nowMonoMs > 0L) {
            startElapsedMs = nowMonoMs;
        }
        long relMs = (startElapsedMs >= 0L && nowMonoMs >= startElapsedMs)
                ? (nowMonoMs - startElapsedMs)
                : Math.max(0L, lastStoredElapsedMs);
        pushTransitionLine(String.format(Locale.US, "[%7.2fs] %s",
                Float.valueOf(relMs / 1000.0f), eventText));
    }

    public synchronized void record(BrakeEvaluator.Snapshot snap, long currentWallTimeMs) {
        if (snap == null) return;
        long now = snap.timestampMs;
        if (startElapsedMs < 0L) {
            startElapsedMs = now;
        }
        long relMs = Math.max(0L, now - startElapsedMs);

        BrakeEvaluator.ObdSample obd = snap.obd;
        long age = obd.ageMsAt(now);
        boolean obdScorable = snap.obdFresh && age >= 0L && age <= MAX_SCORING_OBD_AGE_MS && !snap.manualTestActive;

        // 1. Update live accuracy & lag metrics + discrete transition log
        updateScoringAndTransitions(snap, relMs, obd, obdScorable);

        // 2. Downsample to 1 Hz only while quietly parked with unlit brake lamps so 30-min ring focuses on driving
        boolean quietParked = Boolean.TRUE.equals(snap.gearPark)
                && !snap.modelIlluminated
                && !snap.illuminated
                && (!snap.obdFresh || !obd.brakeLampLit)
                && (snap.speedKmh == null || snap.speedKmh < 0.5f);
        if (quietParked) {
            if (idleParkedSinceMs < 0L) {
                idleParkedSinceMs = now;
            } else if (now - idleParkedSinceMs >= 3000L
                    && lastStoredElapsedMs >= 0L
                    && (relMs - lastStoredElapsedMs) < IDLE_PARK_DOWNSAMPLE_MS) {
                return;
            }
        } else {
            idleParkedSinceMs = -1L;
        }

        // 3. Write sample into zero-allocation primitive ring buffer
        int idx;
        if (size < CAPACITY) {
            idx = (head + size) % CAPACITY;
            size++;
        } else {
            idx = head;
            head = (head + 1) % CAPACITY;
        }

        elapsedMs[idx] = relMs;
        wallTimeMs[idx] = currentWallTimeMs;
        speedKmh[idx] = unboxFloat(snap.speedKmh);
        decelMps2[idx] = snap.decelMps2;
        brakePressure[idx] = unboxFloat(snap.brakePressure);
        accelPedal[idx] = unboxFloat(snap.accelPedal);
        steeringDeg[idx] = unboxFloat(snap.steeringAngleDeg);
        hvSoc[idx] = unboxFloat(snap.hvSocPct);

        int f = 0;
        int known = 0;
        if (snap.gearPark != null) {
            known |= FLAG_GEAR_PARK;
            if (snap.gearPark.booleanValue()) f |= FLAG_GEAR_PARK;
        }
        if (snap.parkingBrake != null) {
            known |= FLAG_PARKING_BRAKE;
            if (snap.parkingBrake.booleanValue()) f |= FLAG_PARKING_BRAKE;
        }
        if (snap.moving != null) {
            known |= FLAG_MOVING;
            if (snap.moving.booleanValue()) f |= FLAG_MOVING;
        }
        if (snap.modelIlluminated) f |= FLAG_MODEL_LIT;
        if (snap.illuminated) f |= FLAG_ICON_LIT;
        if (obd.connected) f |= FLAG_OBD_CONNECTED;
        if (snap.obdFresh && obd.brakeLampLit) f |= FLAG_OBD_LIT;
        if (snap.obdFresh && obd.pedalBit) f |= FLAG_OBD_PEDAL;

        flags[idx] = (byte) f;
        flagKnownMask[idx] = (byte) known;
        modelSourceCode[idx] = encodeModelSource(snap.modelSource);
        transportCode[idx] = encodeTransport(obd.transport);
        obdAgeMs[idx] = (short) (age < 0L ? -1 : Math.min(32000L, age));
        obdLampRaw[idx] = (short) (snap.obdFresh ? obd.rawCode : -1);
        inferredRegen[idx] = (byte) (snap.inferredRegenLevel >= 0 && snap.inferredRegenLevel <= 10
                ? snap.inferredRegenLevel : -1);
        hevRegenRaw[idx] = (short) (snap.hevRegenRaw >= 0
                ? Math.min(32000, snap.hevRegenRaw) : -1);

        lastStoredElapsedMs = relMs;
        totalRecordedRows++;
    }

    private void updateScoringAndTransitions(
            BrakeEvaluator.Snapshot snap,
            long relMs,
            BrakeEvaluator.ObdSample obd,
            boolean obdScorable) {
        boolean modelLit = snap.modelIlluminated;
        boolean obdLit = snap.obdFresh && obd.brakeLampLit;
        int obdRaw = snap.obdFresh ? obd.rawCode : -1;

        if (obdScorable) {
            scoredSamples++;
            if (modelLit && obdLit) {
                tp++;
            } else if (modelLit && !obdLit) {
                fp++;
            } else if (!modelLit && !obdLit) {
                tn++;
            } else {
                fn++;
            }
            if (obdLit && obd.pedalBit) {
                pedalObdOnSamples++;
                if (modelLit) {
                    pedalModelMatchedSamples++;
                }
            } else if (obdLit && !obd.pedalBit) {
                regenObdOnSamples++;
                if (modelLit) {
                    regenModelMatchedSamples++;
                }
            }
            Integer key = Integer.valueOf(obdRaw);
            Long prevCount = rawCodeCounts.get(key);
            rawCodeCounts.put(key, Long.valueOf(prevCount == null ? 1L : prevCount.longValue() + 1L));
        }

        if (!prevStateValid) {
            prevStateValid = true;
            prevModelLit = modelLit;
            prevModelSource = snap.modelSource;
            prevObdFresh = snap.obdFresh;
            prevObdLit = obdLit;
            prevObdRaw = obdRaw;
            prevInferredRegen = snap.inferredRegenLevel;
            prevSccActive = snap.sccActive;
            prevAutoRegenActive = snap.autoRegenActive;
            return;
        }

        boolean modelChanged = (modelLit != prevModelLit) || !snap.modelSource.equals(prevModelSource);
        boolean obdChanged = (snap.obdFresh != prevObdFresh) || (snap.obdFresh && obdRaw != prevObdRaw);
        boolean regenChanged = (snap.inferredRegenLevel >= 0 && snap.inferredRegenLevel != prevInferredRegen);
        boolean sccChanged = (snap.sccActive != prevSccActive);
        boolean autoRegenChanged = (snap.autoRegenActive != prevAutoRegenActive);

        // Edge lag Pairing (signed: positive = HMG-API model lagged behind real OBD)
        if (modelLit && !prevModelLit) {
            lastModelRiseMs = relMs;
            if (pendingObdRiseMs >= 0L && (relMs - pendingObdRiseMs) <= EDGE_MATCH_WINDOW_MS) {
                onsetLagSumMs += (relMs - pendingObdRiseMs);
                onsetCount++;
                pendingObdRiseMs = -1L;
            }
        } else if (!modelLit && prevModelLit) {
            lastModelFallMs = relMs;
            if (pendingObdFallMs >= 0L && (relMs - pendingObdFallMs) <= EDGE_MATCH_WINDOW_MS) {
                releaseLagSumMs += (relMs - pendingObdFallMs);
                releaseCount++;
                pendingObdFallMs = -1L;
            }
        }

        if (snap.obdFresh && prevObdFresh) {
            if (obdLit && !prevObdLit) {
                if (modelLit && lastModelRiseMs >= 0L && (relMs - lastModelRiseMs) <= EDGE_MATCH_WINDOW_MS) {
                    onsetLagSumMs += (lastModelRiseMs - relMs); // negative = model led OBD
                    onsetCount++;
                    pendingObdRiseMs = -1L;
                } else {
                    pendingObdRiseMs = relMs;
                }
            } else if (!obdLit && prevObdLit) {
                if (!modelLit && lastModelFallMs >= 0L && (relMs - lastModelFallMs) <= EDGE_MATCH_WINDOW_MS) {
                    releaseLagSumMs += (lastModelFallMs - relMs);
                    releaseCount++;
                    pendingObdFallMs = -1L;
                } else {
                    pendingObdFallMs = relMs;
                }
            }
        }

        if (sccChanged) {
            pushTransitionLine(String.format(Locale.US,
                    "[%7.2fs] SmartCruiseEst: %s (spd=%s km/h, reg=%s)",
                    Float.valueOf(relMs / 1000.0f),
                    snap.sccActive ? "ACTIVE" : "OFF",
                    fmtFloat(unboxFloat(snap.speedKmh), "%.1f"),
                    BrakeEvaluator.regenLevelLabel(snap.inferredRegenLevel)));
        }

        if (autoRegenChanged) {
            pushTransitionLine(String.format(Locale.US,
                    "[%7.2fs] AutoRegenEst: %s (spd=%s km/h, reg=%s)",
                    Float.valueOf(relMs / 1000.0f),
                    snap.autoRegenActive ? "ACTIVE" : "OFF",
                    fmtFloat(unboxFloat(snap.speedKmh), "%.1f"),
                    BrakeEvaluator.regenLevelLabel(snap.inferredRegenLevel)));
        }

        if (regenChanged) {
            String fromLbl = BrakeEvaluator.regenLevelLabel(prevInferredRegen, prevSccActive, prevAutoRegenActive);
            String toLbl = BrakeEvaluator.regenLevelLabel(snap.inferredRegenLevel, snap.sccActive, snap.autoRegenActive);
            pushTransitionLine(String.format(Locale.US,
                    "[%7.2fs] RegenModeEst: %s -> %s (decel=%.2f m/s^2 @ %s km/h)",
                    Float.valueOf(relMs / 1000.0f),
                    fromLbl,
                    toLbl,
                    Float.valueOf(snap.decelMps2),
                    fmtFloat(unboxFloat(snap.speedKmh), "%.1f")));
        }

        if (modelChanged || obdChanged) {
            StringBuilder line = new StringBuilder(120);
            line.append(String.format(Locale.US, "[%7.2fs] ", Float.valueOf(relMs / 1000.0f)));
            if (snap.obdFresh) {
                line.append("OBD=").append(obdLit ? "ON" : "OFF")
                        .append('(').append(obdRaw)
                        .append(obdLit ? (obd.pedalBit ? ":PEDAL" : ":REGEN") : "")
                        .append(") ");
            } else {
                line.append("OBD=-- ");
            }
            line.append("Model=").append(modelLit ? snap.modelSource : "OFF")
                    .append(" | spd=").append(fmtFloat(unboxFloat(snap.speedKmh), "%.1f"))
                    .append(" decel=").append(String.format(Locale.US, "%.2f", Float.valueOf(snap.decelMps2)))
                    .append(" prsr=").append(fmtFloat(unboxFloat(snap.brakePressure), "%.1f"))
                    .append(" acc=").append(fmtFloat(unboxFloat(snap.accelPedal), "%.1f"));
            if (snap.inferredRegenLevel >= 0 || snap.sccActive || snap.autoRegenActive) {
                line.append(" reg=").append(BrakeEvaluator.regenLevelLabel(
                        snap.inferredRegenLevel, snap.sccActive, snap.autoRegenActive));
            }
            pushTransitionLine(line.toString());
        }

        prevModelLit = modelLit;
        prevModelSource = snap.modelSource;
        prevObdFresh = snap.obdFresh;
        prevObdLit = obdLit;
        prevObdRaw = obdRaw;
        prevInferredRegen = snap.inferredRegenLevel;
        prevSccActive = snap.sccActive;
        prevAutoRegenActive = snap.autoRegenActive;
    }

    private void pushTransitionLine(String s) {
        if (transitionSize < TRANSITION_LOG_CAP) {
            int idx = (transitionHead + transitionSize) % TRANSITION_LOG_CAP;
            transitionRing[idx] = s;
            transitionSize++;
        } else {
            transitionRing[transitionHead] = s;
            transitionHead = (transitionHead + 1) % TRANSITION_LOG_CAP;
        }
    }

    public synchronized ScoreSnapshot scoreSnapshot() {
        long meanOnset = onsetCount > 0 ? Math.round((double) onsetLagSumMs / (double) onsetCount) : 0L;
        long meanRelease = releaseCount > 0 ? Math.round((double) releaseLagSumMs / (double) releaseCount) : 0L;
        return new ScoreSnapshot(
                size,
                totalRecordedRows,
                scoredSamples,
                tp,
                fp,
                tn,
                fn,
                pedalObdOnSamples,
                pedalModelMatchedSamples,
                regenObdOnSamples,
                regenModelMatchedSamples,
                onsetCount,
                meanOnset,
                releaseCount,
                meanRelease);
    }

    public synchronized String recentTransitionsText(int maxLines) {
        if (transitionSize == 0) {
            return "No brake-lamp, regen-level, or HMG-API transitions recorded yet.";
        }
        int count = Math.min(transitionSize, Math.max(1, maxLines));
        int startOffset = transitionSize - count;
        StringBuilder sb = new StringBuilder(count * 96);
        for (int i = count - 1; i >= 0; i--) {
            int idx = (transitionHead + startOffset + i) % TRANSITION_LOG_CAP;
            if (sb.length() > 0) sb.append('\n');
            sb.append(transitionRing[idx]);
        }
        return sb.toString();
    }

    public synchronized String toCsvString() {
        StringBuilder sb = new StringBuilder(Math.max(256, size * 96));
        sb.append("elapsed_ms,wall_iso,hmg_speed_kmh,hmg_decel_mps2,hmg_brake_pressure,")
          .append("hmg_pedal_accel,hmg_steering_deg,hmg_gear_park,hmg_parking_brake,hmg_moving,")
          .append("hmg_hv_soc,model_lit,model_source,icon_lit,obd_connected,obd_transport,")
          .append("obd_age_ms,obd_lamp_raw,obd_lamp_lit,obd_pedal_bit,inferred_regen_level,hev_regen_raw\n");

        SimpleDateFormat iso = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSSZ", Locale.US);
        Date d = new Date();

        for (int i = 0; i < size; i++) {
            int idx = (head + i) % CAPACITY;
            int f = flags[idx] & 0xFF;
            int k = flagKnownMask[idx] & 0xFF;
            short raw = obdLampRaw[idx];
            boolean hasObdVal = raw >= 0;
            byte infReg = inferredRegen[idx];
            short hevRaw = hevRegenRaw[idx];

            d.setTime(wallTimeMs[idx]);
            sb.append(elapsedMs[idx]).append(',')
              .append(iso.format(d)).append(',')
              .append(csvFloat(speedKmh[idx], "%.2f")).append(',')
              .append(String.format(Locale.US, "%.3f", Float.valueOf(decelMps2[idx]))).append(',')
              .append(csvFloat(brakePressure[idx], "%.2f")).append(',')
              .append(csvFloat(accelPedal[idx], "%.2f")).append(',')
              .append(csvFloat(steeringDeg[idx], "%.2f")).append(',')
              .append(csvFlag(f, k, FLAG_GEAR_PARK)).append(',')
              .append(csvFlag(f, k, FLAG_PARKING_BRAKE)).append(',')
              .append(csvFlag(f, k, FLAG_MOVING)).append(',')
              .append(csvFloat(hvSoc[idx], "%.1f")).append(',')
              .append((f & FLAG_MODEL_LIT) != 0 ? '1' : '0').append(',')
              .append(decodeModelSource(modelSourceCode[idx])).append(',')
              .append((f & FLAG_ICON_LIT) != 0 ? '1' : '0').append(',')
              .append((f & FLAG_OBD_CONNECTED) != 0 ? '1' : '0').append(',')
              .append(decodeTransport(transportCode[idx])).append(',')
              .append(obdAgeMs[idx] >= 0 ? Integer.toString(obdAgeMs[idx]) : "").append(',')
              .append(hasObdVal ? Integer.toString(raw) : "").append(',')
              .append(hasObdVal ? ((f & FLAG_OBD_LIT) != 0 ? "1" : "0") : "").append(',')
              .append(hasObdVal ? ((f & FLAG_OBD_PEDAL) != 0 ? "1" : "0") : "").append(',')
              .append(infReg >= 0 ? Integer.toString(infReg) : "").append(',')
              .append(hevRaw >= 0 ? Integer.toString(hevRaw) : "").append('\n');
        }
        return sb.toString();
    }

    public synchronized ExportPayload snapshotForExport(
            IndicatorConfig cfg,
            String obdStatusSummary,
            String hmgStatusSummary,
            String hevProbeCompactSummary,
            String exportTriggerLabel) {
        int rowsNow = size;
        long totalNow = totalRecordedRows;
        String csv = toCsvString();
        String summary = buildSummaryReport(
                cfg, obdStatusSummary, hmgStatusSummary, hevProbeCompactSummary, exportTriggerLabel);
        return new ExportPayload(csv, summary, rowsNow, totalNow);
    }

    public synchronized String buildSummaryReport(IndicatorConfig cfg, String obdStatusSummary) {
        return buildSummaryReport(cfg, obdStatusSummary, null, null, null);
    }

    public synchronized String buildSummaryReport(
            IndicatorConfig cfg,
            String obdStatusSummary,
            String hmgStatusSummary,
            String hevProbeCompactSummary) {
        return buildSummaryReport(cfg, obdStatusSummary, hmgStatusSummary, hevProbeCompactSummary, null);
    }

    public synchronized String buildSummaryReport(
            IndicatorConfig cfg,
            String obdStatusSummary,
            String hmgStatusSummary,
            String hevProbeCompactSummary,
            String exportTriggerLabel) {
        ScoreSnapshot sc = scoreSnapshot();
        StringBuilder sb = new StringBuilder(2304);
        sb.append("=== Brake Light Indicator v0.8.1 Calibration Summary ===\n");
        sb.append("Export Time         : ")
          .append(new SimpleDateFormat("yyyy-MM-dd HH:mm:ss Z", Locale.US).format(new Date()))
          .append('\n');
        if (exportTriggerLabel != null && !exportTriggerLabel.isEmpty()) {
            sb.append("Export Trigger      : ").append(exportTriggerLabel).append('\n');
        }
        if (cfg != null) {
            sb.append("OBD Mode            : ").append(cfg.obdMode.wireValue).append('\n');
            sb.append("HMG-API Model Mode  : ").append(cfg.signalMode.name()).append('\n');
            sb.append("Thresholds          : pressure=").append(cfg.pressureThreshold)
              .append(", decel=").append(cfg.decelThresholdMps2).append(" m/s^2")
              .append(", standstillHold=").append(cfg.standstillHold)
              .append(", autoExportUsb=").append(cfg.autoExportUsb)
              .append(", autoUploadCloud=").append(cfg.autoUploadCloud).append('\n');
        }
        if (hmgStatusSummary != null) {
            sb.append("HMG-API Session     : ").append(hmgStatusSummary).append('\n');
        }
        sb.append("OBD Session Status  : ").append(obdStatusSummary == null ? "-" : obdStatusSummary).append('\n');
        if (hevProbeCompactSummary != null) {
            sb.append("HEV Regen Probe     : ").append(hevProbeCompactSummary).append('\n');
        }
        sb.append("Buffered CSV Rows   : ").append(sc.bufferedRows)
          .append(" / ").append(CAPACITY)
          .append(" (total recorded: ").append(sc.totalRecordedRows).append(")\n\n");

        sb.append("--- HMG-API Model vs Real OBD (770/22BC06) Ground Truth ---\n");
        sb.append("Scored OBD Ticks    : ").append(sc.scoredSamples).append('\n');
        sb.append("Confusion Matrix    : TP=").append(sc.tp)
          .append("  FP=").append(sc.fp)
          .append("  TN=").append(sc.tn)
          .append("  FN=").append(sc.fn).append('\n');
        sb.append("Overall Agreement   : ").append(fmtPct(sc.agreementPct())).append('\n');
        sb.append("Model Precision     : ").append(fmtPct(sc.precisionPct())).append('\n');
        sb.append("Model Recall (All)  : ").append(fmtPct(sc.recallPct())).append('\n');
        sb.append("Pedal Switch Rec.   : ").append(fmtPct(sc.pedalRecallPct()))
          .append(" (").append(sc.pedalModelMatchedSamples).append('/').append(sc.pedalObdOnSamples).append(" ticks)\n");
        sb.append("Pure Regen/SCC Rec. : ").append(fmtPct(sc.regenRecallPct()))
          .append(" (").append(sc.regenModelMatchedSamples).append('/').append(sc.regenObdOnSamples).append(" ticks)\n");
        sb.append("Mean Onset Lag      : ")
          .append(sc.onsetCount > 0 ? (sc.meanOnsetLagMs + " ms (n=" + sc.onsetCount + ")") : "n/a")
          .append('\n');
        sb.append("Mean Release Lag    : ")
          .append(sc.releaseCount > 0 ? (sc.meanReleaseLagMs + " ms (n=" + sc.releaseCount + ")") : "n/a")
          .append('\n');

        sb.append("Observed 22BC06 Raw : ");
        if (rawCodeCounts.isEmpty()) {
            sb.append("none\n");
        } else {
            boolean first = true;
            for (Map.Entry<Integer, Long> e : rawCodeCounts.entrySet()) {
                if (!first) sb.append(", ");
                first = false;
                sb.append(e.getKey()).append('=').append(e.getValue());
            }
            sb.append('\n');
        }

        sb.append("\n--- Recent Transitions (newest first) ---\n");
        sb.append(recentTransitionsText(TRANSITION_LOG_CAP)).append('\n');
        return sb.toString();
    }

    private static float unboxFloat(Float v) {
        return v == null ? Float.NaN : v.floatValue();
    }

    private static String csvFloat(float v, String fmt) {
        if (Float.isNaN(v) || Float.isInfinite(v)) {
            return "";
        }
        return String.format(Locale.US, fmt, Float.valueOf(v));
    }

    private static String fmtFloat(float v, String fmt) {
        if (Float.isNaN(v) || Float.isInfinite(v)) {
            return "-";
        }
        return String.format(Locale.US, fmt, Float.valueOf(v));
    }

    private static String fmtPct(float pct) {
        if (Float.isNaN(pct)) {
            return "n/a";
        }
        return String.format(Locale.US, "%.1f%%", Float.valueOf(pct));
    }

    private static String csvFlag(int flags, int known, int mask) {
        if ((known & mask) == 0) {
            return "";
        }
        return (flags & mask) != 0 ? "1" : "0";
    }

    private static byte encodeModelSource(String s) {
        if ("BRAKE_PRESSURE".equals(s)) return SRC_BRAKE_PRESSURE;
        if ("REGEN_DECEL".equals(s)) return SRC_REGEN_DECEL;
        if ("STANDSTILL_HOLD".equals(s)) return SRC_STANDSTILL_HOLD;
        return SRC_OFF;
    }

    private static String decodeModelSource(byte code) {
        if (code == SRC_BRAKE_PRESSURE) return "BRAKE_PRESSURE";
        if (code == SRC_REGEN_DECEL) return "REGEN_DECEL";
        if (code == SRC_STANDSTILL_HOLD) return "STANDSTILL_HOLD";
        return "OFF";
    }

    private static byte encodeTransport(String t) {
        if ("BLE".equalsIgnoreCase(t)) return TRANS_BLE;
        if ("BRIDGE".equalsIgnoreCase(t)) return TRANS_BRIDGE;
        return TRANS_NONE;
    }

    private static String decodeTransport(byte code) {
        if (code == TRANS_BLE) return "BLE";
        if (code == TRANS_BRIDGE) return "BRIDGE";
        return "";
    }
}
