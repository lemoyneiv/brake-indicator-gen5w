package dev.lemoyne.brakeindicator;

import java.io.File;
import java.util.List;
import java.util.Locale;

/**
 * Coordinates automatic and manual USB dataset exports and post-write RAM buffer resets.
 *
 * Triggers an automatic export whenever:
 *  1. The vehicle shifts into Park ({@code gear.park == true} near standstill) after an active
 *     out-of-Park driving/maneuvering segment ({@link Trigger#SHIFT_TO_PARK}), or
 *  2. The 30-minute in-RAM dataset buffer reaches 95% capacity
 *     ({@link #NEAR_FULL_THRESHOLD_ROWS} / {@link TelemetryRecorder#CAPACITY}, leaving 90 s of
 *     20 Hz headroom before ring wraparound) ({@link Trigger#BUFFER_NEAR_FULL}).
 *
 * Before serializing CSV rows, checks whether a removable USB drive is mounted. If no USB stick
 * is attached (or if writing fails), the in-RAM buffer is preserved intact and retries are
 * throttled ({@link #PARK_USB_RETRY_MS} / {@link #NEAR_FULL_USB_RETRY_MS}) so plugging in a USB
 * stick after shifting to Park or mid-drive exports automatically without busy-looping.
 * Immediately after a successful write, exported rows are cleared for a clean next log.
 */
public final class AutoExportCoordinator {

    /** 95% of 36,000 rows (~28.5 min at 20 Hz; leaves 1,800 rows / 90 s headroom before wrap). */
    public static final int NEAR_FULL_THRESHOLD_ROWS = 34200;

    /** Minimum stationary out-of-Park duration to arm Park export when speed stays 0 km/h. */
    public static final long OUT_OF_PARK_ARM_MS = 1000L;

    /** Retry cadence while parked with an unexported drive waiting for a USB stick. */
    public static final long PARK_USB_RETRY_MS = 3000L;

    /** Retry cadence when buffer is >= 95% full without a USB stick or after an I/O error. */
    public static final long NEAR_FULL_USB_RETRY_MS = 5000L;

    public enum Trigger {
        NONE("", ""),
        SHIFT_TO_PARK("AUTO (SHIFT_TO_PARK)", "Park"),
        BUFFER_NEAR_FULL("AUTO (BUFFER_NEAR_FULL)", "Buffer 95%"),
        MANUAL("MANUAL", "Manual");

        public final String summaryLabel;
        public final String shortLabel;

        Trigger(String summaryLabel, String shortLabel) {
            this.summaryLabel = summaryLabel;
            this.shortLabel = shortLabel;
        }
    }

    public interface UsbRootLocator {
        List<String> findMountedUsbRoots();
    }

    public interface UsbWriter {
        UsbExporter.ExportResult write(String csvContent, String summaryContent, List<String> candidateRoots);
    }

    public interface OutOfParkCheck {
        boolean isCurrentlyOutOfPark();
    }

    public static final UsbWriter DEFAULT_USB_WRITER = new UsbWriter() {
        @Override
        public UsbExporter.ExportResult write(String csvContent, String summaryContent, List<String> candidateRoots) {
            return UsbExporter.exportToDirectories(
                    csvContent, summaryContent, null, candidateRoots, UsbExporter.SUBFOLDER);
        }
    };

    public static final class StatusSnapshot {
        public final int autoExportCount;
        public final int totalExportCount;
        public final int exportSeq;
        public final boolean waitingForUsb;
        public final String waitingReason;
        public final String lastExportFileName;
        public final Trigger lastTrigger;
        public final int lastExportedRows;
        public final String lastNotice;

        StatusSnapshot(
                int autoExportCount,
                int totalExportCount,
                int exportSeq,
                boolean waitingForUsb,
                String waitingReason,
                String lastExportFileName,
                Trigger lastTrigger,
                int lastExportedRows,
                String lastNotice) {
            this.autoExportCount = autoExportCount;
            this.totalExportCount = totalExportCount;
            this.exportSeq = exportSeq;
            this.waitingForUsb = waitingForUsb;
            this.waitingReason = waitingReason;
            this.lastExportFileName = lastExportFileName;
            this.lastTrigger = lastTrigger == null ? Trigger.NONE : lastTrigger;
            this.lastExportedRows = lastExportedRows;
            this.lastNotice = lastNotice;
        }

        public String shortLastSavedLabel() {
            if (lastExportFileName == null || lastExportFileName.isEmpty()) {
                return null;
            }
            String clock = extractClockFromFileName(lastExportFileName);
            String trig = lastTrigger != Trigger.NONE ? lastTrigger.shortLabel : "USB";
            if (clock != null) {
                return clock + " (" + trig + ")";
            }
            return lastExportFileName + " (" + trig + ")";
        }

        private static String extractClockFromFileName(String name) {
            // Matches "brake-dataset-yyyyMMdd-HHmmss.csv" or "brake-dataset-yyyyMMdd-HHmmss_1.csv"
            int dash = name.lastIndexOf('-');
            if (dash < 0 || dash + 7 > name.length()) {
                return null;
            }
            String sub = name.substring(dash + 1, dash + 7);
            for (int i = 0; i < 6; i++) {
                char c = sub.charAt(i);
                if (c < '0' || c > '9') {
                    return null;
                }
            }
            return sub.substring(0, 2) + ":" + sub.substring(2, 4) + ":" + sub.substring(4, 6);
        }
    }

    private boolean sawDriveSinceClear;
    private long outOfParkSinceMonoMs = -1L;
    private boolean parkExportPending;
    private boolean loggedParkWaitNotice;
    private boolean loggedNearFullWaitNotice;
    private boolean loggedWriteErrorNotice;
    private long nextProbeAllowedMonoMs;

    private int autoExportCount;
    private int totalExportCount;
    private int exportSeq;
    private boolean waitingForUsb;
    private String waitingReason;
    private String lastExportFileName;
    private Trigger lastTrigger = Trigger.NONE;
    private int lastExportedRows;
    private String lastNotice;

    public static boolean isOutOfParkState(Boolean gearPark, Float speedKmh, Boolean moving) {
        if (Boolean.FALSE.equals(gearPark)) {
            return true;
        }
        if (Boolean.TRUE.equals(gearPark)) {
            return false;
        }
        return (speedKmh != null && speedKmh.floatValue() >= 1.5f) || Boolean.TRUE.equals(moving);
    }

    /**
     * Lightweight O(1) in-memory state update & trigger check safe to invoke at 20 Hz on evalLoop.
     * Performs zero I/O and zero heap allocations.
     */
    public synchronized Trigger pollDueTrigger(
            boolean autoExportEnabled,
            Boolean gearPark,
            Float speedKmh,
            Boolean moving,
            int bufferedRows,
            long nowMonoMs) {
        boolean movingEvidence = (speedKmh != null && speedKmh.floatValue() >= 0.5f)
                || Boolean.TRUE.equals(moving);

        if (Boolean.FALSE.equals(gearPark)) {
            if (outOfParkSinceMonoMs < 0L) {
                outOfParkSinceMonoMs = nowMonoMs;
            }
            if (movingEvidence || (nowMonoMs - outOfParkSinceMonoMs) >= OUT_OF_PARK_ARM_MS) {
                sawDriveSinceClear = true;
            }
            parkExportPending = false;
            loggedParkWaitNotice = false;
            if (waitingForUsb && "Park".equals(waitingReason)) {
                waitingForUsb = false;
                waitingReason = null;
            }
        } else {
            outOfParkSinceMonoMs = -1L;
            if (!Boolean.TRUE.equals(gearPark)
                    && speedKmh != null
                    && speedKmh.floatValue() >= 2.0f) {
                sawDriveSinceClear = true;
            }
        }

        boolean parkedNearRest = Boolean.TRUE.equals(gearPark)
                && (speedKmh == null || speedKmh.floatValue() < 3.0f);
        if (parkedNearRest && sawDriveSinceClear && bufferedRows > 0) {
            if (!parkExportPending) {
                parkExportPending = true;
                nextProbeAllowedMonoMs = 0L;
            }
        }

        if (!autoExportEnabled || bufferedRows <= 0) {
            waitingForUsb = false;
            waitingReason = null;
            return Trigger.NONE;
        }

        boolean nearFull = bufferedRows >= NEAR_FULL_THRESHOLD_ROWS;
        if (!parkExportPending && !nearFull) {
            loggedNearFullWaitNotice = false;
            loggedWriteErrorNotice = false;
            waitingForUsb = false;
            waitingReason = null;
            return Trigger.NONE;
        }

        if (nowMonoMs < nextProbeAllowedMonoMs) {
            return Trigger.NONE;
        }

        return parkExportPending ? Trigger.SHIFT_TO_PARK : Trigger.BUFFER_NEAR_FULL;
    }

    public UsbExporter.ExportResult executeExport(
            Trigger trigger,
            TelemetryRecorder recorder,
            IndicatorConfig cfg,
            String obdStatusSummary,
            String hmgStatusSummary,
            UsbRootLocator rootLocator,
            UsbWriter writer,
            final boolean currentlyOutOfPark,
            long nowMonoMs) {
        return executeExport(
                trigger,
                recorder,
                cfg,
                obdStatusSummary,
                hmgStatusSummary,
                rootLocator,
                writer,
                new OutOfParkCheck() {
                    @Override
                    public boolean isCurrentlyOutOfPark() {
                        return currentlyOutOfPark;
                    }
                },
                nowMonoMs);
    }

    /**
     * Executes an automatic or manual USB export and clears exported rows from {@code recorder}
     * strictly upon verified write success. Must be serialized externally by the caller's export lock.
     */
    public UsbExporter.ExportResult executeExport(
            Trigger trigger,
            TelemetryRecorder recorder,
            IndicatorConfig cfg,
            String obdStatusSummary,
            String hmgStatusSummary,
            UsbRootLocator rootLocator,
            UsbWriter writer,
            OutOfParkCheck outOfParkCheck,
            long nowMonoMs) {
        Trigger effectiveTrigger = trigger == null ? Trigger.MANUAL : trigger;
        int rowsBefore = recorder.bufferedRows();
        if (rowsBefore <= 0) {
            String lastFile;
            synchronized (this) {
                parkExportPending = false;
                waitingForUsb = false;
                waitingReason = null;
                lastFile = lastExportFileName;
            }
            String msg = lastFile != null
                    ? ("Buffer already exported & cleared (" + lastFile + "). Drive or record new samples first.")
                    : "No telemetry rows buffered yet.";
            return new UsbExporter.ExportResult(false, null, null, msg);
        }

        List<String> roots = rootLocator != null ? rootLocator.findMountedUsbRoots() : null;
        if (roots == null || roots.isEmpty()) {
            if (effectiveTrigger != Trigger.MANUAL) {
                String waitNote = onNoUsbAttached(effectiveTrigger, rowsBefore, nowMonoMs);
                if (waitNote != null) {
                    recorder.recordExternalEvent(nowMonoMs, waitNote);
                }
            }
            return new UsbExporter.ExportResult(
                    false,
                    null,
                    null,
                    "No mounted USB stick found. Insert a FAT32/exFAT USB drive (ensure USB port is in host mode).");
        }

        TelemetryRecorder.ExportPayload payload = recorder.snapshotForExport(
                cfg,
                obdStatusSummary,
                hmgStatusSummary,
                null,
                effectiveTrigger.summaryLabel);

        UsbWriter effectiveWriter = writer != null ? writer : DEFAULT_USB_WRITER;
        UsbExporter.ExportResult res = effectiveWriter.write(payload.csv, payload.summary, roots);
        if (res.ok) {
            String fileName = (res.csvPath != null && !res.csvPath.isEmpty())
                    ? new File(res.csvPath).getName()
                    : "brake-dataset.csv";
            String prefix = (effectiveTrigger == Trigger.MANUAL) ? "Exported " : "Auto-exported ";
            String clearNote = String.format(
                    Locale.US,
                    "%s%,d rows -> %s (%s, buffer cleared)",
                    prefix,
                    Integer.valueOf(payload.exportedRows),
                    fileName,
                    effectiveTrigger.shortLabel);
            recorder.clearAfterExport(payload.exportedTotalRecordedRows, clearNote);
            boolean outOfParkNow = outOfParkCheck != null && outOfParkCheck.isCurrentlyOutOfPark();
            onExportSucceeded(effectiveTrigger, outOfParkNow, fileName, payload.exportedRows);
            return res;
        }

        if (effectiveTrigger != Trigger.MANUAL) {
            String errNote = onExportFailed(effectiveTrigger, res.errorMessage, nowMonoMs);
            if (errNote != null) {
                recorder.recordExternalEvent(nowMonoMs, errNote);
            }
        }
        return res;
    }

    public synchronized void onManualClear(boolean currentlyOutOfPark) {
        sawDriveSinceClear = currentlyOutOfPark;
        outOfParkSinceMonoMs = -1L;
        parkExportPending = false;
        loggedParkWaitNotice = false;
        loggedNearFullWaitNotice = false;
        loggedWriteErrorNotice = false;
        waitingForUsb = false;
        waitingReason = null;
        nextProbeAllowedMonoMs = 0L;
    }

    public synchronized StatusSnapshot statusSnapshot() {
        return new StatusSnapshot(
                autoExportCount,
                totalExportCount,
                exportSeq,
                waitingForUsb,
                waitingReason,
                lastExportFileName,
                lastTrigger,
                lastExportedRows,
                lastNotice);
    }

    private synchronized String onNoUsbAttached(Trigger trigger, int bufferedRows, long nowMonoMs) {
        long retryMs = (trigger == Trigger.SHIFT_TO_PARK) ? PARK_USB_RETRY_MS : NEAR_FULL_USB_RETRY_MS;
        nextProbeAllowedMonoMs = nowMonoMs + retryMs;
        waitingForUsb = true;
        waitingReason = trigger.shortLabel;

        if (trigger == Trigger.SHIFT_TO_PARK) {
            if (!loggedParkWaitNotice) {
                loggedParkWaitNotice = true;
                lastNotice = String.format(
                        Locale.US,
                        "Auto-export waiting for USB (%s, %,d rows kept in RAM).",
                        trigger.shortLabel,
                        Integer.valueOf(bufferedRows));
                return lastNotice;
            }
        } else if (trigger == Trigger.BUFFER_NEAR_FULL) {
            if (!loggedNearFullWaitNotice) {
                loggedNearFullWaitNotice = true;
                lastNotice = String.format(
                        Locale.US,
                        "Auto-export waiting for USB (%s: %,d / %,d rows kept in RAM).",
                        trigger.shortLabel,
                        Integer.valueOf(bufferedRows),
                        Integer.valueOf(TelemetryRecorder.CAPACITY));
                return lastNotice;
            }
        }
        return null;
    }

    private synchronized void onExportSucceeded(
            Trigger trigger,
            boolean currentlyOutOfPark,
            String fileName,
            int exportedRows) {
        parkExportPending = false;
        loggedParkWaitNotice = false;
        loggedNearFullWaitNotice = false;
        loggedWriteErrorNotice = false;
        waitingForUsb = false;
        waitingReason = null;
        nextProbeAllowedMonoMs = 0L;
        sawDriveSinceClear = currentlyOutOfPark;
        outOfParkSinceMonoMs = -1L;

        totalExportCount++;
        if (trigger != Trigger.MANUAL) {
            autoExportCount++;
        }
        exportSeq++;
        lastExportFileName = fileName;
        lastTrigger = trigger;
        lastExportedRows = exportedRows;
        String actionWord = (trigger == Trigger.MANUAL) ? "Saved" : "Auto-saved";
        lastNotice = String.format(
                Locale.US,
                "%s %,d rows: %s (%s, buffer cleared)",
                actionWord,
                Integer.valueOf(exportedRows),
                fileName,
                trigger.shortLabel);
    }

    private synchronized String onExportFailed(Trigger trigger, String errorMessage, long nowMonoMs) {
        nextProbeAllowedMonoMs = nowMonoMs + NEAR_FULL_USB_RETRY_MS;
        waitingForUsb = false;
        waitingReason = null;
        if (!loggedWriteErrorNotice) {
            loggedWriteErrorNotice = true;
            String detail = (errorMessage == null || errorMessage.isEmpty()) ? "write error" : errorMessage;
            lastNotice = "Auto-export deferred (" + trigger.shortLabel + "): " + detail;
            return lastNotice;
        }
        return null;
    }
}
