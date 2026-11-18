package dev.lemoyne.brakeindicator;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Coordinates automatic and manual dataset exports/uploads across GitHub Cloud (LTE/Wi-Fi),
 * the bounded in-RAM {@link PendingUploadQueue}, and removable USB storage, clearing exported
 * rows from {@link TelemetryRecorder} immediately after packaging or verified USB write.
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

    /** Background drain cadence while compressed drives sit in PendingUploadQueue offline. */
    public static final long CLOUD_QUEUE_RETRY_MS = 15000L;

    /** Longer backoff after non-retryable HTTP auth/config error (401/403/404). */
    public static final long CLOUD_AUTH_BACKOFF_MS = 60000L;

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

    public enum TransportKind {
        NONE,
        WIFI,
        CELLULAR
    }

    public static final class TransportInfo {
        public final TransportKind kind;
        public final String label;

        public TransportInfo(TransportKind kind, String label) {
            this.kind = kind == null ? TransportKind.NONE : kind;
            this.label = (label == null || label.isEmpty()) ? "Offline" : label;
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

    public interface NetworkCheck {
        TransportInfo activeTransport();
    }

    public interface CloudUploader {
        GitHubUploader.UploadResult upload(CloudUploadConfig cfg, PendingUploadQueue.QueuedDrive drive);
    }

    public static final UsbWriter DEFAULT_USB_WRITER = new UsbWriter() {
        @Override
        public UsbExporter.ExportResult write(String csvContent, String summaryContent, List<String> candidateRoots) {
            return UsbExporter.exportToDirectories(
                    csvContent, summaryContent, null, candidateRoots, UsbExporter.SUBFOLDER);
        }
    };

    public static final CloudUploader DEFAULT_CLOUD_UPLOADER = new CloudUploader() {
        private final GitHubUploader uploader = new GitHubUploader();

        @Override
        public GitHubUploader.UploadResult upload(CloudUploadConfig cfg, PendingUploadQueue.QueuedDrive drive) {
            return uploader.uploadDrive(cfg, drive);
        }
    };

    public static final class ActionOutcome {
        public final boolean ok;
        public final boolean uploadedToCloud;
        public final boolean queuedInRam;
        public final boolean wroteToUsb;
        public final String primaryTarget;
        public final String message;

        public ActionOutcome(
                boolean ok,
                boolean uploadedToCloud,
                boolean queuedInRam,
                boolean wroteToUsb,
                String primaryTarget,
                String message) {
            this.ok = ok;
            this.uploadedToCloud = uploadedToCloud;
            this.queuedInRam = queuedInRam;
            this.wroteToUsb = wroteToUsb;
            this.primaryTarget = primaryTarget;
            this.message = message;
        }
    }

    public static final class StatusSnapshot {
        public final int autoExportCount;
        public final int totalExportCount;
        public final int cloudUploadCount;
        public final int queuedCloudCount;
        public final int queuedCloudBytes;
        public final long bytesUploadedToday;
        public final boolean cloudConfigured;
        public final String lastDestination;
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
                int cloudUploadCount,
                int queuedCloudCount,
                int queuedCloudBytes,
                long bytesUploadedToday,
                boolean cloudConfigured,
                String lastDestination,
                int exportSeq,
                boolean waitingForUsb,
                String waitingReason,
                String lastExportFileName,
                Trigger lastTrigger,
                int lastExportedRows,
                String lastNotice) {
            this.autoExportCount = autoExportCount;
            this.totalExportCount = totalExportCount;
            this.cloudUploadCount = cloudUploadCount;
            this.queuedCloudCount = queuedCloudCount;
            this.queuedCloudBytes = queuedCloudBytes;
            this.bytesUploadedToday = bytesUploadedToday;
            this.cloudConfigured = cloudConfigured;
            this.lastDestination = lastDestination;
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
            String trig = lastTrigger != Trigger.NONE ? lastTrigger.shortLabel : "Saved";
            String destSuffix = ("GitHub".equals(lastDestination)) ? " \u2192 GitHub" : "";
            if (clock != null) {
                return clock + " (" + trig + destSuffix + ")";
            }
            return lastExportFileName + " (" + trig + destSuffix + ")";
        }

        static String extractClockFromFileName(String name) {
            // Matches "-HHmmss" inside either "brake-dataset-yyyyMMdd-HHmmss.csv"
            // or "brake-drive-yyyyMMdd-HHmmss-park.tar.gz"
            if (name == null) return null;
            for (int i = 0; i + 7 <= name.length(); i++) {
                if (name.charAt(i) != '-') continue;
                boolean sixDigits = true;
                for (int k = 1; k <= 6; k++) {
                    char c = name.charAt(i + k);
                    if (c < '0' || c > '9') {
                        sixDigits = false;
                        break;
                    }
                }
                if (!sixDigits) continue;
                int after = i + 7;
                if (after < name.length()) {
                    char next = name.charAt(after);
                    if (next >= '0' && next <= '9') {
                        // This was "-yyyyMMdd" (8 digits), keep scanning for "-HHmmss"
                        continue;
                    }
                }
                String sub = name.substring(i + 1, i + 7);
                return sub.substring(0, 2) + ":" + sub.substring(2, 4) + ":" + sub.substring(4, 6);
            }
            return null;
        }
    }

    private final PendingUploadQueue pendingQueue;
    private final CloudUploadConfig.DailyByteBudget dailyBudget;

    private boolean sawDriveSinceClear;
    private long outOfParkSinceMonoMs = -1L;
    private boolean parkExportPending;
    private boolean loggedParkWaitNotice;
    private boolean loggedNearFullWaitNotice;
    private boolean loggedWriteErrorNotice;
    private long nextProbeAllowedMonoMs;
    private long nextQueueDrainMonoMs;

    private int autoExportCount;
    private int totalExportCount;
    private int cloudUploadCount;
    private boolean cloudConfiguredSnapshot;
    private String lastDestination;
    private int exportSeq;
    private boolean waitingForUsb;
    private String waitingReason;
    private String lastExportFileName;
    private Trigger lastTrigger = Trigger.NONE;
    private int lastExportedRows;
    private String lastNotice;

    public AutoExportCoordinator() {
        this(new PendingUploadQueue(), new CloudUploadConfig.DailyByteBudget());
    }

    public AutoExportCoordinator(
            PendingUploadQueue pendingQueue,
            CloudUploadConfig.DailyByteBudget dailyBudget) {
        this.pendingQueue = pendingQueue == null ? new PendingUploadQueue() : pendingQueue;
        this.dailyBudget = dailyBudget == null ? new CloudUploadConfig.DailyByteBudget() : dailyBudget;
    }

    public PendingUploadQueue getPendingQueue() {
        return pendingQueue;
    }

    public CloudUploadConfig.DailyByteBudget getDailyBudget() {
        return dailyBudget;
    }

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
     * Parses Linux {@code /proc/net/route} to detect active default IPv4 routes ({@code 00000000})
     * on head units where OEM telematics interfaces ({@code rmnet*}, {@code ccmni*}, {@code usb0})
     * are not registered with Android's {@code ConnectivityManager}.
     */
    public static TransportInfo parseProcNetRoute(String procNetRouteText) {
        if (procNetRouteText == null || procNetRouteText.isEmpty()) {
            return new TransportInfo(TransportKind.NONE, "Offline");
        }
        String bestIface = null;
        int bestMetric = Integer.MAX_VALUE;
        String[] lines = procNetRouteText.split("\n");
        for (int i = 0; i < lines.length; i++) {
            String line = lines[i].trim();
            if (line.isEmpty() || line.startsWith("Iface")) {
                continue;
            }
            String[] cols = line.split("\\s+");
            if (cols.length < 4) {
                continue;
            }
            String iface = cols[0];
            String destHex = cols[1];
            String flagsHex = cols[3];
            if (!"00000000".equals(destHex) || "lo".equals(iface)) {
                continue;
            }
            int flags;
            try {
                flags = Integer.parseInt(flagsHex, 16);
            } catch (NumberFormatException e) {
                continue;
            }
            if ((flags & 0x1) == 0) { // RTF_UP
                continue;
            }
            int metric = 0;
            if (cols.length >= 7) {
                try {
                    metric = Integer.parseInt(cols[6]);
                } catch (NumberFormatException ignored) {}
            }
            if (bestIface == null || metric < bestMetric) {
                bestIface = iface;
                bestMetric = metric;
            }
        }
        if (bestIface == null) {
            return new TransportInfo(TransportKind.NONE, "Offline");
        }
        String lower = bestIface.toLowerCase(Locale.US);
        if (lower.startsWith("wlan") || lower.startsWith("wifi") || lower.startsWith("ap")) {
            return new TransportInfo(TransportKind.WIFI, "Wi-Fi (" + bestIface + ")");
        }
        return new TransportInfo(TransportKind.CELLULAR, "LTE (" + bestIface + ")");
    }

    public static TransportInfo readKernelDefaultRoute() {
        File f = new File("/proc/net/route");
        if (!f.isFile()) {
            return new TransportInfo(TransportKind.NONE, "Offline");
        }
        StringBuilder sb = new StringBuilder(512);
        try {
            BufferedReader br = new BufferedReader(
                    new InputStreamReader(new FileInputStream(f), "UTF-8"), 512);
            try {
                String line;
                while ((line = br.readLine()) != null) {
                    sb.append(line).append('\n');
                }
            } finally {
                br.close();
            }
        } catch (Exception ignored) {
            return new TransportInfo(TransportKind.NONE, "Offline");
        }
        return parseProcNetRoute(sb.toString());
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

    /**
     * Lightweight O(1) check whether queued compressed bundles in RAM are due for a background
     * connectivity/USB retry.
     */
    public synchronized boolean pollQueueDrainDue(long nowMonoMs) {
        return !pendingQueue.isEmpty() && nowMonoMs >= nextQueueDrainMonoMs;
    }

    public synchronized void requestImmediateQueueDrain() {
        nextQueueDrainMonoMs = 0L;
    }

    public synchronized void setCloudConfiguredState(boolean configured) {
        this.cloudConfiguredSnapshot = configured;
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
     * Executes a direct USB export and clears exported rows from {@code recorder} strictly upon
     * verified write success. Preserves backward compatibility with USB-only callers.
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
            onExportSucceeded(effectiveTrigger, outOfParkNow, fileName, "USB", payload.exportedRows);
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

    /**
     * Unified Cloud + PendingUploadQueue + USB Fallback orchestrator.
     *
     * @param forceCloudOnly true when the user explicitly tapped {@code Upload Cloud} (never silently
     *                       diverts to USB-only, though drains any queued bundles to GitHub).
     */
    public ActionOutcome executeCloudOrUsb(
            Trigger trigger,
            boolean forceCloudOnly,
            TelemetryRecorder recorder,
            IndicatorConfig cfg,
            CloudUploadConfig.Store cloudStore,
            String obdStatusSummary,
            String hmgStatusSummary,
            UsbRootLocator rootLocator,
            UsbWriter usbWriter,
            NetworkCheck networkCheck,
            CloudUploader cloudUploader,
            OutOfParkCheck outOfParkCheck,
            long nowMonoMs,
            long nowWallMs) {
        Trigger effectiveTrigger = trigger == null ? Trigger.MANUAL : trigger;
        List<String> roots = rootLocator != null ? rootLocator.findMountedUsbRoots() : null;

        // Opportunistic one-time cloud-upload.json import whenever a USB stick is present
        if (roots != null && !roots.isEmpty() && cloudStore != null) {
            CloudUploadConfig existing = cloudStore.load();
            if (existing == null || !existing.hasCredentials()) {
                CloudUploadConfig.UsbImportResult imp = CloudUploadConfig.tryImportFromUsbRoots(roots, cloudStore);
                if (imp.imported) {
                    recorder.recordExternalEvent(nowMonoMs, imp.message);
                }
            }
        }

        CloudUploadConfig cloudCfg = cloudStore != null ? cloudStore.load() : CloudUploadConfig.unconfigured();
        if (cloudCfg == null) {
            cloudCfg = CloudUploadConfig.unconfigured();
        }
        setCloudConfiguredState(cloudCfg.hasCredentials());

        if (forceCloudOnly && !cloudCfg.hasCredentials()) {
            String msg = "Cloud upload not configured. Place BrakeIndicator/"
                    + CloudUploadConfig.USB_CONFIG_FILE + " on USB and tap Test Cloud.";
            return new ActionOutcome(false, false, false, false, null, msg);
        }

        boolean useCloud = forceCloudOnly
                ? cloudCfg.hasCredentials()
                : cloudCfg.isReadyForAutoUpload(cfg);

        if (!useCloud) {
            // Drain any previously queued cloud bundles to USB if USB is mounted
            if (!pendingQueue.isEmpty() && roots != null && !roots.isEmpty() && cfg.autoExportUsb) {
                drainPendingQueue(
                        false, recorder, cfg, cloudCfg, roots, usbWriter, networkCheck, cloudUploader,
                        nowMonoMs, nowWallMs);
            }
            if (!cfg.autoExportUsb && effectiveTrigger != Trigger.MANUAL) {
                return new ActionOutcome(false, false, false, false, null, "Auto-export disabled.");
            }
            UsbExporter.ExportResult usbRes = executeExport(
                    effectiveTrigger, recorder, cfg, obdStatusSummary, hmgStatusSummary,
                     new FixedRootsLocator(roots), usbWriter, outOfParkCheck, nowMonoMs);
            return new ActionOutcome(
                    usbRes.ok,
                    false,
                    false,
                    usbRes.ok,
                    usbRes.csvPath,
                    usbRes.ok ? ("Saved to USB: " + usbRes.csvPath) : usbRes.errorMessage);
        }

        // Cloud path active: package active buffer (if non-empty) into PendingUploadQueue & clear RAM ring
        int rowsBefore = recorder.bufferedRows();
        if (rowsBefore <= 0 && pendingQueue.isEmpty()) {
            String lastFile;
            synchronized (this) {
                parkExportPending = false;
                waitingForUsb = false;
                waitingReason = null;
                lastFile = lastExportFileName;
            }
            String msg = lastFile != null
                    ? ("Buffer already uploaded & cleared (" + lastFile + "). Drive or record new samples first.")
                    : "No telemetry rows buffered yet.";
            return new ActionOutcome(false, false, false, false, null, msg);
        }

        if (rowsBefore > 0) {
            TelemetryRecorder.ScoreSnapshot score = recorder.scoreSnapshot();
            TelemetryRecorder.ExportPayload payload = recorder.snapshotForExport(
                    cfg,
                    obdStatusSummary,
                    hmgStatusSummary,
                    null,
                    effectiveTrigger.summaryLabel);
            try {
                PendingUploadQueue.QueuedDrive packaged = PendingUploadQueue.packageDrive(
                        payload, score, effectiveTrigger, nowWallMs);
                pendingQueue.enqueue(packaged);
                int kb = Math.max(1, (packaged.tarGzBytes.length + 512) / 1024);
                String clearNote = String.format(
                        Locale.US,
                        "Packaged %,d rows -> %s (%d KB gz, %s, buffer cleared)",
                        Integer.valueOf(payload.exportedRows),
                        packaged.archiveFileName,
                        Integer.valueOf(kb),
                        effectiveTrigger.shortLabel);
                recorder.clearAfterExport(payload.exportedTotalRecordedRows, clearNote);
                boolean outOfParkNow = outOfParkCheck != null && outOfParkCheck.isCurrentlyOutOfPark();
                synchronized (this) {
                    parkExportPending = false;
                    loggedParkWaitNotice = false;
                    loggedNearFullWaitNotice = false;
                    loggedWriteErrorNotice = false;
                    waitingForUsb = false;
                    waitingReason = null;
                    nextProbeAllowedMonoMs = 0L;
                    sawDriveSinceClear = outOfParkNow;
                    outOfParkSinceMonoMs = -1L;
                }
            } catch (Exception e) {
                String err = "Failed to compress drive archive: " + e.getMessage();
                return new ActionOutcome(false, false, false, false, null, err);
            }
        }

        return drainPendingQueue(
                forceCloudOnly,
                recorder,
                cfg,
                cloudCfg,
                roots,
                usbWriter,
                networkCheck,
                cloudUploader,
                nowMonoMs,
                nowWallMs);
    }

    /**
     * Attempts to upload all compressed drives in {@link PendingUploadQueue} to GitHub, or unpacks
     * them to a mounted USB stick when offline (unless {@code forceCloudOnly} is true).
     */
    public ActionOutcome drainPendingQueue(
            boolean forceCloudOnly,
            TelemetryRecorder recorder,
            IndicatorConfig cfg,
            CloudUploadConfig cloudCfg,
            List<String> mountedUsbRoots,
            UsbWriter usbWriter,
            NetworkCheck networkCheck,
            CloudUploader cloudUploader,
            long nowMonoMs,
            long nowWallMs) {
        if (pendingQueue.isEmpty()) {
            return new ActionOutcome(true, false, false, false, null, "No pending cloud uploads.");
        }

        CloudUploader effectiveUploader = cloudUploader != null ? cloudUploader : DEFAULT_CLOUD_UPLOADER;
        UsbWriter effectiveUsbWriter = usbWriter != null ? usbWriter : DEFAULT_USB_WRITER;
        TransportInfo net = networkCheck != null
                ? networkCheck.activeTransport()
                : new TransportInfo(TransportKind.WIFI, "Wi-Fi");

        boolean anyCloudOk = false;
        boolean anyUsbFallbackOk = false;
        String lastTarget = null;
        String lastMsg = null;

        while (!pendingQueue.isEmpty()) {
            PendingUploadQueue.QueuedDrive head = pendingQueue.peekOldest();
            if (head == null) {
                break;
            }

            long estWireBytes = ((long) head.tarGzBytes.length * 4L) / 3L + 512L;
            String blockReason = null;
            if (!cloudCfg.hasCredentials()) {
                blockReason = "Cloud credentials not configured";
            } else if (net.kind == TransportKind.NONE) {
                blockReason = "Offline";
            } else if (net.kind == TransportKind.CELLULAR && !cloudCfg.isLteAllowed(cfg)) {
                blockReason = "Wi-Fi only (LTE upload disabled)";
            } else if (net.kind == TransportKind.CELLULAR
                    && !dailyBudget.canSpend(estWireBytes, cloudCfg.dailyLimitKb, nowWallMs)) {
                blockReason = "Daily LTE cap (" + cloudCfg.dailyLimitKb + " KB) reached";
            }

            boolean retryableFailure = true;
            if (blockReason == null) {
                GitHubUploader.UploadResult upRes = effectiveUploader.upload(cloudCfg, head);
                if (upRes.ok) {
                    if (net.kind == TransportKind.CELLULAR) {
                        long spent = upRes.wireBytesSent > 0L ? upRes.wireBytesSent : estWireBytes;
                        dailyBudget.recordSpent(spent, nowWallMs);
                    }
                    pendingQueue.remove(head);
                    anyCloudOk = true;
                    lastTarget = upRes.remotePath;
                    lastMsg = upRes.message;

                    // Optional dual-write to USB when cloud upload succeeds
                    if (cloudCfg.shouldAlsoWriteUsb(cfg)
                            && mountedUsbRoots != null
                            && !mountedUsbRoots.isEmpty()) {
                        writeQueuedArchiveToUsb(head, mountedUsbRoots, effectiveUsbWriter);
                    }

                    onCloudUploadSucceeded(head, upRes);
                    if (recorder != null) {
                        recorder.recordExternalEvent(
                                nowMonoMs,
                                "GitHub upload OK (" + net.label + "): " + upRes.message);
                    }
                    continue;
                }
                blockReason = upRes.message;
                retryableFailure = upRes.retryable;
            }

            // Cloud blocked or failed -> try unpacking to USB fallback if attached (and not manual Cloud-only)
            if (!forceCloudOnly
                    && cfg != null
                    && cfg.autoExportUsb
                    && mountedUsbRoots != null
                    && !mountedUsbRoots.isEmpty()) {
                UsbExporter.ExportResult usbRes = writeQueuedArchiveToUsb(
                        head, mountedUsbRoots, effectiveUsbWriter);
                if (usbRes.ok) {
                    pendingQueue.remove(head);
                    anyUsbFallbackOk = true;
                    String usbName = usbRes.csvPath != null
                            ? new File(usbRes.csvPath).getName()
                            : "brake-dataset.csv";
                    lastTarget = usbRes.csvPath;
                    lastMsg = "Cloud unavailable (" + blockReason + ") -> saved to USB: " + usbName;
                    onUsbFallbackSucceeded(head, usbName, lastMsg);
                    if (recorder != null) {
                        recorder.recordExternalEvent(nowMonoMs, lastMsg);
                    }
                    continue;
                }
            }

            // Keep safely compressed in PendingUploadQueue for automatic retry when LTE/Wi-Fi/USB returns
            head.attemptCount++;
            head.lastError = blockReason;
            long backoff = retryableFailure ? CLOUD_QUEUE_RETRY_MS : CLOUD_AUTH_BACKOFF_MS;
            int kb = Math.max(1, (head.tarGzBytes.length + 512) / 1024);
            String queueMsg = String.format(
                    Locale.US,
                    "Queued in RAM (%d drive%s, %d KB gz) \u2014 %s",
                    Integer.valueOf(pendingQueue.size()),
                    pendingQueue.size() == 1 ? "" : "s",
                    Integer.valueOf(kb),
                    blockReason);
            synchronized (this) {
                nextQueueDrainMonoMs = nowMonoMs + backoff;
                lastDestination = "Queued";
                lastNotice = queueMsg;
                exportSeq++;
            }
            if (recorder != null && head.attemptCount == 1) {
                recorder.recordExternalEvent(nowMonoMs, queueMsg);
            }
            return new ActionOutcome(false, anyCloudOk, true, anyUsbFallbackOk, lastTarget, queueMsg);
        }

        return new ActionOutcome(
                anyCloudOk || anyUsbFallbackOk,
                anyCloudOk,
                false,
                anyUsbFallbackOk,
                lastTarget,
                lastMsg != null ? lastMsg : "Upload complete.");
    }

    private static UsbExporter.ExportResult writeQueuedArchiveToUsb(
            PendingUploadQueue.QueuedDrive drive,
            List<String> roots,
            UsbWriter writer) {
        try {
            Map<String, byte[]> entries = TarGzArchive.extractEntries(drive.tarGzBytes);
            String csv = "";
            String summary = "";
            for (Map.Entry<String, byte[]> e : entries.entrySet()) {
                if (e.getKey().endsWith(".csv")) {
                    csv = new String(e.getValue(), "UTF-8");
                } else if (e.getKey().endsWith(".txt")) {
                    summary = new String(e.getValue(), "UTF-8");
                }
            }
            return writer.write(csv, summary, roots);
        } catch (Exception e) {
            return new UsbExporter.ExportResult(
                    false, null, null, "Failed to unpack queued archive to USB: " + e.getMessage());
        }
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
        return statusSnapshot(System.currentTimeMillis());
    }

    public synchronized StatusSnapshot statusSnapshot(long nowWallMs) {
        return new StatusSnapshot(
                autoExportCount,
                totalExportCount,
                cloudUploadCount,
                pendingQueue.size(),
                pendingQueue.totalBytes(),
                dailyBudget.bytesUsedToday(nowWallMs),
                cloudConfiguredSnapshot,
                lastDestination,
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
            String destination,
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
        lastDestination = destination;
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

    private synchronized void onCloudUploadSucceeded(
            PendingUploadQueue.QueuedDrive drive,
            GitHubUploader.UploadResult upRes) {
        waitingForUsb = false;
        waitingReason = null;
        nextQueueDrainMonoMs = 0L;

        cloudUploadCount++;
        totalExportCount++;
        if (drive.trigger != Trigger.MANUAL) {
            autoExportCount++;
        }
        exportSeq++;
        lastDestination = "GitHub";
        lastExportFileName = drive.archiveFileName;
        lastTrigger = drive.trigger;
        lastExportedRows = drive.exportedRows;
        lastNotice = upRes.message + " -> GitHub (" + drive.trigger.shortLabel + ")";
    }

    private synchronized void onUsbFallbackSucceeded(
            PendingUploadQueue.QueuedDrive drive,
            String usbFileName,
            String notice) {
        waitingForUsb = false;
        waitingReason = null;
        nextQueueDrainMonoMs = 0L;

        totalExportCount++;
        if (drive.trigger != Trigger.MANUAL) {
            autoExportCount++;
        }
        exportSeq++;
        lastDestination = "USB";
        lastExportFileName = usbFileName;
        lastTrigger = drive.trigger;
        lastExportedRows = drive.exportedRows;
        lastNotice = notice;
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

    private static final class FixedRootsLocator implements UsbRootLocator {
        private final List<String> roots;

        FixedRootsLocator(List<String> roots) {
            this.roots = roots;
        }

        @Override
        public List<String> findMountedUsbRoots() {
            return roots;
        }
    }
}
