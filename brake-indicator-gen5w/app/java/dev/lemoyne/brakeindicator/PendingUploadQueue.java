package dev.lemoyne.brakeindicator;

import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.Deque;
import java.util.List;
import java.util.Locale;

/**
 * Bounded, zero-flash-wear in-RAM queue for compressed {@code .tar.gz} drive logs.
 *
 * When the car shifts to Park in an underground garage or cellular dead zone without a USB stick,
 * the active 36,000-row ring buffer is compressed (~300 KB) and placed here so the ring buffer can
 * reset immediately for the next drive. Queued bundles automatically upload when connectivity
 * returns or unpack to a USB drive if one is plugged in.
 */
public final class PendingUploadQueue {

    public static final int DEFAULT_MAX_ITEMS = 4;
    public static final int DEFAULT_MAX_TOTAL_BYTES = 2 * 1024 * 1024; // 2 MB RAM ceiling

    public static final class QueuedDrive {
        public final String stamp;
        public final String dateFolder;
        public final String archiveFileName;
        public final byte[] tarGzBytes;
        public final AutoExportCoordinator.Trigger trigger;
        public final int exportedRows;
        public final String commitMessage;
        public final long createdWallMs;
        public int attemptCount;
        public String lastError;

        public QueuedDrive(
                String stamp,
                String dateFolder,
                String archiveFileName,
                byte[] tarGzBytes,
                AutoExportCoordinator.Trigger trigger,
                int exportedRows,
                String commitMessage,
                long createdWallMs) {
            this.stamp = stamp;
            this.dateFolder = dateFolder;
            this.archiveFileName = archiveFileName;
            this.tarGzBytes = tarGzBytes == null ? new byte[0] : tarGzBytes;
            this.trigger = trigger == null ? AutoExportCoordinator.Trigger.MANUAL : trigger;
            this.exportedRows = exportedRows;
            this.commitMessage = commitMessage == null ? "telemetry log" : commitMessage;
            this.createdWallMs = createdWallMs;
        }
    }

    private final int maxItems;
    private final int maxTotalBytes;
    private final Deque<QueuedDrive> queue = new ArrayDeque<QueuedDrive>();
    private int totalBytes;

    public PendingUploadQueue() {
        this(DEFAULT_MAX_ITEMS, DEFAULT_MAX_TOTAL_BYTES);
    }

    public PendingUploadQueue(int maxItems, int maxTotalBytes) {
        this.maxItems = Math.max(1, maxItems);
        this.maxTotalBytes = Math.max(65536, maxTotalBytes);
    }

    public static QueuedDrive packageDrive(
            TelemetryRecorder.ExportPayload payload,
            TelemetryRecorder.ScoreSnapshot score,
            AutoExportCoordinator.Trigger trigger,
            long wallTimeMs) throws java.io.IOException {
        long ts = wallTimeMs > 0L ? wallTimeMs : System.currentTimeMillis();
        Date d = new Date(ts);
        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(d);
        String dateFolder = new SimpleDateFormat("yyyy-MM-dd", Locale.US).format(d);
        AutoExportCoordinator.Trigger trig = trigger == null ? AutoExportCoordinator.Trigger.MANUAL : trigger;

        String slug;
        if (trig == AutoExportCoordinator.Trigger.SHIFT_TO_PARK) {
            slug = "park";
        } else if (trig == AutoExportCoordinator.Trigger.BUFFER_NEAR_FULL) {
            slug = "buffer95";
        } else {
            slug = "manual";
        }

        byte[] gz = TarGzArchive.buildDatasetArchive(stamp, payload.csv, payload.summary, ts);
        String archiveName = "brake-drive-" + stamp + "-" + slug + ".tar.gz";
        int kb = Math.max(1, (gz.length + 512) / 1024);

        String scorePart;
        if (score != null && score.scoredSamples > 0L) {
            scorePart = String.format(
                    Locale.US,
                    "OBD=%d samples, agree=%.1f%%",
                    Long.valueOf(score.scoredSamples),
                    Float.valueOf(score.agreementPct()));
        } else {
            scorePart = "OBD=standby";
        }

        String commitMsg = String.format(
                Locale.US,
                "drive(%s): %,d rows, %s (%d KB gz)",
                slug,
                Integer.valueOf(payload.exportedRows),
                scorePart,
                Integer.valueOf(kb));

        return new QueuedDrive(
                stamp,
                dateFolder,
                archiveName,
                gz,
                trig,
                payload.exportedRows,
                commitMsg,
                ts);
    }

    public synchronized boolean enqueue(QueuedDrive item) {
        if (item == null || item.tarGzBytes.length > maxTotalBytes) {
            return false;
        }
        while (!queue.isEmpty()
                && (queue.size() >= maxItems || (totalBytes + item.tarGzBytes.length) > maxTotalBytes)) {
            QueuedDrive dropped = queue.removeFirst();
            totalBytes -= dropped.tarGzBytes.length;
        }
        queue.addLast(item);
        totalBytes += item.tarGzBytes.length;
        return true;
    }

    public synchronized QueuedDrive peekOldest() {
        return queue.peekFirst();
    }

    public synchronized boolean remove(QueuedDrive item) {
        if (queue.remove(item)) {
            totalBytes = Math.max(0, totalBytes - item.tarGzBytes.length);
            return true;
        }
        return false;
    }

    public synchronized int size() {
        return queue.size();
    }

    public synchronized int totalBytes() {
        return totalBytes;
    }

    public synchronized boolean isEmpty() {
        return queue.isEmpty();
    }

    public synchronized void clear() {
        queue.clear();
        totalBytes = 0;
    }

    public synchronized List<QueuedDrive> snapshotItems() {
        return new ArrayList<QueuedDrive>(queue);
    }
}
