package dev.lemoyne.brakeindicator;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Reliable root-side USB drive detector and log exporter for Gen5W head units.
 *
 * Why hmgapid's `usb` command alone was not enough in v0.1.0:
 *   1. `op_usb` in hmgapid only returns `/storage/usbX/BrakeIndicator` if the
 *      `BrakeIndicator/` directory ALREADY exists on the USB stick beforehand.
 *   2. An unprivileged Android 4.4 app UID cannot `mkdirs()` on `/storage/usb0`
 *      without `WRITE_MEDIA_STORAGE`.
 *   3. Checking `new File("/storage/usb0").isDirectory()` as root without checking
 *      `/proc/mounts` would match an unmounted empty tmpfs stub directory.
 *
 * UsbExporter parses `/proc/mounts` to locate genuinely mounted USB filesystems,
 * creates `BrakeIndicator/` on the stick as root, writes and `fsync`s the report,
 * and saves a world-readable fallback copy to `/data/local/tmp/brake-indicator-probe.txt`.
 */
public final class UsbExporter {

    public static final String SUBFOLDER = "BrakeIndicator";
    public static final String FALLBACK_PATH = "/data/local/tmp/brake-indicator-probe.txt";

    private static final String[] USB_MOUNT_PREFIXES = {
        "/storage/usb",
        "/mnt/media_rw/usb",
        "/mnt/usb",
        "/storage/usbdisk",
        "/mnt/usbdisk",
        "/storage/udisk",
        "/mnt/udisk",
        "/udisk"
    };

    private static final String[] INTERNAL_PREFIXES = {
        "/system", "/data", "/cache", "/dev", "/proc", "/sys", "/acct",
        "/oem", "/vendor", "/firmware", "/persist", "/sbin",
        "/storage/emulated", "/storage/sdcard0", "/mnt/shell",
        "/mnt/asec", "/mnt/obb", "/mnt/secure", "/mnt/shm"
    };

    private static final String[] VIRTUAL_FS = {
        "tmpfs", "rootfs", "proc", "sysfs", "devpts", "cgroup",
        "debugfs", "pstore", "selinuxfs", "functionfs", "adb", "none"
    };

    private static final String[] STORAGE_FS = {
        "vfat", "exfat", "texfat", "sdfat", "msdos", "fat",
        "ntfs", "ufsd", "fuse", "fuseblk", "ext4", "ext3", "ext2"
    };

    public static final class ExportResult {
        public final boolean ok;
        public final String savedPath;
        public final String errorMessage;

        public ExportResult(boolean ok, String savedPath, String errorMessage) {
            this.ok = ok;
            this.savedPath = savedPath;
            this.errorMessage = errorMessage;
        }
    }

    private UsbExporter() {}

    public static ExportResult exportReport(String reportContent, List<String> extraUsbFolders) {
        String mountsText = readProcMounts();
        List<String> roots = new ArrayList<String>(parseUsbMountPoints(mountsText));
        if (extraUsbFolders != null) {
            for (String folder : extraUsbFolders) {
                if (folder == null || folder.trim().isEmpty()) {
                    continue;
                }
                File f = new File(folder.trim());
                File parent = SUBFOLDER.equals(f.getName()) ? f.getParentFile() : f;
                if (parent != null && !roots.contains(parent.getAbsolutePath())) {
                    roots.add(parent.getAbsolutePath());
                }
            }
        }
        return exportToDirectories(reportContent, roots, SUBFOLDER, new File(FALLBACK_PATH));
    }

    public static ExportResult exportToDirectories(
            String reportContent,
            List<String> candidateMountRoots,
            String subfolderName,
            File fallbackFile) {
        byte[] payload;
        try {
            payload = (reportContent == null ? "" : reportContent).getBytes("UTF-8");
        } catch (Exception e) {
            return new ExportResult(false, null, "UTF-8 encoding failed: " + e.getMessage());
        }

        boolean wroteFallback = false;
        if (fallbackFile != null) {
            try {
                File parent = fallbackFile.getParentFile();
                if (parent != null && !parent.isDirectory()) {
                    parent.mkdirs();
                }
                writeAndSync(fallbackFile, payload);
                fallbackFile.setReadable(true, false);
                wroteFallback = true;
            } catch (Exception ignored) {
                // Non-fatal
            }
        }

        if (candidateMountRoots == null || candidateMountRoots.isEmpty()) {
            String note = "No mounted USB stick found in /proc/mounts. "
                    + "Check FAT32/exFAT stick (note: USB ADB puts the USB port in device mode).";
            if (wroteFallback) {
                note += " Saved fallback to " + fallbackFile.getAbsolutePath();
            }
            return new ExportResult(false, null, note);
        }

        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        String fileName = "hscan-probe-" + stamp + ".txt";
        String lastErr = "unable to write to USB mount";

        for (String mountRoot : candidateMountRoots) {
            if (mountRoot == null || mountRoot.isEmpty()) {
                continue;
            }
            try {
                File rootDir = new File(mountRoot);
                if (!rootDir.isDirectory()) {
                    continue;
                }
                File targetDir = (subfolderName == null || subfolderName.isEmpty())
                        ? rootDir
                        : new File(rootDir, subfolderName);
                if (!targetDir.isDirectory() && !targetDir.mkdirs()) {
                    lastErr = "mkdirs failed on " + targetDir.getAbsolutePath();
                    continue;
                }
                targetDir.setReadable(true, false);
                targetDir.setExecutable(true, false);

                File outFile = new File(targetDir, fileName);
                writeAndSync(outFile, payload);
                outFile.setReadable(true, false);

                if (outFile.isFile() && outFile.length() > 0) {
                    return new ExportResult(true, outFile.getAbsolutePath(), null);
                }
            } catch (Exception e) {
                lastErr = mountRoot + ": " + e.getClass().getSimpleName()
                        + (e.getMessage() != null ? " (" + e.getMessage() + ")" : "");
            }
        }

        String err = "USB write failed (" + lastErr + ").";
        if (wroteFallback) {
            err += " Saved fallback to " + fallbackFile.getAbsolutePath();
        }
        return new ExportResult(false, null, err);
    }

    /**
     * Parses `/proc/mounts` text and returns mounted external USB root directories,
     * preferring `/storage/usb*` ahead of `/mnt/media_rw/usb*`.
     */
    public static List<String> parseUsbMountPoints(String procMountsText) {
        Set<String> storageMounts = new LinkedHashSet<String>();
        Set<String> rawMounts = new LinkedHashSet<String>();

        if (procMountsText == null || procMountsText.isEmpty()) {
            return new ArrayList<String>();
        }

        String[] lines = procMountsText.split("\n");
        for (String rawLine : lines) {
            String line = rawLine.trim();
            if (line.isEmpty() || line.startsWith("#")) {
                continue;
            }
            String[] cols = line.split("\\s+");
            if (cols.length < 3) {
                continue;
            }
            String dev = unescapeMountField(cols[0]);
            String mountPoint = stripTrailingSlash(unescapeMountField(cols[1]));
            String fsType = cols[2].toLowerCase(Locale.US);

            if (mountPoint.isEmpty() || "/".equals(mountPoint)) {
                continue;
            }
            if (matchesAnyExact(fsType, VIRTUAL_FS) || startsWithAny(mountPoint, INTERNAL_PREFIXES)) {
                continue;
            }
            // Ignore internal eMMC block partitions
            if (dev.startsWith("/dev/block/mmcblk") || dev.startsWith("/dev/mmcblk")) {
                continue;
            }

            boolean usbPath = startsWithAny(mountPoint, USB_MOUNT_PREFIXES);
            boolean usbDev = (dev.startsWith("/dev/block/vold/")
                    || dev.startsWith("/dev/sd")
                    || dev.startsWith("/dev/block/sd"))
                    && matchesAnyExact(fsType, STORAGE_FS);

            if (usbPath && matchesAnyExact(fsType, STORAGE_FS)) {
                if (mountPoint.startsWith("/storage/")) {
                    storageMounts.add(mountPoint);
                } else {
                    rawMounts.add(mountPoint);
                }
            } else if (usbDev) {
                if (mountPoint.startsWith("/storage/")) {
                    storageMounts.add(mountPoint);
                } else {
                    rawMounts.add(mountPoint);
                }
            }
        }

        List<String> ordered = new ArrayList<String>(storageMounts.size() + rawMounts.size());
        ordered.addAll(storageMounts);
        ordered.addAll(rawMounts);
        return ordered;
    }

    private static void writeAndSync(File file, byte[] data) throws Exception {
        FileOutputStream fos = new FileOutputStream(file);
        try {
            fos.write(data);
            fos.flush();
            try {
                fos.getFD().sync();
            } catch (Exception ignored) {
                // Some virtual/host filesystems do not support fsync
            }
        } finally {
            fos.close();
        }
    }

    private static String readProcMounts() {
        File f = new File("/proc/mounts");
        if (!f.isFile()) {
            return "";
        }
        StringBuilder sb = new StringBuilder(2048);
        try {
            BufferedReader br = new BufferedReader(new InputStreamReader(new FileInputStream(f), "UTF-8"), 1024);
            try {
                String line;
                while ((line = br.readLine()) != null) {
                    sb.append(line).append('\n');
                }
            } finally {
                br.close();
            }
        } catch (Exception ignored) {
        }
        return sb.toString();
    }

    private static String unescapeMountField(String s) {
        if (s == null || s.indexOf('\\') < 0) {
            return s == null ? "" : s;
        }
        return s.replace("\\040", " ")
                .replace("\\011", "\t")
                .replace("\\012", "\n")
                .replace("\\134", "\\");
    }

    private static String stripTrailingSlash(String path) {
        if (path == null) {
            return "";
        }
        while (path.length() > 1 && path.endsWith("/")) {
            path = path.substring(0, path.length() - 1);
        }
        return path;
    }

    private static boolean startsWithAny(String value, String[] prefixes) {
        String lower = value.toLowerCase(Locale.US);
        for (String p : prefixes) {
            if (lower.equals(p) || lower.startsWith(p + "/") || lower.startsWith(p)) {
                // Ensure "/mnt/usb" matches "/mnt/usb0" and "/mnt/usb/..." while "/data" doesn't match "/database"
                if (p.endsWith("usb") || p.endsWith("udisk") || p.endsWith("usbdisk")) {
                    return true;
                }
                if (lower.equals(p) || lower.startsWith(p + "/")) {
                    return true;
                }
            }
        }
        return false;
    }

    private static boolean matchesAnyExact(String value, String[] allowed) {
        for (String a : allowed) {
            if (a.equals(value)) {
                return true;
            }
        }
        return false;
    }
}
