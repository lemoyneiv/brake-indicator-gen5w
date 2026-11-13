package dev.lemoyne.brakeindicator;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Non-root external USB storage locator and atomic dataset exporter for Gen5W head units.
 *
 * Strictly writes ONLY to removable USB media under {@code <usb>/BrakeIndicator/} using
 * temporary {@code .part} files + {@code FileDescriptor.sync()} + {@code renameTo()} so
 * sudden 12V power cuts never corrupt FAT32/exFAT sticks and internal eMMC flash is never touched.
 */
public final class UsbExporter {

    public static final String SUBFOLDER = "BrakeIndicator";

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

    private static final String[] DIRECT_USB_CANDIDATES = {
        "/storage/usb0",
        "/storage/usb1",
        "/storage/usb2",
        "/mnt/media_rw/usb0",
        "/mnt/media_rw/usb1",
        "/mnt/usb0",
        "/mnt/usb1"
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
        public final String csvPath;
        public final String summaryPath;
        public final String hevProbePath;
        public final String errorMessage;

        public ExportResult(boolean ok, String csvPath, String summaryPath, String errorMessage) {
            this(ok, csvPath, summaryPath, null, errorMessage);
        }

        public ExportResult(
                boolean ok,
                String csvPath,
                String summaryPath,
                String hevProbePath,
                String errorMessage) {
            this.ok = ok;
            this.csvPath = csvPath;
            this.summaryPath = summaryPath;
            this.hevProbePath = hevProbePath;
            this.errorMessage = errorMessage;
        }
    }

    private UsbExporter() {}

    public static ExportResult exportDatasetAndSummary(
            String csvContent,
            String summaryContent,
            List<String> extraUsbFolders) {
        return exportDatasetAndSummary(csvContent, summaryContent, null, extraUsbFolders);
    }

    public static ExportResult exportDatasetAndSummary(
            String csvContent,
            String summaryContent,
            String hevProbeContent,
            List<String> extraUsbFolders) {
        Set<String> roots = new LinkedHashSet<String>();
        if (extraUsbFolders != null) {
            for (String folder : extraUsbFolders) {
                if (folder == null || folder.trim().isEmpty()) {
                    continue;
                }
                File f = new File(folder.trim());
                File parent = SUBFOLDER.equals(f.getName()) ? f.getParentFile() : f;
                if (parent != null) {
                    roots.add(parent.getAbsolutePath());
                }
            }
        }
        String mountsText = readProcMounts();
        List<String> mountedUsb = parseUsbMountPoints(mountsText);
        roots.addAll(mountedUsb);

        if (mountsText.isEmpty()) {
            for (int i = 0; i < DIRECT_USB_CANDIDATES.length; i++) {
                File dir = new File(DIRECT_USB_CANDIDATES[i]);
                if (dir.isDirectory() && dir.canWrite()) {
                    roots.add(dir.getAbsolutePath());
                }
            }
        }

        return exportToDirectories(csvContent, summaryContent, hevProbeContent, new ArrayList<String>(roots), SUBFOLDER);
    }

    public static ExportResult exportToDirectories(
            String csvContent,
            String summaryContent,
            List<String> candidateMountRoots,
            String subfolderName) {
        return exportToDirectories(csvContent, summaryContent, null, candidateMountRoots, subfolderName);
    }

    public static ExportResult exportToDirectories(
            String csvContent,
            String summaryContent,
            String hevProbeContent,
            List<String> candidateMountRoots,
            String subfolderName) {
        byte[] csvBytes;
        byte[] summaryBytes;
        byte[] probeBytes = null;
        try {
            csvBytes = (csvContent == null ? "" : csvContent).getBytes("UTF-8");
            summaryBytes = (summaryContent == null ? "" : summaryContent).getBytes("UTF-8");
            if (hevProbeContent != null && !hevProbeContent.isEmpty()) {
                probeBytes = hevProbeContent.getBytes("UTF-8");
            }
        } catch (Exception e) {
            return new ExportResult(false, null, null, null, "UTF-8 encoding failed: " + e.getMessage());
        }

        if (candidateMountRoots == null || candidateMountRoots.isEmpty()) {
            return new ExportResult(
                    false,
                    null,
                    null,
                    null,
                    "No mounted USB stick found. Insert a FAT32/exFAT USB drive (ensure USB port is in host mode).");
        }

        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        String desiredCsv = "brake-dataset-" + stamp + ".csv";
        String desiredSummary = "brake-summary-" + stamp + ".txt";
        String desiredProbe = "hev-regen-probe-" + stamp + ".txt";
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

                File csvOut = writeBytesAtomically(targetDir, desiredCsv, csvBytes);
                File sumOut = writeBytesAtomically(targetDir, desiredSummary, summaryBytes);
                File probeOut = (probeBytes != null)
                        ? writeBytesAtomically(targetDir, desiredProbe, probeBytes)
                        : null;

                if (csvOut.isFile() && csvOut.length() > 0) {
                    return new ExportResult(
                            true,
                            csvOut.getAbsolutePath(),
                            sumOut.getAbsolutePath(),
                            probeOut != null ? probeOut.getAbsolutePath() : null,
                            null);
                }
            } catch (Exception e) {
                lastErr = mountRoot + ": " + e.getClass().getSimpleName()
                        + (e.getMessage() != null ? " (" + e.getMessage() + ")" : "");
            }
        }

        return new ExportResult(false, null, null, null, "USB write failed (" + lastErr + ").");
    }

    public static File writeBytesAtomically(File targetDir, String baseFileName, byte[] data) throws IOException {
        File out = uniqueFile(targetDir, baseFileName);
        File tmp = new File(targetDir, "." + out.getName() + ".part");
        FileOutputStream fos = null;
        try {
            fos = new FileOutputStream(tmp);
            if (data != null && data.length > 0) {
                fos.write(data);
            }
            fos.flush();
            try {
                fos.getFD().sync();
            } catch (Throwable ignored) {
                // Some host/virtual filesystems do not support fsync
            }
            fos.close();
            fos = null;
            if (!tmp.renameTo(out)) {
                throw new IOException("Failed to rename temporary file to " + out.getAbsolutePath());
            }
            out.setReadable(true, false);
            return out;
        } finally {
            if (fos != null) {
                try { fos.close(); } catch (Throwable ignored) {}
            }
            if (tmp.exists()) {
                tmp.delete();
            }
        }
    }

    private static File uniqueFile(File dir, String desiredName) {
        File candidate = new File(dir, desiredName);
        if (!candidate.exists()) {
            return candidate;
        }
        int dot = desiredName.lastIndexOf('.');
        String stem = dot > 0 ? desiredName.substring(0, dot) : desiredName;
        String ext = dot > 0 ? desiredName.substring(dot) : "";
        int n = 1;
        while (true) {
            File next = new File(dir, stem + "_" + n + ext);
            if (!next.exists()) {
                return next;
            }
            n++;
        }
    }

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
            if (dev.startsWith("/dev/block/mmcblk") || dev.startsWith("/dev/mmcblk")) {
                continue;
            }

            boolean usbPath = startsWithAny(mountPoint, USB_MOUNT_PREFIXES);
            boolean usbDev = (dev.startsWith("/dev/block/vold/")
                    || dev.startsWith("/dev/sd")
                    || dev.startsWith("/dev/block/sd"))
                    && matchesAnyExact(fsType, STORAGE_FS);

            if ((usbPath && matchesAnyExact(fsType, STORAGE_FS)) || usbDev) {
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
