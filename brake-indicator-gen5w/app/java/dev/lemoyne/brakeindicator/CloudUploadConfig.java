package dev.lemoyne.brakeindicator;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.TimeZone;

/**
 * Cloud log upload configuration, one-time USB {@code cloud-upload.json} provisioning (with
 * automatic plaintext token scrubbing on the USB stick), and in-RAM daily LTE byte budget tracking.
 */
public final class CloudUploadConfig {

    public static final String DEFAULT_REPO = "lemoyneiv/brake-indicator-logs";
    public static final String DEFAULT_BRANCH = "main";
    public static final String DEFAULT_API_HOST = "api.github.com";
    public static final int DEFAULT_API_PORT = 443;
    public static final int DEFAULT_DAILY_LIMIT_KB = 5120; // 5 MB/day (~15-20 compressed 30-min drives)
    public static final String USB_CONFIG_FILE = "cloud-upload.json";

    public interface Store {
        CloudUploadConfig load();
        void save(CloudUploadConfig config);
        void clear();
    }

    public static final class MemoryStore implements Store {
        private volatile CloudUploadConfig current = CloudUploadConfig.unconfigured();

        @Override
        public CloudUploadConfig load() {
            return current;
        }

        @Override
        public void save(CloudUploadConfig config) {
            current = config == null ? CloudUploadConfig.unconfigured() : config;
        }

        @Override
        public void clear() {
            current = CloudUploadConfig.unconfigured();
        }
    }

    public static final class UsbImportResult {
        public final boolean foundFile;
        public final boolean imported;
        public final boolean scrubbedTokenOnUsb;
        public final String sourcePath;
        public final String message;
        public final CloudUploadConfig config;

        UsbImportResult(
                boolean foundFile,
                boolean imported,
                boolean scrubbedTokenOnUsb,
                String sourcePath,
                String message,
                CloudUploadConfig config) {
            this.foundFile = foundFile;
            this.imported = imported;
            this.scrubbedTokenOnUsb = scrubbedTokenOnUsb;
            this.sourcePath = sourcePath;
            this.message = message;
            this.config = config;
        }
    }

    /**
     * Pure in-RAM daily byte budget tracker so normal driving/parking uploads never write to
     * internal eMMC flash.
     */
    public static final class DailyByteBudget {
        private long currentUtcDay = -1L;
        private long bytesUsedToday = 0L;

        public synchronized boolean canSpend(long bytesToSend, int dailyLimitKb, long nowWallMs) {
            rollDayIfNeeded(nowWallMs);
            long limitBytes = Math.max(128L, (long) dailyLimitKb) * 1024L;
            return (bytesUsedToday + Math.max(0L, bytesToSend)) <= limitBytes;
        }

        public synchronized void recordSpent(long bytesSent, long nowWallMs) {
            rollDayIfNeeded(nowWallMs);
            if (bytesSent > 0L) {
                bytesUsedToday += bytesSent;
            }
        }

        public synchronized long bytesUsedToday(long nowWallMs) {
            rollDayIfNeeded(nowWallMs);
            return bytesUsedToday;
        }

        private void rollDayIfNeeded(long nowWallMs) {
            long ts = nowWallMs > 0L ? nowWallMs : System.currentTimeMillis();
            long day = ts / 86400000L;
            if (day != currentUtcDay) {
                currentUtcDay = day;
                bytesUsedToday = 0L;
            }
        }
    }

    public final boolean enabled;
    public final String repo;
    public final String branch;
    public final String token;
    public final boolean allowLte;
    public final boolean usbBackupToo;
    public final int dailyLimitKb;
    public final String apiHost;
    public final int apiPort;

    public CloudUploadConfig(
            boolean enabled,
            String repo,
            String branch,
            String token,
            boolean allowLte,
            boolean usbBackupToo,
            int dailyLimitKb,
            String apiHost,
            int apiPort) {
        this.enabled = enabled;
        this.repo = normalizeRepo(repo);
        this.branch = (branch == null || branch.trim().isEmpty()) ? DEFAULT_BRANCH : branch.trim();
        this.token = token == null ? "" : token.trim();
        this.allowLte = allowLte;
        this.usbBackupToo = usbBackupToo;
        this.dailyLimitKb = Math.max(256, Math.min(102400, dailyLimitKb));
        this.apiHost = (apiHost == null || apiHost.trim().isEmpty()) ? DEFAULT_API_HOST : apiHost.trim();
        this.apiPort = (apiPort <= 0 || apiPort > 65535) ? DEFAULT_API_PORT : apiPort;
    }

    public static CloudUploadConfig unconfigured() {
        return new CloudUploadConfig(
                true,
                DEFAULT_REPO,
                DEFAULT_BRANCH,
                "",
                true,
                false,
                DEFAULT_DAILY_LIMIT_KB,
                DEFAULT_API_HOST,
                DEFAULT_API_PORT);
    }

    public boolean hasCredentials() {
        return !token.isEmpty() && isValidRepoSlug(repo);
    }

    public boolean isReadyForAutoUpload(IndicatorConfig indCfg) {
        boolean modAllows = indCfg == null || indCfg.autoUploadCloud;
        return enabled && modAllows && hasCredentials();
    }

    public boolean isLteAllowed(IndicatorConfig indCfg) {
        boolean modAllows = indCfg == null || indCfg.cloudAllowLte;
        return allowLte && modAllows;
    }

    public boolean shouldAlsoWriteUsb(IndicatorConfig indCfg) {
        return usbBackupToo || (indCfg != null && indCfg.usbBackupWhenCloud);
    }

    public static boolean isValidRepoSlug(String slug) {
        if (slug == null) return false;
        int slash = slug.indexOf('/');
        if (slash <= 0 || slash >= slug.length() - 1 || slug.indexOf('/', slash + 1) >= 0) {
            return false;
        }
        for (int i = 0; i < slug.length(); i++) {
            char c = slug.charAt(i);
            boolean ok = (c >= 'a' && c <= 'z')
                    || (c >= 'A' && c <= 'Z')
                    || (c >= '0' && c <= '9')
                    || c == '-' || c == '_' || c == '.' || c == '/';
            if (!ok) return false;
        }
        return true;
    }

    public static boolean isPlaceholderToken(String tok) {
        if (tok == null) return true;
        String t = tok.trim();
        if (t.isEmpty()) return true;
        if (t.endsWith("...")) return true;
        String upper = t.toUpperCase(Locale.US);
        return upper.contains("YOUR_TOKEN")
                || upper.contains("REPLACE_ME")
                || upper.contains("PASTE_TOKEN")
                || "GITHUB_PAT_...".equals(upper);
    }

    /**
     * Scans mounted USB roots for {@code <usb>/BrakeIndicator/cloud-upload.json} (or root
     * {@code <usb>/cloud-upload.json}), imports valid GitHub upload credentials into {@code store},
     * and scrubs the plaintext token from the USB file so the PAT isn't left exposed on removable media.
     */
    public static UsbImportResult tryImportFromUsbRoots(List<String> usbRoots, Store store) {
        if (usbRoots == null || usbRoots.isEmpty()) {
            return new UsbImportResult(false, false, false, null, "No USB stick mounted.", null);
        }
        for (int i = 0; i < usbRoots.size(); i++) {
            String root = usbRoots.get(i);
            if (root == null || root.isEmpty()) continue;
            File rootDir = new File(root);
            File subCandidate = new File(new File(rootDir, UsbExporter.SUBFOLDER), USB_CONFIG_FILE);
            File rootCandidate = new File(rootDir, USB_CONFIG_FILE);
            File target = subCandidate.isFile() ? subCandidate : (rootCandidate.isFile() ? rootCandidate : null);
            if (target == null) {
                continue;
            }
            return importFile(target, store);
        }
        return new UsbImportResult(false, false, false, null,
                "No BrakeIndicator/" + USB_CONFIG_FILE + " found on USB.", null);
    }

    static UsbImportResult importFile(File jsonFile, Store store) {
        try {
            String raw = readUtf8(jsonFile);
            JSONObject obj = new JSONObject(raw);
            String tok = obj.optString("token", "").trim();
            String alreadyImportedAt = obj.optString("token_imported_at", "").trim();
            if (tok.isEmpty() && !alreadyImportedAt.isEmpty()) {
                CloudUploadConfig existing = store != null ? store.load() : null;
                return new UsbImportResult(
                        true,
                        false,
                        false,
                        jsonFile.getAbsolutePath(),
                        "USB cloud-upload.json was already imported (" + alreadyImportedAt + ").",
                        existing);
            }
            if (isPlaceholderToken(tok)) {
                return new UsbImportResult(
                        true,
                        false,
                        false,
                        jsonFile.getAbsolutePath(),
                        "cloud-upload.json has an empty or placeholder token.",
                        null);
            }
            String repo = normalizeRepo(obj.optString("repo", DEFAULT_REPO));
            if (!isValidRepoSlug(repo)) {
                return new UsbImportResult(
                        true,
                        false,
                        false,
                        jsonFile.getAbsolutePath(),
                        "Invalid repo '" + repo + "' in cloud-upload.json (expected owner/repo).",
                        null);
            }

            boolean enabled = obj.optBoolean("enabled", true);
            String branch = obj.optString("branch", DEFAULT_BRANCH).trim();
            boolean allowLte = obj.optBoolean("allow_lte", true);
            boolean usbBackupToo = obj.optBoolean("usb_backup_too", false);
            int dailyLimitKb = obj.optInt("daily_limit_kb", DEFAULT_DAILY_LIMIT_KB);
            String apiHost = obj.optString("api_host", DEFAULT_API_HOST).trim();
            int apiPort = obj.optInt("api_port", DEFAULT_API_PORT);

            CloudUploadConfig parsed = new CloudUploadConfig(
                    enabled, repo, branch, tok, allowLte, usbBackupToo, dailyLimitKb, apiHost, apiPort);
            if (store != null) {
                store.save(parsed);
            }

            boolean scrubbed = scrubTokenInUsbFile(jsonFile, parsed);
            String msg = "Imported GitHub upload config for " + parsed.repo
                    + (scrubbed ? " (token scrubbed from USB)." : ".");
            return new UsbImportResult(
                    true, true, scrubbed, jsonFile.getAbsolutePath(), msg, parsed);
        } catch (Exception e) {
            return new UsbImportResult(
                    true,
                    false,
                    false,
                    jsonFile.getAbsolutePath(),
                    "Failed to parse " + jsonFile.getName() + ": " + e.getMessage(),
                    null);
        }
    }

    private static boolean scrubTokenInUsbFile(File target, CloudUploadConfig cfg) {
        try {
            SimpleDateFormat fmt = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", Locale.US);
            fmt.setTimeZone(TimeZone.getTimeZone("UTC"));
            JSONObject receipt = new JSONObject();
            receipt.put("enabled", cfg.enabled);
            receipt.put("repo", cfg.repo);
            receipt.put("branch", cfg.branch);
            receipt.put("token", "");
            receipt.put("token_imported_at", fmt.format(new Date()));
            receipt.put("allow_lte", cfg.allowLte);
            receipt.put("usb_backup_too", cfg.usbBackupToo);
            receipt.put("daily_limit_kb", cfg.dailyLimitKb);

            byte[] bytes = (receipt.toString(2) + "\n").getBytes("UTF-8");
            File tmp = new File(target.getParentFile(), "." + target.getName() + ".scrub");
            FileOutputStream fos = new FileOutputStream(tmp);
            try {
                fos.write(bytes);
                fos.flush();
                try {
                    fos.getFD().sync();
                } catch (Throwable ignored) {}
            } finally {
                fos.close();
            }
            if (!tmp.renameTo(target)) {
                tmp.delete();
                return false;
            }
            return true;
        } catch (Throwable t) {
            return false;
        }
    }

    private static String normalizeRepo(String r) {
        if (r == null || r.trim().isEmpty()) return DEFAULT_REPO;
        String t = r.trim();
        if (t.startsWith("https://github.com/")) {
            t = t.substring("https://github.com/".length());
        }
        while (t.endsWith("/")) {
            t = t.substring(0, t.length() - 1);
        }
        if (t.endsWith(".git")) {
            t = t.substring(0, t.length() - 4);
        }
        return t;
    }

    private static String readUtf8(File f) throws IOException {
        StringBuilder sb = new StringBuilder(512);
        BufferedReader br = new BufferedReader(
                new InputStreamReader(new FileInputStream(f), "UTF-8"), 1024);
        try {
            char[] buf = new char[512];
            int n;
            while ((n = br.read(buf)) >= 0) {
                sb.append(buf, 0, n);
                if (sb.length() > 65536) {
                    throw new IOException("Config file exceeds 64 KB limit");
                }
            }
        } finally {
            br.close();
        }
        return sb.toString();
    }
}
