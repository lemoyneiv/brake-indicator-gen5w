package dev.lemoyne.brakeindicator;

import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Uploads compressed {@code .tar.gz} drive archives directly to a GitHub repository via the
 * GitHub REST Contents API ({@code PUT /repos/<owner>/<repo>/contents/drives/<date>/<name>.tar.gz})
 * over {@link TlsHttpClient}, and provides a 4-stage connectivity + permission diagnostic probe.
 */
public final class GitHubUploader {

    private static final char[] B64_ALPHABET =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/".toCharArray();

    public static final class UploadResult {
        public final boolean ok;
        public final boolean retryable;
        public final int httpStatus;
        public final String remotePath;
        public final String commitSha;
        public final long wireBytesSent;
        public final String message;

        public UploadResult(
                boolean ok,
                boolean retryable,
                int httpStatus,
                String remotePath,
                String commitSha,
                long wireBytesSent,
                String message) {
            this.ok = ok;
            this.retryable = retryable;
            this.httpStatus = httpStatus;
            this.remotePath = remotePath;
            this.commitSha = commitSha;
            this.wireBytesSent = wireBytesSent;
            this.message = message;
        }
    }

    private final TlsHttpClient httpClient;
    private final String sniHostOverride;

    public GitHubUploader() {
        this(new TlsHttpClient(), null);
    }

    public GitHubUploader(TlsHttpClient httpClient, String sniHostOverride) {
        this.httpClient = httpClient == null ? new TlsHttpClient() : httpClient;
        this.sniHostOverride = sniHostOverride;
    }

    public UploadResult uploadDrive(CloudUploadConfig cfg, PendingUploadQueue.QueuedDrive drive) {
        if (cfg == null || !cfg.hasCredentials()) {
            return new UploadResult(false, false, 0, null, null, 0L,
                    "Cloud upload not configured (import BrakeIndicator/cloud-upload.json from USB).");
        }
        if (drive == null || drive.tarGzBytes.length == 0) {
            return new UploadResult(false, false, 0, null, null, 0L, "Empty drive archive.");
        }

        String baseRemotePath = "drives/" + drive.dateFolder + "/" + drive.archiveFileName;
        UploadResult first = putContent(cfg, baseRemotePath, drive.commitMessage, drive.tarGzBytes);
        if (!first.ok && first.httpStatus == 422) {
            // Same-second filename already exists in repo without blob sha -> retry with unique suffix
            String uniqueName = appendCollisionSuffix(drive.archiveFileName, 1);
            String retryPath = "drives/" + drive.dateFolder + "/" + uniqueName;
            return putContent(cfg, retryPath, drive.commitMessage, drive.tarGzBytes);
        }
        return first;
    }

    public String probeGitHubConnectivity(CloudUploadConfig cfg, String networkLabel) {
        String connectHost = cfg != null ? cfg.apiHost : CloudUploadConfig.DEFAULT_API_HOST;
        int port = cfg != null ? cfg.apiPort : CloudUploadConfig.DEFAULT_API_PORT;
        String verifyHost = (sniHostOverride != null && !sniHostOverride.isEmpty())
                ? sniHostOverride : connectHost;

        boolean hasCreds = cfg != null && cfg.hasCredentials();
        String path = hasCreds ? ("/repos/" + cfg.repo) : "/rate_limit";
        Map<String, String> headers = buildGitHubHeaders(hasCreds ? cfg.token : null, false);

        String netPrefix = (networkLabel == null || networkLabel.isEmpty())
                ? "" : ("Net " + networkLabel + " · ");

        TlsHttpClient.ProbeResult pr = httpClient.probe(connectHost, port, verifyHost, path, headers);
        if (!pr.ok && pr.httpStatus == 0) {
            return netPrefix + pr.summary;
        }

        if (!hasCreds) {
            if (pr.httpStatus == 200) {
                return netPrefix + pr.summary + " · GitHub reachable! (Add cloud-upload.json on USB to enable uploads)";
            }
            return netPrefix + pr.summary;
        }

        // Verify exact repo push permissions via full GET /repos/<owner>/<repo>
        try {
            TlsHttpClient.HttpResponse resp = httpClient.execute(
                    "GET", connectHost, port, verifyHost, path, headers, null);
            if (resp.statusCode == 200) {
                JSONObject json = new JSONObject(resp.bodyUtf8());
                JSONObject perms = json.optJSONObject("permissions");
                boolean canPush = perms == null || perms.optBoolean("push", true);
                if (!canPush) {
                    return netPrefix + "TLS 1.2 OK, but token lacks Contents: Write permission on " + cfg.repo;
                }
                return netPrefix + pr.summary + " · Repo " + cfg.repo + " READY (write verified)";
            }
            if (resp.statusCode == 401) {
                return netPrefix + "TLS 1.2 OK, HTTP 401: GitHub token invalid or expired.";
            }
            if (resp.statusCode == 403) {
                return netPrefix + "TLS 1.2 OK, HTTP 403: Token lacks access to " + cfg.repo + ".";
            }
            if (resp.statusCode == 404) {
                return netPrefix + "TLS 1.2 OK, HTTP 404: Repo " + cfg.repo
                        + " not found or token not granted access.";
            }
            return netPrefix + "TLS 1.2 OK, HTTP " + resp.statusCode + " from GitHub.";
        } catch (Exception e) {
            return netPrefix + pr.summary + " (" + e.getMessage() + ")";
        }
    }

    private UploadResult putContent(
            CloudUploadConfig cfg,
            String remotePath,
            String commitMessage,
            byte[] rawBytes) {
        byte[] jsonBody;
        try {
            jsonBody = buildContentsPutJson(commitMessage, cfg.branch, rawBytes);
        } catch (IOException e) {
            return new UploadResult(false, false, 0, remotePath, null, 0L,
                    "Failed to encode upload payload: " + e.getMessage());
        }

        String apiPath = "/repos/" + cfg.repo + "/contents/" + remotePath;
        String verifyHost = (sniHostOverride != null && !sniHostOverride.isEmpty())
                ? sniHostOverride : cfg.apiHost;
        Map<String, String> headers = buildGitHubHeaders(cfg.token, true);

        try {
            TlsHttpClient.HttpResponse resp = httpClient.execute(
                    "PUT",
                    cfg.apiHost,
                    cfg.apiPort,
                    verifyHost,
                    apiPath,
                    headers,
                    jsonBody);

            int code = resp.statusCode;
            if (code == 200 || code == 201) {
                String sha = extractShortCommitSha(resp.bodyUtf8());
                int kb = Math.max(1, (rawBytes.length + 512) / 1024);
                String msg = String.format(
                        Locale.US,
                        "Uploaded %s (%d KB gz%s)",
                        remotePath.substring(remotePath.lastIndexOf('/') + 1),
                        Integer.valueOf(kb),
                        sha != null ? (", " + sha) : "");
                return new UploadResult(true, false, code, remotePath, sha, jsonBody.length, msg);
            }

            boolean retryable = (code == 408 || code == 429 || code >= 500);
            String ghMsg = extractGitHubErrorMessage(resp.bodyUtf8());
            String detail;
            if (code == 401) {
                detail = "GitHub HTTP 401: Token invalid or expired.";
            } else if (code == 403) {
                detail = "GitHub HTTP 403: Check token Contents: Write permission (" + ghMsg + ")";
            } else if (code == 404) {
                detail = "GitHub HTTP 404: Repo " + cfg.repo + " or branch '" + cfg.branch + "' not accessible.";
            } else {
                detail = "GitHub HTTP " + code + (ghMsg.isEmpty() ? "" : (": " + ghMsg));
            }
            return new UploadResult(false, retryable, code, remotePath, null, jsonBody.length, detail);
        } catch (IOException ioe) {
            return new UploadResult(false, true, 0, remotePath, null, 0L,
                    "Cloud upload network error: " + ioe.getMessage());
        }
    }

    private static Map<String, String> buildGitHubHeaders(String token, boolean jsonContentType) {
        Map<String, String> h = new LinkedHashMap<String, String>();
        h.put("Accept", "application/vnd.github+json");
        h.put("X-GitHub-Api-Version", "2022-11-28");
        if (token != null && !token.isEmpty()) {
            h.put("Authorization", "Bearer " + token);
        }
        if (jsonContentType) {
            h.put("Content-Type", "application/json; charset=utf-8");
        }
        return h;
    }

    /**
     * Streams {@code {"message":"...","branch":"...","content":"<base64>"}} directly into a single
     * {@code byte[]} without building intermediate multi-megabyte Java Strings.
     */
    static byte[] buildContentsPutJson(String message, String branch, byte[] contentBytes)
            throws IOException {
        int b64Len = ((contentBytes.length + 2) / 3) * 4;
        ByteArrayOutputStream out = new ByteArrayOutputStream(b64Len + 256);
        String prefix = "{\"message\":" + JSONObject.quote(message == null ? "telemetry log" : message)
                + ",\"branch\":" + JSONObject.quote(branch == null ? "main" : branch)
                + ",\"content\":\"";
        out.write(prefix.getBytes("UTF-8"));
        writeBase64Ascii(out, contentBytes);
        out.write('\"');
        out.write('}');
        return out.toByteArray();
    }

    static void writeBase64Ascii(ByteArrayOutputStream out, byte[] data) {
        int len = data.length;
        int i = 0;
        byte[] chunk = new byte[3072];
        int cPos = 0;
        while (i + 3 <= len) {
            int v = ((data[i] & 0xFF) << 16) | ((data[i + 1] & 0xFF) << 8) | (data[i + 2] & 0xFF);
            i += 3;
            chunk[cPos++] = (byte) B64_ALPHABET[(v >>> 18) & 0x3F];
            chunk[cPos++] = (byte) B64_ALPHABET[(v >>> 12) & 0x3F];
            chunk[cPos++] = (byte) B64_ALPHABET[(v >>> 6) & 0x3F];
            chunk[cPos++] = (byte) B64_ALPHABET[v & 0x3F];
            if (cPos >= chunk.length - 4) {
                out.write(chunk, 0, cPos);
                cPos = 0;
            }
        }
        int rem = len - i;
        if (rem == 1) {
            int v = (data[i] & 0xFF) << 16;
            chunk[cPos++] = (byte) B64_ALPHABET[(v >>> 18) & 0x3F];
            chunk[cPos++] = (byte) B64_ALPHABET[(v >>> 12) & 0x3F];
            chunk[cPos++] = '=';
            chunk[cPos++] = '=';
        } else if (rem == 2) {
            int v = ((data[i] & 0xFF) << 16) | ((data[i + 1] & 0xFF) << 8);
            chunk[cPos++] = (byte) B64_ALPHABET[(v >>> 18) & 0x3F];
            chunk[cPos++] = (byte) B64_ALPHABET[(v >>> 12) & 0x3F];
            chunk[cPos++] = (byte) B64_ALPHABET[(v >>> 6) & 0x3F];
            chunk[cPos++] = '=';
        }
        if (cPos > 0) {
            out.write(chunk, 0, cPos);
        }
    }

    private static String appendCollisionSuffix(String fileName, int n) {
        if (fileName.endsWith(".tar.gz")) {
            return fileName.substring(0, fileName.length() - 7) + "_" + n + ".tar.gz";
        }
        int dot = fileName.lastIndexOf('.');
        if (dot > 0) {
            return fileName.substring(0, dot) + "_" + n + fileName.substring(dot);
        }
        return fileName + "_" + n;
    }

    private static String extractShortCommitSha(String body) {
        try {
            JSONObject root = new JSONObject(body);
            JSONObject commit = root.optJSONObject("commit");
            if (commit != null) {
                String sha = commit.optString("sha", "");
                if (sha.length() >= 7) {
                    return sha.substring(0, 7);
                }
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    private static String extractGitHubErrorMessage(String body) {
        if (body == null || body.isEmpty()) return "";
        try {
            JSONObject root = new JSONObject(body);
            return root.optString("message", "").trim();
        } catch (Exception e) {
            return body.length() > 80 ? body.substring(0, 80) : body.trim();
        }
    }
}
