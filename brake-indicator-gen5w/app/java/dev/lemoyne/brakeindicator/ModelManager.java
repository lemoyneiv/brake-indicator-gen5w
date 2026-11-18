package dev.lemoyne.brakeindicator;

import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.net.URL;
import java.net.URLClassLoader;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Manages discovery, zero-flash-wear {@code ETag} polling, ECDSA P-256 verification, versioned
 * Dalvik {@code DexClassLoader} loading, 6-tick pre-swap self-testing, runtime watchdog rollback,
 * and automatic retirement of OTA {@link BrakeModel} plugins.
 *
 * <p>Fetches {@code latest.json} and {@code models/brake-model-<ver>.jar} from the public
 * {@code ota} branch of {@code lemoyneiv/brake-indicator-gen5w} via {@code api.github.com}
 * ({@code Accept: application/vnd.github.raw+json}) in a single TLS 1.2 connection with zero
 * authentication token required and zero S3 {@code 302} redirects.
 */
public final class ModelManager {

    public static final String DEFAULT_OTA_REPO = "lemoyneiv/brake-indicator-gen5w";
    public static final String DEFAULT_OTA_BRANCH = "ota";
    public static final String LATEST_MANIFEST_PATH = "latest.json";
    public static final String USB_MODEL_PREFIX = "brake-model-";

    /** Minimum interval between automatic Park model checks (in-RAM only, zero flash writes). */
    public static final long PARK_CHECK_COOLDOWN_MS = 15L * 60L * 1000L;

    public static final class UpdateOutcome {
        public final boolean ok;
        public final boolean updated;
        public final boolean throttled;
        public final String activeVersion;
        public final boolean activeIsOta;
        public final String message;

        public UpdateOutcome(
                boolean ok,
                boolean updated,
                boolean throttled,
                String activeVersion,
                boolean activeIsOta,
                String message) {
            this.ok = ok;
            this.updated = updated;
            this.throttled = throttled;
            this.activeVersion = activeVersion == null ? "" : activeVersion;
            this.activeIsOta = activeIsOta;
            this.message = message == null ? "" : message;
        }
    }

    private final File modelsDir;
    private final File dexOptRootDir;
    private final BrakeModel builtInModel;
    private final TlsHttpClient httpClient;
    private final String otaRepo;
    private final String otaBranch;
    private final String apiHost;
    private final int apiPort;
    private final String sniHostOverride;
    private final String expectedAuthorUuid;
    private final String expectedAuthorPubKeyB64;

    private final Object swapLock = new Object();
    private final AtomicBoolean updateInFlight = new AtomicBoolean(false);
    private final Set<Long> quarantinedSeqs = Collections.synchronizedSet(new HashSet<Long>());

    private volatile BrakeModel activeModel;
    private volatile boolean activeIsOta;
    private volatile String activeSha256Short = "";

    // Strictly in-RAM ETag & cooldown state (zero eMMC writes during 304 / up-to-date checks)
    private volatile String cachedLatestEtag;
    private volatile long cachedRemoteSeq;
    private volatile long lastAutoCheckMonoMs = -1L;
    private volatile boolean bootParkedCheckDone;

    public ModelManager(File appFilesDir) {
        this(
                appFilesDir,
                new BrakeEvaluator(),
                new TlsHttpClient(),
                DEFAULT_OTA_REPO,
                DEFAULT_OTA_BRANCH,
                CloudUploadConfig.DEFAULT_API_HOST,
                CloudUploadConfig.DEFAULT_API_PORT,
                null,
                ModelBundleVerifier.PINNED_AUTHOR_UUID,
                ModelBundleVerifier.PINNED_AUTHOR_PUBKEY_B64);
    }

    public ModelManager(
            File appFilesDir,
            BrakeModel builtInModel,
            TlsHttpClient httpClient,
            String otaRepo,
            String otaBranch,
            String apiHost,
            int apiPort,
            String sniHostOverride,
            String expectedAuthorUuid,
            String expectedAuthorPubKeyB64) {
        File base = appFilesDir == null ? new File("/tmp/brake-indicator-models") : appFilesDir;
        this.modelsDir = new File(base, "models");
        this.dexOptRootDir = new File(base, "model_dexopt");
        this.builtInModel = builtInModel == null ? new BrakeEvaluator() : builtInModel;
        this.httpClient = httpClient == null ? new TlsHttpClient() : httpClient;
        this.otaRepo = (otaRepo == null || otaRepo.isEmpty()) ? DEFAULT_OTA_REPO : otaRepo;
        this.otaBranch = (otaBranch == null || otaBranch.isEmpty()) ? DEFAULT_OTA_BRANCH : otaBranch;
        this.apiHost = (apiHost == null || apiHost.isEmpty()) ? CloudUploadConfig.DEFAULT_API_HOST : apiHost;
        this.apiPort = apiPort <= 0 ? CloudUploadConfig.DEFAULT_API_PORT : apiPort;
        this.sniHostOverride = sniHostOverride;
        this.expectedAuthorUuid = expectedAuthorUuid;
        this.expectedAuthorPubKeyB64 = expectedAuthorPubKeyB64;
        this.activeModel = this.builtInModel;
        this.activeIsOta = false;
    }

    public BrakeModel getActiveModel() {
        return activeModel;
    }

    public boolean isActiveOta() {
        return activeIsOta;
    }

    public boolean isUpdateInFlight() {
        return updateInFlight.get();
    }

    public boolean shouldRunBootParkedCheck() {
        return !bootParkedCheckDone;
    }

    public void markBootParkedCheckDone() {
        bootParkedCheckDone = true;
    }

    /** Compact label for UI footer, e.g. {@code "2026.11.18.1 (OTA)"} or {@code "2026.11.18.1 (builtin)"}. */
    public String getActiveModelBadge() {
        BrakeModel m = activeModel;
        return m.modelVersion() + (activeIsOta ? " (OTA)" : " (builtin)");
    }

    /** Detailed line stamped into {@code brake-summary-<stamp>.txt}. */
    public String getActiveModelSummaryLine() {
        BrakeModel m = activeModel;
        if (activeIsOta) {
            return m.modelVersion() + " (OTA, seq=" + m.modelSeq()
                    + (activeSha256Short.isEmpty() ? "" : (", sha256=" + activeSha256Short)) + ")";
        }
        return m.modelVersion() + " (builtin, seq=" + m.modelSeq() + ")";
    }

    /** Compact slug included in cloud upload GitHub commit messages. */
    public String getActiveModelCommitTag() {
        BrakeModel m = activeModel;
        return m.modelVersion() + (activeIsOta ? "-ota" : "-builtin");
    }

    /**
     * Loads any previously verified OTA model cached in {@code <filesDir>/models/} at startup,
     * automatically retiring and deleting cached plugins whose {@code model_seq <= builtInModel.modelSeq()}.
     */
    public void initFromDisk(TelemetryRecorder recorder, long nowMonoMs) {
        synchronized (swapLock) {
            if (!modelsDir.isDirectory()) {
                syncRecorderAttribution(recorder);
                return;
            }
            File[] files = modelsDir.listFiles();
            if (files == null || files.length == 0) {
                syncRecorderAttribution(recorder);
                return;
            }

            File bestFile = null;
            ModelBundleVerifier.VerifiedBundle bestVerified = null;

            for (int i = 0; i < files.length; i++) {
                File f = files[i];
                if (!f.isFile()) {
                    continue;
                }
                String name = f.getName();
                if (name.endsWith(".part") || !name.startsWith(USB_MODEL_PREFIX) || !name.endsWith(".jar")) {
                    safeDeleteFile(f);
                    continue;
                }
                try {
                    byte[] raw = ModelBundleVerifier.readFileBytes(f);
                    ModelBundleVerifier.VerifiedBundle ver = ModelBundleVerifier.verifyBundle(
                            raw,
                            BrakeModel.HOST_MODEL_API_VERSION,
                            expectedAuthorUuid,
                            expectedAuthorPubKeyB64);
                    if (!ver.ok) {
                        safeDeleteFile(f);
                        continue;
                    }
                    if (ver.modelSeq <= builtInModel.modelSeq()) {
                        safeDeleteFile(f);
                        deleteDirRecursive(dexOptDirForSeq(ver.modelSeq));
                        if (recorder != null) {
                            recorder.recordExternalEvent(
                                    nowMonoMs,
                                    "Retired older cached OTA model " + ver.modelVersion
                                            + " (built-in APK model is " + builtInModel.modelVersion() + ")");
                        }
                        continue;
                    }
                    if (bestVerified == null || ver.modelSeq > bestVerified.modelSeq) {
                        bestFile = f;
                        bestVerified = ver;
                    }
                } catch (Throwable t) {
                    safeDeleteFile(f);
                }
            }

            if (bestFile != null && bestVerified != null) {
                try {
                    bestFile.setWritable(false);
                    BrakeModel loaded = loadAndSelfTest(bestFile, bestVerified);
                    activeModel = loaded;
                    activeIsOta = true;
                    activeSha256Short = shortSha(bestVerified.bundleSha256);
                    pruneOtherModelsLocked(bestVerified.modelSeq);
                    if (recorder != null) {
                        recorder.recordExternalEvent(
                                nowMonoMs,
                                "Loaded cached OTA model " + bestVerified.modelVersion
                                        + " (seq=" + bestVerified.modelSeq + ")");
                    }
                } catch (Throwable t) {
                    quarantinedSeqs.add(Long.valueOf(bestVerified.modelSeq));
                    safeDeleteFile(bestFile);
                    deleteDirRecursive(dexOptDirForSeq(bestVerified.modelSeq));
                    activeModel = builtInModel;
                    activeIsOta = false;
                    activeSha256Short = "";
                    if (recorder != null) {
                        recorder.recordExternalEvent(
                                nowMonoMs,
                                "Rejected cached OTA model " + bestVerified.modelVersion
                                        + " (" + t.getClass().getSimpleName() + ") -> using built-in");
                    }
                }
            } else {
                pruneOtherModelsLocked(-1L);
            }
            syncRecorderAttribution(recorder);
        }
    }

    /**
     * Evaluates a single 50 ms tick on the active {@link BrakeModel} with an instant runtime
     * {@code try/catch (Throwable)} watchdog that reverts on the exact same tick to
     * {@link #builtInModel} and quarantines any throwing OTA plugin.
     */
    public BrakeEvaluator.Snapshot evaluateTick(
            IndicatorConfig config,
            BrakeEvaluator.Input input,
            Map<String, String> rawReadings,
            TelemetryRecorder recorder,
            long nowMonoMs) {
        BrakeModel current = this.activeModel;
        if (current != builtInModel) {
            try {
                BrakeEvaluator.Snapshot snap = current.evaluate(config, input, rawReadings);
                if (snap != null && !Float.isNaN(snap.decelMps2) && !Float.isInfinite(snap.decelMps2)) {
                    return snap;
                }
                throw new IllegalStateException("OTA BrakeModel returned invalid Snapshot");
            } catch (Throwable fault) {
                quarantineAndFallback(current, fault, recorder, nowMonoMs);
            }
        }
        return builtInModel.evaluate(config, input, rawReadings);
    }

    /**
     * Checks {@code ota/latest.json} (or imports a signed {@code <usb>/BrakeIndicator/brake-model-*.jar}
     * when offline during a manual check), verifies ECDSA signature &amp; 6-tick self-test, and
     * atomically activates any newer {@code model_seq}.
     */
    public UpdateOutcome checkForUpdate(
            boolean manual,
            IndicatorConfig cfg,
            AutoExportCoordinator.TransportInfo net,
            List<String> usbRoots,
            TelemetryRecorder recorder,
            long nowMonoMs) {
        if (!updateInFlight.compareAndSet(false, true)) {
            return new UpdateOutcome(
                    false,
                    false,
                    false,
                    activeModel.modelVersion(),
                    activeIsOta,
                    "Model update check already in progress.");
        }
        try {
            if (!manual) {
                if (cfg != null && !cfg.autoUpdateModel) {
                    return new UpdateOutcome(
                            true, false, true, activeModel.modelVersion(), activeIsOta,
                            "Auto model updates disabled in settings.");
                }
                long prev = lastAutoCheckMonoMs;
                if (prev > 0L && (nowMonoMs - prev) >= 0L && (nowMonoMs - prev) < PARK_CHECK_COOLDOWN_MS) {
                    return new UpdateOutcome(
                            true, false, true, activeModel.modelVersion(), activeIsOta,
                            "Auto model check within 15m cooldown.");
                }
            }

            // On manual Update Model with a USB stick attached containing a signed newer .jar,
            // allow offline USB import whenever network is absent (or even inspect USB if offline).
            AutoExportCoordinator.TransportInfo transport = net == null
                    ? new AutoExportCoordinator.TransportInfo(AutoExportCoordinator.TransportKind.NONE, "Offline")
                    : net;

            if (transport.kind == AutoExportCoordinator.TransportKind.NONE) {
                if (manual && usbRoots != null && !usbRoots.isEmpty()) {
                    UpdateOutcome usbOut = tryImportFromUsbRoots(usbRoots, recorder, nowMonoMs);
                    if (usbOut != null) {
                        return usbOut;
                    }
                }
                return new UpdateOutcome(
                        false,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Offline \u2014 connect LTE/Wi-Fi (or place signed brake-model-*.jar in USB BrakeIndicator/).");
            }

            if (!manual
                    && transport.kind == AutoExportCoordinator.TransportKind.CELLULAR
                    && cfg != null
                    && !cfg.cloudAllowLte) {
                return new UpdateOutcome(
                        true, false, true, activeModel.modelVersion(), activeIsOta,
                        "Skipped auto model check over LTE (LTE disabled in settings).");
            }

            if (!manual) {
                lastAutoCheckMonoMs = nowMonoMs;
            }

            String verifyHost = (sniHostOverride != null && !sniHostOverride.isEmpty())
                    ? sniHostOverride : apiHost;
            String latestPath = "/repos/" + otaRepo + "/contents/" + LATEST_MANIFEST_PATH
                    + "?ref=" + otaBranch;
            Map<String, String> headers = new LinkedHashMap<String, String>();
            headers.put("Accept", "application/vnd.github.raw+json");
            headers.put("X-GitHub-Api-Version", "2022-11-28");
            String prevEtag = cachedLatestEtag;
            if (prevEtag != null
                    && !prevEtag.isEmpty()
                    && cachedRemoteSeq > 0L
                    && cachedRemoteSeq <= activeModel.modelSeq()) {
                headers.put("If-None-Match", prevEtag);
            }

            TlsHttpClient.HttpResponse latestResp = httpClient.execute(
                    "GET", apiHost, apiPort, verifyHost, latestPath, headers, null);
            if (latestResp.statusCode == 304) {
                return new UpdateOutcome(
                        true,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Model up to date: " + getActiveModelBadge());
            }
            if (latestResp.statusCode == 404) {
                return new UpdateOutcome(
                        false,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "OTA channel not found (" + otaRepo + "@" + otaBranch + ").");
            }
            if (latestResp.statusCode != 200) {
                return new UpdateOutcome(
                        false,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "OTA check HTTP " + latestResp.statusCode + " from GitHub.");
            }

            String newEtag = latestResp.headers != null ? latestResp.headers.get("etag") : null;
            JSONObject latest = new JSONObject(latestResp.bodyUtf8());
            String remoteVer = latest.optString("model_version", "").trim();
            long remoteSeq = latest.optLong("model_seq", -1L);
            int minApi = latest.optInt("min_host_api", 1);
            int maxApi = latest.optInt("max_host_api", minApi);
            String jarPath = latest.optString("jar_path", "").trim();
            String expectedJarSha = latest.optString("jar_sha256", "").trim().toLowerCase(Locale.US);

            if (!ModelBundleVerifier.isSafeModelVersion(remoteVer) || remoteSeq <= 0L) {
                return new UpdateOutcome(
                        false, false, false, activeModel.modelVersion(), activeIsOta,
                        "Invalid remote latest.json metadata.");
            }

            if (BrakeModel.HOST_MODEL_API_VERSION < minApi
                    || (maxApi > 0 && BrakeModel.HOST_MODEL_API_VERSION > maxApi)) {
                return new UpdateOutcome(
                        true,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Model " + remoteVer + " requires newer app APK (host API " + minApi + ").");
            }

            if (quarantinedSeqs.contains(Long.valueOf(remoteSeq))) {
                return new UpdateOutcome(
                        false,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Model " + remoteVer + " was quarantined after fault; keeping "
                                + getActiveModelBadge() + ".");
            }

            if (remoteSeq <= activeModel.modelSeq()) {
                cachedLatestEtag = newEtag;
                cachedRemoteSeq = remoteSeq;
                return new UpdateOutcome(
                        true,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Model up to date: " + getActiveModelBadge());
            }

            if (!isSafeRepoJarPath(jarPath)) {
                return new UpdateOutcome(
                        false, false, false, activeModel.modelVersion(), activeIsOta,
                        "Rejected unsafe jar_path in latest.json: " + jarPath);
            }

            // Download signed .jar (~16 KB) directly over the same GitHub Contents raw endpoint
            Map<String, String> jarHeaders = new LinkedHashMap<String, String>();
            jarHeaders.put("Accept", "application/vnd.github.raw+json");
            jarHeaders.put("X-GitHub-Api-Version", "2022-11-28");
            String jarApiPath = "/repos/" + otaRepo + "/contents/" + jarPath + "?ref=" + otaBranch;

            TlsHttpClient.HttpResponse jarResp = httpClient.execute(
                    "GET", apiHost, apiPort, verifyHost, jarApiPath, jarHeaders, null);
            if (jarResp.statusCode != 200 || jarResp.body.length == 0) {
                return new UpdateOutcome(
                        false,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Model download failed (HTTP " + jarResp.statusCode + ").");
            }

            if (!expectedJarSha.isEmpty()) {
                String dlSha = ModelBundleVerifier.sha256Hex(jarResp.body);
                if (!expectedJarSha.equals(dlSha)) {
                    return new UpdateOutcome(
                            false,
                            false,
                            false,
                            activeModel.modelVersion(),
                            activeIsOta,
                            "Downloaded model SHA-256 mismatch against latest.json.");
                }
            }

            ModelBundleVerifier.VerifiedBundle verified = ModelBundleVerifier.verifyBundle(
                    jarResp.body,
                    BrakeModel.HOST_MODEL_API_VERSION,
                    expectedAuthorUuid,
                    expectedAuthorPubKeyB64);
            if (!verified.ok) {
                return new UpdateOutcome(
                        false,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Rejected OTA model: " + verified.error);
            }
            if (verified.modelSeq != remoteSeq || !verified.modelVersion.equals(remoteVer)) {
                return new UpdateOutcome(
                        false,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Rejected OTA model: manifest version/seq differs from latest.json.");
            }

            return installVerifiedBundle(jarResp.body, verified, newEtag, transport.label, recorder, nowMonoMs);
        } catch (IOException ioe) {
            return new UpdateOutcome(
                    false,
                    false,
                    false,
                    activeModel.modelVersion(),
                    activeIsOta,
                    "Model update network error: " + ioe.getMessage());
        } catch (Exception e) {
            return new UpdateOutcome(
                    false,
                    false,
                    false,
                    activeModel.modelVersion(),
                    activeIsOta,
                    "Model update error: " + e.getMessage());
        } finally {
            updateInFlight.set(false);
        }
    }

    /**
     * Reverts immediately to the compiled-in APK {@link BrakeEvaluator} and deletes any cached
     * OTA {@code .jar} from internal storage (invoked on long-pressing {@code Update Model}).
     */
    public UpdateOutcome resetToBuiltIn(TelemetryRecorder recorder, long nowMonoMs) {
        synchronized (swapLock) {
            boolean wasOta = activeIsOta;
            String prevVer = activeModel.modelVersion();
            builtInModel.reset();
            activeModel = builtInModel;
            activeIsOta = false;
            activeSha256Short = "";
            cachedLatestEtag = null;
            cachedRemoteSeq = 0L;
            pruneOtherModelsLocked(-1L);
            syncRecorderAttribution(recorder);
            if (recorder != null) {
                recorder.recordExternalEvent(
                        nowMonoMs,
                        "Reset brake model to built-in " + builtInModel.modelVersion()
                                + (wasOta ? (" (was OTA " + prevVer + ")") : ""));
            }
            String msg = wasOta
                    ? ("Reverted to built-in model " + builtInModel.modelVersion() + " (cleared OTA " + prevVer + ").")
                    : ("Already running built-in model " + builtInModel.modelVersion() + ".");
            return new UpdateOutcome(true, wasOta, false, builtInModel.modelVersion(), false, msg);
        }
    }

    private UpdateOutcome tryImportFromUsbRoots(
            List<String> usbRoots,
            TelemetryRecorder recorder,
            long nowMonoMs) {
        File bestUsbJar = null;
        byte[] bestBytes = null;
        ModelBundleVerifier.VerifiedBundle bestVer = null;

        for (int r = 0; r < usbRoots.size(); r++) {
            File biDir = new File(usbRoots.get(r), UsbExporter.SUBFOLDER);
            if (!biDir.isDirectory()) {
                continue;
            }
            File[] list = biDir.listFiles();
            if (list == null) {
                continue;
            }
            for (int i = 0; i < list.length; i++) {
                File f = list[i];
                if (!f.isFile()
                        || !f.getName().startsWith(USB_MODEL_PREFIX)
                        || !f.getName().endsWith(".jar")
                        || f.length() > ModelBundleVerifier.MAX_BUNDLE_BYTES) {
                    continue;
                }
                try {
                    byte[] raw = ModelBundleVerifier.readFileBytes(f);
                    ModelBundleVerifier.VerifiedBundle ver = ModelBundleVerifier.verifyBundle(
                            raw,
                            BrakeModel.HOST_MODEL_API_VERSION,
                            expectedAuthorUuid,
                            expectedAuthorPubKeyB64);
                    if (!ver.ok || quarantinedSeqs.contains(Long.valueOf(ver.modelSeq))) {
                        continue;
                    }
                    if (bestVer == null || ver.modelSeq > bestVer.modelSeq) {
                        bestUsbJar = f;
                        bestBytes = raw;
                        bestVer = ver;
                    }
                } catch (Exception ignored) {
                }
            }
        }

        if (bestUsbJar == null || bestVer == null) {
            return null;
        }
        if (bestVer.modelSeq <= activeModel.modelSeq()) {
            return new UpdateOutcome(
                    true,
                    false,
                    false,
                    activeModel.modelVersion(),
                    activeIsOta,
                    "USB model " + bestVer.modelVersion + " is not newer than active " + getActiveModelBadge() + ".");
        }
        return installVerifiedBundle(bestBytes, bestVer, null, "USB", recorder, nowMonoMs);
    }

    private UpdateOutcome installVerifiedBundle(
            byte[] jarBytes,
            ModelBundleVerifier.VerifiedBundle verified,
            String newEtag,
            String sourceLabel,
            TelemetryRecorder recorder,
            long nowMonoMs) {
        synchronized (swapLock) {
            if (!modelsDir.exists() && !modelsDir.mkdirs()) {
                return new UpdateOutcome(
                        false, false, false, activeModel.modelVersion(), activeIsOta,
                        "Failed to create internal models directory.");
            }
            File targetJar = new File(modelsDir, USB_MODEL_PREFIX + verified.modelSeq + ".jar");
            File partJar = new File(modelsDir, USB_MODEL_PREFIX + verified.modelSeq + ".jar.part");
            safeDeleteFile(partJar);
            safeDeleteFile(targetJar);

            try {
                FileOutputStream fos = new FileOutputStream(partJar);
                try {
                    fos.write(jarBytes);
                    fos.getFD().sync();
                } finally {
                    fos.close();
                }
                if (!partJar.renameTo(targetJar)) {
                    safeDeleteFile(partJar);
                    return new UpdateOutcome(
                            false, false, false, activeModel.modelVersion(), activeIsOta,
                            "Failed to commit verified model jar.");
                }
                targetJar.setWritable(false);

                BrakeModel loaded = loadAndSelfTest(targetJar, verified);
                String prevVer = activeModel.modelVersion();
                activeModel = loaded;
                activeIsOta = true;
                activeSha256Short = shortSha(verified.bundleSha256);
                if (newEtag != null && !newEtag.isEmpty()) {
                    cachedLatestEtag = newEtag;
                }
                cachedRemoteSeq = verified.modelSeq;
                pruneOtherModelsLocked(verified.modelSeq);
                syncRecorderAttribution(recorder);

                int kb = Math.max(1, (jarBytes.length + 512) / 1024);
                if (recorder != null) {
                    recorder.recordExternalEvent(
                            nowMonoMs,
                            "Updated brake model " + prevVer + " -> " + verified.modelVersion
                                    + " via " + sourceLabel + " (" + kb + " KB, sha256=" + activeSha256Short + ")");
                }
                return new UpdateOutcome(
                        true,
                        true,
                        false,
                        verified.modelVersion,
                        true,
                        "Updated model \u2192 " + verified.modelVersion
                                + " (" + sourceLabel + ", " + kb + " KB)");
            } catch (Throwable fault) {
                quarantinedSeqs.add(Long.valueOf(verified.modelSeq));
                safeDeleteFile(partJar);
                safeDeleteFile(targetJar);
                deleteDirRecursive(dexOptDirForSeq(verified.modelSeq));
                if (recorder != null) {
                    recorder.recordExternalEvent(
                            nowMonoMs,
                            "Rejected OTA model " + verified.modelVersion
                                    + " during self-test (" + fault.getClass().getSimpleName() + ")");
                }
                return new UpdateOutcome(
                        false,
                        false,
                        false,
                        activeModel.modelVersion(),
                        activeIsOta,
                        "Model self-test failed (" + fault.getMessage() + "); kept " + getActiveModelBadge() + ".");
            }
        }
    }

    private BrakeModel loadAndSelfTest(
            File jarFile,
            ModelBundleVerifier.VerifiedBundle verified) throws Exception {
        File dexOptDir = dexOptDirForSeq(verified.modelSeq);
        deleteDirRecursive(dexOptDir);
        dexOptDir.mkdirs();

        ClassLoader parent = BrakeModel.class.getClassLoader();
        ClassLoader pluginLoader;
        try {
            Class<?> dexClClass = Class.forName("dalvik.system.DexClassLoader");
            Constructor<?> ctor = dexClClass.getConstructor(
                    String.class, String.class, String.class, ClassLoader.class);
            pluginLoader = (ClassLoader) ctor.newInstance(
                    jarFile.getAbsolutePath(),
                    dexOptDir.getAbsolutePath(),
                    null,
                    parent);
        } catch (ClassNotFoundException hostJvm) {
            pluginLoader = new URLClassLoader(new URL[]{jarFile.toURI().toURL()}, parent);
        }

        Class<?> implClass = Class.forName(verified.entryClass, true, pluginLoader);
        if (!BrakeModel.class.isAssignableFrom(implClass)) {
            throw new IllegalStateException(verified.entryClass + " does not implement BrakeModel");
        }
        BrakeModel candidate = (BrakeModel) implClass.getDeclaredConstructor().newInstance();
        if (candidate.modelApiVersion() != BrakeModel.HOST_MODEL_API_VERSION) {
            throw new IllegalStateException("ABI mismatch: " + candidate.modelApiVersion());
        }
        if (candidate.modelSeq() != verified.modelSeq
                || !verified.modelVersion.equals(candidate.modelVersion())) {
            throw new IllegalStateException("Class metadata mismatch vs signed manifest");
        }

        runSelfTest(candidate);
        candidate.reset();
        return candidate;
    }

    /**
     * Gate 1 deterministic 6-tick sanity check executed before any OTA model can become active:
     * verifies Park OFF, firm foot-brake ON, highway L3 cruise, urban i-Pedal feathering,
     * standstill, and {@code null}-argument resilience.
     */
    static void runSelfTest(BrakeModel candidate) {
        candidate.reset();
        IndicatorConfig cfg = IndicatorConfig.defaults();
        Map<String, String> emptyRaw = Collections.emptyMap();

        // 1. Quiet Park -> OFF
        BrakeEvaluator.Snapshot s1 = candidate.evaluate(
                cfg,
                new BrakeEvaluator.Input(
                        1000L, 0.0f, 0.0f, 0.0f, 0.0f,
                        Boolean.TRUE, Boolean.TRUE, Boolean.FALSE, 75.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, 1000L),
                emptyRaw);
        if (s1 == null || s1.modelIlluminated || Float.isNaN(s1.decelMps2)) {
            throw new IllegalStateException("Self-test tick 1 (quiet Park) failed");
        }

        // 2. Firm 12.0 bar hydraulic foot brake in Drive -> ON
        BrakeEvaluator.Snapshot s2 = candidate.evaluate(
                cfg,
                new BrakeEvaluator.Input(
                        1100L, 12.0f, 36.0f, 0.0f, 0.0f,
                        Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, 1100L),
                emptyRaw);
        if (s2 == null || !s2.modelIlluminated || Float.isNaN(s2.decelMps2)) {
            throw new IllegalStateException("Self-test tick 2 (12 bar foot brake) failed");
        }

        // 3. Highway neutral cruise (80 km/h @ 24% pedal)
        BrakeEvaluator.Snapshot s3 = candidate.evaluate(
                cfg,
                new BrakeEvaluator.Input(
                        2000L, 0.0f, 80.0f, 24.0f, 0.0f,
                        Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, 2000L),
                emptyRaw);
        if (s3 == null || Float.isNaN(s3.decelMps2) || Float.isInfinite(s3.decelMps2)) {
            throw new IllegalStateException("Self-test tick 3 (highway cruise) failed");
        }

        // 4. Urban feathered deceleration (32 km/h @ 8.5% pedal)
        BrakeEvaluator.Snapshot s4 = candidate.evaluate(
                cfg,
                new BrakeEvaluator.Input(
                        2100L, 0.0f, 31.6f, 8.5f, 4.0f,
                        Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, 2100L),
                emptyRaw);
        if (s4 == null || Float.isNaN(s4.decelMps2) || Float.isInfinite(s4.decelMps2)) {
            throw new IllegalStateException("Self-test tick 4 (urban feather) failed");
        }

        // 5. Standstill in Drive (0 km/h)
        BrakeEvaluator.Snapshot s5 = candidate.evaluate(
                cfg,
                new BrakeEvaluator.Input(
                        3000L, 0.0f, 0.0f, 0.0f, 0.0f,
                        Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 75.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, 3000L),
                emptyRaw);
        if (s5 == null || Float.isNaN(s5.decelMps2)) {
            throw new IllegalStateException("Self-test tick 5 (standstill) failed");
        }

        // 6. Null-input & null-config resilience
        BrakeEvaluator.Snapshot s6 = candidate.evaluate(null, null, null);
        if (s6 == null) {
            throw new IllegalStateException("Self-test tick 6 (null resilience) returned null");
        }
    }

    private void quarantineAndFallback(
            BrakeModel faulty,
            Throwable fault,
            TelemetryRecorder recorder,
            long nowMonoMs) {
        synchronized (swapLock) {
            if (activeModel != faulty) {
                return;
            }
            long badSeq = faulty.modelSeq();
            String badVer = faulty.modelVersion();
            quarantinedSeqs.add(Long.valueOf(badSeq));
            builtInModel.reset();
            activeModel = builtInModel;
            activeIsOta = false;
            activeSha256Short = "";
            safeDeleteFile(new File(modelsDir, USB_MODEL_PREFIX + badSeq + ".jar"));
            deleteDirRecursive(dexOptDirForSeq(badSeq));
            syncRecorderAttribution(recorder);
            if (recorder != null) {
                recorder.recordExternalEvent(
                        nowMonoMs,
                        "OTA model " + badVer + " fault (" + fault.getClass().getSimpleName()
                                + ") -> reverted immediately to built-in " + builtInModel.modelVersion());
            }
        }
    }

    private void syncRecorderAttribution(TelemetryRecorder recorder) {
        if (recorder != null) {
            recorder.setActiveModelInfo(getActiveModelSummaryLine(), getActiveModelCommitTag());
        }
    }

    private File dexOptDirForSeq(long seq) {
        return new File(dexOptRootDir, "m_" + seq);
    }

    private void pruneOtherModelsLocked(long keepSeq) {
        File[] jars = modelsDir.listFiles();
        if (jars != null) {
            String keepName = keepSeq > 0L ? (USB_MODEL_PREFIX + keepSeq + ".jar") : "";
            for (int i = 0; i < jars.length; i++) {
                if (!jars[i].getName().equals(keepName)) {
                    safeDeleteFile(jars[i]);
                }
            }
        }
        File[] optDirs = dexOptRootDir.listFiles();
        if (optDirs != null) {
            String keepOpt = keepSeq > 0L ? ("m_" + keepSeq) : "";
            for (int i = 0; i < optDirs.length; i++) {
                if (!optDirs[i].getName().equals(keepOpt)) {
                    deleteDirRecursive(optDirs[i]);
                }
            }
        }
    }

    private static boolean isSafeRepoJarPath(String path) {
        if (path == null || !path.startsWith("models/brake-model-") || !path.endsWith(".jar")) {
            return false;
        }
        if (path.contains("..") || path.contains("?") || path.contains("#") || path.contains("\\")) {
            return false;
        }
        String mid = path.substring("models/brake-model-".length(), path.length() - 4);
        return ModelBundleVerifier.isSafeModelVersion(mid);
    }

    private static String shortSha(String hex) {
        if (hex == null) return "";
        return hex.length() <= 12 ? hex : hex.substring(0, 12);
    }

    private static void safeDeleteFile(File f) {
        if (f != null && f.exists()) {
            try {
                f.setWritable(true);
            } catch (Throwable ignored) {}
            f.delete();
        }
    }

    private static void deleteDirRecursive(File f) {
        if (f == null || !f.exists()) {
            return;
        }
        File[] children = f.listFiles();
        if (children != null) {
            for (int i = 0; i < children.length; i++) {
                deleteDirRecursive(children[i]);
            }
        }
        safeDeleteFile(f);
    }
}
