package dev.lemoyne.brakeindicator;

import android.content.Context;
import android.content.SharedPreferences;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;

import hmgapi.client.HmgApi;

import org.json.JSONObject;

import java.io.File;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * In-process vehicle & OBD-II evaluation engine shared by {@link OverlayService}
 * and {@link MainActivity}.
 *
 * Polls HMG-API's built-in `Vehicle` readings at 20 Hz (50 ms) (or 650 ms on legacy
 * KitKat `Hub`), aligns `-dv/dt` and `L0..i-Pedal` detection to exact HMG-API poll
 * timestamps, fuses or compares them against {@link ObdBrakeClient} (`@hmgapi-ble`
 * central + `127.0.0.1:35000` bridge coexistence), records a 30-minute synchronized CSV
 * dataset in RAM, and uploads compressed `.tar.gz` logs to GitHub over LTE/Wi-Fi (with
 * in-RAM offline queue and removable USB fallback).
 */
public final class BrakeEngine {

    public interface StateListener {
        void onEngineState(IndicatorConfig config, BrakeEvaluator.Snapshot snapshot, boolean hmgConnected);
    }

    private static final class HmgPollSnapshot {
        final Map<String, String> readings;
        final long monoMs;
        final boolean legacyStall;

        HmgPollSnapshot(Map<String, String> readings, long monoMs, boolean legacyStall) {
            this.readings = readings == null ? Collections.<String, String>emptyMap() : readings;
            this.monoMs = monoMs;
            this.legacyStall = legacyStall;
        }
    }

    private static final String PREFS_NAME = "brake_obd_pairing";
    private static final String KEY_OBD_MAC = "obd_mac";
    private static final String KEY_OBD_NAME = "obd_name";
    private static final String KEY_OBD_ADDR_TYPE = "obd_addr_type";

    private static final String CLOUD_PREFS_NAME = "brake_cloud_upload";
    private static final String KEY_CLOUD_ENABLED = "enabled";
    private static final String KEY_CLOUD_REPO = "repo";
    private static final String KEY_CLOUD_BRANCH = "branch";
    private static final String KEY_CLOUD_TOKEN = "token";
    private static final String KEY_CLOUD_ALLOW_LTE = "allow_lte";
    private static final String KEY_CLOUD_USB_TOO = "usb_backup_too";
    private static final String KEY_CLOUD_DAILY_KB = "daily_limit_kb";
    private static final String KEY_CLOUD_API_HOST = "api_host";
    private static final String KEY_CLOUD_API_PORT = "api_port";

    private static final String[] VEHICLE_READINGS = {
        "power",
        "brake.pressure",
        "speed",
        "pedal.accel",
        "gear.park",
        "parking_brake",
        "moving",
        "steering.angle",
        "hv.soc"
    };

    private static final long EVAL_INTERVAL_MS = 50L;
    private static final long LEGACY_HUB_POLL_MS = 650L;
    private static final long STANDBY_IDLE_POLL_MS = 1000L;
    private static final long HMG_RETRY_MS = 250L;
    private static final long HMG_FRESH_MAX_MS = 1800L;
    private static final long STANDBY_HMG_FRESH_MAX_MS = 2800L;
    private static final long SETTINGS_REFRESH_MS = 3000L;

    private static BrakeEngine sInstance;

    public static synchronized BrakeEngine getInstance(Context ctx) {
        if (sInstance == null) {
            sInstance = new BrakeEngine(ctx.getApplicationContext());
        }
        return sInstance;
    }

    private final Context appContext;
    private final TelemetryRecorder recorder = new TelemetryRecorder();
    private final AutoExportCoordinator autoExport = new AutoExportCoordinator();
    private final HevRegenProbe hevProbe = new HevRegenProbe();
    private final GitHubUploader gitHubUploader = new GitHubUploader();
    private final ModelManager modelManager;
    private final CloudUploadConfig.Store cloudStore;
    private final ObdBrakeClient obdClient;
    private final List<StateListener> listeners = new CopyOnWriteArrayList<StateListener>();
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicInteger markSeq = new AtomicInteger(0);
    private final Object exportLock = new Object();
    private final AtomicBoolean exportInFlight = new AtomicBoolean(false);
    private final AtomicBoolean autoExportQueued = new AtomicBoolean(false);
    private final AtomicBoolean queueDrainInFlight = new AtomicBoolean(false);
    private final AtomicBoolean bootModelCheckInFlight = new AtomicBoolean(false);
    private volatile long nextBootModelCheckMonoMs;
    private final ExecutorService autoExportExecutor = Executors.newSingleThreadExecutor(new ThreadFactory() {
        @Override
        public Thread newThread(Runnable r) {
            Thread t = new Thread(r, "BrakeAutoExport");
            t.setDaemon(true);
            return t;
        }
    });

    private volatile IndicatorConfig config = IndicatorConfig.defaults();
    private volatile BrakeEvaluator.Snapshot latestSnapshot;
    private volatile HmgPollSnapshot latestHmgPoll =
            new HmgPollSnapshot(Collections.<String, String>emptyMap(), -1L, false);
    private volatile boolean hmgConnected;
    private volatile long testUntilMs;
    private volatile boolean forceSettingsReload = true;

    private BrakeEngine(final Context appContext) {
        this.appContext = appContext;
        final SharedPreferences prefs = appContext.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        ObdBrakeClient.PairingStore store = new ObdBrakeClient.PairingStore() {
            @Override
            public String getMac() {
                return prefs.getString(KEY_OBD_MAC, "");
            }

            @Override
            public String getName() {
                return prefs.getString(KEY_OBD_NAME, "");
            }

            @Override
            public int getAddrType() {
                return prefs.getInt(KEY_OBD_ADDR_TYPE, -1);
            }

            @Override
            public void save(String mac, String name, int addrType) {
                // Written ONLY on explicit user Pair tap (never in a background loop)
                prefs.edit()
                        .putString(KEY_OBD_MAC, mac)
                        .putString(KEY_OBD_NAME, name)
                        .putInt(KEY_OBD_ADDR_TYPE, addrType)
                        .commit();
            }

            @Override
            public void clear() {
                prefs.edit()
                        .remove(KEY_OBD_MAC)
                        .remove(KEY_OBD_NAME)
                        .remove(KEY_OBD_ADDR_TYPE)
                        .commit();
            }
        };
        final SharedPreferences cloudPrefs =
                appContext.getSharedPreferences(CLOUD_PREFS_NAME, Context.MODE_PRIVATE);
        this.cloudStore = new CloudUploadConfig.Store() {
            private volatile CloudUploadConfig cached = loadFromPrefs(cloudPrefs);

            @Override
            public CloudUploadConfig load() {
                return cached;
            }

            @Override
            public void save(CloudUploadConfig cfg) {
                // Written ONLY on one-time USB cloud-upload.json import (zero background flash writes)
                CloudUploadConfig safe = cfg == null ? CloudUploadConfig.unconfigured() : cfg;
                cloudPrefs.edit()
                        .putBoolean(KEY_CLOUD_ENABLED, safe.enabled)
                        .putString(KEY_CLOUD_REPO, safe.repo)
                        .putString(KEY_CLOUD_BRANCH, safe.branch)
                        .putString(KEY_CLOUD_TOKEN, safe.token)
                        .putBoolean(KEY_CLOUD_ALLOW_LTE, safe.allowLte)
                        .putBoolean(KEY_CLOUD_USB_TOO, safe.usbBackupToo)
                        .putInt(KEY_CLOUD_DAILY_KB, safe.dailyLimitKb)
                        .putString(KEY_CLOUD_API_HOST, safe.apiHost)
                        .putInt(KEY_CLOUD_API_PORT, safe.apiPort)
                        .commit();
                cached = safe;
                autoExport.setCloudConfiguredState(safe.hasCredentials());
            }

            @Override
            public void clear() {
                cloudPrefs.edit().clear().commit();
                cached = CloudUploadConfig.unconfigured();
                autoExport.setCloudConfiguredState(false);
            }
        };
        this.autoExport.setCloudConfiguredState(this.cloudStore.load().hasCredentials());
        this.obdClient = new ObdBrakeClient(store);
        this.obdClient.setDiagnosticLogger(new ObdBrakeClient.DiagnosticLogger() {
            @Override
            public void onDiagnostic(long monoMs, String message) {
                recorder.recordExternalEvent(monoMs, message);
            }
        });
        this.modelManager = new ModelManager(appContext.getFilesDir());
        long initMono = this.obdClient.monotonicMs();
        this.nextBootModelCheckMonoMs = initMono + 4000L;
        this.modelManager.initFromDisk(this.recorder, initMono);
    }

    private static CloudUploadConfig loadFromPrefs(SharedPreferences p) {
        return new CloudUploadConfig(
                p.getBoolean(KEY_CLOUD_ENABLED, true),
                p.getString(KEY_CLOUD_REPO, CloudUploadConfig.DEFAULT_REPO),
                p.getString(KEY_CLOUD_BRANCH, CloudUploadConfig.DEFAULT_BRANCH),
                p.getString(KEY_CLOUD_TOKEN, ""),
                p.getBoolean(KEY_CLOUD_ALLOW_LTE, true),
                p.getBoolean(KEY_CLOUD_USB_TOO, false),
                p.getInt(KEY_CLOUD_DAILY_KB, CloudUploadConfig.DEFAULT_DAILY_LIMIT_KB),
                p.getString(KEY_CLOUD_API_HOST, CloudUploadConfig.DEFAULT_API_HOST),
                p.getInt(KEY_CLOUD_API_PORT, CloudUploadConfig.DEFAULT_API_PORT));
    }

    public void ensureStarted() {
        if (!running.compareAndSet(false, true)) {
            return;
        }
        Thread hmgThread = new Thread("BrakeHmgPoll") {
            @Override
            public void run() {
                hmgPollLoop();
            }
        };
        hmgThread.setDaemon(true);
        hmgThread.start();

        Thread evalThread = new Thread("BrakeEvalLoop") {
            @Override
            public void run() {
                evalLoop();
            }
        };
        evalThread.setDaemon(true);
        evalThread.start();
    }

    public void addListener(StateListener l) {
        if (l != null && !listeners.contains(l)) {
            listeners.add(l);
            BrakeEvaluator.Snapshot snap = latestSnapshot;
            if (snap != null) {
                try {
                    l.onEngineState(config, snap, hmgConnected);
                } catch (Throwable ignored) {}
            }
        }
    }

    public void removeListener(StateListener l) {
        listeners.remove(l);
    }

    public IndicatorConfig getConfig() {
        return config;
    }

    public BrakeEvaluator.Snapshot getLatestSnapshot() {
        return latestSnapshot;
    }

    public TelemetryRecorder getRecorder() {
        return recorder;
    }

    public ObdBrakeClient getObdClient() {
        return obdClient;
    }

    public HevRegenProbe getHevProbe() {
        return hevProbe;
    }

    public AutoExportCoordinator.StatusSnapshot getAutoExportStatus() {
        return autoExport.statusSnapshot();
    }

    public CloudUploadConfig getCloudUploadConfig() {
        return cloudStore.load();
    }

    public ModelManager getModelManager() {
        return modelManager;
    }

    public String getActiveModelBadge() {
        return modelManager.getActiveModelBadge();
    }

    public ModelManager.UpdateOutcome updateModelNow() {
        return modelManager.checkForUpdate(
                true,
                config,
                detectActiveTransport(),
                findUsbRoots(),
                recorder,
                obdClient.monotonicMs());
    }

    public ModelManager.UpdateOutcome resetToBuiltInModel() {
        return modelManager.resetToBuiltIn(recorder, obdClient.monotonicMs());
    }

    public boolean isExportInFlight() {
        return exportInFlight.get();
    }

    public void clearDataset() {
        BrakeEvaluator.Snapshot snap = latestSnapshot;
        boolean outOfPark = snap != null
                && AutoExportCoordinator.isOutOfParkState(snap.gearPark, snap.speedKmh, snap.moving);
        recorder.clear();
        autoExport.onManualClear(outOfPark);
    }

    public String getHmgStatusSummary() {
        if (!hmgConnected) {
            return "WAITING";
        }
        return latestHmgPoll.legacyStall ? "OK (legacy Hub - update HMG-API)" : "OK (20Hz)";
    }

    public String markDriveEvent() {
        long nowMono = obdClient.monotonicMs();
        int n = markSeq.incrementAndGet();
        BrakeEvaluator.Snapshot snap = latestSnapshot;
        TelemetryRecorder.DriverGroundTruth gt = recorder.getDriverGroundTruth(nowMono);
        String reg = snap != null
                ? BrakeEvaluator.regenLevelLabel(snap.inferredRegenLevel, snap.sccActive, snap.autoRegenActive)
                : "?";
        String spd = (snap != null && snap.speedKmh != null)
                ? String.format(java.util.Locale.US, "%.1f km/h", snap.speedKmh)
                : "- km/h";
        String gtPart = gt.hasRegenConfirmation() ? (", gt=" + gt.formatRegenLabel()) : "";
        String msg = "MARK #" + n + " (regen=" + reg + gtPart + ", speed=" + spd + ")";
        recorder.recordExternalEvent(nowMono, msg);
        return msg;
    }

    public String markRegenBaseline() {
        return markDriveEvent();
    }

    public void setUiFocused(boolean focused) {
        recorder.setUiFocused(focused, obdClient.monotonicMs());
    }

    public TelemetryRecorder.DriverGroundTruth confirmDriverRegenLevel(int level) {
        return recorder.confirmDriverRegenLevel(level, obdClient.monotonicMs(), latestSnapshot);
    }

    public TelemetryRecorder.DriverGroundTruth toggleDriverAutoRegen() {
        return recorder.toggleDriverAutoRegen(obdClient.monotonicMs(), latestSnapshot);
    }

    public TelemetryRecorder.DriverGroundTruth toggleDriverScc() {
        return recorder.toggleDriverScc(obdClient.monotonicMs(), latestSnapshot);
    }

    public TelemetryRecorder.DriverGroundTruth setDriverAutoHold(int state) {
        return recorder.setDriverAutoHold(state, obdClient.monotonicMs());
    }

    public TelemetryRecorder.DriverGroundTruth clearDriverGroundTruth() {
        return recorder.clearDriverGroundTruth(obdClient.monotonicMs());
    }

    public TelemetryRecorder.DriverGroundTruth getDriverGroundTruth() {
        return recorder.getDriverGroundTruth(obdClient.monotonicMs());
    }

    public void triggerManualTest(long durationMs) {
        long clamped = Math.max(500L, Math.min(15000L, durationMs));
        testUntilMs = obdClient.monotonicMs() + clamped;
    }

    public void requestSettingsReload() {
        forceSettingsReload = true;
    }

    /**
     * Cycles `obd_mode` on explicit user tap and persists it to HMG-API's mod settings
     * so `MainActivity` and `HMG-API -> Mods -> Brake Light Indicator` stay in sync.
     */
    public IndicatorConfig.ObdMode cycleObdModeAndSave() {
        final IndicatorConfig.ObdMode next = config.obdMode.next();
        Map<String, String> updated = new LinkedHashMap<String, String>();
        try {
            HmgApi api = HmgApi.connect().timeout(2000);
            JSONObject json = api.settings();
            json.put("obd_mode", next.wireValue);
            api.saveSettings(json);
            updated = jsonToMap(json);
        } catch (Exception e) {
            // Apply in-memory even if hmgapid settings save fails off-car
            updated.put("obd_mode", next.wireValue);
            updated.put("signal_mode", config.signalMode == IndicatorConfig.SignalMode.PRESSURE_ONLY
                    ? "pressure_only" : "pressure_and_decel");
            updated.put("off_state", config.offState == IndicatorConfig.OffState.DIM ? "dim" : "hidden");
            updated.put("anchor", config.anchor.name().toLowerCase(java.util.Locale.US));
            updated.put("x_offset", Integer.toString(config.xOffsetPx));
            updated.put("y_offset", Integer.toString(config.yOffsetPx));
            updated.put("icon_size", Integer.toString(config.iconSizePx));
            updated.put("pressure_threshold", Integer.toString(Math.round(config.pressureThreshold * 10.0f)));
            updated.put("decel_threshold", Integer.toString(Math.round(config.decelThresholdMps2 * 10.0f)));
            updated.put("standstill_hold", Boolean.toString(config.standstillHold));
            updated.put("bridge_port", Integer.toString(config.bridgePort));
            updated.put("auto_export_usb", Boolean.toString(config.autoExportUsb));
            updated.put("auto_upload_cloud", Boolean.toString(config.autoUploadCloud));
            updated.put("cloud_allow_lte", Boolean.toString(config.cloudAllowLte));
            updated.put("usb_backup_when_cloud", Boolean.toString(config.usbBackupWhenCloud));
            updated.put("auto_update_model", Boolean.toString(config.autoUpdateModel));
        }
        IndicatorConfig newCfg = IndicatorConfig.fromMap(updated);
        config = newCfg;
        obdClient.applyConfig(newCfg);
        return next;
    }

    public AutoExportCoordinator.ActionOutcome uploadDatasetToCloud() {
        return performCloudOrUsbExport(AutoExportCoordinator.Trigger.MANUAL, true, false);
    }

    public UsbExporter.ExportResult exportDatasetToUsb() {
        synchronized (exportLock) {
            exportInFlight.set(true);
            try {
                List<String> roots = findUsbRoots();
                if (!roots.isEmpty() && !cloudStore.load().hasCredentials()) {
                    CloudUploadConfig.UsbImportResult imp =
                            CloudUploadConfig.tryImportFromUsbRoots(roots, cloudStore);
                    if (imp.imported) {
                        recorder.recordExternalEvent(obdClient.monotonicMs(), imp.message);
                    }
                }
                // If active buffer was already packaged into PendingUploadQueue offline, unpack to USB
                if (recorder.bufferedRows() <= 0 && !autoExport.getPendingQueue().isEmpty() && !roots.isEmpty()) {
                    AutoExportCoordinator.ActionOutcome drained = autoExport.drainPendingQueue(
                            false,
                            recorder,
                            config,
                            CloudUploadConfig.unconfigured(),
                            roots,
                            AutoExportCoordinator.DEFAULT_USB_WRITER,
                            new AutoExportCoordinator.NetworkCheck() {
                                @Override
                                public AutoExportCoordinator.TransportInfo activeTransport() {
                                    return new AutoExportCoordinator.TransportInfo(
                                            AutoExportCoordinator.TransportKind.NONE, "Offline");
                                }
                            },
                            cloudUploaderBridge(),
                            obdClient.monotonicMs(),
                            System.currentTimeMillis());
                    if (drained.wroteToUsb) {
                        return new UsbExporter.ExportResult(true, drained.primaryTarget, null, null);
                    }
                }
                return performDirectUsbExport(AutoExportCoordinator.Trigger.MANUAL, roots);
            } finally {
                exportInFlight.set(false);
            }
        }
    }

    /**
     * Imports {@code <usb>/BrakeIndicator/cloud-upload.json} if attached, runs the 4-stage
     * DNS -> TCP :443 -> TLS 1.2 -> GitHub REST API diagnostic probe, and drains any queued
     * drives if connectivity is healthy.
     */
    public String testCloudAndImportFromUsb() {
        synchronized (exportLock) {
            exportInFlight.set(true);
            try {
                String importPrefix = "";
                List<String> roots = findUsbRoots();
                if (!roots.isEmpty()) {
                    CloudUploadConfig.UsbImportResult imp =
                            CloudUploadConfig.tryImportFromUsbRoots(roots, cloudStore);
                    if (imp.imported) {
                        importPrefix = imp.message + " ";
                        recorder.recordExternalEvent(obdClient.monotonicMs(), imp.message);
                    } else if (imp.foundFile && !cloudStore.load().hasCredentials()) {
                        importPrefix = imp.message + " ";
                    }
                }

                CloudUploadConfig cloudCfg = cloudStore.load();
                autoExport.setCloudConfiguredState(cloudCfg.hasCredentials());
                AutoExportCoordinator.TransportInfo net = detectActiveTransport();
                String probeMsg = gitHubUploader.probeGitHubConnectivity(cloudCfg, net.label);
                String combined = importPrefix + probeMsg;
                recorder.recordExternalEvent(obdClient.monotonicMs(), "Cloud Probe: " + probeMsg);

                if (cloudCfg.hasCredentials() && !autoExport.getPendingQueue().isEmpty()) {
                    autoExport.requestImmediateQueueDrain();
                    AutoExportCoordinator.ActionOutcome drainRes = autoExport.drainPendingQueue(
                            true,
                            recorder,
                            config,
                            cloudCfg,
                            roots,
                            AutoExportCoordinator.DEFAULT_USB_WRITER,
                            networkCheckBridge(),
                            cloudUploaderBridge(),
                            obdClient.monotonicMs(),
                            System.currentTimeMillis());
                    if (drainRes.uploadedToCloud && drainRes.message != null) {
                        combined = combined + " · " + drainRes.message;
                    }
                }
                return combined;
            } finally {
                exportInFlight.set(false);
            }
        }
    }

    public void drainPendingCloudUploadsAsync() {
        if (autoExport.getPendingQueue().isEmpty()) {
            return;
        }
        autoExport.requestImmediateQueueDrain();
        if (exportInFlight.get() || !queueDrainInFlight.compareAndSet(false, true)) {
            return;
        }
        autoExportExecutor.execute(new Runnable() {
            @Override
            public void run() {
                try {
                    performPendingQueueDrain(true);
                } catch (Throwable ignored) {
                } finally {
                    queueDrainInFlight.set(false);
                }
            }
        });
    }

    public AutoExportCoordinator.TransportInfo detectActiveTransport() {
        try {
            ConnectivityManager cm =
                    (ConnectivityManager) appContext.getSystemService(Context.CONNECTIVITY_SERVICE);
            if (cm != null) {
                NetworkInfo info = cm.getActiveNetworkInfo();
                if (info != null && info.isConnected()) {
                    if (info.getType() == ConnectivityManager.TYPE_WIFI) {
                        return new AutoExportCoordinator.TransportInfo(
                                AutoExportCoordinator.TransportKind.WIFI, "Wi-Fi");
                    }
                    String tName = info.getTypeName();
                    return new AutoExportCoordinator.TransportInfo(
                            AutoExportCoordinator.TransportKind.CELLULAR,
                            (tName == null || tName.isEmpty()) ? "LTE" : ("LTE (" + tName + ")"));
                }
            }
        } catch (Throwable ignored) {
        }
        return AutoExportCoordinator.readKernelDefaultRoute();
    }

    private List<String> findUsbRoots() {
        List<String> extraFolders = null;
        try {
            extraFolders = HmgApi.connect().timeout(800).usbFolders();
        } catch (Exception ignored) {
        }
        return UsbExporter.findMountedUsbRoots(extraFolders);
    }

    private AutoExportCoordinator.UsbRootLocator usbRootLocatorBridge() {
        return new AutoExportCoordinator.UsbRootLocator() {
            @Override
            public List<String> findMountedUsbRoots() {
                return findUsbRoots();
            }
        };
    }

    private AutoExportCoordinator.OutOfParkCheck liveDriveCheckBridge() {
        return new AutoExportCoordinator.OutOfParkCheck() {
            @Override
            public boolean isCurrentlyOutOfPark() {
                BrakeEvaluator.Snapshot s = latestSnapshot;
                return s != null && AutoExportCoordinator.isOutOfParkState(
                        s.gearPark, s.speedKmh, s.moving);
            }
        };
    }

    private AutoExportCoordinator.NetworkCheck networkCheckBridge() {
        return new AutoExportCoordinator.NetworkCheck() {
            @Override
            public AutoExportCoordinator.TransportInfo activeTransport() {
                return detectActiveTransport();
            }
        };
    }

    private AutoExportCoordinator.CloudUploader cloudUploaderBridge() {
        return new AutoExportCoordinator.CloudUploader() {
            @Override
            public GitHubUploader.UploadResult upload(
                    CloudUploadConfig cfg, PendingUploadQueue.QueuedDrive drive) {
                return gitHubUploader.uploadDrive(cfg, drive);
            }
        };
    }

    private UsbExporter.ExportResult performDirectUsbExport(
            AutoExportCoordinator.Trigger trigger,
            final List<String> prelocatedRoots) {
        AutoExportCoordinator.UsbRootLocator locator = prelocatedRoots != null
                ? new AutoExportCoordinator.UsbRootLocator() {
                    @Override
                    public List<String> findMountedUsbRoots() {
                        return prelocatedRoots;
                    }
                }
                : usbRootLocatorBridge();
        return autoExport.executeExport(
                trigger,
                recorder,
                config,
                obdClient.getStatusSummary(),
                getHmgStatusSummary(),
                locator,
                AutoExportCoordinator.DEFAULT_USB_WRITER,
                liveDriveCheckBridge(),
                obdClient.monotonicMs());
    }

    private AutoExportCoordinator.ActionOutcome performCloudOrUsbExport(
            AutoExportCoordinator.Trigger trigger,
            boolean forceCloudOnly,
            boolean notifyToastOnSuccess) {
        synchronized (exportLock) {
            exportInFlight.set(true);
            try {
                AutoExportCoordinator.ActionOutcome out = autoExport.executeCloudOrUsb(
                        trigger,
                        forceCloudOnly,
                        recorder,
                        config,
                        cloudStore,
                        obdClient.getStatusSummary(),
                        getHmgStatusSummary(),
                        usbRootLocatorBridge(),
                        AutoExportCoordinator.DEFAULT_USB_WRITER,
                        networkCheckBridge(),
                        cloudUploaderBridge(),
                        liveDriveCheckBridge(),
                        obdClient.monotonicMs(),
                        System.currentTimeMillis());
                if (notifyToastOnSuccess && out.ok) {
                    notifyAutoToast(out);
                }
                if (!forceCloudOnly
                        && trigger == AutoExportCoordinator.Trigger.SHIFT_TO_PARK
                        && config.autoUpdateModel
                        && !liveDriveCheckBridge().isCurrentlyOutOfPark()) {
                    performParkedAutoModelCheck(notifyToastOnSuccess);
                }
                return out;
            } finally {
                exportInFlight.set(false);
            }
        }
    }

    private void performParkedAutoModelCheck(boolean notifyToastOnUpdate) {
        BrakeEvaluator.Snapshot cur = latestSnapshot;
        if (cur != null && AutoExportCoordinator.isOutOfParkState(cur.gearPark, cur.speedKmh, cur.moving)) {
            return;
        }
        AutoExportCoordinator.TransportInfo net = detectActiveTransport();
        if (net.kind == AutoExportCoordinator.TransportKind.NONE) {
            return;
        }
        modelManager.markBootParkedCheckDone();
        ModelManager.UpdateOutcome mOut = modelManager.checkForUpdate(
                false,
                config,
                net,
                null,
                recorder,
                obdClient.monotonicMs());
        if (mOut.updated && notifyToastOnUpdate) {
            try {
                HmgApi.toast(appContext, mOut.message);
            } catch (Throwable ignored) {}
        }
    }

    private void performPendingQueueDrain(boolean notifyToastOnSuccess) {
        synchronized (exportLock) {
            if (autoExport.getPendingQueue().isEmpty()) {
                return;
            }
            exportInFlight.set(true);
            try {
                AutoExportCoordinator.ActionOutcome out = autoExport.drainPendingQueue(
                        false,
                        recorder,
                        config,
                        cloudStore.load(),
                        findUsbRoots(),
                        AutoExportCoordinator.DEFAULT_USB_WRITER,
                        networkCheckBridge(),
                        cloudUploaderBridge(),
                        obdClient.monotonicMs(),
                        System.currentTimeMillis());
                if (notifyToastOnSuccess && out.ok) {
                    notifyAutoToast(out);
                }
            } finally {
                exportInFlight.set(false);
            }
        }
    }

    private void notifyAutoToast(AutoExportCoordinator.ActionOutcome out) {
        try {
            if (out.uploadedToCloud && out.primaryTarget != null) {
                String name = out.primaryTarget.substring(out.primaryTarget.lastIndexOf('/') + 1);
                HmgApi.toast(appContext, "Uploaded " + name);
            } else if (out.wroteToUsb && out.primaryTarget != null) {
                HmgApi.toast(appContext, "Auto-saved " + new File(out.primaryTarget).getName());
            }
        } catch (Throwable ignored) {
        }
    }

    private void hmgPollLoop() {
        HmgApi settingsApi = HmgApi.connect().timeout(1500);
        long lastSettingsMs = 0L;

        while (running.get()) {
            long startMono = obdClient.monotonicMs();
            long targetPeriodMs = HMG_RETRY_MS;
            try {
                if (forceSettingsReload
                        || (startMono - lastSettingsMs) >= SETTINGS_REFRESH_MS
                        || lastSettingsMs == 0L) {
                    reloadSettings(settingsApi);
                    forceSettingsReload = false;
                    lastSettingsMs = startMono;
                }

                HmgVehiclePoller.PollResult res = HmgVehiclePoller.readVehicle(VEHICLE_READINGS, 900, 90);
                if (res.ok && !res.readings.isEmpty()) {
                    long pollAt = obdClient.monotonicMs();
                    latestHmgPoll = new HmgPollSnapshot(res.readings, pollAt, res.legacyHubStall);
                    boolean powerOn = HmgVehiclePoller.isVehiclePowerActive(res.readings.get("power"));
                    obdClient.setVehiclePowerOn(powerOn);
                    if (!powerOn) {
                        targetPeriodMs = STANDBY_IDLE_POLL_MS;
                    } else {
                        targetPeriodMs = res.legacyHubStall ? LEGACY_HUB_POLL_MS : EVAL_INTERVAL_MS;
                    }
                } else {
                    targetPeriodMs = HMG_RETRY_MS;
                }
            } catch (Throwable ignored) {
            }

            long spent = obdClient.monotonicMs() - startMono;
            long sleep = targetPeriodMs - spent;
            if (sleep > 2L) {
                try {
                    Thread.sleep(sleep);
                } catch (InterruptedException ie) {
                    break;
                }
            }
        }
    }

    private void evalLoop() {
        boolean sawOutOfParkForModel = false;
        while (running.get()) {
            long tickStartMono = obdClient.monotonicMs();
            boolean powerOn = true;
            try {
                HmgPollSnapshot hmgSnap = latestHmgPoll;
                long hmgAt = hmgSnap.monoMs;
                boolean snapPowerOn = HmgVehiclePoller.isVehiclePowerActive(hmgSnap.readings.get("power"));
                long freshMaxMs = snapPowerOn ? HMG_FRESH_MAX_MS : STANDBY_HMG_FRESH_MAX_MS;
                boolean okHmg = hmgAt > 0L
                        && (tickStartMono - hmgAt) >= 0L
                        && (tickStartMono - hmgAt) <= freshMaxMs
                        && !hmgSnap.readings.isEmpty();
                hmgConnected = okHmg;
                Map<String, String> readings = okHmg
                        ? hmgSnap.readings
                        : Collections.<String, String>emptyMap();
                powerOn = !okHmg || HmgVehiclePoller.isVehiclePowerActive(readings.get("power"));
                long hmgSampleElapsedMs = okHmg ? hmgAt : -1L;

                IndicatorConfig cfg = config;
                BrakeEvaluator.ObdSample obd = obdClient.currentSample();

                BrakeEvaluator.Input input = new BrakeEvaluator.Input(
                        tickStartMono,
                        BrakeEvaluator.parseReadingFloat(readings.get("brake.pressure")),
                        BrakeEvaluator.parseReadingFloat(readings.get("speed")),
                        BrakeEvaluator.parseReadingFloat(readings.get("pedal.accel")),
                        BrakeEvaluator.parseReadingFloat(readings.get("steering.angle")),
                        BrakeEvaluator.parseReadingFlag(readings.get("gear.park")),
                        BrakeEvaluator.parseReadingFlag(readings.get("parking_brake")),
                        BrakeEvaluator.parseReadingFlag(readings.get("moving")),
                        BrakeEvaluator.parseReadingFloat(readings.get("hv.soc")),
                        obd,
                        testUntilMs,
                        -1,
                        hmgSampleElapsedMs);

                BrakeEvaluator.Snapshot snap =
                        modelManager.evaluateTick(cfg, input, readings, recorder, tickStartMono);
                latestSnapshot = snap;
                if (powerOn || Boolean.FALSE.equals(snap.gearPark)) {
                    recorder.record(snap, System.currentTimeMillis());
                }

                if (Boolean.FALSE.equals(snap.gearPark)) {
                    sawOutOfParkForModel = true;
                }
                boolean parkedNearRest = Boolean.TRUE.equals(snap.gearPark)
                        && (snap.speedKmh == null || snap.speedKmh.floatValue() < 3.0f);

                boolean anyAutoEnabled = cfg.autoExportUsb
                        || cloudStore.load().isReadyForAutoUpload(cfg);
                final AutoExportCoordinator.Trigger dueTrigger = autoExport.pollDueTrigger(
                        anyAutoEnabled,
                        snap.gearPark,
                        snap.speedKmh,
                        snap.moving,
                        recorder.bufferedRows(),
                        tickStartMono);
                if (dueTrigger != AutoExportCoordinator.Trigger.NONE
                        && !exportInFlight.get()
                        && autoExportQueued.compareAndSet(false, true)) {
                    if (dueTrigger == AutoExportCoordinator.Trigger.SHIFT_TO_PARK) {
                        sawOutOfParkForModel = false;
                    }
                    autoExportExecutor.execute(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                performCloudOrUsbExport(dueTrigger, false, true);
                            } catch (Throwable ignored) {
                            } finally {
                                autoExportQueued.set(false);
                            }
                        }
                    });
                } else if (autoExport.pollQueueDrainDue(tickStartMono)
                        && !exportInFlight.get()
                        && queueDrainInFlight.compareAndSet(false, true)) {
                    autoExportExecutor.execute(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                performPendingQueueDrain(true);
                            } catch (Throwable ignored) {
                            } finally {
                                queueDrainInFlight.set(false);
                            }
                        }
                    });
                } else if (parkedNearRest
                        && cfg.autoUpdateModel
                        && (sawOutOfParkForModel
                                || (modelManager.shouldRunBootParkedCheck()
                                        && tickStartMono >= nextBootModelCheckMonoMs))
                        && !exportInFlight.get()
                        && bootModelCheckInFlight.compareAndSet(false, true)) {
                    sawOutOfParkForModel = false;
                    nextBootModelCheckMonoMs = tickStartMono + 15000L;
                    autoExportExecutor.execute(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                performParkedAutoModelCheck(true);
                            } catch (Throwable ignored) {
                            } finally {
                                bootModelCheckInFlight.set(false);
                            }
                        }
                    });
                }

                for (int i = 0; i < listeners.size(); i++) {
                    try {
                        listeners.get(i).onEngineState(cfg, snap, okHmg);
                    } catch (Throwable ignored) {}
                }
            } catch (Throwable ignored) {
            }

            long spent = obdClient.monotonicMs() - tickStartMono;
            long targetSleep = (powerOn ? EVAL_INTERVAL_MS : STANDBY_IDLE_POLL_MS) - spent;
            if (targetSleep > 2L) {
                try {
                    Thread.sleep(targetSleep);
                } catch (InterruptedException ie) {
                    break;
                }
            }
        }
    }

    private void reloadSettings(HmgApi api) {
        try {
            JSONObject json = api.settings();
            IndicatorConfig loaded = IndicatorConfig.fromMap(jsonToMap(json));
            config = loaded;
            obdClient.applyConfig(loaded);
        } catch (Exception ignored) {
            // Keep active configuration when hmgapid is temporarily unreachable
        }
    }

    private static Map<String, String> jsonToMap(JSONObject json) {
        Map<String, String> map = new LinkedHashMap<String, String>();
        if (json == null) return map;
        Iterator<?> keys = json.keys();
        while (keys.hasNext()) {
            String k = String.valueOf(keys.next());
            Object v = json.opt(k);
            if (v != null && v != JSONObject.NULL) {
                map.put(k, String.valueOf(v));
            }
        }
        return map;
    }
}
