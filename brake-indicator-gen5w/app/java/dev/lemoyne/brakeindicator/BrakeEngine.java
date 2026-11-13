package dev.lemoyne.brakeindicator;

import android.content.Context;
import android.content.SharedPreferences;

import hmgapi.client.HmgApi;

import org.json.JSONObject;

import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * In-process vehicle & OBD-II evaluation engine shared by {@link OverlayService}
 * and {@link MainActivity}.
 *
 * Polls HMG-API's built-in `Vehicle` readings at 20 Hz (50 ms) (or 650 ms on legacy
 * KitKat `Hub`), aligns `-dv/dt` and `L0..i-Pedal` detection to exact HMG-API poll
 * timestamps, fuses or compares them against {@link ObdBrakeClient} (`@gen5w_ble`
 * primary -> `127.0.0.1:35000` fallback), records a 30-minute synchronized CSV dataset
 * in RAM, and exports to removable USB.
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

    private static final String[] VEHICLE_READINGS = {
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
    private static final long HMG_RETRY_MS = 250L;
    private static final long HMG_FRESH_MAX_MS = 1800L;
    private static final long SETTINGS_REFRESH_MS = 3000L;

    private static BrakeEngine sInstance;

    public static synchronized BrakeEngine getInstance(Context ctx) {
        if (sInstance == null) {
            sInstance = new BrakeEngine(ctx.getApplicationContext());
        }
        return sInstance;
    }

    private final Context appContext;
    private final BrakeEvaluator evaluator = new BrakeEvaluator();
    private final TelemetryRecorder recorder = new TelemetryRecorder();
    private final HevRegenProbe hevProbe = new HevRegenProbe();
    private final ObdBrakeClient obdClient;
    private final List<StateListener> listeners = new CopyOnWriteArrayList<StateListener>();
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicInteger markSeq = new AtomicInteger(0);

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
        this.obdClient = new ObdBrakeClient(store);
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

    public String getHmgStatusSummary() {
        if (!hmgConnected) {
            return "WAITING";
        }
        return latestHmgPoll.legacyStall ? "OK (legacy Hub - update HMG-API)" : "OK (20Hz)";
    }

    public String markDriveEvent() {
        int n = markSeq.incrementAndGet();
        BrakeEvaluator.Snapshot snap = latestSnapshot;
        String reg = snap != null
                ? BrakeEvaluator.regenLevelLabel(snap.inferredRegenLevel)
                : "?";
        String spd = (snap != null && snap.speedKmh != null)
                ? String.format(java.util.Locale.US, "%.1f km/h", snap.speedKmh)
                : "- km/h";
        String msg = "MARK #" + n + " (regen=" + reg + ", speed=" + spd + ")";
        recorder.recordExternalEvent(obdClient.monotonicMs(), msg);
        return msg;
    }

    public String markRegenBaseline() {
        return markDriveEvent();
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
        }
        IndicatorConfig newCfg = IndicatorConfig.fromMap(updated);
        config = newCfg;
        obdClient.applyConfig(newCfg);
        return next;
    }

    public UsbExporter.ExportResult exportDatasetToUsb() {
        String csv = recorder.toCsvString();
        String summary = recorder.buildSummaryReport(
                config,
                obdClient.getStatusSummary(),
                getHmgStatusSummary(),
                null);
        List<String> extraFolders = null;
        try {
            extraFolders = HmgApi.connect().timeout(1500).usbFolders();
        } catch (Exception ignored) {
        }
        return UsbExporter.exportDatasetAndSummary(csv, summary, null, extraFolders);
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
                    targetPeriodMs = res.legacyHubStall ? LEGACY_HUB_POLL_MS : EVAL_INTERVAL_MS;
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
        while (running.get()) {
            long tickStartMono = obdClient.monotonicMs();
            try {
                HmgPollSnapshot hmgSnap = latestHmgPoll;
                long hmgAt = hmgSnap.monoMs;
                boolean okHmg = hmgAt > 0L
                        && (tickStartMono - hmgAt) >= 0L
                        && (tickStartMono - hmgAt) <= HMG_FRESH_MAX_MS
                        && !hmgSnap.readings.isEmpty();
                hmgConnected = okHmg;
                Map<String, String> readings = okHmg
                        ? hmgSnap.readings
                        : Collections.<String, String>emptyMap();
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

                BrakeEvaluator.Snapshot snap = evaluator.evaluate(cfg, input);
                latestSnapshot = snap;
                recorder.record(snap, System.currentTimeMillis());

                for (int i = 0; i < listeners.size(); i++) {
                    try {
                        listeners.get(i).onEngineState(cfg, snap, okHmg);
                    } catch (Throwable ignored) {}
                }
            } catch (Throwable ignored) {
            }

            long spent = obdClient.monotonicMs() - tickStartMono;
            long sleep = EVAL_INTERVAL_MS - spent;
            if (sleep > 2L) {
                try {
                    Thread.sleep(sleep);
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
