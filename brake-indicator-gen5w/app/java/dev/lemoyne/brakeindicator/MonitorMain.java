package dev.lemoyne.brakeindicator;

import android.net.LocalServerSocket;
import android.net.LocalSocket;
import android.os.IBinder;
import android.os.SystemClock;

import hmgapi.client.HmgApi;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Background HMG-API service ("user": "system", uid 1000) that:
 *  1. Polls HMG-API vehicle readings + Option 1 ITM_HSCANService OEM getters at 10 Hz.
 *  2. Evaluates brake-lamp illumination state via BrakeEvaluator.
 *  3. Streams updates over @dev.lemoyne.brakeindicator.ipc to OverlayService and MainActivity.
 *  4. Starts OverlayService automatically when Android is up and no overlay client is connected.
 */
public final class MonitorMain {

    private static final String APP_PACKAGE = "dev.lemoyne.brakeindicator";
    private static final String OVERLAY_COMPONENT = "dev.lemoyne.brakeindicator/.OverlayService";
    private static final long POLL_INTERVAL_MS = 100L;
    private static final long SETTINGS_REFRESH_MS = 2500L;
    private static final long OVERLAY_ENSURE_MS = 4000L;
    private static final long HEARTBEAT_PUSH_MS = 500L;

    private final HmgApi api = HmgApi.connect().timeout(2000);
    private final BrakeEvaluator evaluator = new BrakeEvaluator();
    private final HscanProbe hscanProbe;
    private final List<LocalSocket> subscribers = new ArrayList<LocalSocket>();

    private volatile IndicatorConfig config = IndicatorConfig.defaults();
    private volatile BrakeEvaluator.Snapshot latestSnapshot;
    private volatile String latestStateLine = "";
    private volatile long testUntilMs;
    private volatile boolean forceSettingsReload = true;
    private volatile int cachedAppUid = -1;

    private MonitorMain(File dataDir) {
        this.hscanProbe = new HscanProbe(dataDir);
    }

    public static void main(String[] args) throws Exception {
        String dataPath = System.getenv("HMGAPI_DATA");
        File dataDir = dataPath != null && !dataPath.isEmpty()
                ? new File(dataPath)
                : new File("/data/local/tmp");
        System.out.println("brake-indicator: monitor starting (uid=" + android.os.Process.myUid() + ")");
        MonitorMain monitor = new MonitorMain(dataDir);
        monitor.run();
    }

    private void run() throws Exception {
        final LocalServerSocket server = new LocalServerSocket(IpcProtocol.SOCKET_NAME);
        Thread acceptThread = new Thread("brake-ipc-accept") {
            @Override
            public void run() {
                acceptLoop(server);
            }
        };
        acceptThread.setDaemon(true);
        acceptThread.start();

        long lastSettingsMs = 0L;
        long lastOverlayCheckMs = 0L;
        long lastPushMs = 0L;
        boolean lastLit = false;

        while (true) {
            long loopStart = SystemClock.elapsedRealtime();
            try {
                if (forceSettingsReload || loopStart - lastSettingsMs >= SETTINGS_REFRESH_MS) {
                    reloadSettings();
                    forceSettingsReload = false;
                    lastSettingsMs = loopStart;
                }

                IndicatorConfig cfg = config;
                Map<String, String> veh = readHmgVehicle();
                Float hubPressure = BrakeEvaluator.parseReadingFloat(veh.get("brake.pressure"));
                Float speedKmh = BrakeEvaluator.parseReadingFloat(veh.get("speed"));
                Float accelPedal = BrakeEvaluator.parseReadingFloat(veh.get("pedal.accel"));
                Boolean gearPark = BrakeEvaluator.parseReadingFlag(veh.get("gear.park"));
                Boolean parkingBrake = BrakeEvaluator.parseReadingFlag(veh.get("parking_brake"));

                boolean wantDiag = cfg.probeLogging || hasSubscribers();
                HscanProbe.PollResult hscan = hscanProbe.poll(loopStart, wantDiag, speedKmh, hubPressure);

                Float effectivePressure = hubPressure != null ? hubPressure : hscan.directBrakePressure;
                BrakeEvaluator.Input input = new BrakeEvaluator.Input(
                        loopStart,
                        hscan.directStopLamp,
                        hscan.directStopLampSource,
                        effectivePressure,
                        speedKmh,
                        accelPedal,
                        gearPark,
                        parkingBrake,
                        testUntilMs);

                BrakeEvaluator.Snapshot snap = evaluator.evaluate(cfg, input);
                latestSnapshot = snap;

                String encoded = IpcProtocol.encodeSnapshot(
                        snap,
                        cfg,
                        hscan.connected,
                        hscan.directCandidateCount,
                        hscan.diagnosticCandidateCount,
                        hscan.summary);

                boolean litChanged = snap.illuminated != lastLit;
                boolean lineChanged = !encoded.equals(latestStateLine);
                latestStateLine = encoded;
                lastLit = snap.illuminated;

                if (litChanged || (lineChanged && loopStart - lastPushMs >= 100L)
                        || loopStart - lastPushMs >= HEARTBEAT_PUSH_MS) {
                    broadcastState(encoded);
                    lastPushMs = loopStart;
                }

                if (!hasSubscribers() && loopStart - lastOverlayCheckMs >= OVERLAY_ENSURE_MS) {
                    lastOverlayCheckMs = loopStart;
                    ensureOverlayServiceStarted();
                }
            } catch (Throwable t) {
                System.out.println("brake-indicator: loop error: " + t);
            }

            long elapsed = SystemClock.elapsedRealtime() - loopStart;
            long sleepMs = POLL_INTERVAL_MS - elapsed;
            if (sleepMs > 10L) {
                try {
                    Thread.sleep(sleepMs);
                } catch (InterruptedException ie) {
                    break;
                }
            }
        }
    }

    private Map<String, String> readHmgVehicle() {
        try {
            return api.vehicle("brake.pressure", "speed", "pedal.accel", "gear.park", "parking_brake", "moving");
        } catch (IOException e) {
            return new LinkedHashMap<String, String>();
        }
    }

    private void reloadSettings() {
        try {
            JSONObject json = api.settings();
            Map<String, String> map = new LinkedHashMap<String, String>();
            Iterator<?> keys = json.keys();
            while (keys.hasNext()) {
                String k = String.valueOf(keys.next());
                Object v = json.opt(k);
                if (v != null && v != JSONObject.NULL) {
                    map.put(k, String.valueOf(v));
                }
            }
            config = IndicatorConfig.fromMap(map);
        } catch (Exception e) {
            // Keep previous config if hmgapid is momentarily busy
        }
    }

    private void acceptLoop(LocalServerSocket server) {
        while (true) {
            try {
                final LocalSocket client = server.accept();
                Thread handler = new Thread("brake-ipc-client") {
                    @Override
                    public void run() {
                        handleClient(client);
                    }
                };
                handler.setDaemon(true);
                handler.start();
            } catch (Throwable t) {
                System.out.println("brake-indicator: accept error: " + t);
                try {
                    Thread.sleep(500L);
                } catch (InterruptedException ignored) {
                    return;
                }
            }
        }
    }

    private void handleClient(LocalSocket client) {
        try {
            int peerUid = client.getPeerCredentials().getUid();
            if (!isAllowedPeer(peerUid)) {
                client.close();
                return;
            }
            client.setSoTimeout(3000);
            BufferedReader in = new BufferedReader(new InputStreamReader(client.getInputStream(), "UTF-8"), 256);
            String cmd = in.readLine();
            if (cmd == null) {
                client.close();
                return;
            }
            cmd = cmd.trim();
            if ("STREAM".equals(cmd)) {
                client.setSoTimeout(0);
                registerSubscriber(client);
                // Block until the client disconnects so we clean up promptly
                while (in.readLine() != null) {
                    // Drain any keepalive lines
                }
                removeSubscriber(client);
                return;
            }
            OutputStream out = client.getOutputStream();
            if (cmd.startsWith("TEST")) {
                long durationMs = 3000L;
                String[] parts = cmd.split(" +");
                if (parts.length >= 2) {
                    try {
                        durationMs = Math.max(500L, Math.min(15000L, Long.parseLong(parts[1])));
                    } catch (NumberFormatException ignored) {
                    }
                }
                testUntilMs = SystemClock.elapsedRealtime() + durationMs;
                out.write("OK\n".getBytes("UTF-8"));
                out.flush();
                ensureOverlayServiceStarted();
            } else if ("RELOAD".equals(cmd)) {
                forceSettingsReload = true;
                out.write("OK\n".getBytes("UTF-8"));
                out.flush();
            } else if ("REPORT".equals(cmd)) {
                String report = hscanProbe.buildReport(config, latestSnapshot);
                out.write((report + "\nEND_REPORT\n").getBytes("UTF-8"));
                out.flush();
            } else if ("SNAPSHOT".equals(cmd)) {
                out.write((latestStateLine + "\n").getBytes("UTF-8"));
                out.flush();
            }
            client.close();
        } catch (Exception e) {
            removeSubscriber(client);
            try {
                client.close();
            } catch (Exception ignored) {
            }
        }
    }

    private boolean isAllowedPeer(int peerUid) {
        if (peerUid == 0 || peerUid == 1000 || peerUid == 2000) {
            return true;
        }
        int appUid = resolveAppUid();
        return appUid > 0 && peerUid == appUid;
    }

    private int resolveAppUid() {
        int uid = cachedAppUid;
        if (uid > 0) {
            return uid;
        }
        try {
            Class<?> sm = Class.forName("android.os.ServiceManager");
            IBinder pmBinder = (IBinder) sm.getMethod("getService", String.class).invoke(null, "package");
            if (pmBinder != null) {
                Class<?> stub = Class.forName("android.content.pm.IPackageManager$Stub");
                Object pm = stub.getMethod("asInterface", IBinder.class).invoke(null, pmBinder);
                Method getPkgUid = pm.getClass().getMethod("getPackageUid", String.class, Integer.TYPE);
                int resolved = ((Number) getPkgUid.invoke(pm, APP_PACKAGE, 0)).intValue();
                if (resolved > 0) {
                    cachedAppUid = resolved;
                    return resolved;
                }
            }
        } catch (Throwable ignored) {
        }
        return -1;
    }

    private boolean hasSubscribers() {
        synchronized (subscribers) {
            return !subscribers.isEmpty();
        }
    }

    private void registerSubscriber(LocalSocket s) {
        String initial = latestStateLine;
        synchronized (subscribers) {
            subscribers.add(s);
        }
        if (initial != null && !initial.isEmpty()) {
            try {
                OutputStream out = s.getOutputStream();
                out.write((initial + "\n").getBytes("UTF-8"));
                out.flush();
            } catch (IOException e) {
                removeSubscriber(s);
            }
        }
    }

    private void removeSubscriber(LocalSocket s) {
        synchronized (subscribers) {
            subscribers.remove(s);
        }
        try {
            s.close();
        } catch (Exception ignored) {
        }
    }

    private void broadcastState(String line) {
        byte[] payload;
        try {
            payload = (line + "\n").getBytes("UTF-8");
        } catch (Exception e) {
            return;
        }
        synchronized (subscribers) {
            Iterator<LocalSocket> it = subscribers.iterator();
            while (it.hasNext()) {
                LocalSocket s = it.next();
                try {
                    OutputStream out = s.getOutputStream();
                    out.write(payload);
                    out.flush();
                } catch (IOException e) {
                    it.remove();
                    try {
                        s.close();
                    } catch (Exception ignored) {
                    }
                }
            }
        }
    }

    private void ensureOverlayServiceStarted() {
        try {
            // Check sys.boot_completed first so we don't call am before ActivityManager is ready
            Class<?> sp = Class.forName("android.os.SystemProperties");
            String boot = (String) sp.getMethod("get", String.class, String.class)
                    .invoke(null, "sys.boot_completed", "0");
            if (!"1".equals(boot)) {
                return;
            }
            Process p = new ProcessBuilder(
                    "/system/bin/am", "startservice", "--user", "0", "-n", OVERLAY_COMPONENT)
                    .redirectErrorStream(true)
                    .start();
            p.waitFor();
        } catch (Throwable ignored) {
        }
    }
}
