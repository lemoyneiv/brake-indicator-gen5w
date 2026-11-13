package dev.lemoyne.brakeindicator;

import gen5w.ble.Gen5wBleAdvRecord;
import gen5w.ble.Gen5wBleClient;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Dual-transport OBD-II IGMP (`770` -> `778`, UDS `22 BC06`) brake-lamp reader:
 *  1. Primary: Direct BLE GATT session via {@code @gen5w_ble} ({@link Gen5wBleClient}),
 *     with LNEX BVM02 hardware safety exclusion.
 *  2. Fallback: Automatic localhost ELM327 TCP bridge ({@code 127.0.0.1:<port>}, default 35000)
 *     when {@code obd-gen5w} already holds the BLE link (`ALREADY_CONNECTED`) or {@code @gen5w_ble}
 *     is temporarily unavailable. Uses pure `lo` loopback (never touches `wlan0` or wireless projection).
 */
public final class ObdBrakeClient {

    public interface PairingStore {
        String getMac();
        String getName();
        int getAddrType();
        void save(String mac, String name, int addrType);
        void clear();
    }

    public interface Listener {
        void onObdStateChanged();
    }

    public static final class DiscoveredDevice {
        public final String mac;
        public final int addrType;
        public final int rssi;
        public final String name;
        public final boolean likelyObd;
        public final boolean likelyBvm02;

        public DiscoveredDevice(
                String mac,
                int addrType,
                int rssi,
                String name,
                boolean likelyObd,
                boolean likelyBvm02) {
            this.mac = mac;
            this.addrType = addrType;
            this.rssi = rssi;
            this.name = name;
            this.likelyObd = likelyObd;
            this.likelyBvm02 = likelyBvm02;
        }
    }

    private final PairingStore store;
    private final boolean testMode;
    private final long monoOriginNano = System.nanoTime();
    private final List<Listener> listeners = new CopyOnWriteArrayList<Listener>();
    private final Map<String, DiscoveredDevice> discovered = new LinkedHashMap<String, DiscoveredDevice>();
    private final List<String> triedServiceOverrides = new ArrayList<String>();
    private final Map<String, List<String>> discoveredServicesByMac = new LinkedHashMap<String, List<String>>();

    private volatile Gen5wBleClient bleClient;
    private final Elm327BrakeProtocol.PromptBuffer blePromptBuf = new Elm327BrakeProtocol.PromptBuffer();
    private final Object blePromptLock = new Object();
    private String latestBlePrompt = null;

    private final Object notifyReadyLock = new Object();
    private String lastNotifyReadyUuid = "";

    private final AtomicBoolean stopped = new AtomicBoolean(false);
    private volatile boolean enabled = true;
    private volatile int bridgePort = 35000;

    // Paired BLE adapter metadata
    private volatile String pairedMac = "";
    private volatile String pairedName = "";
    private volatile int pairedAddrType = -1;

    // BLE session state
    private volatile boolean bleBridgeReady = false;
    private volatile boolean bleScanning = false;
    private volatile int scanGeneration = 0;
    private volatile boolean bleConnecting = false;
    private volatile boolean bleConnected = false;
    private volatile boolean bleAlreadyOwnedElsewhere = false;
    private volatile boolean bleGattRejected = false;
    private volatile String bleServiceFilter = "*";
    private volatile boolean pendingTargetedSvcRetry = false;
    private volatile long nextBleReconnectMs = 0L;
    private volatile int bleTimeoutStreak = 0;
    private volatile int bleChunkBytes = 20;
    private volatile Elm327BrakeProtocol.GattBinding bleBinding = null;
    private volatile List<Elm327BrakeProtocol.GattBinding> bleCandidates = Collections.emptyList();

    // Loopback TCP fallback state
    private volatile Socket activeBridgeSocket = null;
    private volatile boolean bridgeConnected = false;

    // Shared telemetry snapshot
    private volatile String statusSummary = "Unpaired (or awaiting 127.0.0.1:35000)";
    private volatile float pollHz = 0.0f;
    private volatile BrakeEvaluator.ObdSample latestSample = BrakeEvaluator.ObdSample.disconnected();

    public ObdBrakeClient(PairingStore store) {
        this(store, false);
    }

    private ObdBrakeClient(PairingStore store, boolean testMode) {
        this.store = store;
        this.testMode = testMode;
        if (store != null) {
            this.pairedMac = safeUpper(store.getMac());
            this.pairedName = store.getName() == null ? "" : store.getName();
            this.pairedAddrType = store.getAddrType();
        }
        this.bleClient = new Gen5wBleClient(new BleListener());
        if (!testMode) {
            this.bleClient.connectBridge();
            startCoordinatorLoop();
        }
    }

    /** Deterministic factory for host-JVM unit testing of BLE + loopback fallback behavior. */
    public static ObdBrakeClient createForTest(
            PairingStore store,
            Gen5wBleClient.LineChannel channel,
            int fallbackPort) {
        ObdBrakeClient client = new ObdBrakeClient(store, true);
        client.bridgePort = fallbackPort;
        if (channel != null) {
            client.bleClient.attachCustomChannel(channel);
            client.bleBridgeReady = true;
        }
        return client;
    }

    public long monotonicMs() {
        return Math.max(1L, (System.nanoTime() - monoOriginNano) / 1_000_000L);
    }

    public void addListener(Listener l) {
        if (l != null && !listeners.contains(l)) {
            listeners.add(l);
        }
    }

    public void removeListener(Listener l) {
        listeners.remove(l);
    }

    private void notifyListeners() {
        for (int i = 0; i < listeners.size(); i++) {
            try {
                listeners.get(i).onObdStateChanged();
            } catch (Throwable ignored) {}
        }
    }

    public void applyConfig(IndicatorConfig cfg) {
        if (cfg == null) return;
        boolean wantEnabled = (cfg.obdMode != IndicatorConfig.ObdMode.DISABLED);
        int wantPort = cfg.bridgePort;
        boolean changed = (wantEnabled != this.enabled) || (wantPort != this.bridgePort);
        this.enabled = wantEnabled;
        this.bridgePort = wantPort;
        if (!wantEnabled) {
            disconnectActiveTransports("OBD disabled in settings");
        } else if (changed && !testMode) {
            requestReconnect();
        }
    }

    public BrakeEvaluator.ObdSample currentSample() {
        return enabled ? latestSample : BrakeEvaluator.ObdSample.disconnected();
    }

    public String getStatusSummary() {
        return enabled ? statusSummary : "OBD disabled";
    }

    public float getPollHz() {
        return enabled ? pollHz : 0.0f;
    }

    public boolean isBleBridgeReady() {
        return bleBridgeReady;
    }

    public boolean isBleScanning() {
        return bleScanning;
    }

    public String getPairedMac() {
        return pairedMac;
    }

    public String getPairedName() {
        return pairedName;
    }

    public List<DiscoveredDevice> getDiscoveredDevices() {
        synchronized (discovered) {
            return new ArrayList<DiscoveredDevice>(discovered.values());
        }
    }

    public Gen5wBleClient getBleClientForTest() {
        return bleClient;
    }

    public void startBleScan() {
        if (!enabled) return;
        ensureBleBridgeClientOpen();
        synchronized (discovered) {
            discovered.clear();
        }
        final int gen = ++scanGeneration;
        bleClient.startScan();
        notifyListeners();
        if (!testMode) {
            Thread autoStop = new Thread("BrakeObdScanStop") {
                @Override
                public void run() {
                    sleepQuiet(15000L);
                    if (!stopped.get() && bleScanning && scanGeneration == gen) {
                        stopBleScan();
                    }
                }
            };
            autoStop.setDaemon(true);
            autoStop.start();
        }
    }

    public void stopBleScan() {
        scanGeneration++;
        Gen5wBleClient c = bleClient;
        if (c != null) {
            c.stopScan();
        }
        notifyListeners();
    }

    public boolean pairAdapter(String mac, String name, int addrType) {
        if (mac == null || mac.trim().isEmpty()) return false;
        if (Elm327BrakeProtocol.isLikelyBvm02Device(name)) {
            statusSummary = "Refused BVM02 battery sensor (safety guard)";
            notifyListeners();
            return false;
        }
        String cleanMac = safeUpper(mac);
        String cleanName = (name == null || name.trim().isEmpty()) ? "BLE OBD-II" : name.trim();
        if (!pairedMac.isEmpty() && !pairedMac.equals(cleanMac) && bleClient != null) {
            bleClient.disconnectDevice(pairedMac);
        }
        closeBridgeSocketQuiet();
        this.pairedMac = cleanMac;
        this.pairedName = cleanName;
        this.pairedAddrType = addrType;
        if (store != null) {
            store.save(cleanMac, cleanName, addrType);
        }
        resetBleRetryState();
        scanGeneration++;
        if (bleClient != null) {
            bleClient.stopScan();
            if (bleBridgeReady && enabled) {
                bleConnecting = true;
                statusSummary = "Connecting BLE (" + shortName(cleanName, cleanMac) + ")...";
                bleClient.connectDevice(cleanMac, addrType, "*");
            }
        }
        notifyListeners();
        return true;
    }

    public void unpairAdapter() {
        String oldMac = pairedMac;
        pairedMac = "";
        pairedName = "";
        pairedAddrType = -1;
        if (store != null) {
            store.clear();
        }
        resetBleRetryState();
        bleConnected = false;
        bleConnecting = false;
        bleBinding = null;
        if (!oldMac.isEmpty() && bleClient != null) {
            bleClient.disconnectDevice(oldMac);
        }
        if (!bridgeConnected) {
            latestSample = BrakeEvaluator.ObdSample.disconnected();
            statusSummary = "Unpaired (127.0.0.1:" + bridgePort + " fallback active)";
        }
        notifyListeners();
    }

    public void requestReconnect() {
        resetBleRetryState();
        closeBridgeSocketQuiet();
        if (!pairedMac.isEmpty() && bleClient != null) {
            bleConnected = false;
            bleConnecting = false;
            bleClient.disconnectDevice(pairedMac);
            if (bleBridgeReady && enabled) {
                bleConnecting = true;
                statusSummary = "Reconnecting BLE...";
                bleClient.connectDevice(pairedMac, pairedAddrType, "*");
            }
        }
        notifyListeners();
    }

    public void close() {
        if (!stopped.compareAndSet(false, true)) return;
        disconnectActiveTransports("Stopped");
        Gen5wBleClient c = bleClient;
        if (c != null) {
            c.close();
        }
    }

    private void disconnectActiveTransports(String status) {
        closeBridgeSocketQuiet();
        Gen5wBleClient c = bleClient;
        if (c != null && !pairedMac.isEmpty()) {
            c.disconnectDevice(pairedMac);
        }
        bleConnected = false;
        bleConnecting = false;
        bleBinding = null;
        pollHz = 0.0f;
        latestSample = BrakeEvaluator.ObdSample.disconnected();
        statusSummary = status;
        synchronized (blePromptLock) {
            blePromptLock.notifyAll();
        }
        synchronized (notifyReadyLock) {
            notifyReadyLock.notifyAll();
        }
        notifyListeners();
    }

    private void resetBleRetryState() {
        bleAlreadyOwnedElsewhere = false;
        bleGattRejected = false;
        bleServiceFilter = "*";
        pendingTargetedSvcRetry = false;
        bleTimeoutStreak = 0;
        nextBleReconnectMs = 0L;
        synchronized (triedServiceOverrides) {
            triedServiceOverrides.clear();
        }
    }

    private void ensureBleBridgeClientOpen() {
        Gen5wBleClient c = bleClient;
        if (c == null || !c.isBridgeOpen()) {
            bleBridgeReady = false;
            if (!testMode) {
                Gen5wBleClient next = new Gen5wBleClient(new BleListener());
                this.bleClient = next;
                next.connectBridge();
            }
        }
    }

    private void startCoordinatorLoop() {
        Thread worker = new Thread("BrakeObdCoordinator") {
            @Override
            public void run() {
                coordinatorLoop();
            }
        };
        worker.setDaemon(true);
        worker.start();
    }

    /**
     * Master connection priority state machine:
     *  1. Always try `@gen5w_ble` direct BLE first when a BLE OBD adapter is paired and free.
     *  2. If `@gen5w_ble` is busy (`ALREADY_CONNECTED` by `obd-gen5w`), unreachable, or unpaired,
     *     automatically try `127.0.0.1:<bridgePort>` loopback ELM327 fallback.
     */
    private void coordinatorLoop() {
        while (!stopped.get()) {
            if (!enabled) {
                sleepQuiet(1000L);
                continue;
            }

            ensureBleBridgeClientOpen();
            long now = System.currentTimeMillis();

            // Priority 1: Direct @gen5w_ble BLE session
            if (bleConnected && bleBinding != null) {
                runBlePollSession();
                continue;
            }

            boolean canTryDirectBle = bleBridgeReady
                    && !pairedMac.isEmpty()
                    && !bleGattRejected
                    && !bleAlreadyOwnedElsewhere
                    && now >= nextBleReconnectMs;

            if (canTryDirectBle) {
                if (!bleConnecting) {
                    bleConnecting = true;
                    statusSummary = "Connecting @gen5w_ble (" + shortName(pairedName, pairedMac) + ")...";
                    bleClient.connectDevice(pairedMac, pairedAddrType, bleServiceFilter);
                    notifyListeners();
                }
                sleepQuiet(400L);
                continue;
            }

            // Priority 2: Automatic fallback to 127.0.0.1:<bridgePort> (obd-gen5w loopback bridge)
            if (!bleScanning) {
                boolean ranBridge = tryRunLoopbackBridgeSession(bridgePort);
                if (ranBridge) {
                    // When loopback session ends, immediately allow direct @gen5w_ble retry!
                    bleAlreadyOwnedElsewhere = false;
                    nextBleReconnectMs = 0L;
                    continue;
                }
            }

            // Periodically re-check whether @gen5w_ble direct link became free (e.g. obd-gen5w closed)
            if (bleAlreadyOwnedElsewhere && now >= nextBleReconnectMs) {
                bleAlreadyOwnedElsewhere = false;
            }
            if (!bleConnected && !bleConnecting && !bridgeConnected) {
                if (pairedMac.isEmpty()) {
                    statusSummary = bleBridgeReady
                            ? "Unpaired (Scan BLE or enable 127.0.0.1:" + bridgePort + ")"
                            : "Awaiting @gen5w_ble or 127.0.0.1:" + bridgePort;
                }
                notifyListeners();
            }
            sleepQuiet(2500L);
        }
    }

    // ================= DIRECT @GEN5W_BLE POLLING =================

    private void runBlePollSession() {
        Elm327BrakeProtocol.GattBinding initial = bleBinding;
        if (initial == null) return;

        awaitNotifyReady(initial.notifyUuid, 2200L);
        sleepQuiet(100L);

        String ver = probeBleCandidate(initial);
        List<Elm327BrakeProtocol.GattBinding> all = bleCandidates;
        if ((ver == null || ver.isEmpty()) && all != null && all.size() > 1) {
            for (int i = 1; i < all.size() && bleConnected && enabled && !stopped.get(); i++) {
                Elm327BrakeProtocol.GattBinding next = all.get(i);
                synchronized (notifyReadyLock) {
                    lastNotifyReadyUuid = "";
                }
                bleClient.enableNotifications(pairedMac, next.notifyUuid);
                awaitNotifyReady(next.notifyUuid, 2000L);
                sleepQuiet(80L);
                ver = probeBleCandidate(next);
                if (ver != null && !ver.isEmpty()) {
                    break;
                }
            }
        }

        for (int i = 0; i < Elm327BrakeProtocol.IGMP_INIT_COMMANDS.length && bleConnected && enabled; i++) {
            sendBleCommandBlocking(Elm327BrakeProtocol.IGMP_INIT_COMMANDS[i], 900L);
        }

        int noDataStreak = 0;
        int promptTimeoutStreak = 0;
        int cmdsInWindow = 0;
        long windowStartMs = System.currentTimeMillis();

        while (bleConnected && bleBinding != null && enabled && !stopped.get()) {
            String raw = sendBleCommandBlocking(Elm327BrakeProtocol.CMD_IGMP_BRAKE_LAMP, 550L);
            cmdsInWindow++;
            long nowWall = System.currentTimeMillis();
            if (nowWall - windowStartMs >= 1500L) {
                pollHz = (cmdsInWindow * 1000.0f) / (float) (nowWall - windowStartMs);
                cmdsInWindow = 0;
                windowStartMs = nowWall;
            }

            if (raw == null) {
                promptTimeoutStreak++;
                if (promptTimeoutStreak >= 3) {
                    statusSummary = "BLE adapter stalled; cycling link...";
                    bleConnected = false;
                    bleClient.disconnectDevice(pairedMac);
                    nextBleReconnectMs = System.currentTimeMillis() + 2000L;
                    break;
                }
                continue;
            }
            promptTimeoutStreak = 0;

            Elm327BrakeProtocol.IgmpBrakeReading reading = Elm327BrakeProtocol.decodeIgmpBrakeResponse(raw);
            if (reading != null) {
                noDataStreak = 0;
                latestSample = new BrakeEvaluator.ObdSample(
                        true,
                        "BLE",
                        monotonicMs(),
                        reading.rawCode,
                        reading.brakeLampLit,
                        reading.pedalBit);
                statusSummary = "BLE @gen5w_ble (" + shortName(pairedName, pairedMac) + ")";
                notifyListeners();
                sleepQuiet(15L);
            } else {
                noDataStreak++;
                if (noDataStreak >= 3) {
                    // Check ATI so parked IG OFF enters 5s sleep instead of keeping CAN awake
                    String alive = sendBleCommandBlocking("ATI", 1000L);
                    if (alive == null || alive.trim().isEmpty()) {
                        bleConnected = false;
                        bleClient.disconnectDevice(pairedMac);
                        nextBleReconnectMs = System.currentTimeMillis() + 2500L;
                        break;
                    }
                    pollHz = 0.2f;
                    latestSample = BrakeEvaluator.ObdSample.disconnected();
                    statusSummary = "BLE Standby (IG OFF)";
                    notifyListeners();
                    sleepQuiet(5000L);
                } else {
                    sleepQuiet(150L);
                }
            }
        }
        pollHz = 0.0f;
        if (!bridgeConnected) {
            latestSample = BrakeEvaluator.ObdSample.disconnected();
        }
        notifyListeners();
    }

    private String probeBleCandidate(Elm327BrakeProtocol.GattBinding candidate) {
        if (!bleConnected || candidate == null) return null;
        bleBinding = candidate;
        String ver = sendBleCommandBlocking("ATI", 1400L);
        if ((ver == null || ver.isEmpty()) && bleConnected) {
            ver = sendBleCommandBlocking("ATZ", 1800L);
        }
        if ((ver == null || ver.isEmpty()) && candidate.supportsAlternateWriteMode() && bleConnected) {
            bleBinding = candidate.withWriteWithoutResponse(!candidate.writeWithoutResponse);
            ver = sendBleCommandBlocking("ATI", 1400L);
        }
        return ver;
    }

    private void awaitNotifyReady(String notifyUuid, long timeoutMs) {
        String norm = Gen5wBleAdvRecord.normalizeUuid(notifyUuid);
        long deadline = System.currentTimeMillis() + Math.max(150L, timeoutMs);
        synchronized (notifyReadyLock) {
            while (bleConnected && !norm.equals(lastNotifyReadyUuid)) {
                long rem = deadline - System.currentTimeMillis();
                if (rem <= 0L) break;
                try {
                    notifyReadyLock.wait(rem);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
    }

    public String sendBleCommandBlocking(String cmd, long timeoutMs) {
        if (!Elm327BrakeProtocol.isSafeCommand(cmd)) {
            return null;
        }
        Elm327BrakeProtocol.GattBinding b = bleBinding;
        String mac = pairedMac;
        Gen5wBleClient c = bleClient;
        if (!bleConnected || b == null || mac.isEmpty() || c == null) {
            return null;
        }
        synchronized (blePromptLock) {
            latestBlePrompt = null;
            blePromptBuf.reset();
        }
        List<byte[]> chunks = Elm327BrakeProtocol.encodeCommandChunks(cmd, bleChunkBytes);
        for (int i = 0; i < chunks.size(); i++) {
            c.writeCharacteristic(mac, b.writeUuid, chunks.get(i), b.writeWithoutResponse);
        }
        long deadline = System.currentTimeMillis() + Math.max(100L, timeoutMs);
        synchronized (blePromptLock) {
            while (latestBlePrompt == null && bleConnected && enabled && !stopped.get()) {
                long rem = deadline - System.currentTimeMillis();
                if (rem <= 0L) break;
                try {
                    blePromptLock.wait(rem);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
            return latestBlePrompt;
        }
    }

    // ================= 127.0.0.1 LOOPBACK TCP BRIDGE FALLBACK =================

    /**
     * Connects strictly to `127.0.0.1:<port>` (pure kernel loopback `lo`, zero Wi-Fi/projection impact)
     * and polls `22BC06`. Publicly accessible for deterministic unit tests.
     */
    public boolean tryRunLoopbackBridgeSession(int port) {
        Socket sock = new Socket();
        try {
            InetAddress loopback = InetAddress.getByName("127.0.0.1");
            sock.connect(new InetSocketAddress(loopback, port), 450);
            sock.setTcpNoDelay(true);
            sock.setSoTimeout(1200);
            activeBridgeSocket = sock;
            bridgeConnected = true;
            statusSummary = "Fallback Bridge (127.0.0.1:" + port + ")";
            notifyListeners();

            InputStream in = sock.getInputStream();
            OutputStream out = sock.getOutputStream();
            Elm327BrakeProtocol.PromptBuffer promptBuf = new Elm327BrakeProtocol.PromptBuffer();

            // Verify live ELM327 response before entering continuous loop
            String ver = sendSocketElmCommand(in, out, promptBuf, "ATI", 1200L);
            if (ver == null) {
                return false;
            }
            for (int i = 0; i < Elm327BrakeProtocol.IGMP_INIT_COMMANDS.length && enabled && !stopped.get(); i++) {
                sendSocketElmCommand(in, out, promptBuf, Elm327BrakeProtocol.IGMP_INIT_COMMANDS[i], 900L);
            }

            int noDataStreak = 0;
            int cmdsInWindow = 0;
            long windowStartMs = System.currentTimeMillis();

            while (enabled && !stopped.get() && !sock.isClosed()) {
                String raw = sendSocketElmCommand(in, out, promptBuf, Elm327BrakeProtocol.CMD_IGMP_BRAKE_LAMP, 650L);
                if (raw == null) {
                    break;
                }
                cmdsInWindow++;
                long nowWall = System.currentTimeMillis();
                if (nowWall - windowStartMs >= 1500L) {
                    pollHz = (cmdsInWindow * 1000.0f) / (float) (nowWall - windowStartMs);
                    cmdsInWindow = 0;
                    windowStartMs = nowWall;
                }

                Elm327BrakeProtocol.IgmpBrakeReading reading = Elm327BrakeProtocol.decodeIgmpBrakeResponse(raw);
                if (reading != null) {
                    noDataStreak = 0;
                    latestSample = new BrakeEvaluator.ObdSample(
                            true,
                            "BRIDGE",
                            monotonicMs(),
                            reading.rawCode,
                            reading.brakeLampLit,
                            reading.pedalBit);
                    statusSummary = "Fallback 127.0.0.1:" + port;
                    notifyListeners();
                    if (testMode) {
                        return true;
                    }
                    sleepQuiet(20L);
                } else {
                    noDataStreak++;
                    if (noDataStreak >= 3) {
                        pollHz = 0.2f;
                        latestSample = BrakeEvaluator.ObdSample.disconnected();
                        statusSummary = "Bridge Standby (IG OFF / No BLE)";
                        notifyListeners();
                        if (testMode) {
                            return false;
                        }
                        sleepQuiet(4000L);
                    } else {
                        sleepQuiet(150L);
                    }
                }
            }
            return true;
        } catch (IOException ioe) {
            return false;
        } finally {
            bridgeConnected = false;
            activeBridgeSocket = null;
            pollHz = 0.0f;
            if (!testMode && !bleConnected) {
                latestSample = BrakeEvaluator.ObdSample.disconnected();
            }
            try { sock.close(); } catch (Throwable ignored) {}
            notifyListeners();
        }
    }

    private static String sendSocketElmCommand(
            InputStream in,
            OutputStream out,
            Elm327BrakeProtocol.PromptBuffer promptBuf,
            String cmd,
            long timeoutMs) throws IOException {
        if (!Elm327BrakeProtocol.isSafeCommand(cmd)) {
            return null;
        }
        promptBuf.reset();
        out.write((cmd.trim() + "\r").getBytes(Elm327BrakeProtocol.ASCII));
        out.flush();

        byte[] buf = new byte[128];
        long deadline = System.currentTimeMillis() + Math.max(150L, timeoutMs);
        while (System.currentTimeMillis() < deadline) {
            int n = in.read(buf);
            if (n < 0) {
                return null;
            }
            List<String> done = promptBuf.feed(buf, 0, n);
            if (!done.isEmpty()) {
                return done.get(done.size() - 1);
            }
        }
        return null;
    }

    private void closeBridgeSocketQuiet() {
        Socket s = activeBridgeSocket;
        activeBridgeSocket = null;
        bridgeConnected = false;
        if (s != null) {
            try { s.close(); } catch (Throwable ignored) {}
        }
    }

    // ================= @GEN5W_BLE LISTENER =================

    private final class BleListener extends Gen5wBleClient.SimpleListener {

        @Override
        public void onSessionReady(String serverVersion) {
            bleBridgeReady = true;
            notifyListeners();
        }

        @Override
        public void onScanStateChanged(boolean scanning, String error) {
            bleScanning = scanning;
            notifyListeners();
        }

        @Override
        public void onAdvertisement(String mac, int addrType, int rssi, Gen5wBleAdvRecord record) {
            if (mac == null) return;
            String upper = safeUpper(mac);
            if (upper.equals(pairedMac)) {
                if (addrType == 0 || addrType == 1) {
                    pairedAddrType = addrType;
                }
                nextBleReconnectMs = 0L;
                bleAlreadyOwnedElsewhere = false;
            }
            String localName = (record != null && record.localName != null && !record.localName.isEmpty())
                    ? record.localName
                    : "BLE Device";
            List<String> uuids = record != null ? record.serviceUuids : Collections.<String>emptyList();
            boolean isBvm = Elm327BrakeProtocol.isLikelyBvm02Device(localName);
            boolean isObd = Elm327BrakeProtocol.isLikelyObdAdapter(localName, uuids);
            DiscoveredDevice dev = new DiscoveredDevice(upper, addrType, rssi, localName, isObd, isBvm);
            synchronized (discovered) {
                discovered.put(upper, dev);
            }
            notifyListeners();
        }

        @Override
        public void onConnecting(String mac) {
            if (mac != null && mac.equalsIgnoreCase(pairedMac)) {
                bleConnecting = true;
                synchronized (discoveredServicesByMac) {
                    discoveredServicesByMac.remove(pairedMac);
                }
                notifyListeners();
            }
        }

        @Override
        public void onGattServiceDiscovered(String mac, String serviceUuid, int startHandle, int endHandle) {
            String effective = (mac != null && !mac.isEmpty()) ? safeUpper(mac) : pairedMac;
            if (effective.isEmpty() || serviceUuid == null) return;
            String norm = Elm327BrakeProtocol.normUuid(serviceUuid);
            synchronized (discoveredServicesByMac) {
                List<String> list = discoveredServicesByMac.get(effective);
                if (list == null) {
                    list = new ArrayList<String>();
                    discoveredServicesByMac.put(effective, list);
                }
                if (!list.contains(norm)) {
                    list.add(norm);
                }
            }
        }

        @Override
        public void onConnected(String mac, Map<String, Integer> charPropsByUuid) {
            if (mac == null || !mac.equalsIgnoreCase(pairedMac)) return;
            bleConnecting = false;
            List<Elm327BrakeProtocol.GattBinding> cands = Elm327BrakeProtocol.resolveAdapterBindings(charPropsByUuid);
            if (cands.isEmpty()) {
                List<String> svcs;
                synchronized (discoveredServicesByMac) {
                    List<String> raw = discoveredServicesByMac.get(pairedMac);
                    svcs = raw == null ? Collections.<String>emptyList() : new ArrayList<String>(raw);
                }
                String nextSvc = pickNextServiceOverride(svcs, charPropsByUuid);
                if (nextSvc != null) {
                    bleServiceFilter = nextSvc;
                    pendingTargetedSvcRetry = true;
                    bleClient.disconnectDevice(pairedMac);
                    return;
                }
                bleGattRejected = true;
                bleConnected = false;
                statusSummary = "Incompatible GATT (safety rejected)";
                bleClient.disconnectDevice(pairedMac);
                notifyListeners();
                return;
            }

            closeBridgeSocketQuiet();
            Elm327BrakeProtocol.GattBinding primary = cands.get(0);
            bleCandidates = cands;
            bleBinding = primary;
            bleChunkBytes = 20;
            bleConnected = true;
            bleAlreadyOwnedElsewhere = false;
            bleTimeoutStreak = 0;
            statusSummary = "BLE GATT Ready (" + primary.profileName + ")";
            synchronized (notifyReadyLock) {
                lastNotifyReadyUuid = "";
            }
            bleClient.enableNotifications(pairedMac, primary.notifyUuid);
            notifyListeners();
        }

        @Override
        public void onDisconnected(String mac, int status, String reason) {
            if (mac == null || !mac.equalsIgnoreCase(pairedMac)) return;
            bleConnecting = false;
            bleConnected = false;
            bleBinding = null;
            synchronized (blePromptLock) {
                blePromptLock.notifyAll();
            }
            synchronized (notifyReadyLock) {
                notifyReadyLock.notifyAll();
            }
            if (pendingTargetedSvcRetry) {
                pendingTargetedSvcRetry = false;
                bleConnecting = true;
                bleClient.connectDevice(pairedMac, pairedAddrType, bleServiceFilter);
                return;
            }
            if ("ALREADY_CONNECTED".equals(reason) || status == 253) {
                bleAlreadyOwnedElsewhere = true;
                nextBleReconnectMs = System.currentTimeMillis() + 12000L;
                statusSummary = "BLE held by peer app -> trying 127.0.0.1:" + bridgePort;
            } else {
                bleTimeoutStreak++;
                long backoff = Math.min(15000L, 3000L * bleTimeoutStreak);
                nextBleReconnectMs = System.currentTimeMillis() + backoff;
                if (!bridgeConnected && !bleGattRejected) {
                    statusSummary = "BLE disconnected (" + reason + ")";
                }
            }
            notifyListeners();
        }

        @Override
        public void onMtuChanged(String mac, int mtu, String error) {
            if (mac != null && mac.equalsIgnoreCase(pairedMac) && error == null && mtu >= 23) {
                bleChunkBytes = Math.min(180, Math.max(20, mtu - 3));
            }
        }

        @Override
        public void onNotificationStateChanged(String mac, String uuid, boolean enabledFlag, String error) {
            if (mac != null && mac.equalsIgnoreCase(pairedMac) && uuid != null) {
                synchronized (notifyReadyLock) {
                    lastNotifyReadyUuid = Gen5wBleAdvRecord.normalizeUuid(uuid);
                    notifyReadyLock.notifyAll();
                }
            }
        }

        @Override
        public void onCharacteristicChanged(String mac, String uuid, byte[] value) {
            Elm327BrakeProtocol.GattBinding b = bleBinding;
            if (mac == null || value == null || b == null || !mac.equalsIgnoreCase(pairedMac)) {
                return;
            }
            if (!Gen5wBleAdvRecord.normalizeUuid(b.notifyUuid).equals(Gen5wBleAdvRecord.normalizeUuid(uuid))) {
                return;
            }
            List<String> done = blePromptBuf.feed(value);
            if (!done.isEmpty()) {
                synchronized (blePromptLock) {
                    latestBlePrompt = done.get(done.size() - 1);
                    blePromptLock.notifyAll();
                }
            }
        }

        @Override
        public void onSessionClosed(String reason) {
            bleBridgeReady = false;
            bleConnected = false;
            bleConnecting = false;
            synchronized (blePromptLock) {
                blePromptLock.notifyAll();
            }
            synchronized (notifyReadyLock) {
                notifyReadyLock.notifyAll();
            }
            notifyListeners();
        }
    }

    private String pickNextServiceOverride(List<String> svcs, Map<String, Integer> charProps) {
        if (svcs == null || svcs.isEmpty()) return null;
        if (charProps != null) {
            Integer p4 = charProps.get("fff4");
            Integer p1 = charProps.get("fff1");
            if (p4 != null && (p4.intValue() & (Elm327BrakeProtocol.PROP_NOTIFY | Elm327BrakeProtocol.PROP_INDICATE)) != 0) {
                boolean f1Active = p1 != null && (p1.intValue() & 0x3C) != 0;
                if (!f1Active) return null; // Never retry BVM02!
            }
        }
        synchronized (triedServiceOverrides) {
            for (int i = 0; i < svcs.size(); i++) {
                String s = Elm327BrakeProtocol.normUuid(svcs.get(i));
                if (s.isEmpty() || "1800".equals(s) || "1801".equals(s) || "180a".equals(s) || "180f".equals(s)) {
                    continue;
                }
                if (!triedServiceOverrides.contains(s)) {
                    triedServiceOverrides.add(s);
                    return s;
                }
            }
        }
        return null;
    }

    private static String safeUpper(String s) {
        return s == null ? "" : s.trim().toUpperCase(Locale.US);
    }

    private static String shortName(String name, String mac) {
        if (name != null && !name.trim().isEmpty()) {
            return name.trim();
        }
        return mac;
    }

    private static void sleepQuiet(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
        }
    }
}
