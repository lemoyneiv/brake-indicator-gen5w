package dev.lemoyne.brakeindicator;

import hmgapi.client.Ble;

import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ConnectException;
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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Dual-transport OBD-II IGMP (`770` -> `778`, UDS `22 BC06`) brake-lamp reader built on
 * HMG-API's native Bluetooth Low Energy central (`"ble": {"central": true}`, {@link Ble}):
 *  1. Coexistence-first loopback check: when {@code obd-gen5w}'s ELM327 multiplexer bridge
 *     ({@code 127.0.0.1:<port>}, default 35000) is active and answering {@code ATI}, yields
 *     to {@code 127.0.0.1:<port>} so two apps never interleave stateful {@code ATSH} CAN headers
 *     on a shared {@code @hmgapi-ble} GATT link. Uses pure {@code lo} loopback (never touches
 *     {@code wlan0} or wireless projection).
 *  2. Direct HMG-API BLE central: connects directly via {@code @hmgapi-ble} ({@link Ble}) with
 *     16/128-bit GATT discovery, negotiated ATT MTU chunking, and strict LNEX BVM02 battery
 *     sensor exclusion + {@code PROTECT_CHAR} defense-in-depth.
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
    private final Map<String, Map<String, Integer>> discoveredCharsByMac = new LinkedHashMap<String, Map<String, Integer>>();

    private final HmgBleListener bleListener = new HmgBleListener();
    private final Object sessionOpenLock = new Object();
    private volatile Ble bleSession;

    private final ExecutorService cmdExecutor = Executors.newSingleThreadExecutor(new ThreadFactory() {
        @Override
        public Thread newThread(Runnable r) {
            Thread t = new Thread(r, "BrakeObdCmd");
            t.setDaemon(true);
            return t;
        }
    });

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

    // HMG-API @hmgapi-ble session state
    private volatile boolean bleBridgeReady = false;
    private volatile boolean bleScanning = false;
    private volatile int scanGeneration = 0;
    private volatile boolean bleConnecting = false;
    private volatile long bleConnectStartedMs = 0L;
    private volatile boolean bleConnected = false;
    private volatile boolean bleGattRejected = false;
    private volatile String bleServiceFilter = null;
    private volatile boolean pendingTargetedSvcRetry = false;
    private volatile long nextBleReconnectMs = 0L;
    private volatile int bleTimeoutStreak = 0;
    private volatile int lastWakeScanStreak = 0;
    private volatile int lastRecycledStreak = 0;
    private volatile long lastPairedAdvMs = 0L;
    private final Object pairedAdvLock = new Object();
    private volatile int bleChunkBytes = 20;
    private volatile Elm327BrakeProtocol.GattBinding bleBinding = null;
    private volatile List<Elm327BrakeProtocol.GattBinding> bleCandidates = Collections.emptyList();

    // Loopback TCP bridge coexistence state
    private volatile Socket activeBridgeSocket = null;
    private volatile boolean bridgeConnected = false;
    private volatile long bridgeRetryAfterMs = 0L;

    // Shared telemetry snapshot
    private volatile String statusSummary = "Unpaired (Scan BLE or 127.0.0.1:35000)";
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
        if (!testMode) {
            startCoordinatorLoop();
        }
    }

    /**
     * Deterministic factory for host-JVM unit testing of HMG-API's {@link Ble#attach}
     * streaming protocol + {@code 127.0.0.1} loopback bridge coexistence.
     */
    public static ObdBrakeClient createForTest(
            PairingStore store,
            InputStream serverToClientIn,
            OutputStream clientToServerOut,
            Closeable closeable,
            int fallbackPort) throws IOException {
        ObdBrakeClient client = new ObdBrakeClient(store, true);
        client.bridgePort = fallbackPort;
        if (serverToClientIn != null && clientToServerOut != null) {
            client.bleSession = Ble.attach(serverToClientIn, clientToServerOut, closeable, client.bleListener);
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
            runAsyncOrInline(new Runnable() {
                @Override
                public void run() {
                    disconnectActiveTransports("OBD disabled in settings", true);
                }
            });
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

    public Ble getBleSessionForTest() {
        return bleSession;
    }

    public void startBleScan() {
        if (!enabled) return;
        synchronized (discovered) {
            discovered.clear();
        }
        final int gen = ++scanGeneration;
        statusSummary = "Scanning BLE via @hmgapi-ble...";
        notifyListeners();

        runAsyncOrInline(new Runnable() {
            @Override
            public void run() {
                Ble s = ensureBleSessionOpen();
                if (s != null) {
                    s.startScan();
                } else {
                    statusSummary = "@hmgapi-ble unavailable (check Bluetooth & HMG-API)";
                    notifyListeners();
                }
            }
        });

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
        runAsyncOrInline(new Runnable() {
            @Override
            public void run() {
                Ble s = bleSession;
                if (s != null && s.isOpen()) {
                    s.stopScan();
                }
                bleScanning = false;
                notifyListeners();
            }
        });
    }

    public boolean pairAdapter(String mac, String name, final int addrType) {
        if (mac == null || mac.trim().isEmpty()) return false;
        if (Elm327BrakeProtocol.isLikelyBvm02Device(name)) {
            statusSummary = "Refused BVM02 battery sensor (safety guard)";
            notifyListeners();
            return false;
        }
        final String cleanMac = safeUpper(mac);
        final String cleanName = (name == null || name.trim().isEmpty()) ? "BLE OBD-II" : name.trim();
        final String prevMac = this.pairedMac;

        closeBridgeSocketQuiet();
        this.pairedMac = cleanMac;
        this.pairedName = cleanName;
        this.pairedAddrType = addrType;
        if (store != null) {
            store.save(cleanMac, cleanName, addrType);
        }
        resetBleRetryState();
        scanGeneration++;
        bleScanning = false;

        runAsyncOrInline(new Runnable() {
            @Override
            public void run() {
                Ble s = ensureBleSessionOpen();
                if (s != null) {
                    if (!prevMac.isEmpty() && !prevMac.equals(cleanMac)) {
                        s.disconnectDevice(prevMac);
                    }
                    s.stopScan();
                    if (enabled && (testMode || !probeActiveLoopbackBridge(bridgePort))) {
                        bleConnecting = true;
                        statusSummary = "Connecting @hmgapi-ble (" + shortName(cleanName, cleanMac) + ")...";
                        s.connectDevice(cleanMac, addrType, null);
                    }
                }
                notifyListeners();
            }
        });
        notifyListeners();
        return true;
    }

    public void unpairAdapter() {
        final String oldMac = pairedMac;
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
        if (!bridgeConnected) {
            latestSample = BrakeEvaluator.ObdSample.disconnected();
            statusSummary = "Unpaired (127.0.0.1:" + bridgePort + " bridge active if running)";
        }
        runAsyncOrInline(new Runnable() {
            @Override
            public void run() {
                Ble s = bleSession;
                if (!oldMac.isEmpty() && s != null && s.isOpen()) {
                    s.disconnectDevice(oldMac);
                }
                notifyListeners();
            }
        });
        notifyListeners();
    }

    public void requestReconnect() {
        resetBleRetryState();
        bridgeRetryAfterMs = 0L;
        closeBridgeSocketQuiet();
        final String targetMac = pairedMac;
        final int targetAddrType = pairedAddrType;
        runAsyncOrInline(new Runnable() {
            @Override
            public void run() {
                Ble s = bleSession;
                if (!targetMac.isEmpty() && s != null && s.isOpen()) {
                    bleConnected = false;
                    bleConnecting = false;
                    s.disconnectDevice(targetMac);
                    if (testMode && enabled) {
                        bleConnecting = true;
                        statusSummary = "Reconnecting @hmgapi-ble...";
                        s.connectDevice(targetMac, targetAddrType, null);
                    }
                }
                notifyListeners();
            }
        });
        notifyListeners();
    }

    public void close() {
        if (!stopped.compareAndSet(false, true)) return;
        disconnectActiveTransports("Stopped", true);
        cmdExecutor.shutdownNow();
    }

    private void runAsyncOrInline(Runnable task) {
        if (testMode) {
            task.run();
        } else if (!stopped.get()) {
            try {
                cmdExecutor.execute(task);
            } catch (Throwable ignored) {}
        }
    }

    private void disconnectActiveTransports(String status, boolean closeBleSession) {
        closeBridgeSocketQuiet();
        Ble s = bleSession;
        if (s != null) {
            if (!pairedMac.isEmpty() && s.isOpen()) {
                s.disconnectDevice(pairedMac);
            }
            if (closeBleSession) {
                bleSession = null;
                bleBridgeReady = false;
                s.close();
            }
        }
        bleConnected = false;
        bleConnecting = false;
        bleScanning = false;
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
        bleGattRejected = false;
        bleServiceFilter = null;
        pendingTargetedSvcRetry = false;
        bleTimeoutStreak = 0;
        lastWakeScanStreak = 0;
        lastRecycledStreak = 0;
        nextBleReconnectMs = 0L;
        synchronized (triedServiceOverrides) {
            triedServiceOverrides.clear();
        }
    }

    int effectiveConnectAddrType() {
        return (bleTimeoutStreak >= 1) ? -1 : pairedAddrType;
    }

    void recycleBleSession() {
        synchronized (sessionOpenLock) {
            Ble existing = bleSession;
            bleSession = null;
            bleBridgeReady = false;
            bleScanning = false;
            bleConnected = false;
            bleConnecting = false;
            if (existing != null) {
                existing.close();
            }
        }
    }

    boolean performWakeScanBeforeConnect(Ble s, long maxWaitMs) {
        if (s == null || !s.isOpen() || pairedMac.isEmpty()) return false;
        long startMs = System.currentTimeMillis();
        statusSummary = "Waking BLE adapter (" + shortName(pairedName, pairedMac) + ")...";
        notifyListeners();
        s.startScan();
        long deadline = startMs + Math.max(100L, maxWaitMs);
        boolean seen = false;
        synchronized (pairedAdvLock) {
            while (!stopped.get() && enabled && s.isOpen()) {
                if (lastPairedAdvMs >= startMs) {
                    seen = true;
                    break;
                }
                long rem = deadline - System.currentTimeMillis();
                if (rem <= 0L) break;
                try {
                    pairedAdvLock.wait(rem);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        if (s.isOpen()) {
            s.stopScan();
        }
        bleScanning = false;
        sleepQuiet(60L);
        return seen;
    }

    private Ble ensureBleSessionOpen() {
        Ble existing = bleSession;
        if (existing != null && existing.isOpen()) {
            bleBridgeReady = true;
            return existing;
        }
        if (testMode || stopped.get() || !enabled) {
            return existing;
        }
        synchronized (sessionOpenLock) {
            existing = bleSession;
            if (existing != null && existing.isOpen()) {
                bleBridgeReady = true;
                return existing;
            }
            try {
                Ble opened = Ble.connect(bleListener);
                this.bleSession = opened;
                this.bleBridgeReady = true;
                return opened;
            } catch (IOException e) {
                this.bleSession = null;
                this.bleBridgeReady = false;
                return null;
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
     * Master connection & coexistence state machine:
     *  1. If {@code obd-gen5w}'s {@code 127.0.0.1:<bridgePort>} ELM327 TCP bridge is listening
     *     and answers {@code ATI}, poll via {@code 127.0.0.1:<bridgePort>} so {@code obd-gen5w}
     *     arbitrates ELM327 {@code ATSH} CAN headers without GATT write interleaving.
     *  2. Otherwise connect directly to the paired BLE OBD-II adapter via HMG-API's built-in
     *     {@code @hmgapi-ble} central ({@link Ble}).
     */
    private void coordinatorLoop() {
        while (!stopped.get()) {
            if (!enabled) {
                sleepQuiet(1000L);
                continue;
            }

            // Priority 1: Active @hmgapi-ble GATT session
            if (bleConnected && bleBinding != null) {
                runBlePollSession();
                continue;
            }

            long now = System.currentTimeMillis();

            // Safety watchdog if a BLE connect request never received a terminal callback
            if (bleConnecting && bleConnectStartedMs > 0L && now - bleConnectStartedMs > 18000L) {
                bleConnecting = false;
                bleTimeoutStreak++;
                Ble cur = bleSession;
                if (cur != null && cur.isOpen() && !pairedMac.isEmpty()) {
                    cur.disconnectDevice(pairedMac);
                }
                nextBleReconnectMs = now + 1500L;
            }

            // Priority 2: Coexistence check — yield to obd-gen5w's 127.0.0.1:<bridgePort> if active
            if (!bleScanning && !bleConnecting && now >= bridgeRetryAfterMs) {
                boolean ranBridge = tryRunLoopbackBridgeSession(bridgePort);
                if (ranBridge) {
                    nextBleReconnectMs = 0L;
                    continue;
                }
            }

            // Recycle @hmgapi-ble stream after every 3 consecutive connection failures
            if (!testMode
                    && bleTimeoutStreak >= 3
                    && (bleTimeoutStreak % 3 == 0)
                    && lastRecycledStreak != bleTimeoutStreak
                    && !bleScanning
                    && !bleConnecting
                    && now >= nextBleReconnectMs) {
                lastRecycledStreak = bleTimeoutStreak;
                recycleBleSession();
            }

            // Priority 3: Direct BLE via HMG-API's @hmgapi-ble central
            Ble s = (!pairedMac.isEmpty() || bleScanning) ? ensureBleSessionOpen() : bleSession;
            boolean canTryDirectBle = s != null
                    && s.isOpen()
                    && !pairedMac.isEmpty()
                    && !bleGattRejected
                    && now >= nextBleReconnectMs;

            if (canTryDirectBle) {
                if (!bleConnecting) {
                    if (!testMode
                            && bleTimeoutStreak >= 2
                            && (bleTimeoutStreak % 2 == 0)
                            && lastWakeScanStreak != bleTimeoutStreak) {
                        lastWakeScanStreak = bleTimeoutStreak;
                        performWakeScanBeforeConnect(s, 2200L);
                        if (!s.isOpen() || !enabled || stopped.get()) {
                            continue;
                        }
                    }
                    bleConnecting = true;
                    bleConnectStartedMs = System.currentTimeMillis();
                    statusSummary = "Connecting @hmgapi-ble (" + shortName(pairedName, pairedMac) + ")...";
                    s.connectDevice(pairedMac, effectiveConnectAddrType(), bleServiceFilter);
                    notifyListeners();
                }
                sleepQuiet(400L);
                continue;
            }

            if (!bleConnected && !bleConnecting && !bridgeConnected) {
                if (pairedMac.isEmpty()) {
                    statusSummary = bleBridgeReady
                            ? "Unpaired (Scan BLE or run 127.0.0.1:" + bridgePort + ")"
                            : "Scan BLE (@hmgapi-ble) or run 127.0.0.1:" + bridgePort;
                }
                notifyListeners();
            }
            sleepQuiet(2500L);
        }
    }

    // ================= DIRECT @HMGAPI-BLE POLLING =================

    private void runBlePollSession() {
        Elm327BrakeProtocol.GattBinding initial = bleBinding;
        Ble s = bleSession;
        if (initial == null || s == null || !s.isOpen()) return;

        awaitNotifyReady(initial.notifyUuid, 2200L);
        sleepQuiet(100L);

        String ver = probeBleCandidate(initial);
        List<Elm327BrakeProtocol.GattBinding> all = bleCandidates;
        if ((ver == null || ver.isEmpty()) && all != null && all.size() > 1) {
            Map<String, Integer> charProps;
            synchronized (discoveredCharsByMac) {
                charProps = discoveredCharsByMac.get(pairedMac);
            }
            for (int i = 1; i < all.size() && bleConnected && enabled && !stopped.get(); i++) {
                Elm327BrakeProtocol.GattBinding next = all.get(i);
                synchronized (notifyReadyLock) {
                    lastNotifyReadyUuid = "";
                }
                enableCharacteristicStream(s, pairedMac, next.notifyUuid, charProps);
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
        long lastBridgeProbeMs = windowStartMs;

        while (bleConnected && bleBinding != null && enabled && !stopped.get()) {
            long nowCheck = System.currentTimeMillis();
            if (!testMode && nowCheck - lastBridgeProbeMs >= 10000L && nowCheck >= bridgeRetryAfterMs) {
                lastBridgeProbeMs = nowCheck;
                if (probeActiveLoopbackBridge(bridgePort)) {
                    statusSummary = "Yielding to active 127.0.0.1:" + bridgePort + " bridge...";
                    bleConnected = false;
                    s.disconnectDevice(pairedMac);
                    notifyListeners();
                    break;
                }
            }

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
                    s.disconnectDevice(pairedMac);
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
                statusSummary = "BLE @hmgapi-ble (" + shortName(pairedName, pairedMac) + ")";
                notifyListeners();
                sleepQuiet(15L);
            } else {
                noDataStreak++;
                if (noDataStreak >= 3) {
                    // Check ATI so parked IG OFF enters 5s sleep instead of keeping CAN awake
                    String alive = sendBleCommandBlocking("ATI", 1000L);
                    if (alive == null || alive.trim().isEmpty()) {
                        bleConnected = false;
                        s.disconnectDevice(pairedMac);
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

    private static void enableCharacteristicStream(
            Ble ble,
            String mac,
            String notifyUuid,
            Map<String, Integer> charProps) {
        if (ble == null || !ble.isOpen()) return;
        String norm = Elm327BrakeProtocol.normUuid(notifyUuid);
        Integer props = charProps != null ? charProps.get(norm) : null;
        boolean indicateOnly = props != null
                && (props.intValue() & Elm327BrakeProtocol.PROP_NOTIFY) == 0
                && (props.intValue() & Elm327BrakeProtocol.PROP_INDICATE) != 0;
        if (indicateOnly) {
            ble.enableIndications(mac, notifyUuid);
        } else {
            ble.enableNotifications(mac, notifyUuid);
        }
    }

    private void awaitNotifyReady(String notifyUuid, long timeoutMs) {
        String norm = Elm327BrakeProtocol.normUuid(notifyUuid);
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
        Ble s = bleSession;
        if (!bleConnected || b == null || mac.isEmpty() || s == null || !s.isOpen()) {
            return null;
        }
        synchronized (blePromptLock) {
            latestBlePrompt = null;
            blePromptBuf.reset();
        }
        boolean withResponse = !b.writeWithoutResponse;
        List<byte[]> chunks = Elm327BrakeProtocol.encodeCommandChunks(cmd, bleChunkBytes);
        for (int i = 0; i < chunks.size(); i++) {
            s.writeCharacteristic(mac, b.writeUuid, chunks.get(i), withResponse);
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

    // ================= 127.0.0.1 LOOPBACK TCP BRIDGE COEXISTENCE =================

    /**
     * Sub-millisecond when nothing is listening (`ECONNREFUSED`), or verifies live `ATI`
     * answer when `obd-gen5w`'s loopback ELM327 multiplexer is open on `127.0.0.1:<port>`.
     */
    public boolean probeActiveLoopbackBridge(int port) {
        Socket sock = new Socket();
        try {
            InetAddress loopback = InetAddress.getByName("127.0.0.1");
            sock.connect(new InetSocketAddress(loopback, port), 250);
            sock.setTcpNoDelay(true);
            sock.setSoTimeout(700);
            Elm327BrakeProtocol.PromptBuffer promptBuf = new Elm327BrakeProtocol.PromptBuffer();
            String ver = sendSocketElmCommand(sock.getInputStream(), sock.getOutputStream(), promptBuf, "ATI", 700L);
            if (ver != null && !ver.trim().isEmpty()) {
                bridgeRetryAfterMs = 0L;
                return true;
            }
            bridgeRetryAfterMs = System.currentTimeMillis() + 12000L;
            return false;
        } catch (ConnectException ce) {
            return false;
        } catch (IOException ioe) {
            return false;
        } finally {
            try { sock.close(); } catch (Throwable ignored) {}
        }
    }

    /**
     * Connects strictly to `127.0.0.1:<port>` (pure kernel loopback `lo`, zero Wi-Fi/projection impact)
     * and polls `22BC06`. Publicly accessible for deterministic unit tests.
     */
    public boolean tryRunLoopbackBridgeSession(int port) {
        Socket sock = new Socket();
        try {
            InetAddress loopback = InetAddress.getByName("127.0.0.1");
            sock.connect(new InetSocketAddress(loopback, port), 350);
            sock.setTcpNoDelay(true);
            sock.setSoTimeout(1200);

            InputStream in = sock.getInputStream();
            OutputStream out = sock.getOutputStream();
            Elm327BrakeProtocol.PromptBuffer promptBuf = new Elm327BrakeProtocol.PromptBuffer();

            // Verify live ELM327 response before claiming bridge session
            String ver = sendSocketElmCommand(in, out, promptBuf, "ATI", 1000L);
            if (ver == null || ver.trim().isEmpty()) {
                bridgeRetryAfterMs = System.currentTimeMillis() + 12000L;
                return false;
            }

            // Ensure our direct @hmgapi-ble peripheral session is detached while using obd-gen5w's bridge
            Ble s = bleSession;
            if (bleConnected && s != null && s.isOpen() && !pairedMac.isEmpty()) {
                bleConnected = false;
                s.disconnectDevice(pairedMac);
            }

            activeBridgeSocket = sock;
            bridgeConnected = true;
            bridgeRetryAfterMs = 0L;
            statusSummary = "Loopback Bridge (127.0.0.1:" + port + ")";
            notifyListeners();

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
                    statusSummary = "Bridge 127.0.0.1:" + port;
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
        } catch (ConnectException ce) {
            return false;
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

    // ================= HMG-API @HMGAPI-BLE LISTENER =================

    private final class HmgBleListener extends Ble.SimpleListener {

        @Override
        public void onSessionReady() {
            bleBridgeReady = true;
            notifyListeners();
        }

        @Override
        public void onScanStateChanged(boolean scanning) {
            bleScanning = scanning;
            notifyListeners();
        }

        @Override
        public void onAdvertisement(Ble.AdvRecord record) {
            if (record == null || record.mac == null) return;
            String upper = safeUpper(record.mac);
            if (upper.equals(pairedMac)) {
                if (record.addrType == 0 || record.addrType == 1) {
                    pairedAddrType = record.addrType;
                    if (store != null) {
                        store.save(pairedMac, pairedName, record.addrType);
                    }
                }
                nextBleReconnectMs = 0L;
                synchronized (pairedAdvLock) {
                    lastPairedAdvMs = System.currentTimeMillis();
                    pairedAdvLock.notifyAll();
                }
            }
            String localName = (record.localName != null && !record.localName.isEmpty())
                    ? record.localName
                    : "BLE Device";
            List<String> uuids = record.serviceUuids != null
                    ? record.serviceUuids
                    : Collections.<String>emptyList();
            boolean isBvm = Elm327BrakeProtocol.isLikelyBvm02Device(localName);
            boolean isObd = Elm327BrakeProtocol.isLikelyObdAdapter(localName, uuids);
            DiscoveredDevice dev = new DiscoveredDevice(upper, record.addrType, record.rssi, localName, isObd, isBvm);
            synchronized (discovered) {
                discovered.put(upper, dev);
            }
            notifyListeners();
        }

        @Override
        public void onConnectionStateChanged(String mac, boolean connected, int handleOrReason) {
            if (mac == null || !mac.equalsIgnoreCase(pairedMac)) return;
            String upper = safeUpper(mac);
            if (connected) {
                bleConnecting = true;
                bleConnectStartedMs = System.currentTimeMillis();
                synchronized (discoveredServicesByMac) {
                    discoveredServicesByMac.remove(upper);
                }
                synchronized (discoveredCharsByMac) {
                    discoveredCharsByMac.remove(upper);
                }
                notifyListeners();
                return;
            }

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
                Ble s = bleSession;
                if (s != null && s.isOpen() && enabled) {
                    bleConnecting = true;
                    bleConnectStartedMs = System.currentTimeMillis();
                    s.connectDevice(pairedMac, effectiveConnectAddrType(), bleServiceFilter);
                }
                return;
            }
            bleTimeoutStreak++;
            long backoff = Math.min(15000L, 2500L * bleTimeoutStreak);
            nextBleReconnectMs = System.currentTimeMillis() + backoff;
            if (!bridgeConnected && !bleGattRejected) {
                statusSummary = String.format(Locale.US, "BLE disconnected (0x%02x)",
                        Integer.valueOf(handleOrReason & 0xFF));
            }
            notifyListeners();
        }

        @Override
        public void onServiceDiscovered(String mac, String serviceUuid, int startHandle, int endHandle) {
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
        public void onCharacteristicDiscovered(
                String mac,
                String serviceUuid,
                String charUuid,
                int valueHandle,
                int properties,
                int cccdHandle) {
            String effective = (mac != null && !mac.isEmpty()) ? safeUpper(mac) : pairedMac;
            if (effective.isEmpty() || charUuid == null) return;
            String norm = Elm327BrakeProtocol.normUuid(charUuid);
            synchronized (discoveredCharsByMac) {
                Map<String, Integer> map = discoveredCharsByMac.get(effective);
                if (map == null) {
                    map = new LinkedHashMap<String, Integer>();
                    discoveredCharsByMac.put(effective, map);
                }
                map.put(norm, Integer.valueOf(properties));
            }
        }

        @Override
        public void onGattReady(String mac, int mtu) {
            String effective = (mac != null && !mac.isEmpty()) ? safeUpper(mac) : pairedMac;
            if (effective.isEmpty() || !effective.equalsIgnoreCase(pairedMac)) return;
            bleConnecting = false;

            Map<String, Integer> charProps;
            synchronized (discoveredCharsByMac) {
                Map<String, Integer> raw = discoveredCharsByMac.get(pairedMac);
                charProps = raw == null
                        ? Collections.<String, Integer>emptyMap()
                        : new LinkedHashMap<String, Integer>(raw);
            }

            Ble s = bleSession;
            // Defense-in-depth LNEX BVM02 read-only fff1 protection before resolving bindings
            Integer p4 = charProps.get("fff4");
            Integer p1 = charProps.get("fff1");
            if (p4 != null && (p4.intValue() & (Elm327BrakeProtocol.PROP_NOTIFY | Elm327BrakeProtocol.PROP_INDICATE)) != 0) {
                boolean f1Active = p1 != null && (p1.intValue() & 0x3C) != 0;
                if (!f1Active && s != null && s.isOpen()) {
                    s.protectCharacteristic(pairedMac, "fff1");
                }
            }

            List<Elm327BrakeProtocol.GattBinding> cands = Elm327BrakeProtocol.resolveAdapterBindings(charProps);
            if (cands.isEmpty()) {
                List<String> svcs;
                synchronized (discoveredServicesByMac) {
                    List<String> raw = discoveredServicesByMac.get(pairedMac);
                    svcs = raw == null ? Collections.<String>emptyList() : new ArrayList<String>(raw);
                }
                String nextSvc = pickNextServiceOverride(svcs, charProps);
                if (nextSvc != null && s != null && s.isOpen()) {
                    bleServiceFilter = nextSvc;
                    pendingTargetedSvcRetry = true;
                    s.disconnectDevice(pairedMac);
                    return;
                }
                bleGattRejected = true;
                bleConnected = false;
                statusSummary = "Incompatible GATT (safety rejected)";
                if (s != null && s.isOpen()) {
                    s.disconnectDevice(pairedMac);
                }
                notifyListeners();
                return;
            }

            closeBridgeSocketQuiet();
            Elm327BrakeProtocol.GattBinding primary = cands.get(0);
            bleCandidates = cands;
            bleBinding = primary;
            bleChunkBytes = Math.min(180, Math.max(20, mtu - 3));
            bleConnected = true;
            bleTimeoutStreak = 0;
            lastWakeScanStreak = 0;
            lastRecycledStreak = 0;
            statusSummary = "BLE GATT Ready (" + primary.profileName + ")";
            synchronized (notifyReadyLock) {
                lastNotifyReadyUuid = "";
            }
            enableCharacteristicStream(s, pairedMac, primary.notifyUuid, charProps);
            notifyListeners();
        }

        @Override
        public void onMtuChanged(String mac, int mtu) {
            if ((mac == null || mac.equalsIgnoreCase(pairedMac)) && mtu >= 23) {
                bleChunkBytes = Math.min(180, Math.max(20, mtu - 3));
            }
        }

        @Override
        public void onNotifyStateChanged(String mac, String charUuid, boolean enabledFlag) {
            if ((mac == null || mac.equalsIgnoreCase(pairedMac)) && charUuid != null && enabledFlag) {
                synchronized (notifyReadyLock) {
                    lastNotifyReadyUuid = Elm327BrakeProtocol.normUuid(charUuid);
                    notifyReadyLock.notifyAll();
                }
            }
        }

        @Override
        public void onCharacteristicChanged(String mac, String charUuid, byte[] value) {
            Elm327BrakeProtocol.GattBinding b = bleBinding;
            if (value == null || b == null) return;
            if (mac != null && !mac.isEmpty() && !mac.equalsIgnoreCase(pairedMac)) return;
            if (!Elm327BrakeProtocol.normUuid(b.notifyUuid).equals(Elm327BrakeProtocol.normUuid(charUuid))) {
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
        public void onError(String mac, String error) {
            if (mac != null && !mac.isEmpty() && !mac.equalsIgnoreCase(pairedMac)) {
                return;
            }
            String err = error == null ? "UNKNOWN" : error.trim();
            if (err.startsWith("CONNECT_")
                    || err.startsWith("MAX_CONNECTIONS")
                    || err.startsWith("SHARED_ACL")
                    || err.startsWith("GATT_DISCOVERY")
                    || err.startsWith("NOT_CONNECTED")) {
                bleConnecting = false;
                bleConnected = false;
                bleBinding = null;
                bleTimeoutStreak++;
                long backoff = Math.min(15000L, 2500L * bleTimeoutStreak);
                nextBleReconnectMs = System.currentTimeMillis() + backoff;
                if (!bridgeConnected && !bleGattRejected) {
                    statusSummary = err.startsWith("CONNECT_ALREADY_EXISTS")
                            ? "BLE link busy; resetting (" + err + ")"
                            : "BLE retry pending (" + err + ")";
                }
                if (err.startsWith("CONNECT_ALREADY_EXISTS")) {
                    Ble s = bleSession;
                    if (s != null && s.isOpen() && !pairedMac.isEmpty()) {
                        s.disconnectDevice(pairedMac);
                    }
                }
                synchronized (blePromptLock) {
                    blePromptLock.notifyAll();
                }
                synchronized (notifyReadyLock) {
                    notifyReadyLock.notifyAll();
                }
                notifyListeners();
            }
        }

        @Override
        public void onSessionClosed(String reason) {
            bleSession = null;
            bleBridgeReady = false;
            bleScanning = false;
            bleConnected = false;
            bleConnecting = false;
            bleBinding = null;
            if (!bridgeConnected && enabled && !stopped.get()) {
                statusSummary = "BLE session closed (" + reason + ")";
            }
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
