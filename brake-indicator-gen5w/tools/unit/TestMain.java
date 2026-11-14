package dev.lemoyne.brakeindicator;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.Closeable;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.lang.reflect.Method;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Host JVM unit tests for v0.7.1:
 * - HmgVehiclePoller (fast <2ms LFD1 vehicle reads + resilient 90ms legacy KitKat Hub stall recovery)
 * - Elm327BrakeProtocol (IGMP 770->778 22BC06 multi-frame ISO-TP decoder with 0x01 pedal switch vs 0x2C regen,
 *   LNEX BVM02 exclusion, VIN/write safety)
 * - ObdBrakeClient (HMG-API native @hmgapi-ble central session + PROTECT_CHAR guard + 127.0.0.1 bridge coexistence)
 * - IndicatorConfig & BrakeEvaluator (poll-aligned 650 ms legacy & 50-250 ms HMG-API speed handling,
 *   settled L0..L3 coasting chord + i-Pedal & SCC auto-detection, E-GMP R13-H regen hysteresis, standstill hold)
 * - HevRegenProbe (strict VIN/MAC/phone/mutator safety filter, 0xFA USM sentinel rejection, baseline diff)
 * - TelemetryRecorder & UsbExporter (confusion matrix, Pedal vs Pure Regen recall, i-Pedal labels, atomic USB export)
 */
public final class TestMain {

    private static int passed;
    private static int failed;

    public static void main(String[] args) throws Exception {
        testHmgVehiclePollerFastAndLegacyHubStall();
        testElm327IgmpBrakeDecoding();
        testElm327SafetyAndBvm02Exclusion();
        testObdBrakeClientBleAndLoopbackFallback();
        testIndicatorConfigModesAndLegacyCompatibility();
        testBrakeEvaluatorCachedSpeedAndObdModes();
        testBrakeEvaluatorLegacyHub650msPollAlignment();
        testBrakeEvaluatorCoastingRegenAndIpedalInference();
        testBrakeEvaluatorStandstillHoldAndRelease();
        testBrakeEvaluatorV060SccAutoHoldAeroAndRelease();
        testHevRegenProbeSafetyAndBaselineDiff();
        testTelemetryRecorderScoringAndCsv();
        testUsbExporterMountsAndAtomicWrite();

        System.out.println(passed + " ok, " + failed + " failed");
        if (failed > 0) {
            System.exit(1);
        }
    }

    private static void testHmgVehiclePollerFastAndLegacyHubStall() throws Exception {
        final String[] names = {"speed", "brake.pressure", "pedal.accel"};
        final byte[] oPayload = "speed 54.2\nbrake.pressure 0\npedal.accel 12.5\n".getBytes("UTF-8");

        // 1. Patched Hub fast path: 'O' frame followed immediately by 'X'(0)
        ByteArrayOutputStream wireFast = new ByteArrayOutputStream();
        writeLfdFrame(wireFast, (byte) 'O', oPayload);
        writeLfdFrame(wireFast, (byte) 'X', new byte[]{0, 0, 0, 0});

        ByteArrayOutputStream sentReq = new ByteArrayOutputStream();
        final int[] adjustedTimeout = {-1};
        HmgVehiclePoller.PollResult fastRes = HmgVehiclePoller.readVehicleOverStreams(
                new ByteArrayInputStream(wireFast.toByteArray()),
                sentReq,
                new HmgVehiclePoller.TimeoutSetter() {
                    @Override
                    public void setReadTimeoutMs(int timeoutMs) {
                        adjustedTimeout[0] = timeoutMs;
                    }
                },
                names,
                90);
        byte[] reqBytes = sentReq.toByteArray();
        eq("LFD1 header magic", "LFD1", new String(reqBytes, 0, 4, "UTF-8"));
        eq("Patched Hub fast path ok=true", Boolean.TRUE, Boolean.valueOf(fastRes.ok));
        eq("Patched Hub fast path legacyHubStall=false", Boolean.FALSE, Boolean.valueOf(fastRes.legacyHubStall));
        eq("Patched Hub parsed speed=54.2", "54.2", fastRes.readings.get("speed"));
        eq("Patched Hub shortened post-output timeout to 90ms", Integer.valueOf(90), Integer.valueOf(adjustedTimeout[0]));

        // 2. Unpatched KitKat Hub stall path: 'O' arrives immediately, then read() throws SocketTimeoutException
        // before the 3,000 ms hmgapid EOF timeout! Must recover all readings with legacyHubStall=true.
        ByteArrayOutputStream wireOnlyO = new ByteArrayOutputStream();
        writeLfdFrame(wireOnlyO, (byte) 'O', oPayload);
        final byte[] onlyOBytes = wireOnlyO.toByteArray();
        InputStream stallingIn = new InputStream() {
            int pos = 0;
            @Override
            public int read() throws IOException {
                if (pos < onlyOBytes.length) {
                    return onlyOBytes[pos++] & 0xff;
                }
                throw new SocketTimeoutException("simulated 90ms post-O grace timeout");
            }
        };
        HmgVehiclePoller.PollResult stallRes = HmgVehiclePoller.readVehicleOverStreams(
                stallingIn, new ByteArrayOutputStream(), null, names, 90);
        eq("Legacy KitKat Hub stall recovers ok=true", Boolean.TRUE, Boolean.valueOf(stallRes.ok));
        eq("Legacy KitKat Hub flags legacyHubStall=true", Boolean.TRUE, Boolean.valueOf(stallRes.legacyHubStall));
        eq("Legacy KitKat Hub recovers speed=54.2", "54.2", stallRes.readings.get("speed"));
        eq("Legacy KitKat Hub recovers pedal.accel=12.5", "12.5", stallRes.readings.get("pedal.accel"));

        // 3. Unpatched KitKat Hub if 'X'(1) arrives after 'O': still recovers readings with legacyHubStall=true
        ByteArrayOutputStream wireExit1 = new ByteArrayOutputStream();
        writeLfdFrame(wireExit1, (byte) 'O', oPayload);
        writeLfdFrame(wireExit1, (byte) 'X', new byte[]{1, 0, 0, 0});
        HmgVehiclePoller.PollResult exit1Res = HmgVehiclePoller.readVehicleOverStreams(
                new ByteArrayInputStream(wireExit1.toByteArray()), new ByteArrayOutputStream(), null, names, 90);
        eq("Legacy Hub exit=1 after 'O' recovers ok=true", Boolean.TRUE, Boolean.valueOf(exit1Res.ok));
        eq("Legacy Hub exit=1 flags legacyHubStall=true", Boolean.TRUE, Boolean.valueOf(exit1Res.legacyHubStall));
    }

    private static void writeLfdFrame(ByteArrayOutputStream out, byte type, byte[] payload) {
        out.write(type);
        int len = payload.length;
        out.write(len & 0xff);
        out.write((len >> 8) & 0xff);
        out.write((len >> 16) & 0xff);
        out.write((len >> 24) & 0xff);
        out.write(payload, 0, len);
    }

    private static void testElm327IgmpBrakeDecoding() {
        // 1. Real 11-byte multi-frame EV6 / Ioniq 5 IGMP 22BC06 OFF response (rawCode = 0x00)
        String offResp = "778100B62BC0670400003\r7782100000000000000\r";
        Elm327BrakeProtocol.IgmpBrakeReading off = Elm327BrakeProtocol.decodeIgmpBrakeResponse(offResp);
        eq("IGMP OFF decoded", Boolean.TRUE, Boolean.valueOf(off != null));
        eq("IGMP OFF rawCode=0", Integer.valueOf(0), Integer.valueOf(off.rawCode));
        eq("IGMP OFF brakeLampLit=false", Boolean.FALSE, Boolean.valueOf(off.brakeLampLit));
        eq("IGMP OFF pedalBit=false", Boolean.FALSE, Boolean.valueOf(off.pedalBit));

        // 2. MY25 8-byte multi-frame IGMP 22BC06 OFF response
        String my25OffResp = "778100862BC06004000\r7782100000000000000\r";
        Elm327BrakeProtocol.IgmpBrakeReading my25Off = Elm327BrakeProtocol.decodeIgmpBrakeResponse(my25OffResp);
        eq("MY25 8-byte OFF decoded", Boolean.TRUE, Boolean.valueOf(my25Off != null && !my25Off.brakeLampLit));

        // 3. Pure Regen / i-Pedal / SCC / Auto-Hold brake lamps ON without pedal (bit 0 = 0):
        //    4 (0x04), 36 (0x24), 40 (0x28), 44 (0x2C) -> brakeLampLit=true, pedalBit=false
        int[] pureRegenCodes = {4, 36, 40, 44};
        for (int i = 0; i < pureRegenCodes.length; i++) {
            int code = pureRegenCodes[i];
            String hex = String.format("%02X", Integer.valueOf(code));
            String resp = "778100B62BC0670400003\r77821" + hex + "000000000000\r";
            Elm327BrakeProtocol.IgmpBrakeReading r = Elm327BrakeProtocol.decodeIgmpBrakeResponse(resp);
            eq("IGMP Pure Regen/SCC code " + code + " lit=true, pedalBit=false",
                    Boolean.TRUE, Boolean.valueOf(r != null && r.rawCode == code && r.brakeLampLit && !r.pedalBit));
        }

        // 4. Physical foot-pedal switch active codes (bit 0 = 1):
        //    37 (0x25), 41 (0x29), 45 (0x2D) -> brakeLampLit=true, pedalBit=true
        int[] pedalCodes = {37, 41, 45};
        for (int i = 0; i < pedalCodes.length; i++) {
            int code = pedalCodes[i];
            String hex = String.format("%02X", Integer.valueOf(code));
            String resp = "778100B62BC0670400003\r77821" + hex + "000000000000\r";
            Elm327BrakeProtocol.IgmpBrakeReading r = Elm327BrakeProtocol.decodeIgmpBrakeResponse(resp);
            eq("IGMP Pedal switch code " + code + " lit=true, pedalBit=true",
                    Boolean.TRUE, Boolean.valueOf(r != null && r.rawCode == code && r.brakeLampLit && r.pedalBit));
        }

        // 5. Reject asleep/placeholder IGMP frames padded with 0xAA
        String asleepAa = "778100B62BC06AAAAAA\r77821AAAAAAAAAA0000\r";
        eq("IGMP 0xAA asleep frame rejected", null, Elm327BrakeProtocol.decodeIgmpBrakeResponse(asleepAa));

        // 6. Chunked 20-byte BLE GATT notification reassembly via PromptBuffer
        Elm327BrakeProtocol.PromptBuffer pb = new Elm327BrakeProtocol.PromptBuffer();
        List<String> c1 = pb.feed("778100B62BC06704000".getBytes(Elm327BrakeProtocol.ASCII));
        eq("PromptBuffer partial chunk -> 0 prompts", Integer.valueOf(0), Integer.valueOf(c1.size()));
        List<String> c2 = pb.feed("03\r778212C00000000>".getBytes(Elm327BrakeProtocol.ASCII));
        eq("PromptBuffer '>' completes 1 prompt", Integer.valueOf(1), Integer.valueOf(c2.size()));
        Elm327BrakeProtocol.IgmpBrakeReading chunked = Elm327BrakeProtocol.decodeIgmpBrakeResponse(c2.get(0));
        eq("Chunked BLE prompt decodes rawCode=44 pure regen",
                Boolean.TRUE, Boolean.valueOf(chunked != null && chunked.rawCode == 44 && chunked.brakeLampLit && !chunked.pedalBit));
    }

    private static void testElm327SafetyAndBvm02Exclusion() {
        // Strict VIN & write command blocking
        eq("ATSH770 allowed", Boolean.TRUE, Boolean.valueOf(Elm327BrakeProtocol.isSafeCommand("ATSH770")));
        eq("22BC06 allowed", Boolean.TRUE, Boolean.valueOf(Elm327BrakeProtocol.isSafeCommand("22 BC 06")));
        eq("VIN 0902 blocked", Boolean.FALSE, Boolean.valueOf(Elm327BrakeProtocol.isSafeCommand("0902")));
        eq("VIN 22F190 blocked", Boolean.FALSE, Boolean.valueOf(Elm327BrakeProtocol.isSafeCommand("22F190")));
        eq("VIN 1A90 blocked", Boolean.FALSE, Boolean.valueOf(Elm327BrakeProtocol.isSafeCommand("1A90")));
        eq("UDS WriteDataByIdentifier 2EBC06 blocked",
                Boolean.FALSE, Boolean.valueOf(Elm327BrakeProtocol.isSafeCommand("2EBC0600")));
        eq("ClearDTC 14FFFFFF blocked",
                Boolean.FALSE, Boolean.valueOf(Elm327BrakeProtocol.isSafeCommand("14FFFFFF")));

        // LNEX BVM02 battery sensor exclusion (name + GATT fingerprint)
        eq("BVM02 name flagged", Boolean.TRUE, Boolean.valueOf(Elm327BrakeProtocol.isLikelyBvm02Device("LNEX BVM02")));
        eq("BL-680A name flagged", Boolean.TRUE, Boolean.valueOf(Elm327BrakeProtocol.isLikelyBvm02Device("BL-680A")));
        eq("BVM02 never classified as OBD adapter even with FFF0 UUID",
                Boolean.FALSE,
                Boolean.valueOf(Elm327BrakeProtocol.isLikelyObdAdapter("BVM02", Arrays.asList("fff0"))));
        eq("OBDCheck BLE+ classified as OBD adapter",
                Boolean.TRUE,
                Boolean.valueOf(Elm327BrakeProtocol.isLikelyObdAdapter("OBDCheck BLE+", Arrays.asList("fff0"))));

        // BVM02 GATT table: fff1 READ-only (0x02), fff2 WRITE (0x08), fff4 NOTIFY (0x10) -> MUST reject!
        Map<String, Integer> bvmGatt = new LinkedHashMap<String, Integer>();
        bvmGatt.put("fff1", Integer.valueOf(Elm327BrakeProtocol.PROP_READ));
        bvmGatt.put("fff2", Integer.valueOf(Elm327BrakeProtocol.PROP_WRITE));
        bvmGatt.put("fff4", Integer.valueOf(Elm327BrakeProtocol.PROP_NOTIFY));
        eq("BVM02 GATT table rejected (0 bindings)",
                Integer.valueOf(0), Integer.valueOf(Elm327BrakeProtocol.resolveAdapterBindings(bvmGatt).size()));

        // Veepeak BLE GATT table: fff1 NOTIFY (0x10), fff2 WRITE_NO_RESP | WRITE (0x0C) -> accepted!
        Map<String, Integer> veepeakGatt = new LinkedHashMap<String, Integer>();
        veepeakGatt.put("fff1", Integer.valueOf(Elm327BrakeProtocol.PROP_NOTIFY));
        veepeakGatt.put("fff2", Integer.valueOf(Elm327BrakeProtocol.PROP_WRITE | Elm327BrakeProtocol.PROP_WRITE_NO_RESP));
        List<Elm327BrakeProtocol.GattBinding> vp = Elm327BrakeProtocol.resolveAdapterBindings(veepeakGatt);
        eq("Veepeak FFF0 resolved", Boolean.TRUE, Boolean.valueOf(!vp.isEmpty()));
        eq("Veepeak notify=fff1", "fff1", Elm327BrakeProtocol.normUuid(vp.get(0).notifyUuid));
        eq("Veepeak write=fff2", "fff2", Elm327BrakeProtocol.normUuid(vp.get(0).writeUuid));
    }

    private static void testObdBrakeClientBleAndLoopbackFallback() throws Exception {
        final List<String> sentBleLines = Collections.synchronizedList(new ArrayList<String>());

        // Start a mock 127.0.0.1 loopback ELM327 bridge server (simulating obd-gen5w's TcpElmBridgeServer)
        final ServerSocket server = new ServerSocket(0, 8, InetAddress.getByName("127.0.0.1"));
        final int port = server.getLocalPort();
        Thread mockBridgeThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    server.setSoTimeout(5000);
                    while (!server.isClosed()) {
                        Socket client = null;
                        try {
                            client = server.accept();
                            InputStream in = client.getInputStream();
                            OutputStream out = client.getOutputStream();
                            StringBuilder line = new StringBuilder();
                            int b;
                            while ((b = in.read()) != -1) {
                                if (b == '\r' || b == '\n') {
                                    String cmd = line.toString().trim();
                                    line.setLength(0);
                                    if (cmd.isEmpty()) continue;
                                    String reply;
                                    if ("ATI".equalsIgnoreCase(cmd)) {
                                        reply = "ELM327 v1.5\r>";
                                    } else if ("22BC06".equalsIgnoreCase(cmd)) {
                                        reply = "778100B62BC0670400003\r778212C000000000000\r>";
                                    } else {
                                        reply = "OK\r>";
                                    }
                                    out.write(reply.getBytes(Elm327BrakeProtocol.ASCII));
                                    out.flush();
                                } else {
                                    line.append((char) b);
                                }
                            }
                        } catch (Throwable ignored) {
                            break;
                        } finally {
                            if (client != null) {
                                try { client.close(); } catch (Throwable ignored) {}
                            }
                        }
                    }
                } catch (Throwable ignored) {
                } finally {
                    try { server.close(); } catch (Throwable ignored) {}
                }
            }
        }, "MockLoopbackBridge");
        mockBridgeThread.setDaemon(true);
        mockBridgeThread.start();

        // Loopback socket pair simulating @hmgapi-ble inside com.android.bluetooth
        final ServerSocket bleServerSock = new ServerSocket(0, 2, InetAddress.getByName("127.0.0.1"));
        final Socket bleClientSock = new Socket("127.0.0.1", bleServerSock.getLocalPort());
        final Socket bleServerConn = bleServerSock.accept();
        bleServerSock.close();
        bleClientSock.setTcpNoDelay(true);
        bleServerConn.setTcpNoDelay(true);
        final BufferedWriter serverWriter = new BufferedWriter(
                new OutputStreamWriter(bleServerConn.getOutputStream(), "UTF-8"));

        Thread mockBleDaemon = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    BufferedReader r = new BufferedReader(
                            new InputStreamReader(bleServerConn.getInputStream(), "UTF-8"));
                    String line;
                    while ((line = r.readLine()) != null) {
                        String trimmed = line.trim();
                        synchronized (sentBleLines) {
                            sentBleLines.add(trimmed);
                            sentBleLines.notifyAll();
                        }
                        if ("STREAM_SESSION".equals(trimmed)) {
                            sendBleServerEvent(serverWriter, "OK STREAM_READY");
                        } else if (trimmed.startsWith("@AA:BB:CC:DD:EE:01 WRITE_CHAR_NR ")
                                && trimmed.toLowerCase().endsWith("3232424330360d")) {
                            String payloadHex = Elm327BrakeProtocol.bytesToHex(
                                    "778100B62BC0670400003\r778212D00000000>\r"
                                            .getBytes(Elm327BrakeProtocol.ASCII));
                            sendBleServerEvent(
                                    serverWriter,
                                    "@AA:BB:CC:DD:EE:01 CHAR_NOTIFY 0000fff1-0000-1000-8000-00805f9b34fb "
                                            + payloadHex);
                        } else if ("CLOSE".equals(trimmed)) {
                            break;
                        }
                    }
                } catch (Throwable ignored) {
                } finally {
                    try { serverWriter.close(); } catch (Throwable ignored) {}
                    try { bleServerConn.close(); } catch (Throwable ignored) {}
                }
            }
        }, "MockHmgApiBleServer");
        mockBleDaemon.setDaemon(true);
        mockBleDaemon.start();

        InMemoryPairingStore store = new InMemoryPairingStore();
        Closeable closeBleSockets = new Closeable() {
            @Override
            public void close() throws IOException {
                try { bleServerConn.close(); } catch (Throwable ignored) {}
                try { bleClientSock.close(); } catch (Throwable ignored) {}
            }
        };

        ObdBrakeClient client = ObdBrakeClient.createForTest(
                store,
                bleClientSock.getInputStream(),
                bleClientSock.getOutputStream(),
                closeBleSockets,
                port);
        try {
            eq("Ble.attach negotiated STREAM_SESSION + MULTICONN 1",
                    Boolean.TRUE, Boolean.valueOf(waitForBleLine(sentBleLines, "MULTICONN 1", 1000L)));

            // 1. Pairing refuses BVM02 battery sensor by advertised name
            eq("pairAdapter refuses BVM02",
                    Boolean.FALSE, Boolean.valueOf(client.pairAdapter("11:22:33:44:55:66", "LNEX BVM02", 0)));

            // 2. Pairing accepts Veepeak BLE adapter and issues @<mac> CONNECT <addrType> on @hmgapi-ble
            eq("pairAdapter accepts Veepeak",
                    Boolean.TRUE, Boolean.valueOf(client.pairAdapter("AA:BB:CC:DD:EE:01", "OBDCheck BLE", 0)));
            eq("sent @mac CONNECT 0 on @hmgapi-ble",
                    Boolean.TRUE,
                    Boolean.valueOf(waitForBleLine(sentBleLines, "@AA:BB:CC:DD:EE:01 CONNECT 0", 1000L)));

            // 3. Simulate @hmgapi-ble GATT service/char discovery + GATT_READY + direct BLE 22BC06 round-trip
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:01 CONN_STATE CONNECTED AA:BB:CC:DD:EE:01 0x0040");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:01 GATT_SERVICE 0000fff0-0000-1000-8000-00805f9b34fb 0x0010 0x0015");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:01 GATT_CHAR 0000fff0-0000-1000-8000-00805f9b34fb 0000fff1-0000-1000-8000-00805f9b34fb 0x0012 0x10 0x0013");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:01 GATT_CHAR 0000fff0-0000-1000-8000-00805f9b34fb 0000fff2-0000-1000-8000-00805f9b34fb 0x0015 0x0c 0x0000");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:01 GATT_READY AA:BB:CC:DD:EE:01 mtu=185");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:01 NOTIFY_STATE 0000fff1-0000-1000-8000-00805f9b34fb ENABLED");

            eq("GATT_READY enables notifications on fff1",
                    Boolean.TRUE,
                    Boolean.valueOf(waitForBleLinePrefix(sentBleLines, "@AA:BB:CC:DD:EE:01 ENABLE_NOTIFY ", 1000L)));

            String bleRaw = client.sendBleCommandBlocking("22BC06", 1000L);
            Elm327BrakeProtocol.IgmpBrakeReading bleDecoded = Elm327BrakeProtocol.decodeIgmpBrakeResponse(bleRaw);
            eq("Direct @hmgapi-ble 22BC06 round-trip decoded rawCode=45 (pedal switch)",
                    Boolean.TRUE, Boolean.valueOf(bleDecoded != null && bleDecoded.rawCode == 45 && bleDecoded.pedalBit));
            eq("Direct @hmgapi-ble blocks unsafe VIN 0902", null, client.sendBleCommandBlocking("0902", 300L));

            // 4. Simulate @hmgapi-ble ERR CONNECT_TIMEOUT -> AUTO addrType (-1), wake scan, & CONNECT_ALREADY_EXISTS reset
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:01 ERR CONNECT_TIMEOUT AA:BB:CC:DD:EE:01");
            long errDeadline = System.currentTimeMillis() + 1000L;
            while (!client.getStatusSummary().contains("BLE retry pending") && System.currentTimeMillis() < errDeadline) {
                Thread.sleep(10L);
            }
            eq("ERR CONNECT_TIMEOUT transitions to retry backoff",
                    Boolean.TRUE, Boolean.valueOf(client.getStatusSummary().contains("BLE retry pending")));
            eq("ERR CONNECT_TIMEOUT switches effectiveConnectAddrType to -1 (AUTO)",
                    Integer.valueOf(-1), Integer.valueOf(client.effectiveConnectAddrType()));

            // 4b. Wake scan wakes sleeping adapter, parses zero-padded ADV, & heals stored addrType (0 -> 1)
            sentBleLines.clear();
            Thread advFeeder = new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        if (waitForBleLine(sentBleLines, "SCAN_START", 500L)) {
                            sendBleServerEvent(serverWriter,
                                    "ADV AA:BB:CC:DD:EE:01 1 -59 02010608095645455045414b00000000000000");
                        }
                    } catch (Throwable ignored) {}
                }
            }, "MockWakeScanFeeder");
            advFeeder.setDaemon(true);
            advFeeder.start();
            boolean woke = client.performWakeScanBeforeConnect(client.getBleSessionForTest(), 800L);
            eq("performWakeScanBeforeConnect detects paired adapter ADV", Boolean.TRUE, Boolean.valueOf(woke));
            eq("performWakeScanBeforeConnect stops scan cleanly",
                    Boolean.TRUE, Boolean.valueOf(waitForBleLine(sentBleLines, "SCAN_STOP", 500L)));
            eq("ADV heals persisted addrType in PairingStore (0 -> 1)",
                    Integer.valueOf(1), Integer.valueOf(store.getAddrType()));

            // 4c. ERR CONNECT_ALREADY_EXISTS explicitly disconnects stale link
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:01 ERR CONNECT_ALREADY_EXISTS AA:BB:CC:DD:EE:01");
            eq("CONNECT_ALREADY_EXISTS sends immediate @mac DISCONNECT",
                    Boolean.TRUE,
                    Boolean.valueOf(waitForBleLine(sentBleLines, "@AA:BB:CC:DD:EE:01 DISCONNECT", 1000L)));
            eq("CONNECT_ALREADY_EXISTS surfaces busy reset status",
                    Boolean.TRUE, Boolean.valueOf(client.getStatusSummary().contains("BLE link busy; resetting")));

            // 5. Defense-in-depth BVM02 GATT table on unnamed device -> PROTECT_CHAR fff1 + DISCONNECT
            eq("pairAdapter allows unnamed device for GATT inspection",
                    Boolean.TRUE, Boolean.valueOf(client.pairAdapter("AA:BB:CC:DD:EE:02", "Generic BLE", 0)));
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:02 CONN_STATE CONNECTED AA:BB:CC:DD:EE:02 0x0041");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:02 GATT_SERVICE 0000fff0-0000-1000-8000-00805f9b34fb 0x0010 0x0018");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:02 GATT_CHAR 0000fff0-0000-1000-8000-00805f9b34fb 0000fff1-0000-1000-8000-00805f9b34fb 0x0012 0x02 0x0000");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:02 GATT_CHAR 0000fff0-0000-1000-8000-00805f9b34fb 0000fff2-0000-1000-8000-00805f9b34fb 0x0014 0x08 0x0000");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:02 GATT_CHAR 0000fff0-0000-1000-8000-00805f9b34fb 0000fff4-0000-1000-8000-00805f9b34fb 0x0016 0x10 0x0017");
            sendBleServerEvent(serverWriter, "@AA:BB:CC:DD:EE:02 GATT_READY AA:BB:CC:DD:EE:02 mtu=23");

            eq("BVM02 GATT sends PROTECT_CHAR fff1",
                    Boolean.TRUE,
                    Boolean.valueOf(waitForBleLine(sentBleLines, "@AA:BB:CC:DD:EE:02 PROTECT_CHAR fff1", 1000L)));
            eq("BVM02 GATT disconnects immediately",
                    Boolean.TRUE,
                    Boolean.valueOf(waitForBleLine(sentBleLines, "@AA:BB:CC:DD:EE:02 DISCONNECT", 1000L)));
            eq("BVM02 GATT marks safety rejected",
                    "Incompatible GATT (safety rejected)", client.getStatusSummary());

            // 6. Verify coexistence probe + 127.0.0.1:<port> loopback bridge session (obd-gen5w multiplexer)
            eq("probeActiveLoopbackBridge detects live obd-gen5w bridge",
                    Boolean.TRUE, Boolean.valueOf(client.probeActiveLoopbackBridge(port)));
            boolean okBridge = client.tryRunLoopbackBridgeSession(port);
            eq("127.0.0.1 loopback bridge session succeeds", Boolean.TRUE, Boolean.valueOf(okBridge));
            BrakeEvaluator.ObdSample bridgeSample = client.currentSample();
            eq("Loopback sample transport=BRIDGE", "BRIDGE", bridgeSample.transport);
            eq("Loopback sample rawCode=44 (Pure Regen ON)", Integer.valueOf(44), Integer.valueOf(bridgeSample.rawCode));
            eq("Loopback sample brakeLampLit=true", Boolean.TRUE, Boolean.valueOf(bridgeSample.brakeLampLit));
            eq("Loopback sample pedalBit=false", Boolean.FALSE, Boolean.valueOf(bridgeSample.pedalBit));
        } finally {
            client.close();
            try { server.close(); } catch (Throwable ignored) {}
        }
    }

    private static void sendBleServerEvent(BufferedWriter serverWriter, String line) throws IOException {
        synchronized (serverWriter) {
            serverWriter.write(line);
            serverWriter.write('\n');
            serverWriter.flush();
        }
    }

    private static boolean waitForBleLine(List<String> lines, String expected, long timeoutMs)
            throws InterruptedException {
        long deadline = System.currentTimeMillis() + timeoutMs;
        synchronized (lines) {
            while (!lines.contains(expected)) {
                long rem = deadline - System.currentTimeMillis();
                if (rem <= 0L) return false;
                lines.wait(rem);
            }
            return true;
        }
    }

    private static boolean waitForBleLinePrefix(List<String> lines, String prefix, long timeoutMs)
            throws InterruptedException {
        long deadline = System.currentTimeMillis() + timeoutMs;
        synchronized (lines) {
            while (true) {
                for (int i = 0; i < lines.size(); i++) {
                    if (lines.get(i).startsWith(prefix)) return true;
                }
                long rem = deadline - System.currentTimeMillis();
                if (rem <= 0L) return false;
                lines.wait(rem);
            }
        }
    }

    private static void testIndicatorConfigModesAndLegacyCompatibility() {
        IndicatorConfig def = IndicatorConfig.defaults();
        eq("default obdMode is OBD_WHEN_CONNECTED",
                IndicatorConfig.ObdMode.OBD_WHEN_CONNECTED, def.obdMode);
        eq("default signalMode is PRESSURE_AND_DECEL",
                IndicatorConfig.SignalMode.PRESSURE_AND_DECEL, def.signalMode);
        eq("default bridgePort is 35000", Integer.valueOf(35000), Integer.valueOf(def.bridgePort));

        Map<String, String> m = new HashMap<String, String>();
        m.put("obd_mode", "model_only_icon");
        m.put("signal_mode", "native"); // v0.2.0 legacy value -> maps cleanly to PRESSURE_ONLY
        m.put("off_state", "dim");
        m.put("anchor", "top_left");
        m.put("icon_size", "12"); // clamped to 20
        m.put("pressure_threshold", "15"); // 1.5
        m.put("decel_threshold", "14"); // 1.4 m/s^2
        m.put("standstill_hold", "false");
        m.put("bridge_port", "35100");

        IndicatorConfig parsed = IndicatorConfig.fromMap(m);
        eq("parsed obd_mode", IndicatorConfig.ObdMode.MODEL_ONLY_ICON, parsed.obdMode);
        eq("legacy 'native' maps to PRESSURE_ONLY",
                IndicatorConfig.SignalMode.PRESSURE_ONLY, parsed.signalMode);
        eq("clamped icon_size=20", Integer.valueOf(20), Integer.valueOf(parsed.iconSizePx));
        eq("parsed pressure_threshold=1.5", Boolean.TRUE, Boolean.valueOf(Math.abs(parsed.pressureThreshold - 1.5f) < 0.001f));
        eq("parsed decel_threshold=1.4", Boolean.TRUE, Boolean.valueOf(Math.abs(parsed.decelThresholdMps2 - 1.4f) < 0.001f));
        eq("parsed bridge_port=35100", Integer.valueOf(35100), Integer.valueOf(parsed.bridgePort));
        eq("ObdMode cycle order", IndicatorConfig.ObdMode.DISABLED, parsed.obdMode.next());
    }

    private static void testBrakeEvaluatorCachedSpeedAndObdModes() {
        IndicatorConfig defaultCfg = IndicatorConfig.defaults(); // OBD_WHEN_CONNECTED + PRESSURE_AND_DECEL
        Map<String, String> modelOnlyMap = new HashMap<String, String>();
        modelOnlyMap.put("obd_mode", "model_only_icon");
        IndicatorConfig modelIconCfg = IndicatorConfig.fromMap(modelOnlyMap);

        BrakeEvaluator ev = new BrakeEvaluator();

        // 1. CAN invalid pressure sentinel 409.5 must be ignored
        BrakeEvaluator.Snapshot sInvalid = ev.evaluate(defaultCfg, new BrakeEvaluator.Input(
                1000L, 409.5f, 60.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L));
        eq("409.5 invalid brake.pressure ignored", Boolean.FALSE, Boolean.valueOf(sInvalid.illuminated));

        // 2. Simulate HMG-API's 250 ms `speed` cache polled at 20 Hz (50 ms):
        // Speed updates ONLY every 5th tick (every 250 ms) by -1.30 km/h (~1.44 m/s^2) at 8% i-Pedal throttle!
        BrakeEvaluator evRegen = new BrakeEvaluator();
        float cachedSpeed = 80.0f;
        int litTicksAfterOnset = 0;
        BrakeEvaluator.Snapshot lastSnap = null;
        for (int tick = 0; tick < 20; tick++) {
            long t = 2000L + tick * 50L;
            if (tick > 0 && tick % 5 == 0) {
                cachedSpeed -= 1.30f;
            }
            lastSnap = evRegen.evaluate(defaultCfg, new BrakeEvaluator.Input(
                    t, 0.0f, cachedSpeed, 8.0f, -2.5f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 74.5f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            if (tick >= 6 && lastSnap.illuminated) {
                litTicksAfterOnset++;
            }
        }
        eq("250 ms cached HMG-API speed lights regen cleanly",
                Boolean.TRUE, Boolean.valueOf(lastSnap != null && lastSnap.illuminated));
        eq("Zero sawtooth dropout across all 14 ticks (including 4x cached speed repeats)",
                Integer.valueOf(14), Integer.valueOf(litTicksAfterOnset));
        eq("Model source is REGEN_DECEL", "REGEN_DECEL", lastSnap.modelSource);

        // 3. Verify OBD_WHEN_CONNECTED vs MODEL_ONLY_ICON when OBD and Model disagree:
        // Suppose speed is steady 70 km/h (Model = OFF), while OBD 22BC06 reports Pure Regen ON (rawCode=44)
        BrakeEvaluator evObd = new BrakeEvaluator();
        BrakeEvaluator.ObdSample obdOn = new BrakeEvaluator.ObdSample(true, "BLE", 5000L, 44, true, false);
        BrakeEvaluator.Snapshot sObdIcon = evObd.evaluate(defaultCfg, new BrakeEvaluator.Input(
                5040L, 0.0f, 70.0f, 15.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f, obdOn, 0L));
        eq("OBD_WHEN_CONNECTED lights icon from OBD", Boolean.TRUE, Boolean.valueOf(sObdIcon.illuminated));
        eq("OBD_WHEN_CONNECTED source OBD_REGEN[44]", "OBD_REGEN[44]", sObdIcon.source);
        eq("Model still independently evaluated as OFF for scoring",
                Boolean.FALSE, Boolean.valueOf(sObdIcon.modelIlluminated));

        BrakeEvaluator.Snapshot sModelIcon = evObd.evaluate(modelIconCfg, new BrakeEvaluator.Input(
                5090L, 0.0f, 70.0f, 15.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f, obdOn, 0L));
        eq("MODEL_ONLY_ICON keeps icon driven by HMG-API model (OFF)",
                Boolean.FALSE, Boolean.valueOf(sModelIcon.illuminated));
        eq("MODEL_ONLY_ICON still keeps obdFresh=true for background scoring",
                Boolean.TRUE, Boolean.valueOf(sModelIcon.obdFresh));
    }

    private static void testBrakeEvaluatorLegacyHub650msPollAlignment() {
        IndicatorConfig cfg = IndicatorConfig.defaults();
        BrakeEvaluator ev = new BrakeEvaluator();

        // Simulate real EV6 on legacy 650 ms KitKat Hub (13 eval ticks @ 50 ms per 650 ms HMG poll)
        // with live OBD reporting 44 (0x2C, pure regen ON, pedalBit=false) during L2 coast (~0.98 m/s^2).
        long pollMonoMs = 10000L;
        float speedKmh = 64.4f;
        BrakeEvaluator.Snapshot firstTickAfterSecondPoll = null;
        BrakeEvaluator.Snapshot twelfthTickAfterSecondPoll = null;
        BrakeEvaluator.Snapshot lastSnap = null;

        for (int pollIdx = 0; pollIdx < 4; pollIdx++) {
            if (pollIdx > 0) {
                pollMonoMs += 650L;
                // -2.30 km/h per 650 ms = 0.983 m/s^2 (L2 settled coast)
                speedKmh -= 2.30f;
            }
            for (int sub = 0; sub < 13; sub++) {
                long nowMs = pollMonoMs + sub * 50L;
                BrakeEvaluator.ObdSample obd44 = new BrakeEvaluator.ObdSample(
                        true, "BLE", nowMs - 20L, 44, true, false);
                lastSnap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                        nowMs, 0.0f, speedKmh, 0.0f, 0.0f,
                        Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                        obd44, 0L, -1, pollMonoMs));
                if (pollIdx == 1 && sub == 0) {
                    firstTickAfterSecondPoll = lastSnap;
                } else if (pollIdx == 1 && sub == 12) {
                    twelfthTickAfterSecondPoll = lastSnap;
                }
            }
        }

        eq("650ms legacy Hub poll-aligned decel has zero mid-poll decay across 13 ticks",
                Boolean.TRUE,
                Boolean.valueOf(firstTickAfterSecondPoll != null
                        && twelfthTickAfterSecondPoll != null
                        && firstTickAfterSecondPoll.decelMps2 > 0.90f
                        && Math.abs(firstTickAfterSecondPoll.decelMps2 - twelfthTickAfterSecondPoll.decelMps2) < 0.001f));
        eq("OBD raw=44 (pedalBit=false) allows coasting L2 auto-detection on 650ms Hub",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_2),
                Integer.valueOf(lastSnap.inferredRegenLevel));
        eq("L2 coasting on 650ms Hub lights HMG-API model (REGEN_DECEL)",
                Boolean.TRUE,
                Boolean.valueOf(lastSnap.modelIlluminated && "REGEN_DECEL".equals(lastSnap.modelSource)));
    }

    private static void testBrakeEvaluatorCoastingRegenAndIpedalInference() {
        IndicatorConfig cfg = IndicatorConfig.defaults();
        eq("classifyCoastingRegenLevel L0", Integer.valueOf(0), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(0.32f, -1)));
        eq("classifyCoastingRegenLevel L1", Integer.valueOf(1), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(0.60f, -1)));
        eq("classifyCoastingRegenLevel L2", Integer.valueOf(2), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(0.95f, -1)));
        eq("classifyCoastingRegenLevel L3", Integer.valueOf(3), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(1.75f, -1)));
        eq("classifyCoastingRegenLevel hysteresis holds L2 at 0.74",
                Integer.valueOf(2), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(0.74f, 2)));
        eq("regenLevelLabel formats 4 as i-Pedal", "i-Pedal", BrakeEvaluator.regenLevelLabel(BrakeEvaluator.REGEN_LEVEL_IPEDAL));
        eq("regenLevelLabel formats -1 as ?", "?", BrakeEvaluator.regenLevelLabel(BrakeEvaluator.REGEN_LEVEL_UNKNOWN));

        // Simulate 100 ms updated HMG-API speed steps during off-throttle coasting in L1, L2, L3, and i-Pedal
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 20000L;
        float spd = 72.0f;
        BrakeEvaluator.Snapshot snap = null;

        // 1. Coast in L1 (~0.58 m/s^2 -> -0.21 km/h every 100 ms) for 1200 ms (settle 350ms + chord >= 750ms)
        for (int tick = 0; tick <= 24; tick++) {
            if (tick > 0 && tick % 2 == 0) spd -= 0.21f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, 1));
            t += 50L;
        }
        eq("Coasting decel ~0.58 m/s^2 infers Regen L1", Integer.valueOf(1), Integer.valueOf(snap.inferredRegenLevel));
        eq("hevRegenRaw passed through to Snapshot", Integer.valueOf(1), Integer.valueOf(snap.hevRegenRaw));

        // 2. Driver clicks paddle to L2 (~0.97 m/s^2 -> -0.35 km/h every 100 ms)
        for (int tick = 1; tick <= 24; tick++) {
            if (tick % 2 == 0) spd -= 0.35f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, 2));
            t += 50L;
        }
        eq("Coasting decel ~0.97 m/s^2 infers Regen L2", Integer.valueOf(2), Integer.valueOf(snap.inferredRegenLevel));

        // 3. Driver presses throttle (18%): inferredRegenLevel stays latched at L2 until next coast or Park
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd + 0.5f, 18.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, 2));
        t += 100L;
        eq("Throttle press preserves learned L2", Integer.valueOf(2), Integer.valueOf(snap.inferredRegenLevel));

        // 4. Coast in L3 (~1.80 m/s^2 -> -0.65 km/h every 100 ms)
        for (int tick = 1; tick <= 24; tick++) {
            if (tick % 2 == 0) spd -= 0.65f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, 3));
            t += 50L;
        }
        eq("Coasting decel ~1.80 m/s^2 infers Regen L3", Integer.valueOf(3), Integer.valueOf(snap.inferredRegenLevel));
        eq("L3 coast also illuminates HMG-API model", Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));

        // 5. i-Pedal Signature B1: Low-speed zero-pedal full-stop regen (12 -> 5 km/h at ~1.94 m/s^2, no creep taper)
        spd = 12.6f;
        for (int tick = 0; tick <= 20; tick++) {
            if (tick > 0 && tick % 2 == 0) spd -= 0.70f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        eq("Low-speed zero-pedal full-stop regen promotes mode to i-Pedal (4)",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_IPEDAL), Integer.valueOf(snap.inferredRegenLevel));

        // 6. i-Pedal Signature B3: Low-speed creep taper (15 -> 12 km/h at ~0.55 m/s^2) demotes i-Pedal (4) -> L3 (3)
        spd = 15.5f;
        for (int tick = 0; tick <= 24; tick++) {
            if (tick > 0 && tick % 2 == 0) spd -= 0.20f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        eq("Low-speed creep taper demotes i-Pedal (4) back to L3 (3)",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_3), Integer.valueOf(snap.inferredRegenLevel));

        // 7. i-Pedal Signature B2: Steady speed cruise (48 km/h) at 26% neutral pedal for >= 1.1s promotes to i-Pedal (4)
        t += 2500L; // new cruise segment at 48 km/h
        spd = 48.0f;
        for (int tick = 0; tick <= 34; tick++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 26.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        eq("Steady cruise at 26% neutral pedal infers i-Pedal (4)",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_IPEDAL), Integer.valueOf(snap.inferredRegenLevel));

        // 8. Shifting into Park resets inferredRegenLevel to -1
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.0f, 0.0f, Boolean.TRUE, Boolean.TRUE, Boolean.FALSE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1));
        eq("Park resets inferredRegenLevel to -1", Integer.valueOf(-1), Integer.valueOf(snap.inferredRegenLevel));
    }

    private static void testBrakeEvaluatorStandstillHoldAndRelease() {
        IndicatorConfig cfg = IndicatorConfig.defaults();
        BrakeEvaluator ev = new BrakeEvaluator();

        // Decelerate from 15 km/h to 0 km/h at ~2.0 m/s^2 in 250 ms HMG-API speed steps (-1.8 km/h per 250 ms)
        long t = 10000L;
        float spd = 14.4f;
        BrakeEvaluator.Snapshot snap = null;
        for (int tick = 0; tick <= 45; tick++) {
            if (tick > 0 && tick % 5 == 0) {
                spd = Math.max(0.0f, spd - 1.8f);
            }
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE,
                    Boolean.valueOf(spd > 0.1f), 68.0f, BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        // Hold at 0.0 km/h for 2 seconds
        for (int i = 0; i < 10; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, 0.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE,
                    Boolean.FALSE, 68.0f, BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 200L;
        }
        eq("Standstill hold stays lit at 0 km/h", Boolean.TRUE, Boolean.valueOf(snap.illuminated));
        eq("Standstill hold source", "STANDSTILL_HOLD", snap.source);

        // Tap throttle (2.5%) -> immediately releases standstill hold
        BrakeEvaluator.Snapshot released = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 2.5f, 0.0f, Boolean.FALSE, Boolean.FALSE,
                Boolean.FALSE, 68.0f, BrakeEvaluator.ObdSample.disconnected(), 0L));
        eq("Throttle >= 2% releases standstill hold", Boolean.FALSE, Boolean.valueOf(released.illuminated));
    }

    private static void testBrakeEvaluatorV060SccAutoHoldAeroAndRelease() {
        IndicatorConfig cfg = IndicatorConfig.defaults();

        // 1. Auto-Hold regression test (Drive 2 880.5s..933.0s):
        //    Braking briskly across 12.9 -> 4.8 -> 0.0 km/h on 650 ms Hub, holding foot brake
        //    at 0.0 km/h for 18 seconds, then releasing foot brake into Auto-Hold for 30 seconds.
        BrakeEvaluator evAutoHold = new BrakeEvaluator();
        long t = 50000L;
        evAutoHold.evaluate(cfg, new BrakeEvaluator.Input(
                t, 5.0f, 12.9f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 650L;
        evAutoHold.evaluate(cfg, new BrakeEvaluator.Input(
                t, 13.4f, 4.8f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 650L;
        // Hold foot brake at 0.0 km/h for 18 seconds
        for (int i = 0; i < 28; i++) {
            evAutoHold.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 19.3f, 0.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 650L;
        }
        // Release foot brake (prsr -> 0.0) into Auto-Hold for 30 seconds
        BrakeEvaluator.Snapshot autoHoldSnap = null;
        for (int i = 0; i < 46; i++) {
            autoHoldSnap = evAutoHold.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, 0.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 650L;
        }
        eq("Auto-Hold stays STANDSTILL_HOLD after 4.8->0.0 km/h stop and 18s foot hold",
                Boolean.TRUE,
                Boolean.valueOf(autoHoldSnap.modelIlluminated && "STANDSTILL_HOLD".equals(autoHoldSnap.modelSource)));

        // Engaging EPB (`parking_brake = true`) immediately releases standstill hold
        BrakeEvaluator.Snapshot epbSnap = evAutoHold.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.TRUE, Boolean.FALSE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        eq("EPB (parking_brake=true) releases standstill hold",
                Boolean.FALSE, Boolean.valueOf(epbSnap.modelIlluminated));

        // 2. Smart Cruise Control (SCC) stop-and-go detection, L2 mode preservation, & exit:
        BrakeEvaluator evScc = new BrakeEvaluator();
        t = 100000L;
        float spd = 56.0f;
        BrakeEvaluator.Snapshot sccSnap = null;
        // First learn L2 (~0.98 m/s^2 coasting)
        for (int tick = 0; tick <= 30; tick++) {
            if (tick > 0 && tick % 2 == 0) spd -= 0.35f;
            sccSnap = evScc.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        eq("Pre-SCC learned mode is L2", Integer.valueOf(2), Integer.valueOf(sccSnap.inferredRegenLevel));

        // Driver engages SCC in a new segment: 0% pedal while vehicle accelerates from 38.6 -> 42.2 km/h over 2.2s
        t += 2500L;
        spd = 38.6f;
        for (int tick = 0; tick <= 44; tick++) {
            if (tick >= 16 && tick % 4 == 0) spd += 0.6f;
            sccSnap = evScc.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        eq("Zero-pedal acceleration activates sccActive", Boolean.TRUE, Boolean.valueOf(sccSnap.sccActive));
        eq("regenLevelLabel appends (SCC)", "L2 (SCC)",
                BrakeEvaluator.regenLevelLabel(sccSnap.inferredRegenLevel, sccSnap.sccActive));

        // SCC brakes briskly through low speed (12 -> 4 km/h at ~2.5 m/s^2): MUST NOT corrupt L2 -> i-Pedal!
        spd = 12.6f;
        for (int tick = 0; tick <= 20; tick++) {
            if (tick > 0 && tick % 2 == 0) spd -= 0.90f;
            sccSnap = evScc.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, Math.max(2.5f, spd), 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        eq("SCC low-speed full-stop braking lights REGEN_DECEL", Boolean.TRUE, Boolean.valueOf(sccSnap.modelIlluminated));
        eq("SCC freezes driver regen mode at L2 (does NOT corrupt to i-Pedal)",
                Integer.valueOf(2), Integer.valueOf(sccSnap.inferredRegenLevel));

        // Driver taps physical brake pedal -> cancels SCC immediately
        sccSnap = evScc.evaluate(cfg, new BrakeEvaluator.Input(
                t, 4.5f, 2.5f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L));
        eq("Foot brake cancels sccActive immediately", Boolean.FALSE, Boolean.valueOf(sccSnap.sccActive));

        // 3. Creep-taper gate (< 20.5 km/h in L2) & L2 gentle throttle release (5.5% >= 4.8%):
        t += 1000L;
        // Release brake and coast at 18.0 km/h with mild 0.40 m/s^2 creep-taper decel -> must stay OFF
        spd = 18.4f;
        for (int tick = 0; tick <= 12; tick++) {
            if (tick > 0 && tick % 4 == 0) spd -= 0.28f;
            sccSnap = evScc.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        eq("L2 low-speed (<20.5 km/h) creep-taper coast does not false-trigger",
                Boolean.FALSE, Boolean.valueOf(sccSnap.modelIlluminated));

        // Lift off at 44 km/h in L2 -> immediate onset, then gentle 5.5% throttle -> immediate release!
        sccSnap = evScc.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 44.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L));
        eq("L2 lift-off at 44 km/h lights immediately", Boolean.TRUE, Boolean.valueOf(sccSnap.modelIlluminated));
        t += 50L;
        sccSnap = evScc.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 43.8f, 5.5f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L));
        eq("L2 gentle 5.5% throttle releases REGEN_DECEL immediately without 1.5s tail",
                Boolean.FALSE, Boolean.valueOf(sccSnap.modelIlluminated));

        // 4. Highway 70 mph (108 km/h) aerodynamic drag compensation:
        //    L1 coasting at 108 km/h has total decel ~0.86 m/s^2 (~0.67 motor + ~0.19 wind drag) -> stays L1!
        BrakeEvaluator evHwy = new BrakeEvaluator();
        t = 200000L;
        spd = 108.0f;
        BrakeEvaluator.Snapshot hwySnap = null;
        for (int tick = 0; tick <= 30; tick++) {
            if (tick > 0 && tick % 2 == 0) spd -= 0.31f; // ~0.86 m/s^2 total decel
            hwySnap = evHwy.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L));
            t += 50L;
        }
        eq("108 km/h highway L1 coast (~0.86 m/s^2 total) compensates aero drag and stays L1",
                Integer.valueOf(1), Integer.valueOf(hwySnap.inferredRegenLevel));
        eq("108 km/h highway L1 coast keeps brake lamps OFF",
                Boolean.FALSE, Boolean.valueOf(hwySnap.modelIlluminated));
    }

    @SuppressWarnings("unused")
    private interface FakeOemService {
        int getRecuperationLevel();
        boolean isSmartRegenActive();
        String getVinNumber();
        int getDeviceSerial();
        void setRecuperationLevel(int level);
    }

    private static void testHevRegenProbeSafetyAndBaselineDiff() throws Exception {
        // 1. Privacy & mutator safety checks
        eq("Blocks VIN signal name", Boolean.FALSE, Boolean.valueOf(HevRegenProbe.isSafeSignalName("HEVFlag.CAR_VIN_CODE")));
        eq("Blocks MAC signal name", Boolean.FALSE, Boolean.valueOf(HevRegenProbe.isSafeSignalName("BCM.getBtMacAddress")));
        eq("Blocks GPS signal name", Boolean.FALSE, Boolean.valueOf(HevRegenProbe.isSafeSignalName("CLU.getGpsLatitude")));
        eq("Allows EV_RECUP_LEVEL signal name", Boolean.TRUE, Boolean.valueOf(HevRegenProbe.isSafeSignalName("HEVFlag.EV_RECUP_LEVEL(12)")));

        Method okInt = FakeOemService.class.getMethod("getRecuperationLevel");
        Method okBool = FakeOemService.class.getMethod("isSmartRegenActive");
        Method badVinString = FakeOemService.class.getMethod("getVinNumber");
        Method badSerialInt = FakeOemService.class.getMethod("getDeviceSerial");
        Method badMutator = FakeOemService.class.getMethod("setRecuperationLevel", int.class);
        eq("Allows primitive int getter", Boolean.TRUE, Boolean.valueOf(HevRegenProbe.isSafeGetterMethod(okInt)));
        eq("Allows primitive boolean getter", Boolean.TRUE, Boolean.valueOf(HevRegenProbe.isSafeGetterMethod(okBool)));
        eq("Blocks String VIN getter", Boolean.FALSE, Boolean.valueOf(HevRegenProbe.isSafeGetterMethod(badVinString)));
        eq("Blocks serial int getter", Boolean.FALSE, Boolean.valueOf(HevRegenProbe.isSafeGetterMethod(badSerialInt)));
        eq("Blocks mutator method", Boolean.FALSE, Boolean.valueOf(HevRegenProbe.isSafeGetterMethod(badMutator)));

        // 2. Rejects 250 (0xFA USM unsupported sentinel) as direct regen candidate
        final Map<String, String> liveSignals = new LinkedHashMap<String, String>();
        liveSignals.put("HEVFlag.EV_READY(1)", "1");
        liveSignals.put("USM.getSmartRecuperation", "250"); // 0xFA unsupported sentinel -> must NOT become direct candidate
        liveSignals.put("HEVFlag.ODOMETER(4)", "123450");   // noisy counter -> must not trigger false candidate alerts
        liveSignals.put("BCM.getVinCode", "SECRET");        // unsafe name -> must be dropped even if returned

        HevRegenProbe probe = new HevRegenProbe(new HevRegenProbe.SignalSource() {
            @Override
            public String serviceStatusSummary() {
                return "HEV=YES(3 flags) USM=YES CLU=YES";
            }

            @Override
            public Map<String, String> readSignals(boolean fullScan) {
                return new LinkedHashMap<String, String>(liveSignals);
            }
        });

        final List<String> events = new ArrayList<String>();
        probe.setChangeListener(new HevRegenProbe.ChangeListener() {
            @Override
            public void onHevSignalChanged(String key, String oldVal, String newVal, boolean regenKeywordMatch) {
                events.add(key + ":" + oldVal + "->" + newVal + ":" + regenKeywordMatch);
            }
        });

        probe.poll(true);
        eq("250 (0xFA USM sentinel) ignored as direct regen candidate (-1)",
                Integer.valueOf(-1), Integer.valueOf(probe.getLatestDirectRegenRaw()));

        liveSignals.put("HEVFlag.EV_RECUP_LEVEL(12)", "1");
        probe.poll(true);
        eq("Initial direct regen candidate = 1", Integer.valueOf(1), Integer.valueOf(probe.getLatestDirectRegenRaw()));
        events.clear();

        String markMsg = probe.markBaseline();
        eq("markBaseline counts only safe signals (4)", Boolean.TRUE, Boolean.valueOf(markMsg.contains("4 HEV/USM/CLU signals")));

        // Simulate odometer tick + driver clicking left paddle from Level 1 -> Level 2
        liveSignals.put("HEVFlag.ODOMETER(4)", "123451");
        liveSignals.put("HEVFlag.EV_RECUP_LEVEL(12)", "2");
        probe.poll(false);

        eq("Only non-noisy paddle signal fired ChangeListener", Integer.valueOf(1), Integer.valueOf(events.size()));
        eq("ChangeListener flagged regen candidate",
                "HEVFlag.EV_RECUP_LEVEL(12):1->2:true", events.get(0));
        eq("Updated direct regen candidate = 2", Integer.valueOf(2), Integer.valueOf(probe.getLatestDirectRegenRaw()));
        eq("Baseline diff count = 1", Integer.valueOf(1), Integer.valueOf(probe.getBaselineChangedCount()));

        String report = probe.buildReport(false);
        eq("Probe report highlights REGEN/MODE CANDIDATE",
                Boolean.TRUE, Boolean.valueOf(report.contains("HEVFlag.EV_RECUP_LEVEL(12) : 1 -> 2") && report.contains("REGEN/MODE CANDIDATE")));
        eq("Probe report excludes unsafe VIN signal",
                Boolean.FALSE, Boolean.valueOf(report.contains("SECRET") || report.contains("Vin")));
    }

    private static void testTelemetryRecorderScoringAndCsv() {
        TelemetryRecorder rec = new TelemetryRecorder();
        long wall = 1763000000000L;

        // Tick 0 (t=1000): Both OFF -> TN (inferred regen L3)
        rec.record(makeScoredSnapshot(1000L, false, "OFF", 0, false, false, 3, -1), wall);
        // External bookmark event
        rec.recordExternalEvent(1025L, "MARK #1 (regen=L3, speed=64.0 km/h)");
        // Tick 1 (t=1050): OBD turns ON via Pure Regen 44 (0x2C, pedalBit=false), Model still OFF -> FN (regen transitions L3 -> i-Pedal)
        rec.record(makeScoredSnapshot(1050L, false, "OFF", 44, true, false, 4, -1), wall + 50L);
        // Tick 2 (t=1150): Model turns ON via REGEN_DECEL -> TP, onset lag = 150 - 50 = +100 ms!
        rec.record(makeScoredSnapshot(1150L, true, "REGEN_DECEL", 44, true, false, 4, -1), wall + 150L);
        // Tick 3 (t=1200): Driver taps brake pedal (raw=45, pedalBit=true), Model ON via BRAKE_PRESSURE -> TP
        rec.record(makeScoredSnapshot(1200L, true, "BRAKE_PRESSURE", 45, true, true, 4, -1), wall + 200L);
        // Tick 4 (t=1250): OBD turns OFF (0), Model still holding -> FP (pending OBD fall at rel=250ms)
        rec.record(makeScoredSnapshot(1250L, true, "REGEN_DECEL", 0, false, false, 4, -1), wall + 250L);
        // Tick 5 (t=1330): Model turns OFF -> TN, release lag = 330 - 250 = +80 ms!
        rec.record(makeScoredSnapshot(1330L, false, "OFF", 0, false, false, 4, -1), wall + 330L);

        TelemetryRecorder.ScoreSnapshot sc = rec.scoreSnapshot();
        eq("6 buffered rows", Integer.valueOf(6), Integer.valueOf(sc.bufferedRows));
        eq("6 scored ticks", Long.valueOf(6L), Long.valueOf(sc.scoredSamples));
        eq("TP=2", Long.valueOf(2L), Long.valueOf(sc.tp));
        eq("FP=1", Long.valueOf(1L), Long.valueOf(sc.fp));
        eq("TN=2", Long.valueOf(2L), Long.valueOf(sc.tn));
        eq("FN=1", Long.valueOf(1L), Long.valueOf(sc.fn));
        eq("Pedal switch recall 1/1 ticks", Long.valueOf(1L), Long.valueOf(sc.pedalModelMatchedSamples));
        eq("Pure Regen recall 1/2 ticks", Long.valueOf(1L), Long.valueOf(sc.regenModelMatchedSamples));
        eq("Onset lag +100 ms", Long.valueOf(100L), Long.valueOf(sc.meanOnsetLagMs));
        eq("Release lag +80 ms", Long.valueOf(80L), Long.valueOf(sc.meanReleaseLagMs));

        String csv = rec.toCsvString();
        String[] lines = csv.trim().split("\n");
        eq("CSV has header + 6 data rows", Integer.valueOf(7), Integer.valueOf(lines.length));
        eq("CSV header includes inferred_regen_level,hev_regen_raw",
                Boolean.TRUE, Boolean.valueOf(lines[0].endsWith("inferred_regen_level,hev_regen_raw")));
        eq("CSV row 3 records REGEN_DECEL, raw 44, pedal=0, i-Pedal(4)",
                Boolean.TRUE, Boolean.valueOf(lines[3].contains(",1,REGEN_DECEL,1,1,BLE,") && lines[3].endsWith(",44,1,0,4,")));
        eq("CSV row 4 records BRAKE_PRESSURE, raw 45, pedal=1, i-Pedal(4)",
                Boolean.TRUE, Boolean.valueOf(lines[4].contains(",1,BRAKE_PRESSURE,1,1,BLE,") && lines[4].endsWith(",45,1,1,4,")));

        String summary = rec.buildSummaryReport(
                IndicatorConfig.defaults(), "BLE @hmgapi-ble (Veepeak)", "OK (20Hz)", null);
        eq("Summary banner is v0.7.1",
                Boolean.TRUE, Boolean.valueOf(summary.contains("v0.7.1 Calibration Summary")));
        eq("Summary includes confusion matrix and Pedal Switch Rec.",
                Boolean.TRUE, Boolean.valueOf(summary.contains("TP=2  FP=1  TN=2  FN=1")
                        && summary.contains("Pedal Switch Rec.   : 100.0% (1/1 ticks)")
                        && summary.contains("Pure Regen/SCC Rec. : 50.0% (1/2 ticks)")));
        eq("Summary formats i-Pedal transition cleanly",
                Boolean.TRUE, Boolean.valueOf(summary.contains("MARK #1")
                        && summary.contains("RegenModeEst: L3 -> i-Pedal")
                        && summary.contains("reg=i-Pedal")));
    }

    private static BrakeEvaluator.Snapshot makeScoredSnapshot(
            long tMs,
            boolean modelLit,
            String modelSrc,
            int obdRaw,
            boolean obdLit,
            boolean pedalBit,
            int inferredRegen,
            int hevRaw) {
        BrakeEvaluator.ObdSample obd = new BrakeEvaluator.ObdSample(true, "BLE", tMs - 10L, obdRaw, obdLit, pedalBit);
        return new BrakeEvaluator.Snapshot(
                tMs,
                obdLit,
                obdLit ? "OBD_REGEN[36]" : "OBD_OFF",
                modelLit,
                modelSrc,
                Float.valueOf(0.0f),
                Float.valueOf(64.0f),
                Float.valueOf(4.0f),
                Float.valueOf(0.0f),
                modelLit ? 1.55f : 0.10f,
                Boolean.FALSE,
                Boolean.FALSE,
                Boolean.TRUE,
                Float.valueOf(72.0f),
                obd,
                true,
                false,
                inferredRegen,
                hevRaw);
    }

    private static void testUsbExporterMountsAndAtomicWrite() throws Exception {
        String unmountedProcMounts =
                "rootfs / rootfs ro,relatime 0 0\n"
                + "tmpfs /storage tmpfs rw,relatime,mode=0555,gid=1028 0 0\n"
                + "/dev/block/mmcblk0p15 /system ext4 ro,relatime 0 0\n"
                + "/dev/block/mmcblk0p28 /data ext4 rw,nosuid,nodev 0 0\n";
        List<String> none = UsbExporter.parseUsbMountPoints(unmountedProcMounts);
        eq("no USB stick -> 0 mount points", Integer.valueOf(0), Integer.valueOf(none.size()));

        String mountedProcMounts = unmountedProcMounts
                + "/dev/block/vold/8:1 /mnt/media_rw/usb0 vfat rw,dirsync,nosuid,nodev 0 0\n"
                + "/dev/fuse /storage/usb0 fuse rw,nosuid,nodev,relatime 0 0\n";
        List<String> mounts = UsbExporter.parseUsbMountPoints(mountedProcMounts);
        eq("prefers /storage/usb0 ahead of /mnt/media_rw/usb0", "/storage/usb0", mounts.get(0));

        File tmpRoot = Files.createTempDirectory("brake-usb-v4").toFile();
        try {
            File fakeUsb0 = new File(tmpRoot, "usb0");
            fakeUsb0.mkdirs();

            UsbExporter.ExportResult emptyRes = UsbExporter.exportToDirectories(
                    "a,b\n1,2\n", "summary\n", "probe\n", Collections.<String>emptyList(), "BrakeIndicator");
            eq("empty mounts returns ok=false", Boolean.FALSE, Boolean.valueOf(emptyRes.ok));

            UsbExporter.ExportResult okRes = UsbExporter.exportToDirectories(
                    "elapsed_ms,obd_lamp_lit\n0,1\n",
                    "=== Summary ===\n",
                    "=== Gen5W HEVFlag Probe ===\n",
                    Arrays.asList(fakeUsb0.getAbsolutePath()),
                    "BrakeIndicator");
            eq("export dataset+summary+probe succeeds", Boolean.TRUE, Boolean.valueOf(okRes.ok));
            eq("CSV file written", Boolean.TRUE,
                    Boolean.valueOf(okRes.csvPath != null && new File(okRes.csvPath).isFile()));
            eq("Summary file written", Boolean.TRUE,
                    Boolean.valueOf(okRes.summaryPath != null && new File(okRes.summaryPath).isFile()));
            eq("HEV probe file written", Boolean.TRUE,
                    Boolean.valueOf(okRes.hevProbePath != null && new File(okRes.hevProbePath).isFile()));
            // Ensure no leftover .part files remain after atomic rename
            String[] names = new File(fakeUsb0, "BrakeIndicator").list();
            boolean anyPart = false;
            if (names != null) {
                for (int i = 0; i < names.length; i++) {
                    if (names[i].endsWith(".part")) anyPart = true;
                }
            }
            eq("no temporary .part files left behind", Boolean.FALSE, Boolean.valueOf(anyPart));
        } finally {
            deleteRecursive(tmpRoot);
        }
    }

    private static final class InMemoryPairingStore implements ObdBrakeClient.PairingStore {
        String mac = "";
        String name = "";
        int addrType = -1;

        @Override public String getMac() { return mac; }
        @Override public String getName() { return name; }
        @Override public int getAddrType() { return addrType; }

        @Override
        public void save(String mac, String name, int addrType) {
            this.mac = mac;
            this.name = name;
            this.addrType = addrType;
        }

        @Override
        public void clear() {
            this.mac = "";
            this.name = "";
            this.addrType = -1;
        }
    }

    private static void deleteRecursive(File f) {
        if (f == null || !f.exists()) return;
        File[] children = f.listFiles();
        if (children != null) {
            for (File c : children) deleteRecursive(c);
        }
        f.delete();
    }

    private static void eq(String name, Object expected, Object actual) {
        boolean same = expected == null ? actual == null : expected.equals(actual);
        if (same) {
            passed++;
            System.out.println("ok    " + name);
        } else {
            failed++;
            System.out.println("FAIL  " + name + ": wanted [" + expected + "], got [" + actual + "]");
        }
    }
}
