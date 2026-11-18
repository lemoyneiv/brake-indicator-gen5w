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
 * Host JVM unit tests for v0.7.13:
 * - HmgVehiclePoller (fast <2ms LFD1 vehicle reads + resilient 90ms legacy KitKat Hub stall recovery)
 * - Elm327BrakeProtocol (IGMP 770->778 22BC06 multi-frame ISO-TP decoder with 0x01 pedal switch vs 0x2C regen,
 *   LNEX BVM02 exclusion, VIN/write safety)
 * - ObdBrakeClient (HMG-API native @hmgapi-ble central session, uninterrupted manual BLE scan guard,
 *   reconnect 0x16 backoff suppression, diagnostic transition logging, sticky ADV_IND+SCAN_RSP merge,
 *   priority sorting, transient empty-GATT auto-retry, PROTECT_CHAR guard, + 127.0.0.1 bridge coexistence)
 * - IndicatorConfig & BrakeEvaluator (poll-aligned 650 ms legacy & 50-250 ms HMG-API speed handling,
 *   settled L0..L3 coasting chord + first-class Auto Regen detection & traffic-stop stability,
 *   cold-start & mid-drive Auto/Paddle -> i-Pedal 16-125 km/h cruise & 6.8-22.5% feathered braking promotion,
 *   shift-interlock forward <7 km/h 0%-pedal liftoff vs Reverse creep guard & 2.4 km/h crawl release,
 *   1.5-135 km/h crawl-to-highway i-Pedal feathered onset/release curves, sharp-turn boost & 0 km/h standstill hysteresis,
 *   light-throttle cold-onset guard & sustained feather release, highway <1.7% deadzone & active R13-H strong-decel hold,
 *   intersection trail-braking turn scrub suppression, strict i-Pedal vs Auto mutual exclusion,
 *   early >=1.5 km/h brake-release standstill clear, 20 Hz flat-speed quantization ceiling,
 *   pressureLatched-gated post-brake bleed & gentle standstill feather hold,
 *   L2/L3/i-Pedal VCU torque-ramp delay, piecewise speed-dependent i-Pedal feathered braking & low-speed crawl,
 *   full-lock steering scrub compensation, Auto-Hold ON vs OFF standstill inference, <14.2 km/h creep-taper release,
 *   strong-decel throttle cancel, EPB actuator release & carryover hold)
 * - HevRegenProbe (strict VIN/MAC/phone/mutator safety filter, 0xFA USM sentinel rejection, baseline diff)
 * - TelemetryRecorder & UsbExporter (confusion matrix, Pedal vs Pure Regen recall, Auto/i-Pedal labels, atomic USB export)
 */
public final class TestMain {

    private static int passed;
    private static int failed;

    public static void main(String[] args) throws Exception {
        testHmgVehiclePollerFastAndLegacyHubStall();
        testElm327IgmpBrakeDecoding();
        testElm327SafetyAndBvm02Exclusion();
        testObdBrakeClientBleAndLoopbackFallback();
        testV072BleDiscoveryMergePriorityAndEvaluatorCalibration();
        testV073MultiRegenTorqueRampCreepTaperAndThrottleCancel();
        testV074IpedalFeatheredBrakingLaunchAndRamp();
        testV075LowSpeedIpedalCrawlReverseAndPostWotCoast();
        testV076AutoRegenSteeringScrubAndAutoHoldOffStandstill();
        testV077AutoRegenDetectionHysteresisAndQuantizationDecay();
        testV078BleScanReconnectAndAutoRegenFeatherCalibration();
        testV079AutoRegenTrafficStabilityThrottleOnsetAndTurnBleed();
        testV0710MidDriveAutoToIpedalHighwayFeatherAndStandstill();
        testV0712IpedalCrawlCurveHighwayFeatherAndSharpTurn();
        testV0713ColdStartIpedalForwardCrawlAndUrbanGrade();
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
        eq("Summary banner is v0.7.13",
                Boolean.TRUE, Boolean.valueOf(summary.contains("v0.7.13 Calibration Summary")));
        eq("Summary includes confusion matrix and Pedal Switch Rec.",
                Boolean.TRUE, Boolean.valueOf(summary.contains("TP=2  FP=1  TN=2  FN=1")
                        && summary.contains("Pedal Switch Rec.   : 100.0% (1/1 ticks)")
                        && summary.contains("Pure Regen/SCC Rec. : 50.0% (1/2 ticks)")));
        eq("Summary formats i-Pedal transition cleanly",
                Boolean.TRUE, Boolean.valueOf(summary.contains("MARK #1")
                        && summary.contains("RegenModeEst: L3 -> i-Pedal")
                        && summary.contains("reg=i-Pedal")));
    }

    private static void testV072BleDiscoveryMergePriorityAndEvaluatorCalibration() throws Exception {
        // 1. BLE scan sticky ADV_IND + SCAN_RSP merge, persistent BVM02 MAC memory, & OBD priority sorting
        final ServerSocket mockBle = new ServerSocket(0, 2, InetAddress.getByName("127.0.0.1"));
        final Socket cliSock = new Socket("127.0.0.1", mockBle.getLocalPort());
        final Socket srvSock = mockBle.accept();
        mockBle.close();
        cliSock.setTcpNoDelay(true);
        srvSock.setTcpNoDelay(true);
        final BufferedWriter w = new BufferedWriter(new OutputStreamWriter(srvSock.getOutputStream(), "UTF-8"));
        final List<String> lines = Collections.synchronizedList(new ArrayList<String>());
        Thread acceptThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    BufferedReader rdr = new BufferedReader(new InputStreamReader(srvSock.getInputStream(), "UTF-8"));
                    String l;
                    while ((l = rdr.readLine()) != null) {
                        String trimmed = l.trim();
                        synchronized (lines) {
                            lines.add(trimmed);
                            lines.notifyAll();
                        }
                        if ("STREAM_SESSION".equals(trimmed)) {
                            sendBleServerEvent(w, "OK STREAM_READY");
                        } else if ("CLOSE".equals(trimmed)) {
                            break;
                        }
                    }
                } catch (Throwable ignored) {}
            }
        }, "MockBleV072");
        acceptThread.setDaemon(true);
        acceptThread.start();

        Closeable closeBoth = new Closeable() {
            @Override
            public void close() {
                try { srvSock.close(); } catch (Throwable ignored) {}
                try { cliSock.close(); } catch (Throwable ignored) {}
            }
        };
        InMemoryPairingStore store = new InMemoryPairingStore();
        ObdBrakeClient client = ObdBrakeClient.createForTest(
                store, cliSock.getInputStream(), cliSock.getOutputStream(), closeBoth, 35999);
        try {
            waitForBleLine(lines, "MULTICONN 1", 1000L);

            // Emit 4 generic beacons + 1 BVM02 BEFORE Veepeak so Veepeak arrives at insertion index 5 (> old 4-row cap)
            sendBleServerEvent(w, "ADV CC:00:00:00:00:01 1 -52 020106");
            sendBleServerEvent(w, "ADV CC:00:00:00:00:02 1 -55 020106");
            sendBleServerEvent(w, "ADV CC:00:00:00:00:03 1 -58 020106");
            sendBleServerEvent(w, "ADV CC:00:00:00:00:04 1 -60 020106");
            // BVM02 advertises local name "BVM02" in SCAN_RSP, then later sends an unmerged ADV_IND with 0xFFF0 UUID (0303f0ff) and NO name:
            sendBleServerEvent(w, "ADV BB:BB:BB:BB:BB:02 0 -48 020106060942564d3032");
            sendBleServerEvent(w, "ADV BB:BB:BB:BB:BB:02 0 -47 0201060303f0ff");
            // Veepeak advertises "VEEPEAK" in SCAN_RSP, then later sends an unmerged ADV_IND with 0xFFF0 UUID and NO name:
            sendBleServerEvent(w, "ADV AA:BB:CC:DD:EE:77 0 -65 02010608095645455045414b");
            sendBleServerEvent(w, "ADV AA:BB:CC:DD:EE:77 0 -64 0201060303f0ff");

            long deadline = System.currentTimeMillis() + 1000L;
            while (client.getDiscoveredDevices().size() < 6 && System.currentTimeMillis() < deadline) {
                Thread.sleep(10L);
            }
            List<ObdBrakeClient.DiscoveredDevice> sorted = client.getDiscoveredDevices();
            eq("v0.7.2 priority sort puts VEEPEAK at index 0 ahead of 5 earlier devices",
                    "AA:BB:CC:DD:EE:77", sorted.get(0).mac);
            eq("v0.7.2 sticky ADV merge preserves VEEPEAK name across nameless ADV_IND",
                    "VEEPEAK", sorted.get(0).name);
            eq("v0.7.2 sticky ADV merge keeps VEEPEAK likelyObd=true",
                    Boolean.TRUE, Boolean.valueOf(sorted.get(0).likelyObd && !sorted.get(0).likelyBvm02));

            ObdBrakeClient.DiscoveredDevice lastDev = sorted.get(sorted.size() - 1);
            eq("v0.7.2 priority sort sinks BVM02 to bottom of list",
                    "BB:BB:BB:BB:BB:02", lastDev.mac);
            eq("v0.7.2 BVM02 never masquerades as likelyObd on nameless 0xFFF0 ADV_IND",
                    Boolean.TRUE, Boolean.valueOf(lastDev.likelyBvm02 && !lastDev.likelyObd));
            eq("v0.7.2 pairAdapter refuses knownBvm02Mac even if passed generic name",
                    Boolean.FALSE, Boolean.valueOf(client.pairAdapter("BB:BB:BB:BB:BB:02", "BLE Device", 0)));

            // 2. Transient empty GATT_READY on VEEPEAK retries instead of permanently latching bleGattRejected
            eq("v0.7.2 pairAdapter accepts VEEPEAK",
                    Boolean.TRUE, Boolean.valueOf(client.pairAdapter("AA:BB:CC:DD:EE:77", "VEEPEAK", 0)));
            waitForBleLine(lines, "@AA:BB:CC:DD:EE:77 CONNECT 0", 1000L);
            lines.clear();
            sendBleServerEvent(w, "@AA:BB:CC:DD:EE:77 CONN_STATE CONNECTED AA:BB:CC:DD:EE:77 0x0042");
            sendBleServerEvent(w, "@AA:BB:CC:DD:EE:77 GATT_READY AA:BB:CC:DD:EE:77 mtu=23");
            eq("v0.7.2 empty GATT_READY disconnects for clean retry",
                    Boolean.TRUE, Boolean.valueOf(waitForBleLine(lines, "@AA:BB:CC:DD:EE:77 DISCONNECT", 1000L)));
            eq("v0.7.2 empty GATT_READY does NOT latch bleGattRejected on VEEPEAK",
                    Boolean.FALSE, Boolean.valueOf(client.isBleGattRejectedForTest()));
            eq("v0.7.2 empty GATT_READY surfaces retrying status",
                    "BLE GATT incomplete; retrying...", client.getStatusSummary());
        } finally {
            client.close();
        }

        // 3. BrakeEvaluator: L1 foot-brake stop -> pedal release at 10 km/h does NOT corrupt L1->i-Pedal or drag REGEN_DECEL
        IndicatorConfig cfg = IndicatorConfig.defaults();
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 10000L;
        float spd = 64.0f;
        BrakeEvaluator.Snapshot snap = null;
        for (int i = 0; i <= 26; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.21f; // ~0.58 m/s^2 L1 coast
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.2 learned L1 before foot brake", Integer.valueOf(1), Integer.valueOf(snap.inferredRegenLevel));

        // Foot-brake down from 22 -> 10 km/h at ~2.5 m/s^2 (prsr = 8.0 bar)
        spd = 22.0f;
        for (int i = 0; i < 24; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.90f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 8.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        // Release foot brake (prsr -> 0.0) while rolling at ~11.2 km/h: must turn OFF immediately and stay L1!
        for (int i = 0; i < 6; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.2 foot-brake release in L1 does NOT corrupt RegenModeEst to i-Pedal",
                Integer.valueOf(1), Integer.valueOf(snap.inferredRegenLevel));
        eq("v0.7.2 foot-brake release in L1 clears hydraulic -dv/dt tail immediately",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 4. Park-carryover EPB auto-release hold stays STANDSTILL_HOLD until throttle >= 1.5%
        BrakeEvaluator evEpb = new BrakeEvaluator();
        t = 50000L;
        evEpb.evaluate(cfg, new BrakeEvaluator.Input(
                t, 18.0f, 0.0f, 0.0f, 0.0f, Boolean.TRUE, Boolean.TRUE, Boolean.FALSE, 80.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 100L;
        // Shift Park -> Drive with foot on brake while EPB=true is still engaged from Park
        evEpb.evaluate(cfg, new BrakeEvaluator.Input(
                t, 18.0f, 0.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.TRUE, Boolean.FALSE, 80.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 100L;
        // Release foot brake (prsr -> 0.0) while Park-carryover EPB auto-releases (obd=44 in real vehicle logs)
        BrakeEvaluator.Snapshot epbHold = evEpb.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.TRUE, Boolean.FALSE, 80.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        eq("v0.7.2 Park-carryover EPB auto-release holds STANDSTILL_HOLD",
                Boolean.TRUE, Boolean.valueOf(epbHold.modelIlluminated && "STANDSTILL_HOLD".equals(epbHold.modelSource)));
        t += 100L;
        BrakeEvaluator.Snapshot epbGo = evEpb.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 2.7f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 80.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        eq("v0.7.2 throttle >= 1.5% releases Park-carryover EPB hold",
                Boolean.FALSE, Boolean.valueOf(epbGo.modelIlluminated));
    }

    private static void testV073MultiRegenTorqueRampCreepTaperAndThrottleCancel() {
        IndicatorConfig cfg = IndicatorConfig.defaults();

        // 1. Unlearned post-acceleration L2 coasting waits for forward acceleration (< -0.55 m/s^2)
        //    to subside before starting coastStartMs -> classifies directly ? -> L2 without false L1 step!
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 10000L;
        float spd = 48.0f;
        BrakeEvaluator.Snapshot snap = null;
        // Brisk forward acceleration (~ -1.35 m/s^2 decel) from 48 -> 52 km/h at 25% throttle
        for (int i = 0; i < 8; i++) {
            spd += 0.50f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 25.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        // Lift off at t: VCU takes ~400ms to ramp +drive -> 0 before settled ~1.05 m/s^2 L2 coast begins
        for (int i = 0; i < 4; i++) {
            spd += 0.20f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        boolean sawFalseL1 = false;
        for (int i = 0; i < 16; i++) {
            spd -= 0.39f; // ~1.08 m/s^2 settled L2 coast
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.inferredRegenLevel == 1) {
                sawFalseL1 = true;
            }
            t += 100L;
        }
        eq("v0.7.3 post-accel coast avoids false intermediate L1 classification",
                Boolean.FALSE, Boolean.valueOf(sawFalseL1));
        eq("v0.7.3 post-accel coast classifies directly as L2",
                Integer.valueOf(2), Integer.valueOf(snap.inferredRegenLevel));

        // 2. Step up to L3 (~1.65 m/s^2), accelerate briskly (-1.5 m/s^2), then lift off:
        //    - During initial <550ms VCU +drive->0 torque ramp: brake lamps stay OFF (matching 22BC06=0)
        //    - At >=550ms into lift-off: lights ON and stays ON continuously without mid-ramp notch!
        for (int i = 0; i < 16; i++) {
            spd -= 0.60f; // ~1.67 m/s^2 settled L3 coast
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.3 learned L3 mode", Integer.valueOf(3), Integer.valueOf(snap.inferredRegenLevel));

        // Brisk acceleration from 36 -> 42 km/h at 32% throttle (~ -1.67 m/s^2 decel)
        spd = 36.0f;
        for (int i = 0; i < 10; i++) {
            spd += 0.60f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 32.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        // Full lift-off from brisk acceleration: at +200ms and +500ms (<550ms ramp), must still be OFF!
        BrakeEvaluator.Snapshot snap200 = null;
        BrakeEvaluator.Snapshot snap500 = null;
        for (int i = 0; i < 6; i++) {
            if (i < 2) spd += 0.20f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (i == 2) snap200 = snap;
            if (i == 5) snap500 = snap;
            t += 100L;
        }
        eq("v0.7.3 L3 lift-off from brisk accel stays OFF at +200ms during VCU torque ramp",
                Boolean.FALSE, Boolean.valueOf(snap200.modelIlluminated));
        eq("v0.7.3 L3 lift-off from brisk accel stays OFF at +500ms during VCU torque ramp",
                Boolean.FALSE, Boolean.valueOf(snap500.modelIlluminated));

        // From +600ms through +1600ms as regen builds 0.45 -> 1.75 m/s^2: lights ON with ZERO dropout notch!
        boolean allOnAfterRamp = true;
        for (int i = 0; i < 11; i++) {
            spd -= (i < 3) ? 0.22f : 0.64f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (!snap.modelIlluminated) {
                allOnAfterRamp = false;
            }
            t += 100L;
        }
        eq("v0.7.3 L3 lift-off lights after 550ms ramp and holds with zero mid-ramp notch",
                Boolean.TRUE, Boolean.valueOf(allOnAfterRamp));

        // 3. Light feathering (7.0% < 7.5%) during strong L3 decel keeps REGEN_DECEL ON,
        //    whereas deliberate >=13.0% throttle (13.7%) cancels REGEN_DECEL immediately!
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd - 0.60f, 7.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        eq("v0.7.3 L3 light 7.0% feathering keeps REGEN_DECEL ON",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));
        t += 100L;
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd - 1.10f, 13.7f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        eq("v0.7.3 L3 >=13.0% throttle cancels strong-decel REGEN_DECEL immediately",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 4. Low-speed (<14.2 km/h) L3 creep taper (~0.70 m/s^2 at 13.5 km/h) releases REGEN_DECEL cleanly
        //    without corrupting L3 -> i-Pedal:
        t += 200L;
        spd = 24.0f;
        for (int i = 0; i < 16; i++) {
            spd -= 0.60f; // coast down from 24 -> 14.4 km/h in L3
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.3 L3 regen stays ON above 14.2 km/h", Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));
        for (int i = 0; i < 8; i++) {
            spd -= 0.25f; // ~0.69 m/s^2 E-GMP creep taper across 14.1 -> 12.1 km/h
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.3 L3 low-speed (<14.2 km/h) creep taper releases REGEN_DECEL",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));
        eq("v0.7.3 L3 creep taper preserves L3 mode (does NOT false-promote to i-Pedal)",
                Integer.valueOf(3), Integer.valueOf(snap.inferredRegenLevel));
    }

    private static void testV074IpedalFeatheredBrakingLaunchAndRamp() {
        IndicatorConfig cfg = IndicatorConfig.defaults();

        // 1. Verify piecewise speed-dependent i-Pedal onset & release pedal curves:
        eq("v0.7.5 ipedalOnsetPedalPct at 5 km/h is 5.5%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(5.0f) - 5.5f) < 0.01f));
        eq("v0.7.5 ipedalOnsetPedalPct at 17 km/h is 11.0%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(17.0f) - 11.0f) < 0.01f));
        eq("v0.7.5 ipedalOnsetPedalPct at 24 km/h is 13.4%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(24.0f) - 13.4f) < 0.01f));
        eq("v0.7.4 ipedalOnsetPedalPct at 55 km/h is 16.0%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(55.0f) - 16.0f) < 0.01f));
        eq("v0.7.12 ipedalReleasePedalPct at 8 km/h is 9.6%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(8.0f) - 9.6f) < 0.01f));
        eq("v0.7.5 ipedalReleasePedalPct at 21 km/h is 16.8%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(21.0f) - 16.8f) < 0.01f));
        eq("v0.7.4 ipedalReleasePedalPct at 55 km/h is 18.6%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(55.0f) - 18.6f) < 0.01f));

        // 2. Brisk L1 launch at 33% pedal (-2.5 m/s^2 = +0.90 km/h per 100 ms) MUST NOT trigger i-Pedal:
        BrakeEvaluator evL1 = new BrakeEvaluator();
        long t = 600000L;
        float spd = 9.0f;
        BrakeEvaluator.Snapshot snap = null;
        for (int i = 0; i < 20; i++) {
            spd += 0.90f;
            snap = evL1.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 33.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.4 brisk L1 launch (-2.5 m/s^2 at 33% pedal) stays UNKNOWN (-1)",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_UNKNOWN), Integer.valueOf(snap.inferredRegenLevel));

        // 3. Post-Park i-Pedal urban launch (33.5% pedal yielding moderate -1.11 m/s^2 = +0.40 km/h per 100 ms):
        BrakeEvaluator evIp = new BrakeEvaluator();
        t = 700000L;
        spd = 9.0f;
        for (int i = 0; i < 20; i++) {
            spd += 0.40f;
            snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 33.5f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.4 post-Park gentle urban launch at 33.5% pedal auto-learns i-Pedal (4)",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_IPEDAL), Integer.valueOf(snap.inferredRegenLevel));

        // 4. Feathered one-pedal braking at 55 km/h (14.5% pedal, mild ~0.44 m/s^2 = -0.16 km/h per 100 ms):
        t += 2500L;
        spd = 56.0f;
        // Settle cruise first at 24% neutral pedal
        for (int i = 0; i < 6; i++) {
            snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 24.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        // Ease down to 14.5% feathered braking
        for (int i = 0; i < 6; i++) {
            spd -= 0.16f;
            snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 14.5f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.4 i-Pedal feathered braking (14.5% pedal, ~0.44 m/s^2 at 55 km/h) lights REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));

        // Feather slightly up into hysteresis band (17.2% < 18.6% release at 55 km/h) -> holds ON!
        spd -= 0.12f;
        snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd, 17.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 100L;
        eq("v0.7.4 i-Pedal hysteresis band (17.2% < 18.6% at 55 km/h) holds REGEN_DECEL ON",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));

        // Press back to neutral (19.2% >= 18.6%) -> releases immediately!
        snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd, 19.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 100L;
        eq("v0.7.4 i-Pedal pressing back to 19.2% neutral releases REGEN_DECEL immediately",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 5. Brief 400 ms VCU ramp (0.45 m/s^2 at 12 km/h) does NOT falsely demote i-Pedal -> L3,
        //    and 0.8 km/h bridges seamlessly into 0.0 km/h STANDSTILL_HOLD:
        t += 2500L;
        spd = 12.8f;
        for (int i = 0; i < 4; i++) {
            spd -= 0.16f; // ~0.44 m/s^2 for 400 ms (< 780 ms CREEP_TAPER_CONFIRM_MS)
            snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.4 400ms VCU ramp at 12 km/h preserves i-Pedal (4) without false L3 dip",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_IPEDAL), Integer.valueOf(snap.inferredRegenLevel));

        // Decelerate all the way through 0.8 km/h (< 1.2 km/h!) into 0.0 km/h:
        float[] stopProfile = { 9.6f, 6.4f, 3.2f, 1.6f, 0.8f, 0.0f };
        boolean allLitToZero = true;
        for (int i = 0; i < stopProfile.length; i++) {
            float s = stopProfile[i];
            snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, s, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE,
                    Boolean.valueOf(s >= 1.6f), 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (!snap.modelIlluminated) {
                allLitToZero = false;
            }
            t += 100L;
        }
        eq("v0.7.4 i-Pedal stays continuously lit across 0.8 km/h into 0.0 km/h STANDSTILL_HOLD",
                Boolean.TRUE, Boolean.valueOf(allLitToZero && "STANDSTILL_HOLD".equals(snap.modelSource)));

        // 6. Reverse 0%-pedal creep acceleration (0 -> 9.7 km/h at 0% pedal, -0.75 m/s^2) stays OFF:
        t += 2500L;
        spd = 5.6f;
        boolean reverseCreepLit = false;
        for (int i = 0; i < 10; i++) {
            spd += 0.28f; // accelerating while acc == 0.0%
            snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) {
                reverseCreepLit = true;
            }
            t += 100L;
        }
        eq("v0.7.4 Reverse 0%-pedal creep acceleration never false-triggers REGEN_DECEL",
                Boolean.FALSE, Boolean.valueOf(reverseCreepLit));
    }

    private static void testV075LowSpeedIpedalCrawlReverseAndPostWotCoast() {
        IndicatorConfig cfg = IndicatorConfig.defaults();

        // 1. Gentle urban creep-up cruise (25.5% pedal across 34 -> 35.4 km/h, ~ -0.28 m/s^2 for 1.3 s)
        //    auto-learns i-Pedal (4) early (matching 131433.csv t=43.7s):
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 800000L;
        float spd = 34.0f;
        BrakeEvaluator.Snapshot snap = null;
        for (int i = 0; i < 16; i++) {
            spd += 0.10f; // -0.28 m/s^2 gentle creep-up
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 25.5f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.5 gentle 25.5% urban creep-up cruise auto-learns i-Pedal (4)",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_IPEDAL), Integer.valueOf(snap.inferredRegenLevel));

        // 2. Low-speed UN ECE R13-H parking-lot / neighborhood crawl in i-Pedal:
        //    Holding 10.5% pedal at 12 -> 10 km/h (~0.55 m/s^2) stays OFF (22BC06 = 0),
        //    whereas 0% pedal full-stop regen (~1.65 m/s^2) lights ON, and pressing back to 11.8%
        //    at 8.0 km/h cancels REGEN_DECEL immediately even with lagged >1.8 m/s^2 decel!
        t += 2500L;
        spd = 12.5f;
        boolean lowSpdCrawlLit = false;
        for (int i = 0; i < 10; i++) {
            spd -= 0.20f; // ~0.55 m/s^2 gentle crawl at 10.5% pedal (> 5.5..8.9% onset curve)
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 10.5f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) {
                lowSpdCrawlLit = true;
            }
            t += 100L;
        }
        eq("v0.7.5 low-speed i-Pedal crawl (10.5% pedal, ~0.55 m/s^2 at 10-12 km/h) stays OFF",
                Boolean.FALSE, Boolean.valueOf(lowSpdCrawlLit));

        // Lift to 0% pedal for strong low-speed regen (~1.67 m/s^2 down to 8.1 km/h) -> lights ON:
        for (int i = 0; i < 4; i++) {
            spd -= 0.60f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        eq("v0.7.5 0%-pedal low-speed i-Pedal stop (~1.67 m/s^2 at 8 km/h) lights REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));

        // Press crawl throttle back to 11.8% (>= 11.4% strong-cancel at 8.0 km/h) while speed step
        // still shows lagged strong decel -> cancels immediately (matching 131433.csv t=393.0s):
        spd -= 0.65f;
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd, 11.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 100L;
        eq("v0.7.5 pressing 11.8% crawl throttle at <8 km/h cancels lagged strong decel immediately",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 3. Post-WOT acceleration crest (131433.csv t=385.8..388.5s):
        //    99.6% WOT pulse -> 0% lift-off at 21.7 km/h while delayed cluster speed surges to 30.6 km/h
        //    (-3.3 m/s^2) after coastStartMs began, followed by i-Pedal regen down to 24 km/h ->
        //    resets coastStartMs across the crest and NEVER false-demotes i-Pedal to L0 or L1!
        t += 2500L;
        spd = 20.5f;
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd, 99.6f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 100L;
        boolean sawFalseLowRegen = false;
        // 0% lift-off while cluster speed surges 21.7 -> 30.5 km/h over 800 ms (~ -3.05 m/s^2)
        spd = 21.7f;
        for (int i = 0; i < 8; i++) {
            if (i > 0) spd += 1.25f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.inferredRegenLevel != BrakeEvaluator.REGEN_LEVEL_IPEDAL) {
                sawFalseLowRegen = true;
            }
            t += 100L;
        }
        // Now decelerate in i-Pedal from 30.5 -> 22.1 km/h (~1.75 m/s^2) over 1.4 s:
        for (int i = 0; i < 14; i++) {
            spd -= 0.60f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.inferredRegenLevel != BrakeEvaluator.REGEN_LEVEL_IPEDAL) {
                sawFalseLowRegen = true;
            }
            t += 100L;
        }
        eq("v0.7.5 post-WOT speed surge resets coastStartMs and preserves i-Pedal (zero L0/L1 glitch)",
                Boolean.FALSE, Boolean.valueOf(sawFalseLowRegen));

        // 4. Reverse 0%-pedal steady 7.2 km/h creep plateau (0.00 m/s^2 for 1.5 s after reverse creep accel)
        //    and <4.5 km/h mild post-foot-brake release bleed stay OFF (matching 131433.csv t=480.8s & 485.4s):
        t += 2500L;
        spd = 4.0f;
        boolean reversePlateauOrBleedLit = false;
        for (int i = 0; i < 8; i++) {
            spd += 0.40f; // Reverse creep up to 7.2 km/h
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) reversePlateauOrBleedLit = true;
            t += 100L;
        }
        // Hold steady 7.2 km/h plateau at 0.0% pedal for 1.5 s (smoothedDecelMps2 == 0.0f):
        for (int i = 0; i < 15; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, 7.2f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) reversePlateauOrBleedLit = true;
            t += 100L;
        }
        // Foot-brake down to 3.8 km/h, then release foot brake with mild 0.55 m/s^2 bleed:
        for (int i = 0; i < 8; i++) {
            spd -= 0.42f;
            ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 6.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 100L;
        }
        for (int i = 0; i < 8; i++) {
            spd -= 0.18f; // mild ~0.50 m/s^2 rolling tail below 4.5 km/h after releasing foot brake
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) reversePlateauOrBleedLit = true;
            t += 100L;
        }
        eq("v0.7.5 Reverse 7.2 km/h steady creep plateau and <4.5 km/h post-brake bleed stay OFF",
                Boolean.FALSE, Boolean.valueOf(reversePlateauOrBleedLit));
    }

    private static void testV076AutoRegenSteeringScrubAndAutoHoldOffStandstill() {
        IndicatorConfig cfg = IndicatorConfig.defaults();

        // 1. EPB caliper actuator mid-release (parking_brake: true -> null at 277.25s in 205022.csv)
        //    and 0%-throttle idle creep (0.0 -> 0.8 -> 1.6 -> 2.4 km/h) confirming console AUTO HOLD is OFF:
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 900000L;
        BrakeEvaluator.Snapshot snap = null;
        // Start in Park with foot on brake + EPB=true, shift to Drive, release foot brake:
        ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 32.0f, 0.0f, 0.0f, -12.0f, Boolean.TRUE, Boolean.TRUE, Boolean.FALSE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.0f, -12.0f, Boolean.FALSE, Boolean.TRUE, Boolean.FALSE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        // EPB actuator begins winding back (parking_brake transitions TRUE -> null): releases STANDSTILL_HOLD!
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.0f, -12.0f, Boolean.FALSE, null, Boolean.FALSE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        eq("v0.7.6 parking_brake TRUE->null actuator release clears STANDSTILL_HOLD",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // Vehicle creeps forward 0.0 -> 0.8 -> 1.6 -> 2.4 km/h at 0% throttle & 0 brake pressure:
        float[] creepSpds = { 0.8f, 0.8f, 1.6f, 1.6f, 2.4f };
        for (int i = 0; i < creepSpds.length; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, creepSpds[i], 0.0f, -12.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }

        // Subsequent drive-through stop: brake 12 -> 0 km/h with foot brake, then release foot brake (prsr -> 0.0):
        // Because AUTO HOLD was confirmed OFF by idle creep, STANDSTILL_HOLD releases immediately!
        float spd = 12.0f;
        for (int i = 0; i < 16; i++) {
            spd = Math.max(0.0f, spd - 0.8f);
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 8.5f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.valueOf(spd > 0.5f), 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.6 drive-through stop with foot on brake lights BRAKE_PRESSURE",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "BRAKE_PRESSURE".equals(snap.modelSource)));
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        eq("v0.7.6 releasing foot brake at 0 km/h with Auto-Hold confirmed OFF releases immediately",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 2. Learn L0 coasting, verify 5.5-second constant-speed slight-downgrade coast in L0 does NOT false-trigger SCC:
        t += 2500L;
        spd = 68.4f;
        for (int i = 0; i < 40; i++) {
            if (i > 0 && i % 16 == 0) spd -= 0.8f; // ~0.28 m/s^2 total (~0.22 m/s^2 motor/rolling) flat-road L0 coast
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.6 learned L0 coasting mode", Integer.valueOf(0), Integer.valueOf(snap.inferredRegenLevel));
        for (int i = 0; i < 110; i++) { // 5.5 s steady 66.8 km/h slight-downgrade coast in L0 (< 9.0 s L0 guard)
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.6 5.5s constant-speed slight-downgrade coast in L0 does NOT false-activate SCC",
                Boolean.FALSE, Boolean.valueOf(snap.sccActive));

        // 3. Full-lock parking-lot steering scrub (|steering.angle| = 479 deg at 13.7 -> 8.8 km/h in L0, ~1.76 m/s^2):
        //    subtracts passive Ackermann tire scrub so REGEN_DECEL stays OFF and L0 does NOT jump to i-Pedal!
        t += 2500L;
        spd = 13.7f;
        boolean scrubLit = false;
        for (int i = 0; i < 18; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.62f; // ~1.72 m/s^2 passive full-lock scrub drag
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 479.4f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) scrubLit = true;
            t += 50L;
        }
        eq("v0.7.6 full-lock 479 deg parking turn scrub in L0 stays OFF",
                Boolean.FALSE, Boolean.valueOf(scrubLit));
        eq("v0.7.6 full-lock 479 deg parking turn scrub preserves L0 (does NOT false-promote to i-Pedal)",
                Integer.valueOf(0), Integer.valueOf(snap.inferredRegenLevel));

        // 4. Post-foot-brake >= 20 km/h strong Auto-Regen/blended continuation (28 -> 24 km/h at ~3.0 m/s^2, 22BC06=36/40)
        //    stays ON without a 650 ms dropout, and pressing >= 2.0% throttle cancels immediately:
        t += 2500L;
        spd = 34.0f;
        for (int i = 0; i < 12; i++) {
            if (i > 0 && i % 2 == 0) spd -= 1.10f; // ~3.05 m/s^2
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 4.5f, spd, 0.0f, 20.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        // Release foot brake (prsr -> 0.0) while Auto Regen keeps pulling ~3.0 m/s^2 at ~28 km/h:
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd - 0.55f, 0.0f, 20.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        eq("v0.7.6 >=20 km/h strong Auto-Regen continuation after foot-brake lift stays lit (zero dropout notch)",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));
        // Press gentle 2.5% throttle right after brake release in L0 -> cancels immediately!
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd - 0.55f, 2.5f, 20.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        eq("v0.7.6 pressing 2.5% throttle in L0 cancels post-brake regen immediately",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 5. Auto-Regen radar L2 taper release (< 0.82 m/s^2 after >= 1.5 s) and stale L2 -> L0 step-down on < 2% feather coast:
        t += 2500L;
        spd = 46.0f;
        for (int i = 0; i < 36; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.38f; // ~1.05 m/s^2 Auto-Regen radar L2 braking
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.6 Auto-Regen radar braking learns L2 and lights REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.inferredRegenLevel == 2 && snap.modelIlluminated));
        // Radar tapers regen down to ~0.65 m/s^2 (< 0.82 m/s^2) after 1.8 s of 0%-pedal coast:
        for (int i = 0; i < 12; i++) {
            if (i % 4 == 0) spd -= 0.45f; // ~0.62 m/s^2
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.6 settled L2 Auto-Regen taper (<0.82 m/s^2) releases REGEN_DECEL promptly",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));
        // Driver holds 1.4% feather pedal at 65 km/h with ~0.18 m/s^2 passive coast for 1.8 s -> steps L2 down to L0!
        t += 2500L;
        spd = 65.2f;
        for (int i = 0; i < 36; i++) {
            if (i == 8 || i == 20 || i == 32) spd -= 0.4f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 1.4f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.6 near-zero decel at 1.4% feather pedal above 25 km/h steps stale L2 back to L0",
                Integer.valueOf(0), Integer.valueOf(snap.inferredRegenLevel));
    }

    private static void testV077AutoRegenDetectionHysteresisAndQuantizationDecay() {
        eq("v0.7.7 regenLevelLabel L0 Auto", "L0 (Auto)", BrakeEvaluator.regenLevelLabel(0, false, true));
        eq("v0.7.7 regenLevelLabel L2 Auto", "L2 (Auto)", BrakeEvaluator.regenLevelLabel(2, false, true));
        eq("v0.7.7 regenLevelLabel SCC overrides Auto", "L0 (SCC)", BrakeEvaluator.regenLevelLabel(0, true, true));
        eq("v0.7.7 regenLevelLabel i-Pedal ignores Auto", "i-Pedal", BrakeEvaluator.regenLevelLabel(4, false, true));

        IndicatorConfig cfg = IndicatorConfig.defaults();
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 1000L;
        BrakeEvaluator.Snapshot snap = null;

        // 1. Intra-coast Auto-Regen modulation: open-road L0 coast (~0.18 m/s^2) ramping into radar L2 (~1.10 m/s^2)
        float spd = 68.0f;
        for (int i = 0; i < 36; i++) {
            if (i > 0 && i % 8 == 0) spd -= 0.25f; // ~0.17 m/s^2 L0 open-road coast
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        // Lead car encountered in same coast -> radar ramps regen to ~1.12 m/s^2
        for (int i = 0; i < 34; i++) {
            if (i > 0 && i % 4 == 0) spd -= 0.80f; // 0.8 km/h every 200 ms = 1.11 m/s^2
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.7 intra-coast L0->L2 radar modulation latches autoRegenActive and REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.autoRegenActive && snap.inferredRegenLevel >= 2 && snap.modelIlluminated));

        // 2. Pressing >= 3.5% throttle in Auto Regen resets temporary radar L2/L3 back to base L0,
        //    and subsequent open-road lift-off at 72 km/h (0.0 m/s^2) does NOT false-fire liftoffAnticipation!
        spd = 72.4f;
        for (int i = 0; i < 16; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 12.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.7 resuming throttle in Auto Regen resets temporary radar level back to L0",
                Integer.valueOf(0), Integer.valueOf(snap.inferredRegenLevel));
        boolean anyOpenRoadLiftoffFp = false;
        for (int i = 0; i < 18; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) anyOpenRoadLiftoffFp = true;
            t += 50L;
        }
        eq("v0.7.7 open-road 0%-pedal lift-off in Auto Regen produces zero liftoffAnticipation FPs",
                Boolean.FALSE, Boolean.valueOf(anyOpenRoadLiftoffFp));

        // 3. Auto-Regen moderate radar braking (15..48 km/h): ~0.89 m/s^2 lights REGEN_DECEL and holds down to ~0.67 m/s^2,
        //    whereas above 48 km/h, tapering below 0.75 m/s^2 releases promptly.
        t += 3000L;
        spd = 34.0f;
        for (int i = 0; i < 6; i++) {
            ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 12.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 0; i < 19; i++) {
            if (i % 3 == 0) spd -= 0.48f; // 0.48 km/h every 150 ms = 0.89 m/s^2
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.7 Auto-Regen ~0.89 m/s^2 radar braking at 32 km/h lights REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));
        for (int i = 1; i <= 18; i++) {
            if (i % 3 == 0) spd -= 0.36f; // 0.36 km/h every 150 ms = exact 0.67 m/s^2 (>= 0.60 m/s^2 mid-speed hold)
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.7 Auto-Regen holds REGEN_DECEL at ~0.67 m/s^2 between 15 and 48 km/h",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));

        // Sub-latch 0.20 bar brake brush during active 1.35 m/s^2 regen must NOT trigger postFootBrakeDecelBleed:
        for (int i = 1; i <= 12; i++) {
            if (i % 3 == 0) spd -= 0.80f; // ~1.48 m/s^2
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.20f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.7 sub-latch 0.20 bar pedal brush does NOT drop active REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));

        // Above 39.5 km/h in Auto Regen, tapering motor decel below 0.60 m/s^2 (~0.51 m/s^2) releases REGEN_DECEL promptly:
        t += 3000L;
        spd = 58.0f;
        for (int i = 0; i < 6; i++) {
            ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 14.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 18; i++) {
            if (i % 3 == 0) spd -= 0.62f; // ~1.15 m/s^2 total (~1.06 m/s^2 motor) -> lights REGEN_DECEL
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 24; i++) {
            if (i % 3 == 0) spd -= 0.32f; // ~0.59 m/s^2 total (~0.51 m/s^2 motor < 0.60 m/s^2)
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.8 Auto-Regen releases REGEN_DECEL below 0.60 m/s^2 motor decel at >= 39.5 km/h",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 4. 20 Hz flat-speed quantization ceiling & gentle standstill feather hold:
        //    Braking at ~1.9 m/s^2 to 12.1 km/h then releasing to flat 12.1 km/h must bound decelMps2 and never false-promote i-Pedal.
        BrakeEvaluator evQuant = new BrakeEvaluator();
        long tq = 1000L;
        float qSpd = 18.5f;
        for (int i = 0; i < 20; i++) {
            if (i > 0 && i % 3 == 0) qSpd -= 0.80f;
            snap = evQuant.evaluate(cfg, new BrakeEvaluator.Input(
                    tq, 2.4f, qSpd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tq));
            tq += 50L;
        }
        // Release foot brake cleanly while speed sits flat at qSpd (~13.7 km/h -> 12.1 km/h) for 500 ms:
        for (int i = 0; i < 10; i++) {
            snap = evQuant.evaluate(cfg, new BrakeEvaluator.Input(
                    tq, 0.0f, qSpd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tq));
            tq += 50L;
        }
        eq("v0.7.7 20 Hz flat-speed ceiling decays stale decel (< 0.70 m/s^2) and prevents false i-Pedal promotion",
                Boolean.TRUE, Boolean.valueOf(snap.decelMps2 < 0.70f && snap.inferredRegenLevel != 4));

        // Gentle < 0.85 bar feather stop (0.50 -> 0.30 -> 0.22 bar at 0.0 km/h) holds BRAKE_PRESSURE down to 0.18 bar:
        qSpd = 2.4f;
        for (int i = 0; i < 8; i++) {
            if (i % 2 == 1 && qSpd > 0.0f) qSpd = Math.max(0.0f, qSpd - 0.80f);
            float prsr = (i < 3) ? 0.50f : 0.22f;
            snap = evQuant.evaluate(cfg, new BrakeEvaluator.Input(
                    tq, prsr, qSpd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.valueOf(qSpd > 0.1f), 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tq));
            tq += 50L;
        }
        eq("v0.7.7 gentle <=0.85 bar feather stop holds BRAKE_PRESSURE at 0.22 bar at 0.0 km/h",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "BRAKE_PRESSURE".equals(snap.modelSource)));

        // 5. Exiting Auto Regen cleanly after 3 consecutive fixed manual L2/L3 coasts:
        for (int c = 0; c < 3; c++) {
            t += 3000L;
            spd = 58.0f;
            for (int i = 0; i < 10; i++) {
                ev.evaluate(cfg, new BrakeEvaluator.Input(
                        t, 0.0f, spd, 18.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
                t += 50L;
            }
            for (int i = 1; i <= 42; i++) {
                if (i % 3 == 0) spd -= 0.75f; // steady ~1.39 m/s^2 fixed manual L3 coast throughout
                snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                        t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
                t += 50L;
            }
        }
        eq("v0.7.7 3 consecutive fixed high-decel manual coasts exit autoRegenActive cleanly",
                Boolean.FALSE, Boolean.valueOf(snap.autoRegenActive));
    }

    private static void testV078BleScanReconnectAndAutoRegenFeatherCalibration() throws Exception {
        // 1. ObdBrakeClient v0.7.8: uninterrupted manual BLE scan guard, reconnect 0x16 backoff suppression,
        //    non-advertising scan summary, and deduplicated DiagnosticLogger events:
        final List<String> lines = new ArrayList<String>();
        final List<String> diagnostics = Collections.synchronizedList(new ArrayList<String>());
        final ServerSocket ss = new ServerSocket(0, 2, InetAddress.getByName("127.0.0.1"));
        final Socket cliSock = new Socket("127.0.0.1", ss.getLocalPort());
        final Socket srvSock = ss.accept();
        ss.close();
        cliSock.setTcpNoDelay(true);
        srvSock.setTcpNoDelay(true);
        final BufferedWriter w = new BufferedWriter(new OutputStreamWriter(srvSock.getOutputStream(), "UTF-8"));

        Thread srvReader = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    BufferedReader r = new BufferedReader(new InputStreamReader(srvSock.getInputStream(), "UTF-8"));
                    String ln;
                    while ((ln = r.readLine()) != null) {
                        String trimmed = ln.trim();
                        synchronized (lines) {
                            lines.add(trimmed);
                            lines.notifyAll();
                        }
                        if ("STREAM_SESSION".equals(trimmed)) {
                            sendBleServerEvent(w, "OK STREAM_READY");
                        } else if ("CLOSE".equals(trimmed)) {
                            break;
                        }
                    }
                } catch (Throwable ignored) {
                } finally {
                    try { w.close(); } catch (Throwable ignored) {}
                    try { srvSock.close(); } catch (Throwable ignored) {}
                }
            }
        }, "MockV078BleServer");
        srvReader.setDaemon(true);
        srvReader.start();

        Closeable closeBoth = new Closeable() {
            @Override
            public void close() {
                try { srvSock.close(); } catch (Throwable ignored) {}
                try { cliSock.close(); } catch (Throwable ignored) {}
            }
        };
        InMemoryPairingStore store = new InMemoryPairingStore();
        ObdBrakeClient client = ObdBrakeClient.createForTest(
                store, cliSock.getInputStream(), cliSock.getOutputStream(), closeBoth, 35998);
        client.setDiagnosticLogger(new ObdBrakeClient.DiagnosticLogger() {
            @Override
            public void onDiagnostic(long monoMs, String message) {
                diagnostics.add(message);
            }
        });
        try {
            waitForBleLine(lines, "MULTICONN 1", 1000L);
            eq("v0.7.8 pairAdapter accepts VEEPEAK",
                    Boolean.TRUE, Boolean.valueOf(client.pairAdapter("AA:BB:CC:DD:EE:88", "VEEPEAK", 0)));
            waitForBleLine(lines, "@AA:BB:CC:DD:EE:88 CONNECT 0", 1000L);

            // Tapping Reconnect sends DISCONNECT, and BleCentral echoes CONN_STATE DISCONNECTED 0x0016:
            // v0.7.8 must NOT penalize self-initiated disconnect with bleTimeoutStreak=1 or overwrite statusSummary!
            lines.clear();
            client.requestReconnect();
            eq("v0.7.8 requestReconnect sends DISCONNECT",
                    Boolean.TRUE, Boolean.valueOf(waitForBleLine(lines, "@AA:BB:CC:DD:EE:88 DISCONNECT", 1000L)));
            sendBleServerEvent(w, "@AA:BB:CC:DD:EE:88 CONN_STATE DISCONNECTED AA:BB:CC:DD:EE:88 0x0016");
            Thread.sleep(50L);
            eq("v0.7.8 self-disconnect 0x16 does NOT penalize effectiveConnectAddrType to -1",
                    Integer.valueOf(0), Integer.valueOf(client.effectiveConnectAddrType()));
            eq("v0.7.8 self-disconnect 0x16 does NOT overwrite Reconnecting status with 0x16 error",
                    Boolean.FALSE, Boolean.valueOf(client.getStatusSummary().contains("0x16")));

            // Simulate real CONNECT_TIMEOUT -> logs diagnostic milestone
            sendBleServerEvent(w, "@AA:BB:CC:DD:EE:88 ERR CONNECT_TIMEOUT AA:BB:CC:DD:EE:88");
            long errWait = System.currentTimeMillis() + 1000L;
            while (diagnostics.isEmpty() && System.currentTimeMillis() < errWait) {
                Thread.sleep(10L);
            }
            eq("v0.7.8 ERR CONNECT_TIMEOUT emits diagnostic log entry",
                    Boolean.TRUE,
                    Boolean.valueOf(!diagnostics.isEmpty() && diagnostics.get(0).contains("CONNECT_TIMEOUT")));

            // Start manual BLE scan: direct BLE connect attempts MUST be suppressed during scan!
            lines.clear();
            client.startBleScan();
            eq("v0.7.8 startBleScan sends SCAN_START",
                    Boolean.TRUE, Boolean.valueOf(waitForBleLine(lines, "SCAN_START", 1000L)));
            eq("v0.7.8 canAttemptDirectBleNowForTest is FALSE while BLE scan is active",
                    Boolean.FALSE, Boolean.valueOf(client.canAttemptDirectBleNowForTest()));

            // Feed 2 nearby BLE devices (neither is paired VEEPEAK), then stop scan:
            sendBleServerEvent(w, "ADV CC:11:22:33:44:01 1 -58 020106");
            sendBleServerEvent(w, "ADV CC:11:22:33:44:02 1 -63 020106");
            long advWait = System.currentTimeMillis() + 1000L;
            while (client.getDiscoveredDevices().size() < 2 && System.currentTimeMillis() < advWait) {
                Thread.sleep(10L);
            }
            client.stopBleScan();
            eq("v0.7.8 stopBleScan clears stuck Scanning status and reports paired adapter not advertising",
                    "BLE scan: VEEPEAK not advertising (2 other BLE seen)", client.getStatusSummary());
            boolean loggedNotAdv = false;
            synchronized (diagnostics) {
                for (int i = 0; i < diagnostics.size(); i++) {
                    if (diagnostics.get(i).contains("VEEPEAK not advertising (2 other BLE seen)")) {
                        loggedNotAdv = true;
                    }
                }
            }
            eq("v0.7.8 stopBleScan logs non-advertising scan result to DiagnosticLogger",
                    Boolean.TRUE, Boolean.valueOf(loggedNotAdv));
        } finally {
            client.close();
        }

        // 2. BrakeEvaluator v0.7.8: Auto-Regen light throttle feathering, mid-speed settled coast release,
        //    i-Pedal vs Auto mutual exclusion, and fast-Hub early foot-brake release standstill clear:
        IndicatorConfig cfg = IndicatorConfig.defaults();
        BrakeEvaluator evAuto = new BrakeEvaluator();
        long t = 120000L;
        float spd = 62.0f;
        BrakeEvaluator.Snapshot snap = null;

        // Confirmed L0 coast (~0.22 m/s^2) followed by radar L2 modulation (~1.08 m/s^2) on 20 Hz Hub:
        for (int i = 0; i <= 36; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.08f;
            snap = evAuto.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.8 baseline L0 coast learned", Integer.valueOf(0), Integer.valueOf(snap.inferredRegenLevel));

        for (int i = 1; i <= 30; i++) {
            if (i % 2 == 0) spd -= 0.42f; // ~1.17 m/s^2 total (~1.07 m/s^2 motor) radar regen above 52 km/h
            snap = evAuto.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.8 confirmed L0 -> L2 modulation on 20 Hz Hub latches autoRegenActive immediately",
                Boolean.TRUE, Boolean.valueOf(snap.autoRegenActive && snap.modelIlluminated));

        // Highway Auto-Regen light feathering (6.4% throttle at ~53 km/h while radar regen holds ~1.06 m/s^2 motor):
        for (int i = 0; i < 6; i++) {
            if (i % 2 == 1) spd -= 0.42f;
            snap = evAuto.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 6.4f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.8 Auto-Regen >=48 km/h light 6.4% throttle feathering keeps REGEN_DECEL ON",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));

        // Urban Auto-Regen light feathering (2.8% throttle at ~33 km/h while radar regen holds ~1.02 m/s^2):
        t += 1500L;
        spd = 35.0f;
        for (int i = 0; i < 16; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.38f;
            float featherAccel = (i >= 10) ? 2.8f : 0.0f;
            snap = evAuto.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, featherAccel, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.8 Auto-Regen urban 2.8% throttle feathering at ~32 km/h keeps REGEN_DECEL ON",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));

        // Settled (>2.05 s + 220 ms hold) 14.2-39.5 km/h Auto-Regen coast easing to ~0.69 m/s^2 (< 0.88 m/s^2) releases cleanly:
        for (int i = 0; i < 52; i++) {
            if (i % 2 == 1) spd -= 0.25f; // ~0.69 m/s^2 across ~32 -> 25.5 km/h over 2.6 s
            snap = evAuto.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.8 settled (>2.05 s) mid-speed Auto-Regen coast at ~0.69 m/s^2 releases REGEN_DECEL",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 3. Strict i-Pedal vs Auto-Regen mutual exclusion across brief (<1.45 s) accel -> coast transitions:
        BrakeEvaluator evIp = new BrakeEvaluator();
        long ti = 300000L;
        float ipSpd = 9.0f;
        for (int i = 0; i < 34; i++) {
            ipSpd += 0.20f; // +1.11 m/s^2 over 1.70 s (>= 1.35 s confirm)
            snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    ti, 0.0f, ipSpd, 33.5f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, ti));
            ti += 50L;
        }
        eq("v0.7.8 post-Park urban launch learns i-Pedal (4)",
                Integer.valueOf(4), Integer.valueOf(snap.inferredRegenLevel));
        ipSpd = 38.0f;
        for (int i = 0; i < 12; i++) {
            ipSpd += 0.25f;
            evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    ti, 0.0f, ipSpd, 32.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, ti));
            ti += 50L;
        }
        // Brief 1.25 s coast during VCU +drive -> regen transition (~0.33 m/s^2 transient):
        for (int i = 0; i < 25; i++) {
            if (i % 2 == 1) ipSpd -= 0.12f;
            snap = evIp.evaluate(cfg, new BrakeEvaluator.Input(
                    ti, 0.0f, ipSpd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, ti));
            ti += 50L;
        }
        eq("v0.7.8 i-Pedal brief <1.45 s transient coast preserves i-Pedal (4) and keeps autoRegenActive=false",
                Boolean.TRUE, Boolean.valueOf(snap.inferredRegenLevel == 4 && !snap.autoRegenActive));

        // 4. Fast-Hub (20 Hz) early foot-brake release (>=1.5 km/h) rolling unbraked to 0.0 km/h clears standstill latch:
        BrakeEvaluator evEarlyRel = new BrakeEvaluator();
        long te = 400000L;
        float eSpd = 8.0f;
        for (int i = 0; i < 14; i++) {
            if (i > 0 && i % 2 == 0) eSpd -= 0.80f; // brake down from 8.0 -> 2.4 km/h
            evEarlyRel.evaluate(cfg, new BrakeEvaluator.Input(
                    te, 6.0f, eSpd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, te));
            te += 50L;
        }
        // Release foot brake completely at 2.4 km/h and roll unbraked to 0.0 km/h:
        float[] rollSpds = {2.4f, 2.4f, 1.6f, 1.6f, 0.8f, 0.8f, 0.0f, 0.0f};
        for (int i = 0; i < rollSpds.length; i++) {
            float rs = rollSpds[i];
            snap = evEarlyRel.evaluate(cfg, new BrakeEvaluator.Input(
                    te, 0.0f, rs, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.valueOf(rs > 0.1f), 72.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, te));
            te += 50L;
        }
        eq("v0.7.8 early foot-brake release at 2.4 km/h rolling unbraked to 0.0 km/h does NOT false-latch STANDSTILL_HOLD",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));
    }

    private static void testV079AutoRegenTrafficStabilityThrottleOnsetAndTurnBleed() {
        IndicatorConfig cfg = IndicatorConfig.defaults();
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 500000L;
        float spd = 62.0f;
        BrakeEvaluator.Snapshot snap = null;

        // 1. Learn Auto Regen via settled L0 coast (~0.22 m/s^2) -> radar L2 coast (~1.08 m/s^2):
        for (int i = 0; i <= 36; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.08f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 54.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 30; i++) {
            if (i % 2 == 0) spd -= 0.42f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 54.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.9 baseline Auto Regen latched", Boolean.TRUE, Boolean.valueOf(snap.autoRegenActive));

        // 4 traffic radar high-decel coasts separated by >=3.5% throttle (which resets inferredRegenLevel to 0)
        // and open-road L0/L1 coasts MUST NOT accumulate consecutiveFixedHighCoasts >= 3 or drop autoRegenActive:
        boolean anyAutoDropout = false;
        for (int stop = 0; stop < 4; stop++) {
            // Accelerate back up with 14% throttle (resets inferredRegenLevel -> 0)
            spd = 58.0f;
            for (int i = 0; i < 12; i++) {
                snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                        t, 0.0f, spd, 14.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 54.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
                if (!snap.autoRegenActive) anyAutoDropout = true;
                t += 50L;
            }
            // Open-road settled L0 (even stops) or L1 (odd stops) coast while prevLevel is already 0:
            for (int i = 1; i <= 38; i++) {
                if (i % 2 == 0) spd -= (stop % 2 == 0 ? 0.08f : 0.24f); // ~0.16 m/s^2 L0 or ~0.60 m/s^2 L1 motor
                snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                        t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 54.0f,
                        BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
                if (!snap.autoRegenActive) anyAutoDropout = true;
                t += 50L;
            }
            // Re-accelerate briefly, then perform two back-to-back radar L3 traffic coasts (~1.38 m/s^2):
            for (int hc = 0; hc < 2; hc++) {
                spd = 56.0f;
                for (int i = 0; i < 10; i++) {
                    snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                            t, 0.0f, spd, 14.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 54.0f,
                            BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
                    if (!snap.autoRegenActive) anyAutoDropout = true;
                    t += 50L;
                }
                for (int i = 1; i <= 40; i++) {
                    if (i % 3 == 0) spd -= 0.75f; // ~1.39 m/s^2 high-decel radar stop
                    snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                            t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 54.0f,
                            BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
                    if (!snap.autoRegenActive) anyAutoDropout = true;
                    t += 50L;
                }
            }
        }
        eq("v0.7.9 AutoRegenEst stays ACTIVE across 8 radar traffic coasts interleaved with settled L0/L1 coasts",
                Boolean.FALSE, Boolean.valueOf(anyAutoDropout));

        // 2. Light-throttle cold-onset guard (!decelLatched && accVal >= 1.8%) & sustained >=350ms light-throttle release:
        t += 2000L;
        spd = 54.0f;
        for (int i = 0; i < 12; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 12.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 52.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        // Cold uphill coast with 3.6% throttle (~1.05 m/s^2 motor decel < 1.25 m/s^2 cold-onset guard) stays OFF:
        boolean coldThrottleFp = false;
        for (int i = 1; i <= 18; i++) {
            if (i % 2 == 0) spd -= 0.41f; // ~1.14 m/s^2 total (~1.06 m/s^2 motor)
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 3.6f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 52.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) coldThrottleFp = true;
            t += 50L;
        }
        eq("v0.7.9 cold 3.6% uphill throttle at ~1.06 m/s^2 does NOT false-fire REGEN_DECEL",
                Boolean.FALSE, Boolean.valueOf(coldThrottleFp));

        // Latch off-pedal radar regen (~1.12 m/s^2 motor), feather 2.8% briefly (<350 ms) at ~0.86 m/s^2 -> stays ON (>=0.78),
        // then hold 2.8% throttle past >=350 ms + 220 ms hold at ~0.86 m/s^2 (< 1.02 m/s^2 sustained release) -> releases cleanly:
        spd = 52.0f;
        for (int i = 1; i <= 16; i++) {
            if (i % 2 == 0) spd -= 0.43f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 52.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.9 off-pedal radar regen latches REGEN_DECEL", Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));
        for (int i = 1; i <= 4; i++) { // 200 ms (< 350 ms) at 2.8% pedal, ~0.86 m/s^2 motor (>= 0.78 m/s^2 brief hold)
            if (i % 2 == 0) spd -= 0.33f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 2.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 52.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.9 brief <350 ms 2.8% feather keeps REGEN_DECEL ON", Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));
        for (int i = 1; i <= 16; i++) { // +800 ms (1000 ms total) at ~0.86 m/s^2 motor (< 1.02 m/s^2 sustained release)
            if (i % 2 == 0) spd -= 0.33f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 2.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 52.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.9 sustained >=350 ms 2.8% throttle below 1.02 m/s^2 releases REGEN_DECEL",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 3. Highway (>=56 km/h) <1.7% pedal deadzone onset & active R13-H >=1.18 m/s^2 strong-decel hold up to 12.8% pedal:
        t += 2000L;
        spd = 68.0f;
        for (int i = 0; i < 8; i++) {
            ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 12.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 50.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 14; i++) {
            if (i % 2 == 0) spd -= 0.36f; // ~1.00 m/s^2 total (~0.84 m/s^2 motor >= 0.68 m/s^2) at 1.2% deadzone pedal
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 1.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 50.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.9 highway >=56 km/h 1.2% pedal deadzone catches ~0.84 m/s^2 radar regen onset",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));

        // Foot-brake at 44 km/h (~2.2 m/s^2), release foot brake and at 750..950 ms post-release (<1200 ms noRecentFootBrake)
        // apply 10.5% throttle while radar holds ~1.55 m/s^2 straight ahead (< 12.8% active R13-H strong-decel hold):
        t += 2000L;
        spd = 44.0f;
        for (int i = 1; i <= 12; i++) {
            if (i % 2 == 0) spd -= 0.80f; // ~2.22 m/s^2
            ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 3.8f, spd, 0.0f, 10.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 50.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 18; i++) { // 900 ms total (< 1200 ms post-foot-brake)
            if (i % 2 == 0) spd -= 0.58f; // ~1.61 m/s^2 (>= 1.18 m/s^2)
            float ped = (i >= 15) ? 10.5f : 0.0f; // 0% pedal re-latches at 700 ms, then 10.5% pedal at 750..900 ms
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, ped, 10.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 50.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.9 active >=1.18 m/s^2 R13-H strong-decel holds REGEN_DECEL at 10.5% pedal right after foot-brake lift",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));

        // 4. Intersection trail-braking turn scrub (speed >= 18 km/h, |steering| >= 80 deg) suppresses post-brake carryover:
        t += 2500L;
        spd = 31.0f;
        for (int i = 1; i <= 12; i++) {
            if (i % 2 == 0) spd -= 0.80f; // ~2.22 m/s^2 foot braking into intersection
            ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 4.2f, spd, 0.0f, 35.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 49.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        // Release foot brake while turning sharply (115 deg >= 80 deg) at ~25->21 km/h with ~2.05 m/s^2 lateral tire scrub:
        boolean turnScrubFp = false;
        for (int i = 1; i <= 16; i++) { // 800 ms (< 950 ms turn settle window)
            if (i % 2 == 0) spd -= 0.74f; // ~2.05 m/s^2 scrub drag
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 115.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 49.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) turnScrubFp = true;
            t += 50L;
        }
        eq("v0.7.9 >=80 deg intersection trail-braking turn scrub suppresses post-foot-brake REGEN_DECEL FP",
                Boolean.FALSE, Boolean.valueOf(turnScrubFp));
    }

    private static void testV0710MidDriveAutoToIpedalHighwayFeatherAndStandstill() {
        IndicatorConfig cfg = IndicatorConfig.defaults();

        // 1. Extended highway (55-135 km/h) piecewise i-Pedal onset & release curves (preserving <=55 km/h):
        eq("v0.7.10 ipedalOnsetPedalPct(55 km/h) == 16.0%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(55.0f) - 16.0f) < 0.02f));
        eq("v0.7.10 ipedalOnsetPedalPct(85 km/h) == 21.0%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(85.0f) - 21.0f) < 0.02f));
        eq("v0.7.10 ipedalOnsetPedalPct(125 km/h) == 22.6%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(125.0f) - 22.6f) < 0.02f));
        eq("v0.7.10 ipedalReleasePedalPct(55 km/h) == 18.6%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(55.0f) - 18.6f) < 0.02f));
        eq("v0.7.10 ipedalReleasePedalPct(60 km/h) == 19.0%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(60.0f) - 19.0f) < 0.02f));
        eq("v0.7.10 ipedalReleasePedalPct(92 km/h) == 24.8%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(92.0f) - 24.8f) < 0.02f));

        // 2a. Mid-drive Auto Regen -> i-Pedal promotion via >=1.65 s highway neutral cruise (Signature B2):
        BrakeEvaluator evCruise = new BrakeEvaluator();
        long t = 600000L;
        float spd = 62.0f;
        BrakeEvaluator.Snapshot snap = null;
        for (int i = 0; i <= 36; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.08f;
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 79.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 30; i++) {
            if (i % 2 == 0) spd -= 0.42f;
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 79.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.10 pre-switch Auto Regen active", Boolean.TRUE, Boolean.valueOf(snap.autoRegenActive));

        // Settle speed at 82 km/h on normal Auto-Regen throttle (15%), then verify a brief 600 ms uphill 34% press does NOT promote:
        spd = 82.0f;
        for (int i = 0; i < 14; i++) {
            float s = spd + ((i / 6) % 2 == 0 ? 0.0f : 0.08f);
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, s, 15.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 78.5f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 0; i < 12; i++) { // 600 ms (< 1650 ms)
            float s = spd + ((i / 6) % 2 == 0 ? 0.0f : 0.08f);
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, s, 34.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 78.5f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.10 brief 600 ms uphill 34% press in Auto Regen does NOT false-promote i-Pedal",
                Boolean.TRUE, Boolean.valueOf(snap.autoRegenActive && snap.inferredRegenLevel != 4));

        // Reset briefly to 15% pedal, then switch to i-Pedal and hold 82 km/h neutral cruise at 34.5% pedal for 1.9 s (>= 1.65 s):
        for (int i = 0; i < 4; i++) {
            evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, 82.0f, 15.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 78.5f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 0; i < 38; i++) {
            float s = 82.0f + ((i / 12) % 2 == 0 ? 0.0f : 0.08f);
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, s, 34.5f, 2.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 78.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.10 >=1.65 s highway neutral cruise promotes Auto Regen -> i-Pedal (4) and clears autoRegenActive",
                Boolean.TRUE, Boolean.valueOf(snap.inferredRegenLevel == 4 && !snap.autoRegenActive));

        // 2b. Mid-drive Auto Regen -> i-Pedal promotion via >=1.15 s feathered highway one-pedal braking (Signature B2c):
        BrakeEvaluator evFeatherPromote = new BrakeEvaluator();
        long tf = 700000L;
        float fSpd = 62.0f;
        for (int i = 0; i <= 36; i++) {
            if (i > 0 && i % 2 == 0) fSpd -= 0.08f;
            evFeatherPromote.evaluate(cfg, new BrakeEvaluator.Input(
                    tf, 0.0f, fSpd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 79.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tf));
            tf += 50L;
        }
        for (int i = 1; i <= 30; i++) {
            if (i % 2 == 0) fSpd -= 0.42f;
            snap = evFeatherPromote.evaluate(cfg, new BrakeEvaluator.Input(
                    tf, 0.0f, fSpd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 79.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tf));
            tf += 50L;
        }
        eq("v0.7.10 second evaluator starts in Auto Regen", Boolean.TRUE, Boolean.valueOf(snap.autoRegenActive));
        tf += 2500L;
        fSpd = 78.0f;
        for (int i = 0; i < 8; i++) {
            float s = fSpd + ((i / 4) % 2 == 0 ? 0.0f : 0.08f);
            evFeatherPromote.evaluate(cfg, new BrakeEvaluator.Input(
                    tf, 0.0f, s, 20.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 78.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tf));
            tf += 50L;
        }
        for (int i = 1; i <= 34; i++) { // 1.70 s (>= 1.15 s confirm) at 17.5% pedal with ~1.08 m/s^2 motor decel
            if (i % 2 == 0) fSpd -= 0.42f;
            snap = evFeatherPromote.evaluate(cfg, new BrakeEvaluator.Input(
                    tf, 0.0f, fSpd, 17.5f, 3.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 78.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tf));
            tf += 50L;
        }
        eq("v0.7.10 >=1.15 s feathered highway braking promotes Auto Regen -> i-Pedal (4) and lights REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.inferredRegenLevel == 4 && !snap.autoRegenActive && snap.modelIlluminated));

        // 3. Highway i-Pedal cold above-onset spike guard (58.7 km/h @ 18.4% pedal) vs true highway feathered braking (86 km/h):
        t += 1500L;
        spd = 60.0f;
        for (int i = 0; i < 10; i++) {
            evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 28.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 77.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        boolean coldHwyAboveOnsetFp = false;
        for (int i = 1; i <= 10; i++) {
            if (i % 2 == 0) spd -= 0.36f; // ~1.00 m/s^2 total (~0.96 m/s^2 motor < 1.15 m/s^2 standard onset guard)
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 18.4f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 77.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            if (snap.modelIlluminated) coldHwyAboveOnsetFp = true;
            t += 50L;
        }
        eq("v0.7.10 cold >=50 km/h above-onset 18.4% pedal transient (~0.96 m/s^2) stays OFF",
                Boolean.FALSE, Boolean.valueOf(coldHwyAboveOnsetFp));

        // True highway feathered i-Pedal braking at ~86 km/h (20.2% pedal <= 21.04% onset, ~0.99 m/s^2 motor decel):
        t += 1500L;
        spd = 88.0f;
        for (int i = 0; i < 10; i++) {
            evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 35.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 76.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 12; i++) {
            if (i % 2 == 0) spd -= 0.40f; // ~1.11 m/s^2 total (~0.99 m/s^2 motor)
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 20.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 76.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.10 86 km/h feathered i-Pedal braking at 20.2% pedal latches REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));

        // Hold at 22.2% pedal (< ~23.5% highway release) keeps REGEN_DECEL ON, then 26.8% pedal cancels cleanly:
        for (int i = 1; i <= 6; i++) {
            if (i % 2 == 0) spd -= 0.36f;
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 22.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 76.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.10 84 km/h feathered i-Pedal holds REGEN_DECEL at 22.2% pedal",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));
        for (int i = 0; i < 4; i++) {
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 26.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 76.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.10 84 km/h feathered i-Pedal releases REGEN_DECEL at 26.8% pedal",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 4. i-Pedal 0 km/h standstill hold pedal hysteresis (<0.6% latch, <1.0% hold, >=1.0% release):
        t += 4000L; // expire any prior lastBrakingNearStopMs arm window
        for (int i = 0; i < 4; i++) {
            snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, 0.0f, 1.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.10 0 km/h i-Pedal with 1.2% pedal stays OFF",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));
        snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        eq("v0.7.10 unlatched 0 km/h i-Pedal at 0.8% (>=0.6%) does NOT cold-latch STANDSTILL_HOLD",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));
        snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.4f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        eq("v0.7.10 0 km/h i-Pedal easing to 0.4% (<0.6%) latches STANDSTILL_HOLD",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "STANDSTILL_HOLD".equals(snap.modelSource)));
        snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 0.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        eq("v0.7.10 latched 0 km/h i-Pedal holds STANDSTILL_HOLD at 0.8% (<1.0% hysteresis)",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "STANDSTILL_HOLD".equals(snap.modelSource)));
        snap = evCruise.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, 0.0f, 1.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.FALSE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        eq("v0.7.10 latched 0 km/h i-Pedal releases STANDSTILL_HOLD at 1.2% (>=1.0%)",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));
    }

    private static void testV0712IpedalCrawlCurveHighwayFeatherAndSharpTurn() {
        IndicatorConfig cfg = IndicatorConfig.defaults();

        // 1. Verify v0.7.12 crawl-to-urban speed-dependent i-Pedal onset & release curves:
        eq("v0.7.12 ipedalOnsetPedalPct at 1.5 km/h is 1.8%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(1.5f) - 1.8f) < 0.01f));
        eq("v0.7.12 ipedalOnsetPedalPct at 5.0 km/h is 5.5%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(5.0f) - 5.5f) < 0.01f));
        eq("v0.7.12 ipedalOnsetPedalPct at 8.0 km/h is 7.5%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalOnsetPedalPct(8.0f) - 7.5f) < 0.01f));
        eq("v0.7.12 ipedalReleasePedalPct at 1.5 km/h is 2.1%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(1.5f) - 2.1f) < 0.01f));
        eq("v0.7.12 ipedalReleasePedalPct at 4.0 km/h is 6.1%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(4.0f) - 6.1f) < 0.01f));
        eq("v0.7.12 ipedalReleasePedalPct at 8.0 km/h is 9.6%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(8.0f) - 9.6f) < 0.01f));
        eq("v0.7.12 ipedalReleasePedalPct at 15.0 km/h is 12.8%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(15.0f) - 12.8f) < 0.01f));
        eq("v0.7.12 ipedalReleasePedalPct at 21.0 km/h is 16.8%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(21.0f) - 16.8f) < 0.01f));

        // 2. Fast-poll Highway Signature B promotion during gentle neutral-pedal deceleration
        //    (~64.5 km/h at 25.0% pedal slowing at ~0.24 m/s^2 for >= 1.65 s, matching 111713.csv):
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 10000L;
        float spd = 65.4f;
        BrakeEvaluator.Snapshot snap = null;
        for (int i = 0; i < 42; i++) {
            if (i > 0 && i % 4 == 0) spd -= 0.17f; // ~0.24 m/s^2 gentle neutral deceleration
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 25.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.12 gentle-decel highway neutral cruise (64.5 km/h, 25% pedal, ~0.24 m/s^2) promotes i-Pedal",
                Boolean.TRUE, Boolean.valueOf(snap.inferredRegenLevel == BrakeEvaluator.REGEN_LEVEL_IPEDAL
                        && !snap.autoRegenActive));

        // 3. Highway feathered i-Pedal at 78 km/h (18.2% pedal <= 19.8% onset, ~0.46 m/s^2 motor decel)
        //    latches REGEN_DECEL, and throttle re-application to 21.5% (> onset+1.2%, < 22.2% release)
        //    with mild <0.46 m/s^2 trailing motor decel releases cleanly via aboveOnsetReapply:
        t += 2200L;
        spd = 79.2f;
        for (int i = 0; i < 12; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 28.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 12; i++) {
            if (i % 2 == 0) spd -= 0.20f; // ~0.56 m/s^2 total -> ~0.47 m/s^2 motor decel
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 18.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.12 highway feathered i-Pedal (78 km/h, 18.2% pedal, ~0.47 m/s^2 motor decel) lights REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));
        for (int i = 1; i <= 8; i++) {
            if (i % 2 == 0) spd -= 0.13f; // ~0.36 m/s^2 total -> ~0.27 m/s^2 motor decel (>0.20 old hold, <0.46 new)
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 21.5f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.12 highway throttle reapply above onset+1.2% (21.5% pedal) sheds trailing -dv/dt tail",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        // 4. Low-speed crawl release (< 4 km/h at 6.8% pedal cancels immediately) and sharp-steering
        //    intersection turn boost (13.5 km/h, |steer|=300 deg, 12.6% pedal, ~0.80 m/s^2 lights ON):
        t += 2200L;
        spd = 7.2f;
        for (int i = 0; i < 10; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.65f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.12 0%-pedal crawl stop at ~4.6 km/h lights REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));
        spd = 3.2f;
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd, 6.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        t += 50L;
        eq("v0.7.12 pressing 6.8% crawl throttle at 3.2 km/h (>4.8% crawl release) cancels REGEN_DECEL",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));

        t += 2200L;
        spd = 15.2f;
        for (int i = 0; i < 12; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 24.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        for (int i = 1; i <= 12; i++) {
            if (i % 2 == 0) spd -= 0.29f; // ~0.81 m/s^2 (>= 0.55 m/s^2 sharp-turn motor decel)
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 12.6f, 300.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.12 sharp 300 deg i-Pedal intersection turn at 13.5 km/h & 12.6% pedal lights REGEN_DECEL",
                Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated && "REGEN_DECEL".equals(snap.modelSource)));
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd, 12.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
        eq("v0.7.12 unwinding steering to 0 deg at 13.5 km/h & 12.8% pedal releases REGEN_DECEL",
                Boolean.FALSE, Boolean.valueOf(snap.modelIlluminated));
    }

    private static void testV0713ColdStartIpedalForwardCrawlAndUrbanGrade() {
        IndicatorConfig cfg = IndicatorConfig.defaults();

        // 1. Sub-4 km/h crawl release curve anchor at 2.4 km/h (3.9%):
        eq("v0.7.13 ipedalReleasePedalPct at 2.4 km/h is 3.9%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(2.4f) - 3.9f) < 0.01f));
        eq("v0.7.13 ipedalReleasePedalPct at 4.0 km/h remains 6.1%",
                Boolean.TRUE, Boolean.valueOf(Math.abs(BrakeEvaluator.ipedalReleasePedalPct(4.0f) - 6.1f) < 0.01f));

        // 2. Cold-start deep feathered braking promotion (6.8..13.5% pedal at >= 18 km/h
        //    sustaining >= 1.05 m/s^2 motor decel for >= 1.15 s, matching 170504/171339 cold start):
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 10000L;
        float spd = 34.0f;
        BrakeEvaluator.Snapshot snap = null;
        for (int i = 0; i < 30; i++) {
            if (i > 0 && i % 2 == 0) spd -= 0.44f; // ~1.22 m/s^2 decel at 9.8% pedal
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 9.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, t));
            t += 50L;
        }
        eq("v0.7.13 cold-start deep feathered regen (9.8% pedal, ~1.2 m/s^2) promotes i-Pedal in <1.5 s",
                Boolean.valueOf(true),
                Boolean.valueOf(snap.inferredRegenLevel == BrakeEvaluator.REGEN_LEVEL_IPEDAL
                        && snap.modelIlluminated));

        // Negative guard: brief 600 ms transient (< 1150 ms confirm) or recent foot-brake bleed does NOT promote i-Pedal
        BrakeEvaluator evGuard = new BrakeEvaluator();
        long tg = 10000L;
        float spdg = 34.0f;
        BrakeEvaluator.Snapshot snapG = null;
        snapG = evGuard.evaluate(cfg, new BrakeEvaluator.Input(
                tg, 2.5f, spdg, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tg));
        tg += 100L;
        for (int i = 0; i < 26; i++) {
            if (i > 0 && i % 2 == 0) spdg -= 0.45f;
            snapG = evGuard.evaluate(cfg, new BrakeEvaluator.Input(
                    tg, 0.0f, spdg, 9.8f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tg));
            tg += 50L;
        }
        eq("v0.7.13 post-foot-brake hydraulic settle blocks false deep-feather i-Pedal promotion",
                Boolean.FALSE, Boolean.valueOf(snapG.inferredRegenLevel == BrakeEvaluator.REGEN_LEVEL_IPEDAL));

        // 3. Urban 21.2% neutral cruise (22 km/h) & high-speed 108 km/h highway cruise (42.0% pedal) promote i-Pedal:
        BrakeEvaluator evUrb = new BrakeEvaluator();
        long tu = 10000L;
        BrakeEvaluator.Snapshot snapU = null;
        for (int i = 0; i < 40; i++) {
            snapU = evUrb.evaluate(cfg, new BrakeEvaluator.Input(
                    tu, 0.0f, 22.0f, 21.2f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tu));
            tu += 50L;
        }
        eq("v0.7.13 steady 22 km/h urban cruise at 21.2% pedal promotes i-Pedal",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_IPEDAL), Integer.valueOf(snapU.inferredRegenLevel));

        BrakeEvaluator evHwy = new BrakeEvaluator();
        long th = 10000L;
        BrakeEvaluator.Snapshot snapH = null;
        for (int i = 0; i < 52; i++) {
            snapH = evHwy.evaluate(cfg, new BrakeEvaluator.Input(
                    th, 0.0f, 108.0f, 42.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, th));
            th += 50L;
        }
        eq("v0.7.13 108 km/h highway neutral cruise at 42.0% pedal promotes i-Pedal",
                Integer.valueOf(BrakeEvaluator.REGEN_LEVEL_IPEDAL), Integer.valueOf(snapH.inferredRegenLevel));

        // 4. Shift-interlock confirmedForwardDrive: after >=18 km/h forward travel (evUrb above),
        //    <7 km/h 0%-pedal liftoff lights REGEN_DECEL right after 150 ms ramp even across a flat
        //    0.8 km/h quantized step, whereas foot-braking below 4.5 km/h clears confirmedForwardDrive
        //    so Reverse 0%-pedal creep stays OFF:
        tu += 500L;
        for (int i = 0; i < 5; i++) {
            snapU = evUrb.evaluate(cfg, new BrakeEvaluator.Input(
                    tu, 0.0f, 5.6f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tu));
            tu += 50L;
        }
        eq("v0.7.13 confirmed-forward <7 km/h 0%-pedal liftoff lights REGEN_DECEL after 150 ms ramp",
                Boolean.TRUE, Boolean.valueOf(snapU.modelIlluminated && "REGEN_DECEL".equals(snapU.modelSource)));

        // Press foot brake at 1.6 km/h (enables shifting D -> R), release, then creep at 4.8 km/h & 0% pedal
        tu += 500L;
        evUrb.evaluate(cfg, new BrakeEvaluator.Input(
                tu, 3.0f, 1.6f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tu));
        tu += 1500L;
        evUrb.evaluate(cfg, new BrakeEvaluator.Input(
                tu, 0.0f, 4.8f, 8.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tu));
        tu += 200L;
        for (int i = 0; i < 5; i++) {
            snapU = evUrb.evaluate(cfg, new BrakeEvaluator.Input(
                    tu, 0.0f, 4.8f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 75.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, -1, tu));
            tu += 50L;
        }
        eq("v0.7.13 foot-brake <4.5 km/h resets confirmedForwardDrive so Reverse 0%-pedal flat creep stays OFF",
                Boolean.FALSE, Boolean.valueOf(snapU.modelIlluminated));
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
