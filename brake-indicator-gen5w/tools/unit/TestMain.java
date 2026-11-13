package dev.lemoyne.brakeindicator;

import gen5w.ble.Gen5wBleClient;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
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
 * Host JVM unit tests for v0.4.0:
 * - HmgVehiclePoller (fast <2ms LFD1 vehicle reads + resilient 90ms legacy KitKat Hub stall recovery)
 * - Elm327BrakeProtocol (IGMP 770->778 22BC06 multi-frame ISO-TP decoder, LNEX BVM02 exclusion, VIN/write safety)
 * - ObdBrakeClient (@gen5w_ble direct BLE primary session + ALREADY_CONNECTED 127.0.0.1 loopback fallback)
 * - IndicatorConfig & BrakeEvaluator (100/250 ms cached HMG-API speed handling, L0..L3 coasting regen estimator,
 *   i-Pedal/SCC regen, standstill hold, ObdMode)
 * - HevRegenProbe (strict VIN/MAC/phone/mutator safety filter, Mark Regen baseline, live paddle-click diff)
 * - TelemetryRecorder & UsbExporter (confusion matrix, regen columns, external events, 3-file atomic USB export)
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
        testBrakeEvaluatorCoastingRegenInference();
        testBrakeEvaluatorStandstillHoldAndRelease();
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

        // 3. Pure Regen / Smart Cruise Control brake lamps ON without pedal (0x24 = 36 and 0x04 = 4)
        String regen36Resp = "778100B62BC0670400003\r7782124000000000000\r";
        Elm327BrakeProtocol.IgmpBrakeReading regen36 = Elm327BrakeProtocol.decodeIgmpBrakeResponse(regen36Resp);
        eq("IGMP Regen 36 lit=true", Boolean.TRUE, Boolean.valueOf(regen36 != null && regen36.brakeLampLit));
        eq("IGMP Regen 36 rawCode=36", Integer.valueOf(36), Integer.valueOf(regen36.rawCode));
        eq("IGMP Regen 36 pedalBit=false", Boolean.FALSE, Boolean.valueOf(regen36.pedalBit));

        String regen4Resp = "778100B62BC0670400003\r7782104000000000000\r";
        Elm327BrakeProtocol.IgmpBrakeReading regen4 = Elm327BrakeProtocol.decodeIgmpBrakeResponse(regen4Resp);
        eq("IGMP Regen 4 lit=true, pedalBit=false",
                Boolean.TRUE, Boolean.valueOf(regen4 != null && regen4.brakeLampLit && !regen4.pedalBit));

        // 4. Foot pedal / brake switch active codes: 40 (0x28), 41 (0x29), 44 (0x2C), 45 (0x2D)
        int[] pedalCodes = {40, 41, 44, 45};
        for (int i = 0; i < pedalCodes.length; i++) {
            int code = pedalCodes[i];
            String hex = String.format("%02X", Integer.valueOf(code));
            String resp = "778100B62BC0670400003\r77821" + hex + "000000000000\r";
            Elm327BrakeProtocol.IgmpBrakeReading r = Elm327BrakeProtocol.decodeIgmpBrakeResponse(resp);
            eq("IGMP Pedal code " + code + " lit+pedalBit",
                    Boolean.TRUE, Boolean.valueOf(r != null && r.rawCode == code && r.brakeLampLit && r.pedalBit));
        }

        // 5. Reject asleep/placeholder IGMP frames padded with 0xAA
        String asleepAa = "778100B62BC06AAAAAA\r77821AAAAAAAAAA0000\r";
        eq("IGMP 0xAA asleep frame rejected", null, Elm327BrakeProtocol.decodeIgmpBrakeResponse(asleepAa));

        // 6. Chunked 20-byte BLE GATT notification reassembly via PromptBuffer
        Elm327BrakeProtocol.PromptBuffer pb = new Elm327BrakeProtocol.PromptBuffer();
        List<String> c1 = pb.feed("778100B62BC06704000".getBytes(Elm327BrakeProtocol.ASCII));
        eq("PromptBuffer partial chunk -> 0 prompts", Integer.valueOf(0), Integer.valueOf(c1.size()));
        List<String> c2 = pb.feed("03\r778212400000000>".getBytes(Elm327BrakeProtocol.ASCII));
        eq("PromptBuffer '>' completes 1 prompt", Integer.valueOf(1), Integer.valueOf(c2.size()));
        Elm327BrakeProtocol.IgmpBrakeReading chunked = Elm327BrakeProtocol.decodeIgmpBrakeResponse(c2.get(0));
        eq("Chunked BLE prompt decodes rawCode=36",
                Boolean.TRUE, Boolean.valueOf(chunked != null && chunked.rawCode == 36 && chunked.brakeLampLit));
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
        final ServerSocket server = new ServerSocket(0, 4, InetAddress.getByName("127.0.0.1"));
        final int port = server.getLocalPort();
        Thread mockBridgeThread = new Thread(new Runnable() {
            @Override
            public void run() {
                Socket client = null;
                try {
                    server.setSoTimeout(4000);
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
                                reply = "778100B62BC0670400003\r7782124000000000000\r>";
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
                } finally {
                    if (client != null) {
                        try { client.close(); } catch (Throwable ignored) {}
                    }
                    try { server.close(); } catch (Throwable ignored) {}
                }
            }
        }, "MockLoopbackBridge");
        mockBridgeThread.setDaemon(true);
        mockBridgeThread.start();

        InMemoryPairingStore store = new InMemoryPairingStore();
        final ObdBrakeClient[] ref = new ObdBrakeClient[1];
        Gen5wBleClient.LineChannel channel = new Gen5wBleClient.LineChannel() {
            @Override
            public void sendLine(String line) {
                sentBleLines.add(line);
                // When WRITE_CHAR_NR 22BC06 arrives over @gen5w_ble, reply with CHAR_NOTIFY 22BC06 pedal=40!
                if (line.startsWith("@AA:BB:CC:DD:EE:01 WRITE_CHAR_NR ")
                        && line.toLowerCase().endsWith("3232424330360d")) {
                    final ObdBrakeClient c = ref[0];
                    if (c != null) {
                        new Thread(new Runnable() {
                            @Override
                            public void run() {
                                String payloadHex = Elm327BrakeProtocol.bytesToHex(
                                        "778100B62BC0670400003\r778212800000000>\r"
                                                .getBytes(Elm327BrakeProtocol.ASCII));
                                c.getBleClientForTest().handleServerLine(
                                        "@AA:BB:CC:DD:EE:01 CHAR_NOTIFY 0000fff1-0000-1000-8000-00805f9b34fb "
                                                + payloadHex);
                            }
                        }).start();
                    }
                }
            }

            @Override
            public void close() {}
        };

        ObdBrakeClient client = ObdBrakeClient.createForTest(store, channel, port);
        ref[0] = client;
        try {
            // 1. Pairing refuses BVM02 battery sensor
            eq("pairAdapter refuses BVM02",
                    Boolean.FALSE, Boolean.valueOf(client.pairAdapter("11:22:33:44:55:66", "LNEX BVM02", 0)));

            // 2. Pairing accepts Veepeak BLE adapter and issues @gen5w_ble CONNECT first
            eq("pairAdapter accepts Veepeak",
                    Boolean.TRUE, Boolean.valueOf(client.pairAdapter("AA:BB:CC:DD:EE:01", "OBDCheck BLE", 0)));
            eq("sent CONNECT on @gen5w_ble",
                    Boolean.TRUE, Boolean.valueOf(sentBleLines.contains("CONNECT AA:BB:CC:DD:EE:01 0 *")));

            // 3. Simulate @gen5w_ble GATT_READY + direct BLE 22BC06 command round-trip
            Gen5wBleClient ble = client.getBleClientForTest();
            ble.handleServerLine("GATT_READY AA:BB:CC:DD:EE:01 0000fff1-0000-1000-8000-00805f9b34fb:10,0000fff2-0000-1000-8000-00805f9b34fb:0c");
            ble.handleServerLine("@AA:BB:CC:DD:EE:01 OP_OK ENABLE_NOTIFY 0000fff1-0000-1000-8000-00805f9b34fb");

            String bleRaw = client.sendBleCommandBlocking("22BC06", 1000L);
            Elm327BrakeProtocol.IgmpBrakeReading bleDecoded = Elm327BrakeProtocol.decodeIgmpBrakeResponse(bleRaw);
            eq("Direct @gen5w_ble 22BC06 round-trip decoded rawCode=40",
                    Boolean.TRUE, Boolean.valueOf(bleDecoded != null && bleDecoded.rawCode == 40 && bleDecoded.pedalBit));
            eq("Direct @gen5w_ble blocks unsafe VIN 0902", null, client.sendBleCommandBlocking("0902", 300L));

            // 4. Simulate obd-gen5w holding BLE link (status=253 reason=ALREADY_CONNECTED) -> fallback to 127.0.0.1:<port>
            ble.handleServerLine("CONN_STATE DISCONNECTED AA:BB:CC:DD:EE:01 status=253 reason=ALREADY_CONNECTED");
            eq("Status reflects fallback after ALREADY_CONNECTED",
                    Boolean.TRUE, Boolean.valueOf(client.getStatusSummary().contains("127.0.0.1:" + port)));

            boolean okBridge = client.tryRunLoopbackBridgeSession(port);
            eq("127.0.0.1 loopback fallback session succeeds", Boolean.TRUE, Boolean.valueOf(okBridge));
            BrakeEvaluator.ObdSample bridgeSample = client.currentSample();
            eq("Loopback sample transport=BRIDGE", "BRIDGE", bridgeSample.transport);
            eq("Loopback sample rawCode=36 (Regen ON)", Integer.valueOf(36), Integer.valueOf(bridgeSample.rawCode));
            eq("Loopback sample brakeLampLit=true", Boolean.TRUE, Boolean.valueOf(bridgeSample.brakeLampLit));
            eq("Loopback sample pedalBit=false", Boolean.FALSE, Boolean.valueOf(bridgeSample.pedalBit));
        } finally {
            client.close();
            try { server.close(); } catch (Throwable ignored) {}
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
        IndicatorConfig defaultCfg = IndicatorConfig.defaults(); // OBD_WHEN_CONNECTED + PRESSURE_AND_DECEL (1.2 m/s^2)
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
        // Speed updates ONLY every 5th tick (every 250 ms) by -1.30 km/h (~1.44 m/s^2) at 12% i-Pedal throttle!
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
                    t, 0.0f, cachedSpeed, 12.0f, -2.5f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 74.5f,
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
        // Suppose speed is steady 70 km/h (Model = OFF), while OBD 22BC06 reports Regen ON (rawCode=36)
        BrakeEvaluator evObd = new BrakeEvaluator();
        BrakeEvaluator.ObdSample obdOn = new BrakeEvaluator.ObdSample(true, "BLE", 5000L, 36, true, false);
        BrakeEvaluator.Snapshot sObdIcon = evObd.evaluate(defaultCfg, new BrakeEvaluator.Input(
                5040L, 0.0f, 70.0f, 5.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f, obdOn, 0L));
        eq("OBD_WHEN_CONNECTED lights icon from OBD", Boolean.TRUE, Boolean.valueOf(sObdIcon.illuminated));
        eq("OBD_WHEN_CONNECTED source OBD_REGEN[36]", "OBD_REGEN[36]", sObdIcon.source);
        eq("Model still independently evaluated as OFF for scoring",
                Boolean.FALSE, Boolean.valueOf(sObdIcon.modelIlluminated));

        BrakeEvaluator.Snapshot sModelIcon = evObd.evaluate(modelIconCfg, new BrakeEvaluator.Input(
                5090L, 0.0f, 70.0f, 5.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f, obdOn, 0L));
        eq("MODEL_ONLY_ICON keeps icon driven by HMG-API model (OFF)",
                Boolean.FALSE, Boolean.valueOf(sModelIcon.illuminated));
        eq("MODEL_ONLY_ICON still keeps obdFresh=true for background scoring",
                Boolean.TRUE, Boolean.valueOf(sModelIcon.obdFresh));
    }

    private static void testBrakeEvaluatorCoastingRegenInference() {
        IndicatorConfig cfg = IndicatorConfig.defaults();
        eq("classifyCoastingRegenLevel L0", Integer.valueOf(0), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(0.10f, -1)));
        eq("classifyCoastingRegenLevel L1", Integer.valueOf(1), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(0.60f, -1)));
        eq("classifyCoastingRegenLevel L2", Integer.valueOf(2), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(1.15f, -1)));
        eq("classifyCoastingRegenLevel L3", Integer.valueOf(3), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(1.85f, -1)));
        eq("classifyCoastingRegenLevel hysteresis holds L2 at 0.80",
                Integer.valueOf(2), Integer.valueOf(BrakeEvaluator.classifyCoastingRegenLevel(0.80f, 2)));

        // Simulate 100 ms updated HMG-API speed steps during off-throttle coasting in L1, then L2, then L3
        BrakeEvaluator ev = new BrakeEvaluator();
        long t = 20000L;
        float spd = 72.0f;
        BrakeEvaluator.Snapshot snap = null;

        // 1. Coast in L1 (~0.58 m/s^2 -> -0.21 km/h every 100 ms) for 900 ms
        for (int tick = 0; tick <= 18; tick++) {
            if (tick > 0 && tick % 2 == 0) spd -= 0.21f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, 1));
            t += 50L;
        }
        eq("Coasting decel ~0.58 m/s^2 infers Regen L1", Integer.valueOf(1), Integer.valueOf(snap.inferredRegenLevel));
        eq("hevRegenRaw passed through to Snapshot", Integer.valueOf(1), Integer.valueOf(snap.hevRegenRaw));

        // 2. Driver clicks paddle to L2 (~1.16 m/s^2 -> -0.42 km/h every 100 ms)
        for (int tick = 1; tick <= 16; tick++) {
            if (tick % 2 == 0) spd -= 0.42f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, 2));
            t += 50L;
        }
        eq("Coasting decel ~1.16 m/s^2 infers Regen L2", Integer.valueOf(2), Integer.valueOf(snap.inferredRegenLevel));

        // 3. Driver presses throttle (18%): inferredRegenLevel stays latched at L2 until next coast or Park
        snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t, 0.0f, spd + 0.5f, 18.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                BrakeEvaluator.ObdSample.disconnected(), 0L, 2));
        t += 100L;
        eq("Throttle press preserves learned L2", Integer.valueOf(2), Integer.valueOf(snap.inferredRegenLevel));

        // 4. Coast in L3 (~1.80 m/s^2 -> -0.65 km/h every 100 ms)
        for (int tick = 1; tick <= 18; tick++) {
            if (tick % 2 == 0) spd -= 0.65f;
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, 0.0f, spd, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, Boolean.TRUE, 70.0f,
                    BrakeEvaluator.ObdSample.disconnected(), 0L, 3));
            t += 50L;
        }
        eq("Coasting decel ~1.80 m/s^2 infers Regen L3", Integer.valueOf(3), Integer.valueOf(snap.inferredRegenLevel));
        eq("L3 coast also illuminates HMG-API model", Boolean.TRUE, Boolean.valueOf(snap.modelIlluminated));

        // 5. Shifting into Park resets inferredRegenLevel to -1
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

        // 2. Baseline marking & live paddle-click diffing
        final Map<String, String> liveSignals = new LinkedHashMap<String, String>();
        liveSignals.put("HEVFlag.EV_READY(1)", "1");
        liveSignals.put("HEVFlag.EV_RECUP_LEVEL(12)", "1");
        liveSignals.put("USM.getSmartRecuperation", "0");
        liveSignals.put("HEVFlag.ODOMETER(4)", "123450"); // noisy counter -> must not trigger false candidate alerts
        liveSignals.put("BCM.getVinCode", "SECRET");      // unsafe name -> must be dropped even if returned

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
        eq("Initial direct regen candidate = 1", Integer.valueOf(1), Integer.valueOf(probe.getLatestDirectRegenRaw()));
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

        // Tick 0 (t=1000): Both OFF -> TN (inferred regen L1, hev raw 1)
        rec.record(makeScoredSnapshot(1000L, false, "OFF", 0, false, false, 1, 1), wall);
        // External baseline marker event
        rec.recordExternalEvent(1025L, "MARK #1 (42 HEV/USM/CLU signals)");
        // Tick 1 (t=1050): OBD turns ON via Regen (36), Model still OFF -> FN (inferred regen transitions L1 -> L2)
        rec.record(makeScoredSnapshot(1050L, false, "OFF", 36, true, false, 2, 2), wall + 50L);
        // Tick 2 (t=1150): Model turns ON via REGEN_DECEL -> TP, onset lag = 150 - 50 = +100 ms!
        rec.record(makeScoredSnapshot(1150L, true, "REGEN_DECEL", 36, true, false, 2, 2), wall + 150L);
        // Tick 3 (t=1200): Both ON via Regen -> TP
        rec.record(makeScoredSnapshot(1200L, true, "REGEN_DECEL", 36, true, false, 2, 2), wall + 200L);
        // Tick 4 (t=1250): OBD turns OFF (0), Model still holding -> FP (pending OBD fall at rel=250ms)
        rec.record(makeScoredSnapshot(1250L, true, "REGEN_DECEL", 0, false, false, 2, 2), wall + 250L);
        // Tick 5 (t=1330): Model turns OFF -> TN, release lag = 330 - 250 = +80 ms!
        rec.record(makeScoredSnapshot(1330L, false, "OFF", 0, false, false, 2, 2), wall + 330L);

        TelemetryRecorder.ScoreSnapshot sc = rec.scoreSnapshot();
        eq("6 buffered rows", Integer.valueOf(6), Integer.valueOf(sc.bufferedRows));
        eq("6 scored ticks", Long.valueOf(6L), Long.valueOf(sc.scoredSamples));
        eq("TP=2", Long.valueOf(2L), Long.valueOf(sc.tp));
        eq("FP=1", Long.valueOf(1L), Long.valueOf(sc.fp));
        eq("TN=2", Long.valueOf(2L), Long.valueOf(sc.tn));
        eq("FN=1", Long.valueOf(1L), Long.valueOf(sc.fn));
        eq("Regen recall 2/3 ticks", Long.valueOf(2L), Long.valueOf(sc.regenModelMatchedSamples));
        eq("Onset lag +100 ms", Long.valueOf(100L), Long.valueOf(sc.meanOnsetLagMs));
        eq("Release lag +80 ms", Long.valueOf(80L), Long.valueOf(sc.meanReleaseLagMs));

        String csv = rec.toCsvString();
        String[] lines = csv.trim().split("\n");
        eq("CSV has header + 6 data rows", Integer.valueOf(7), Integer.valueOf(lines.length));
        eq("CSV header includes inferred_regen_level,hev_regen_raw",
                Boolean.TRUE, Boolean.valueOf(lines[0].endsWith("inferred_regen_level,hev_regen_raw")));
        eq("CSV row 3 records REGEN_DECEL, raw 36, L2, hev=2",
                Boolean.TRUE, Boolean.valueOf(lines[3].contains(",1,REGEN_DECEL,1,1,BLE,") && lines[3].endsWith(",36,1,0,2,2")));

        String summary = rec.buildSummaryReport(
                IndicatorConfig.defaults(), "BLE @gen5w_ble (Veepeak)", "OK (20Hz)", "EV_RECUP_LEVEL=2 | diff=1");
        eq("Summary includes confusion matrix",
                Boolean.TRUE, Boolean.valueOf(summary.contains("TP=2  FP=1  TN=2  FN=1")));
        eq("Summary includes external MARK #1 and CoastRegenEst transitions",
                Boolean.TRUE, Boolean.valueOf(summary.contains("MARK #1") && summary.contains("CoastRegenEst: L1 -> L2")));
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
