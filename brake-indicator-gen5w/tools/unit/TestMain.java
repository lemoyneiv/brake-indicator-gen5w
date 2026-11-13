package dev.lemoyne.brakeindicator;

import java.util.HashMap;
import java.util.Map;

/**
 * Host unit tests for CandidateFilter, IndicatorConfig, BrakeEvaluator, and IpcProtocol.
 */
public final class TestMain {

    private static int passed;
    private static int failed;

    public static void main(String[] args) {
        testCandidateFilterSafetyAndRoles();
        testInterpretBinaryLampValue();
        testIndicatorConfigParsingAndClamping();
        testBrakeEvaluatorNativeAndPressure();
        testBrakeEvaluatorRegenFallbackAndParkGate();
        testIpcProtocolRoundTrip();

        System.out.println(passed + " ok, " + failed + " failed");
        if (failed > 0) {
            System.exit(1);
        }
    }

    private static void testCandidateFilterSafetyAndRoles() {
        // Direct stop-lamp / brake-switch getters (including underscore & navi_get styles)
        eq("getStopLampSw", CandidateFilter.Role.DIRECT_STOP_LAMP,
                CandidateFilter.classify("getStopLampSw", 0, Integer.TYPE));
        eq("isBrakeLampOn", CandidateFilter.Role.DIRECT_STOP_LAMP,
                CandidateFilter.classify("isBrakeLampOn", 0, Boolean.TYPE));
        eq("get_IEB_BrkLampSta", CandidateFilter.Role.DIRECT_STOP_LAMP,
                CandidateFilter.classify("get_IEB_BrkLampSta", 0, Byte.TYPE));
        eq("navi_getBrakeSw", CandidateFilter.Role.DIRECT_STOP_LAMP,
                CandidateFilter.classify("navi_getBrakeSw", 0, Integer.TYPE));
        eq("getActBrkLmpSta", CandidateFilter.Role.DIRECT_STOP_LAMP,
                CandidateFilter.classify("getActBrkLmpSta", 0, Integer.TYPE));

        // Cylinder pressure getter
        eq("getCylPrsrVal", CandidateFilter.Role.BRAKE_PRESSURE,
                CandidateFilter.classify("getCylPrsrVal", 0, Float.TYPE));

        // Parking brake, fluid switch, notch, lamp fault must NOT be direct stop lamps
        eq("getParkBrakeStatus -> diag", CandidateFilter.Role.DIAGNOSTIC_CANDIDATE,
                CandidateFilter.classify("getParkBrakeStatus", 0, Integer.TYPE));
        eq("getBrakeFluidSw -> diag", CandidateFilter.Role.DIAGNOSTIC_CANDIDATE,
                CandidateFilter.classify("getBrakeFluidSw", 0, Integer.TYPE));
        eq("getBrakeNotchStatus -> diag", CandidateFilter.Role.DIAGNOSTIC_CANDIDATE,
                CandidateFilter.classify("getBrakeNotchStatus", 0, Integer.TYPE));
        eq("getStopLampFail -> diag", CandidateFilter.Role.DIAGNOSTIC_CANDIDATE,
                CandidateFilter.classify("getStopLampFail", 0, Integer.TYPE));
        eq("getAccelPdlVal -> diag", CandidateFilter.Role.DIAGNOSTIC_CANDIDATE,
                CandidateFilter.classify("getAccelPdlVal", 0, Float.TYPE));

        // Hard safety blocks: VIN, GPS, IMEI, MSISDN, USIM, getDriveMode, setters, String returns, args > 0
        eq("getVIN blocked", CandidateFilter.Role.NONE,
                CandidateFilter.classify("getVIN", 0, String.class));
        eq("getVinStatus blocked", CandidateFilter.Role.NONE,
                CandidateFilter.classify("getVinStatus", 0, Integer.TYPE));
        eq("getGpsBrake blocked", CandidateFilter.Role.NONE,
                CandidateFilter.classify("getGpsBrake", 0, Integer.TYPE));
        eq("getImei blocked", CandidateFilter.Role.NONE,
                CandidateFilter.classify("getImei", 0, Integer.TYPE));
        eq("getMsisdn blocked", CandidateFilter.Role.NONE,
                CandidateFilter.classify("getMsisdn", 0, Integer.TYPE));
        eq("getDriveMode blocked", CandidateFilter.Role.NONE,
                CandidateFilter.classify("getDriveMode", 0, Integer.TYPE));
        eq("setBrakeLamp blocked", CandidateFilter.Role.NONE,
                CandidateFilter.classify("setBrakeLamp", 0, Integer.TYPE));
        eq("getStopLampReq blocked", CandidateFilter.Role.NONE,
                CandidateFilter.classify("getStopLampReq", 0, Integer.TYPE));
        eq("getCarSpeed(byte) blocked due to param", CandidateFilter.Role.NONE,
                CandidateFilter.classify("getCarSpeed", 1, Float.TYPE));
    }

    private static void testInterpretBinaryLampValue() {
        eq("boolean true", Boolean.TRUE, CandidateFilter.interpretBinaryLampValue(Boolean.TRUE));
        eq("boolean false", Boolean.FALSE, CandidateFilter.interpretBinaryLampValue(Boolean.FALSE));
        eq("int 0 -> false", Boolean.FALSE, CandidateFilter.interpretBinaryLampValue(Integer.valueOf(0)));
        eq("int 1 -> true", Boolean.TRUE, CandidateFilter.interpretBinaryLampValue(Integer.valueOf(1)));
        eq("2-bit CAN error 2 -> null", null, CandidateFilter.interpretBinaryLampValue(Integer.valueOf(2)));
        eq("2-bit CAN invalid 3 -> null", null, CandidateFilter.interpretBinaryLampValue(Integer.valueOf(3)));
        eq("0xFF CAN invalid -> null", null, CandidateFilter.interpretBinaryLampValue(Integer.valueOf(0xFF)));
        eq("0xFFFF CAN invalid -> null", null, CandidateFilter.interpretBinaryLampValue(Integer.valueOf(0xFFFF)));
        eq("negative error -> null", null, CandidateFilter.interpretBinaryLampValue(Integer.valueOf(-1)));
    }

    private static void testIndicatorConfigParsingAndClamping() {
        IndicatorConfig def = IndicatorConfig.defaults();
        eq("default mode", IndicatorConfig.SignalMode.NATIVE, def.signalMode);
        eq("default offState", IndicatorConfig.OffState.HIDDEN, def.offState);
        eq("default anchor", IndicatorConfig.Anchor.TOP_RIGHT, def.anchor);
        eq("default iconWidthPx", Integer.valueOf(51), Integer.valueOf(def.iconWidthPx()));

        Map<String, String> m = new HashMap<String, String>();
        m.put("signal_mode", "native_and_decel");
        m.put("off_state", "dim");
        m.put("anchor", "top_left");
        m.put("x_offset", "120");
        m.put("y_offset", "8");
        m.put("icon_size", "10"); // below 20 -> clamped to 20
        m.put("pressure_threshold", "22"); // 2.2
        m.put("decel_threshold", "15"); // 1.5 m/s^2
        m.put("probe_logging", "false");

        IndicatorConfig parsed = IndicatorConfig.fromMap(m);
        eq("parsed mode", IndicatorConfig.SignalMode.NATIVE_AND_DECEL, parsed.signalMode);
        eq("parsed off", IndicatorConfig.OffState.DIM, parsed.offState);
        eq("parsed anchor", IndicatorConfig.Anchor.TOP_LEFT, parsed.anchor);
        eq("clamped icon_size", Integer.valueOf(20), Integer.valueOf(parsed.iconSizePx));
        eq("parsed pressure_threshold", Boolean.TRUE, Math.abs(parsed.pressureThreshold - 2.2f) < 0.001f);
        eq("parsed decel_threshold", Boolean.TRUE, Math.abs(parsed.decelThresholdMps2 - 1.5f) < 0.001f);
        eq("parsed probe_logging", Boolean.FALSE, Boolean.valueOf(parsed.probeLogging));
    }

    private static void testBrakeEvaluatorNativeAndPressure() {
        IndicatorConfig cfg = IndicatorConfig.defaults(); // NATIVE, pressureThreshold = 1.5
        BrakeEvaluator ev = new BrakeEvaluator();

        // 1. CAN invalid sentinel 409.5 must NOT illuminate brake indicator
        BrakeEvaluator.Snapshot s1 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1000L, null, "", 409.5f, 50.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("409.5 invalid pressure ignored", Boolean.FALSE, Boolean.valueOf(s1.illuminated));

        // 2. Cylinder pressure above 1.5 lights indicator and latches with hysteresis down to 1.05
        BrakeEvaluator.Snapshot s2 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1100L, null, "", 2.4f, 50.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("pressure 2.4 lights", Boolean.TRUE, Boolean.valueOf(s2.illuminated));
        eq("pressure source", "BRAKE_PRESSURE", s2.source);

        BrakeEvaluator.Snapshot s3 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1200L, null, "", 1.2f, 49.5f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("hysteresis holds at 1.2", Boolean.TRUE, Boolean.valueOf(s3.illuminated));

        BrakeEvaluator.Snapshot s4 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1300L, null, "", 0.8f, 49.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("releases below 1.05", Boolean.FALSE, Boolean.valueOf(s4.illuminated));

        // 3. Direct OEM stop-lamp getter takes precedence even at 0 hydraulic pressure (pure regen)
        BrakeEvaluator.Snapshot s5 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1400L, Boolean.TRUE, "getStopLampSw=1", 0.0f, 65.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("native stop lamp lights at 0 pressure", Boolean.TRUE, Boolean.valueOf(s5.illuminated));
        eq("native stop lamp source", "OEM_LAMP:getStopLampSw=1", s5.source);
    }

    private static void testBrakeEvaluatorRegenFallbackAndParkGate() {
        Map<String, String> m = new HashMap<String, String>();
        m.put("signal_mode", "native_and_decel");
        IndicatorConfig fallbackCfg = IndicatorConfig.fromMap(m);
        IndicatorConfig nativeOnlyCfg = IndicatorConfig.defaults();

        BrakeEvaluator evNative = new BrakeEvaluator();
        BrakeEvaluator evFallback = new BrakeEvaluator();

        // Simulate strong one-pedal regen from 72 km/h (20 m/s) dropping 0.8 km/h per 100 ms (~2.22 m/s^2)
        float speed = 72.0f;
        BrakeEvaluator.Snapshot snapNative = null;
        BrakeEvaluator.Snapshot snapFallback = null;
        for (int i = 0; i < 6; i++) {
            long t = 2000L + i * 100L;
            speed -= 0.8f;
            BrakeEvaluator.Input in = new BrakeEvaluator.Input(
                    t, null, "", 0.0f, speed, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L);
            snapNative = evNative.evaluate(nativeOnlyCfg, in);
            snapFallback = evFallback.evaluate(fallbackCfg, in);
        }
        eq("NATIVE mode ignores speed decel", Boolean.FALSE, Boolean.valueOf(snapNative.illuminated));
        eq("NATIVE_AND_DECEL lights on 2.2 m/s^2 regen", Boolean.TRUE, Boolean.valueOf(snapFallback.illuminated));
        eq("regen source", "REGEN_DECEL", snapFallback.source);

        // Shifting to Park suppresses regen decel immediately
        BrakeEvaluator.Snapshot inPark = evFallback.evaluate(fallbackCfg, new BrakeEvaluator.Input(
                2700L, null, "", 0.0f, 5.0f, 0.0f, Boolean.TRUE, Boolean.TRUE, 0L));
        eq("Park clears regen decel", Boolean.FALSE, Boolean.valueOf(inPark.illuminated));

        // Manual 3s test overrides everything
        BrakeEvaluator.Snapshot manual = evFallback.evaluate(fallbackCfg, new BrakeEvaluator.Input(
                2800L, null, "", 0.0f, 0.0f, 0.0f, Boolean.TRUE, Boolean.TRUE, 5000L));
        eq("manual test lights", Boolean.TRUE, Boolean.valueOf(manual.illuminated));
        eq("manual test source", "MANUAL_TEST", manual.source);
    }

    private static void testIpcProtocolRoundTrip() {
        IndicatorConfig cfg = IndicatorConfig.defaults();
        BrakeEvaluator.Snapshot snap = new BrakeEvaluator.Snapshot(
                1000L, true, "OEM_LAMP:getStopLampSw=1", Boolean.TRUE, "getStopLampSw=1",
                0.0f, 62.5f, 0.0f, 1.85f, Boolean.FALSE, Boolean.FALSE);
        String line = IpcProtocol.encodeSnapshot(snap, cfg, true, 1, 4, "getStopLampSw=1, getCylPrsrVal=0.0");
        Map<String, String> decoded = IpcProtocol.decodeLine(line);
        eq("ipc type", "STATE", decoded.get("_type"));
        eq("ipc lit", "1", decoded.get("lit"));
        eq("ipc src", "OEM_LAMP:getStopLampSw=1", decoded.get("src"));
        eq("ipc spd", "62.5", decoded.get("spd"));
        eq("ipc hscan", "1", decoded.get("hscan"));
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
