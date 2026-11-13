package dev.lemoyne.brakeindicator;

import java.io.File;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Host unit tests for CandidateFilter, IndicatorConfig, BrakeEvaluator, IpcProtocol, and UsbExporter.
 */
public final class TestMain {

    private static int passed;
    private static int failed;

    public static void main(String[] args) throws Exception {
        testCandidateFilterSafetyAndRoles();
        testSafeReadOnlyGetterInventory();
        testInterpretBinaryLampValue();
        testIndicatorConfigParsingAndClamping();
        testBrakeEvaluatorNativeAndPressure();
        testBrakeEvaluatorSlidingWindowAndModulatedIpedal();
        testBrakeEvaluatorStandstillHoldAndCruiseStop();
        testUsbExporterMountsAndWrite();
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

        // Parking brake, fluid switch, notch, lamp fault, SCC, XEV must be diagnostic candidates
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
        eq("getSCC_MainInfo -> diag", CandidateFilter.Role.DIAGNOSTIC_CANDIDATE,
                CandidateFilter.classify("getSCC_MainInfo", 0, Integer.TYPE));
        eq("getXEV_HcuRegenLv -> diag", CandidateFilter.Role.DIAGNOSTIC_CANDIDATE,
                CandidateFilter.classify("getXEV_HcuRegenLv", 0, Integer.TYPE));

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

    private static void testSafeReadOnlyGetterInventory() {
        eq("obscure safe int getter allowed", Boolean.TRUE,
                CandidateFilter.isSafeReadOnlyGetter("getCF_Clu_GearSelDisp", 0, Integer.TYPE));
        eq("IsCarMoving allowed", Boolean.TRUE,
                CandidateFilter.isSafeReadOnlyGetter("IsCarMoving", 0, Boolean.TYPE));
        eq("getVIN rejected in safe inventory", Boolean.FALSE,
                CandidateFilter.isSafeReadOnlyGetter("getVIN", 0, Integer.TYPE));
        eq("getGpsLat rejected in safe inventory", Boolean.FALSE,
                CandidateFilter.isSafeReadOnlyGetter("getGpsLat", 0, Double.TYPE));
        eq("getDriveMode rejected in safe inventory", Boolean.FALSE,
                CandidateFilter.isSafeReadOnlyGetter("getDriveMode", 0, Integer.TYPE));
        eq("request method rejected in safe inventory", Boolean.FALSE,
                CandidateFilter.isSafeReadOnlyGetter("getBatteryReq", 0, Integer.TYPE));
        eq("String return rejected in safe inventory", Boolean.FALSE,
                CandidateFilter.isSafeReadOnlyGetter("getGearText", 0, String.class));
        eq("getAccelPdlVal marked noisy continuous", Boolean.TRUE,
                CandidateFilter.isNoisyContinuousGetter("getAccelPdlVal"));
        eq("getSCC_TakeOverReq NOT marked noisy", Boolean.FALSE,
                CandidateFilter.isNoisyContinuousGetter("getSCC_TakeOverReq"));
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
        eq("default mode is NATIVE_AND_DECEL", IndicatorConfig.SignalMode.NATIVE_AND_DECEL, def.signalMode);
        eq("default offState", IndicatorConfig.OffState.HIDDEN, def.offState);
        eq("default anchor", IndicatorConfig.Anchor.TOP_RIGHT, def.anchor);
        eq("default pressureThreshold 0.4", Boolean.TRUE, Math.abs(def.pressureThreshold - 0.4f) < 0.001f);
        eq("default decelThreshold 1.2", Boolean.TRUE, Math.abs(def.decelThresholdMps2 - 1.2f) < 0.001f);
        eq("default standstillHold true", Boolean.TRUE, Boolean.valueOf(def.standstillHold));
        eq("default iconWidthPx", Integer.valueOf(51), Integer.valueOf(def.iconWidthPx()));

        Map<String, String> m = new HashMap<String, String>();
        m.put("signal_mode", "native");
        m.put("off_state", "dim");
        m.put("anchor", "top_left");
        m.put("x_offset", "120");
        m.put("y_offset", "8");
        m.put("icon_size", "10"); // below 20 -> clamped to 20
        m.put("pressure_threshold", "22"); // 2.2
        m.put("decel_threshold", "15"); // 1.5 m/s^2
        m.put("standstill_hold", "false");
        m.put("probe_logging", "false");

        IndicatorConfig parsed = IndicatorConfig.fromMap(m);
        eq("parsed mode native", IndicatorConfig.SignalMode.NATIVE, parsed.signalMode);
        eq("parsed off", IndicatorConfig.OffState.DIM, parsed.offState);
        eq("parsed anchor", IndicatorConfig.Anchor.TOP_LEFT, parsed.anchor);
        eq("clamped icon_size", Integer.valueOf(20), Integer.valueOf(parsed.iconSizePx));
        eq("parsed pressure_threshold", Boolean.TRUE, Math.abs(parsed.pressureThreshold - 2.2f) < 0.001f);
        eq("parsed decel_threshold", Boolean.TRUE, Math.abs(parsed.decelThresholdMps2 - 1.5f) < 0.001f);
        eq("parsed standstill_hold", Boolean.FALSE, Boolean.valueOf(parsed.standstillHold));
        eq("parsed probe_logging", Boolean.FALSE, Boolean.valueOf(parsed.probeLogging));
    }

    private static void testBrakeEvaluatorNativeAndPressure() {
        IndicatorConfig cfg = IndicatorConfig.defaults(); // pressureThreshold = 0.4
        BrakeEvaluator ev = new BrakeEvaluator();

        // 1. CAN invalid sentinel 409.5 must NOT illuminate brake indicator
        BrakeEvaluator.Snapshot s1 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1000L, null, "", 409.5f, 50.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("409.5 invalid pressure ignored", Boolean.FALSE, Boolean.valueOf(s1.illuminated));

        // 2. Light initial pedal cylinder pressure (0.5 >= 0.4) lights immediately with hysteresis down to 0.28
        BrakeEvaluator.Snapshot s2 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1040L, null, "", 0.5f, 50.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("light pedal pressure 0.5 lights immediately", Boolean.TRUE, Boolean.valueOf(s2.illuminated));
        eq("pressure source", "BRAKE_PRESSURE", s2.source);

        BrakeEvaluator.Snapshot s3 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1080L, null, "", 0.32f, 50.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("hysteresis holds at 0.32", Boolean.TRUE, Boolean.valueOf(s3.illuminated));

        BrakeEvaluator.Snapshot s4 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1120L, null, "", 0.1f, 50.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("releases below 0.28", Boolean.FALSE, Boolean.valueOf(s4.illuminated));

        // 3. Direct OEM stop-lamp getter takes precedence even at 0 hydraulic pressure
        BrakeEvaluator.Snapshot s5 = ev.evaluate(cfg, new BrakeEvaluator.Input(
                1160L, Boolean.TRUE, "getStopLampSw=1", 0.0f, 65.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("native stop lamp lights at 0 pressure", Boolean.TRUE, Boolean.valueOf(s5.illuminated));
        eq("native stop lamp source", "OEM_LAMP:getStopLampSw=1", s5.source);
    }

    private static void testBrakeEvaluatorSlidingWindowAndModulatedIpedal() {
        IndicatorConfig defaultCfg = IndicatorConfig.defaults(); // NATIVE_AND_DECEL, 1.2 m/s^2
        Map<String, String> m = new HashMap<String, String>();
        m.put("signal_mode", "native");
        IndicatorConfig nativeOnlyCfg = IndicatorConfig.fromMap(m);

        BrakeEvaluator evNative = new BrakeEvaluator();
        BrakeEvaluator evDefault = new BrakeEvaluator();

        // Simulate 25 Hz (40 ms) polling where CAN speed updates only every 120 ms (every 3rd tick)
        // in 0.7 km/h steps (~1.62 m/s^2) while driver feathers i-Pedal at 12% throttle!
        float canSpeed = 72.0f;
        BrakeEvaluator.Snapshot snapNative = null;
        BrakeEvaluator.Snapshot snapDefault = null;
        int litTicksAfterOnset = 0;

        for (int tick = 0; tick < 15; tick++) {
            long t = 2000L + tick * 40L;
            if (tick > 0 && tick % 3 == 0) {
                canSpeed -= 0.7f;
            }
            BrakeEvaluator.Input in = new BrakeEvaluator.Input(
                    t, null, "", 0.0f, canSpeed, 12.0f, Boolean.FALSE, Boolean.FALSE, 0L);
            snapNative = evNative.evaluate(nativeOnlyCfg, in);
            snapDefault = evDefault.evaluate(defaultCfg, in);
            if (tick >= 6 && snapDefault.illuminated) {
                litTicksAfterOnset++;
            }
        }
        eq("NATIVE mode ignores pure regen decel", Boolean.FALSE, Boolean.valueOf(snapNative.illuminated));
        eq("default NATIVE_AND_DECEL lights on quantized CAN decel + 12% i-Pedal",
                Boolean.TRUE, Boolean.valueOf(snapDefault.illuminated));
        eq("stay lit across all 9 ticks without sawtooth dropout",
                Integer.valueOf(9), Integer.valueOf(litTicksAfterOnset));
        eq("regen source", "REGEN_DECEL", snapDefault.source);
    }

    private static void testBrakeEvaluatorStandstillHoldAndCruiseStop() {
        IndicatorConfig cfg = IndicatorConfig.defaults();
        BrakeEvaluator ev = new BrakeEvaluator();

        // Simulate Smart Cruise Control / i-Pedal decelerating smoothly from 14.4 km/h (4 m/s) to 0 km/h at 2.0 m/s^2
        long t = 5000L;
        float speed = 14.4f;
        BrakeEvaluator.Snapshot snap = null;
        while (speed > 0.0f) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, null, "", 0.0f, speed, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
            t += 40L;
            speed = Math.max(0.0f, speed - 0.288f); // 2.0 m/s^2 = 7.2 km/h/s = 0.288 km/h per 40 ms
        }

        // Now stopped at 0.0 km/h for 3 seconds (0 hydraulic pressure, 0 throttle, in Drive)
        for (int i = 0; i < 20; i++) {
            snap = ev.evaluate(cfg, new BrakeEvaluator.Input(
                    t, null, "", 0.0f, 0.0f, 0.0f, Boolean.FALSE, Boolean.FALSE, 0L));
            t += 150L;
        }
        eq("stays illuminated at 0 km/h after regen/SCC stop", Boolean.TRUE, Boolean.valueOf(snap.illuminated));
        eq("source transitions to STANDSTILL_HOLD", "STANDSTILL_HOLD", snap.source);

        // Driver taps accelerator (3.0%) -> releases standstill hold immediately
        BrakeEvaluator.Snapshot released = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t + 40L, null, "", 0.0f, 0.0f, 3.0f, Boolean.FALSE, Boolean.FALSE, 0L));
        eq("throttle tap releases standstill hold immediately", Boolean.FALSE, Boolean.valueOf(released.illuminated));

        // Manual 3s test overrides everything even in Park
        BrakeEvaluator.Snapshot manual = ev.evaluate(cfg, new BrakeEvaluator.Input(
                t + 100L, null, "", 0.0f, 0.0f, 0.0f, Boolean.TRUE, Boolean.TRUE, t + 3000L));
        eq("manual test lights", Boolean.TRUE, Boolean.valueOf(manual.illuminated));
        eq("manual test source", "MANUAL_TEST", manual.source);
    }

    private static void testUsbExporterMountsAndWrite() throws Exception {
        // 1. Unmounted system (only internal eMMC + tmpfs /storage): must return empty mount list
        String unmountedProcMounts =
                "rootfs / rootfs ro,relatime 0 0\n"
                + "tmpfs /storage tmpfs rw,relatime,mode=0555,gid=1028 0 0\n"
                + "/dev/block/mmcblk0p15 /system ext4 ro,relatime 0 0\n"
                + "/dev/block/mmcblk0p28 /data ext4 rw,nosuid,nodev 0 0\n";
        List<String> none = UsbExporter.parseUsbMountPoints(unmountedProcMounts);
        eq("no USB stick -> 0 mount points", Integer.valueOf(0), Integer.valueOf(none.size()));

        // 2. Mounted USB stick (vold vfat on /mnt/media_rw/usb0 + FUSE mirror on /storage/usb0)
        String mountedProcMounts = unmountedProcMounts
                + "/dev/block/vold/8:1 /mnt/media_rw/usb0 vfat rw,dirsync,nosuid,nodev 0 0\n"
                + "/dev/fuse /storage/usb0 fuse rw,nosuid,nodev,relatime 0 0\n";
        List<String> mounts = UsbExporter.parseUsbMountPoints(mountedProcMounts);
        eq("mounted USB -> 2 candidate paths", Integer.valueOf(2), Integer.valueOf(mounts.size()));
        eq("prefers /storage/usb0 ahead of /mnt/media_rw/usb0", "/storage/usb0", mounts.get(0));
        eq("includes /mnt/media_rw/usb0 fallback", "/mnt/media_rw/usb0", mounts.get(1));

        // 3. Export onto a fresh empty USB root directory (no pre-existing BrakeIndicator/ subfolder!)
        File tmpRoot = Files.createTempDirectory("brake-usb-test").toFile();
        try {
            File fakeUsb0 = new File(tmpRoot, "usb0");
            fakeUsb0.mkdirs();
            File fallback = new File(tmpRoot, "fallback-probe.txt");

            UsbExporter.ExportResult emptyRes = UsbExporter.exportToDirectories(
                    "probe-content", Collections.<String>emptyList(), "BrakeIndicator", fallback);
            eq("empty mounts returns ok=false", Boolean.FALSE, Boolean.valueOf(emptyRes.ok));
            eq("fallback written even when no USB mounted", Boolean.TRUE, Boolean.valueOf(fallback.isFile()));

            UsbExporter.ExportResult okRes = UsbExporter.exportToDirectories(
                    "=== Probe Report v0.2.0 ===\n", Arrays.asList(fakeUsb0.getAbsolutePath()), "BrakeIndicator", fallback);
            eq("export creates BrakeIndicator/ and succeeds", Boolean.TRUE, Boolean.valueOf(okRes.ok));
            eq("saved file exists and non-empty", Boolean.TRUE,
                    Boolean.valueOf(okRes.savedPath != null && new File(okRes.savedPath).length() > 10));
            eq("BrakeIndicator subfolder created automatically", Boolean.TRUE,
                    Boolean.valueOf(new File(fakeUsb0, "BrakeIndicator").isDirectory()));
        } finally {
            deleteRecursive(tmpRoot);
        }
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

    private static void deleteRecursive(File f) {
        if (f == null || !f.exists()) {
            return;
        }
        File[] children = f.listFiles();
        if (children != null) {
            for (File c : children) {
                deleteRecursive(c);
            }
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
