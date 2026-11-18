package dev.lemoyne.brakeindicator;

import java.util.Map;

/**
 * Immutable configuration parsed from HMG-API settings.json values.
 * Note: HMG-API's SettingsForm stores "number" inputs via Long.parseLong,
 * so fractional thresholds are stored in integer tenths (e.g. 12 -> 1.2).
 */
public final class IndicatorConfig {

    public enum ObdMode {
        OBD_WHEN_CONNECTED("obd_when_connected"),
        MODEL_ONLY_ICON("model_only_icon"),
        DISABLED("disabled");

        public final String wireValue;

        ObdMode(String wireValue) {
            this.wireValue = wireValue;
        }

        public static ObdMode fromString(String s) {
            if (s == null) {
                return OBD_WHEN_CONNECTED;
            }
            String t = s.trim();
            if ("model_only_icon".equalsIgnoreCase(t)) {
                return MODEL_ONLY_ICON;
            }
            if ("disabled".equalsIgnoreCase(t) || "off".equalsIgnoreCase(t)) {
                return DISABLED;
            }
            return OBD_WHEN_CONNECTED;
        }

        public ObdMode next() {
            if (this == OBD_WHEN_CONNECTED) {
                return MODEL_ONLY_ICON;
            }
            if (this == MODEL_ONLY_ICON) {
                return DISABLED;
            }
            return OBD_WHEN_CONNECTED;
        }

        public String shortLabel() {
            if (this == MODEL_ONLY_ICON) {
                return "Model Icon (Score vs OBD)";
            }
            if (this == DISABLED) {
                return "OBD Disabled (HMG-API Only)";
            }
            return "Real OBD -> Model Fallback";
        }
    }

    public enum SignalMode {
        PRESSURE_ONLY,
        PRESSURE_AND_DECEL;

        public static SignalMode fromString(String s) {
            if (s != null) {
                String t = s.trim();
                if ("pressure_only".equalsIgnoreCase(t) || "native".equalsIgnoreCase(t)) {
                    return PRESSURE_ONLY;
                }
            }
            return PRESSURE_AND_DECEL;
        }
    }

    public enum OffState {
        HIDDEN,
        DIM;

        public static OffState fromString(String s) {
            if ("dim".equalsIgnoreCase(s)) {
                return DIM;
            }
            return HIDDEN;
        }
    }

    public enum Anchor {
        TOP_RIGHT,
        TOP_CENTER,
        TOP_LEFT;

        public static Anchor fromString(String s) {
            if ("top_left".equalsIgnoreCase(s)) {
                return TOP_LEFT;
            }
            if ("top_center".equalsIgnoreCase(s)) {
                return TOP_CENTER;
            }
            return TOP_RIGHT;
        }
    }

    public final ObdMode obdMode;
    public final SignalMode signalMode;
    public final OffState offState;
    public final Anchor anchor;
    public final int xOffsetPx;
    public final int yOffsetPx;
    public final int iconSizePx;
    public final float pressureThreshold;
    public final float decelThresholdMps2;
    public final boolean standstillHold;
    public final int bridgePort;
    public final boolean autoExportUsb;
    public final boolean autoUploadCloud;
    public final boolean cloudAllowLte;
    public final boolean usbBackupWhenCloud;
    public final boolean autoUpdateModel;

    public IndicatorConfig(
            ObdMode obdMode,
            SignalMode signalMode,
            OffState offState,
            Anchor anchor,
            int xOffsetPx,
            int yOffsetPx,
            int iconSizePx,
            float pressureThreshold,
            float decelThresholdMps2,
            boolean standstillHold,
            int bridgePort) {
        this(
                obdMode,
                signalMode,
                offState,
                anchor,
                xOffsetPx,
                yOffsetPx,
                iconSizePx,
                pressureThreshold,
                decelThresholdMps2,
                standstillHold,
                bridgePort,
                true,
                true,
                true,
                false,
                true);
    }

    public IndicatorConfig(
            ObdMode obdMode,
            SignalMode signalMode,
            OffState offState,
            Anchor anchor,
            int xOffsetPx,
            int yOffsetPx,
            int iconSizePx,
            float pressureThreshold,
            float decelThresholdMps2,
            boolean standstillHold,
            int bridgePort,
            boolean autoExportUsb) {
        this(
                obdMode,
                signalMode,
                offState,
                anchor,
                xOffsetPx,
                yOffsetPx,
                iconSizePx,
                pressureThreshold,
                decelThresholdMps2,
                standstillHold,
                bridgePort,
                autoExportUsb,
                true,
                true,
                false,
                true);
    }

    public IndicatorConfig(
            ObdMode obdMode,
            SignalMode signalMode,
            OffState offState,
            Anchor anchor,
            int xOffsetPx,
            int yOffsetPx,
            int iconSizePx,
            float pressureThreshold,
            float decelThresholdMps2,
            boolean standstillHold,
            int bridgePort,
            boolean autoExportUsb,
            boolean autoUploadCloud,
            boolean cloudAllowLte,
            boolean usbBackupWhenCloud) {
        this(
                obdMode,
                signalMode,
                offState,
                anchor,
                xOffsetPx,
                yOffsetPx,
                iconSizePx,
                pressureThreshold,
                decelThresholdMps2,
                standstillHold,
                bridgePort,
                autoExportUsb,
                autoUploadCloud,
                cloudAllowLte,
                usbBackupWhenCloud,
                true);
    }

    public IndicatorConfig(
            ObdMode obdMode,
            SignalMode signalMode,
            OffState offState,
            Anchor anchor,
            int xOffsetPx,
            int yOffsetPx,
            int iconSizePx,
            float pressureThreshold,
            float decelThresholdMps2,
            boolean standstillHold,
            int bridgePort,
            boolean autoExportUsb,
            boolean autoUploadCloud,
            boolean cloudAllowLte,
            boolean usbBackupWhenCloud,
            boolean autoUpdateModel) {
        this.obdMode = obdMode == null ? ObdMode.OBD_WHEN_CONNECTED : obdMode;
        this.signalMode = signalMode == null ? SignalMode.PRESSURE_AND_DECEL : signalMode;
        this.offState = offState == null ? OffState.HIDDEN : offState;
        this.anchor = anchor == null ? Anchor.TOP_RIGHT : anchor;
        this.xOffsetPx = clampInt(xOffsetPx, -1920, 1920);
        this.yOffsetPx = clampInt(yOffsetPx, 0, 360);
        this.iconSizePx = clampInt(iconSizePx, 20, 96);
        this.pressureThreshold = clampFloat(pressureThreshold, 0.1f, 200.0f);
        this.decelThresholdMps2 = clampFloat(decelThresholdMps2, 0.3f, 6.0f);
        this.standstillHold = standstillHold;
        this.bridgePort = clampInt(bridgePort, 1024, 65535);
        this.autoExportUsb = autoExportUsb;
        this.autoUploadCloud = autoUploadCloud;
        this.cloudAllowLte = cloudAllowLte;
        this.usbBackupWhenCloud = usbBackupWhenCloud;
        this.autoUpdateModel = autoUpdateModel;
    }

    public static IndicatorConfig defaults() {
        return new IndicatorConfig(
                ObdMode.OBD_WHEN_CONNECTED,
                SignalMode.PRESSURE_AND_DECEL,
                OffState.HIDDEN,
                Anchor.TOP_RIGHT,
                260,
                6,
                36,
                0.4f,
                1.2f,
                true,
                35000,
                true,
                true,
                true,
                false,
                true);
    }

    public static IndicatorConfig fromMap(Map<String, String> values) {
        if (values == null || values.isEmpty()) {
            return defaults();
        }
        ObdMode obdMode = ObdMode.fromString(values.get("obd_mode"));
        SignalMode mode = SignalMode.fromString(values.get("signal_mode"));
        OffState off = OffState.fromString(values.get("off_state"));
        Anchor anchor = Anchor.fromString(values.get("anchor"));
        int xOff = parseInt(values.get("x_offset"), 260);
        int yOff = parseInt(values.get("y_offset"), 6);
        int size = parseInt(values.get("icon_size"), 36);
        int pressureTenths = parseInt(values.get("pressure_threshold"), 4);
        int decelTenths = parseInt(values.get("decel_threshold"), 12);
        boolean standstill = parseBoolean(values.get("standstill_hold"), true);
        int bridgePort = parseInt(values.get("bridge_port"), 35000);
        boolean autoExport = parseBoolean(values.get("auto_export_usb"), true);
        boolean autoCloud = parseBoolean(values.get("auto_upload_cloud"), true);
        boolean allowLte = parseBoolean(values.get("cloud_allow_lte"), true);
        boolean usbBackup = parseBoolean(values.get("usb_backup_when_cloud"), false);
        boolean autoModel = parseBoolean(values.get("auto_update_model"), true);
        return new IndicatorConfig(
                obdMode,
                mode,
                off,
                anchor,
                xOff,
                yOff,
                size,
                pressureTenths / 10.0f,
                decelTenths / 10.0f,
                standstill,
                bridgePort,
                autoExport,
                autoCloud,
                allowLte,
                usbBackup,
                autoModel);
    }

    /** Icon cell width matches Gen5W's ~1.42:1 status bar cell aspect ratio (51x36 at height 36). */
    public int iconWidthPx() {
        return Math.max(28, Math.round(iconSizePx * 1.42f));
    }

    private static int parseInt(String s, int fallback) {
        if (s == null || s.trim().isEmpty()) {
            return fallback;
        }
        try {
            return (int) Math.round(Double.parseDouble(s.trim()));
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    private static boolean parseBoolean(String s, boolean fallback) {
        if (s == null || s.trim().isEmpty()) {
            return fallback;
        }
        if ("true".equalsIgnoreCase(s.trim()) || "1".equals(s.trim())) {
            return true;
        }
        if ("false".equalsIgnoreCase(s.trim()) || "0".equals(s.trim())) {
            return false;
        }
        return fallback;
    }

    private static int clampInt(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static float clampFloat(float v, float lo, float hi) {
        if (Float.isNaN(v) || Float.isInfinite(v)) {
            return lo;
        }
        return Math.max(lo, Math.min(hi, v));
    }
}
