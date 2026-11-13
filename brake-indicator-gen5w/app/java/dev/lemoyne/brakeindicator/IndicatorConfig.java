package dev.lemoyne.brakeindicator;

import java.util.Map;

/**
 * Immutable configuration parsed from HMG-API settings.json values.
 * Note: HMG-API's SettingsForm stores "number" inputs via Long.parseLong,
 * so fractional thresholds are stored in integer tenths (e.g. 15 -> 1.5).
 */
public final class IndicatorConfig {

    public enum SignalMode {
        NATIVE,
        NATIVE_AND_DECEL;

        public static SignalMode fromString(String s) {
            if ("native_and_decel".equalsIgnoreCase(s)) {
                return NATIVE_AND_DECEL;
            }
            return NATIVE;
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

    public final SignalMode signalMode;
    public final OffState offState;
    public final Anchor anchor;
    public final int xOffsetPx;
    public final int yOffsetPx;
    public final int iconSizePx;
    public final float pressureThreshold;
    public final float decelThresholdMps2;
    public final boolean probeLogging;

    public IndicatorConfig(
            SignalMode signalMode,
            OffState offState,
            Anchor anchor,
            int xOffsetPx,
            int yOffsetPx,
            int iconSizePx,
            float pressureThreshold,
            float decelThresholdMps2,
            boolean probeLogging) {
        this.signalMode = signalMode == null ? SignalMode.NATIVE : signalMode;
        this.offState = offState == null ? OffState.HIDDEN : offState;
        this.anchor = anchor == null ? Anchor.TOP_RIGHT : anchor;
        this.xOffsetPx = clampInt(xOffsetPx, -1920, 1920);
        this.yOffsetPx = clampInt(yOffsetPx, 0, 360);
        this.iconSizePx = clampInt(iconSizePx, 20, 96);
        this.pressureThreshold = clampFloat(pressureThreshold, 0.1f, 200.0f);
        this.decelThresholdMps2 = clampFloat(decelThresholdMps2, 0.3f, 6.0f);
        this.probeLogging = probeLogging;
    }

    public static IndicatorConfig defaults() {
        return new IndicatorConfig(
                SignalMode.NATIVE,
                OffState.HIDDEN,
                Anchor.TOP_RIGHT,
                260,
                6,
                36,
                1.5f,
                1.3f,
                true);
    }

    public static IndicatorConfig fromMap(Map<String, String> values) {
        if (values == null || values.isEmpty()) {
            return defaults();
        }
        SignalMode mode = SignalMode.fromString(values.get("signal_mode"));
        OffState off = OffState.fromString(values.get("off_state"));
        Anchor anchor = Anchor.fromString(values.get("anchor"));
        int xOff = parseInt(values.get("x_offset"), 260);
        int yOff = parseInt(values.get("y_offset"), 6);
        int size = parseInt(values.get("icon_size"), 36);
        int pressureTenths = parseInt(values.get("pressure_threshold"), 15);
        int decelTenths = parseInt(values.get("decel_threshold"), 13);
        boolean probe = parseBoolean(values.get("probe_logging"), true);
        return new IndicatorConfig(
                mode,
                off,
                anchor,
                xOff,
                yOff,
                size,
                pressureTenths / 10.0f,
                decelTenths / 10.0f,
                probe);
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
