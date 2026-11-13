package dev.lemoyne.brakeindicator;

import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Lightweight line-based IPC framing between MonitorMain (uid 1000 service)
 * and OverlayService / MainActivity over an abstract UNIX domain socket.
 */
public final class IpcProtocol {

    public static final String SOCKET_NAME = "dev.lemoyne.brakeindicator.ipc";

    private IpcProtocol() {}

    public static String encodeSnapshot(
            BrakeEvaluator.Snapshot snap,
            IndicatorConfig cfg,
            boolean hscanConnected,
            int directCount,
            int diagCount,
            String candidateSummary) {
        StringBuilder sb = new StringBuilder(256);
        sb.append("STATE");
        appendKv(sb, "lit", snap.illuminated ? "1" : "0");
        appendKv(sb, "src", sanitize(snap.source));
        appendKv(sb, "lamp", snap.nativeStopLamp == null ? "-" : (snap.nativeStopLamp ? "1" : "0"));
        appendKv(sb, "lamp_name", sanitize(snap.nativeStopLampName));
        appendKv(sb, "prsr", fmtFloat(snap.brakePressure, 1));
        appendKv(sb, "spd", fmtFloat(snap.speedKmh, 1));
        appendKv(sb, "accel", fmtFloat(snap.accelPedal, 1));
        appendKv(sb, "decel", String.format(Locale.US, "%.2f", snap.decelMps2));
        appendKv(sb, "park", snap.gearPark == null ? "-" : (snap.gearPark ? "1" : "0"));
        appendKv(sb, "epb", snap.parkingBrake == null ? "-" : (snap.parkingBrake ? "1" : "0"));
        appendKv(sb, "mode", cfg.signalMode.name());
        appendKv(sb, "off", cfg.offState.name());
        appendKv(sb, "anchor", cfg.anchor.name());
        appendKv(sb, "x", String.valueOf(cfg.xOffsetPx));
        appendKv(sb, "y", String.valueOf(cfg.yOffsetPx));
        appendKv(sb, "size", String.valueOf(cfg.iconSizePx));
        appendKv(sb, "hscan", hscanConnected ? "1" : "0");
        appendKv(sb, "n_lamp", String.valueOf(directCount));
        appendKv(sb, "n_diag", String.valueOf(diagCount));
        appendKv(sb, "cands", sanitize(candidateSummary));
        return sb.toString();
    }

    public static Map<String, String> decodeLine(String line) {
        Map<String, String> map = new LinkedHashMap<String, String>();
        if (line == null) {
            return map;
        }
        String[] parts = line.trim().split("\t");
        if (parts.length == 0) {
            return map;
        }
        map.put("_type", parts[0]);
        for (int i = 1; i < parts.length; i++) {
            int eq = parts[i].indexOf('=');
            if (eq > 0) {
                map.put(parts[i].substring(0, eq), parts[i].substring(eq + 1));
            }
        }
        return map;
    }

    private static void appendKv(StringBuilder sb, String k, String v) {
        sb.append('\t').append(k).append('=').append(v == null ? "" : v);
    }

    private static String fmtFloat(Float v, int decimals) {
        if (v == null) {
            return "-";
        }
        return String.format(Locale.US, "%." + decimals + "f", v);
    }

    private static String sanitize(String s) {
        if (s == null) {
            return "";
        }
        return s.replace('\t', ' ').replace('\n', ' ').replace('\r', ' ').trim();
    }
}
