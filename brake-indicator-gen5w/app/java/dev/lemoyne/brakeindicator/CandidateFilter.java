package dev.lemoyne.brakeindicator;

import java.util.Locale;

/**
 * Pure-Java safety filter and classifier for read-only reflection against OEM
 * Gen5W Binder interfaces (primarily ITM_HSCANService in com.hkmc.ext.hcan.jar).
 *
 * Hard rules enforced:
 *  - Zero parameters only.
 *  - Primitive boolean or numeric return types only (never String/byte[]/Object).
 *  - Never touches per-car identity or location (VIN, GPS, IMEI, MSISDN, USIM, MAC, etc.).
 *  - Never touches state-mutating methods or known side-effecting getters (e.g. getDriveMode).
 */
public final class CandidateFilter {

    public enum Role {
        /** Ignored or blocked for safety. */
        NONE,
        /** Direct binary stop-lamp / brake-switch getter (0 = off, >0 / true = on). */
        DIRECT_STOP_LAMP,
        /** Brake master/IEB cylinder pressure getter (e.g. getCylPrsrVal). */
        BRAKE_PRESSURE,
        /** Other read-only brake/regen/chassis getter logged for Option 1 on-car discovery. */
        DIAGNOSTIC_CANDIDATE
    }

    private static final String[] BLOCKED_SUBSTRINGS = {
        // Per-car identity, location, credentials, telecom
        "vin", "gps", "lat", "lon", "coord", "nmea",
        "imei", "msisdn", "imsi", "usim", "iccid", "eid", "pcid",
        "serial", "mac", "btaddr", "ssid", "bssid", "passwd", "password",
        "token", "cert", "secret", "crypt", "auth", "login", "account",
        "phone", "tel", "nad", "modem",
        // Mutating / request / side-effecting verbs
        "set", "send", "write", "put", "post", "req", "reset", "clear",
        "init", "open", "close", "register", "callback", "listener",
        "notify", "dispatch", "trigger", "exec", "cmd", "command",
        "diag", "ota", "flash", "reboot", "shutdown", "delete", "remove",
        // Known side-effecting getter on HKMC Gen5W BCM (triggers OSD popup)
        "drivemode"
    };

    private static final String[] STOP_LAMP_FAULTS_OR_PARKING = {
        "park", "epb", "fluid", "notch", "pad", "wear",
        "fail", "fault", "err", "warn", "lampfail", "bulb"
    };

    private static final String[] DIRECT_LAMP_PATTERNS = {
        "stoplamp", "stoplight", "stopsw", "stplmp", "stpsw",
        "brakelamp", "brakelight", "brakesw", "brakeswitch", "brakepedal", "brakepdl",
        "brklamp", "brklight", "brklmp", "brksw", "brkpdl",
        "actbrklmp", "regenbrklmp", "regenstoplamp", "iebbrklmp",
        "hmsl", "chmsl"
    };

    private static final String[] PRESSURE_PATTERNS = {
        "cylprsr", "brakeprsr", "brkprsr", "brakepressure", "cylinderpressure"
    };

    private static final String[] DIAGNOSTIC_PATTERNS = {
        "brake", "brk", "stop", "stplmp", "cyl", "prsr",
        "pedal", "pdl", "regen", "decel", "ieb", "esc",
        "autohold", "avh", "epb", "hac", "hill"
    };

    private CandidateFilter() {}

    public static boolean isAllowedReturnType(Class<?> returnType) {
        if (returnType == null || !returnType.isPrimitive() || returnType == Void.TYPE) {
            return false;
        }
        return returnType == Boolean.TYPE
                || returnType == Byte.TYPE
                || returnType == Short.TYPE
                || returnType == Integer.TYPE
                || returnType == Long.TYPE
                || returnType == Float.TYPE
                || returnType == Double.TYPE;
    }

    public static Role classify(String methodName, int paramCount, Class<?> returnType) {
        if (methodName == null || paramCount != 0 || !isAllowedReturnType(returnType)) {
            return Role.NONE;
        }
        String lower = methodName.toLowerCase(Locale.US);
        String stem;
        if (lower.startsWith("get")) {
            stem = lower.substring(3);
        } else if (lower.startsWith("is")) {
            stem = lower.substring(2);
        } else if (lower.startsWith("navi_get")) {
            stem = lower.substring(8);
        } else if (lower.startsWith("navi_is")) {
            stem = lower.substring(7);
        } else {
            return Role.NONE;
        }
        String compact = stem.replace("_", "");
        if (compact.isEmpty()) {
            return Role.NONE;
        }
        // Reject Object/Binder plumbing
        if ("class".equals(compact) || "hashcode".equals(compact) || "interfacedescriptor".equals(compact)) {
            return Role.NONE;
        }
        for (String blocked : BLOCKED_SUBSTRINGS) {
            if (lower.contains(blocked) || compact.contains(blocked)) {
                return Role.NONE;
            }
        }
        // Guard against "start..." methods disguised behind get/is
        if (compact.startsWith("start")) {
            return Role.NONE;
        }

        for (String p : PRESSURE_PATTERNS) {
            if (compact.contains(p)) {
                return Role.BRAKE_PRESSURE;
            }
        }

        boolean hasFaultOrParking = false;
        for (String excl : STOP_LAMP_FAULTS_OR_PARKING) {
            if (compact.contains(excl)) {
                hasFaultOrParking = true;
                break;
            }
        }

        if (!hasFaultOrParking) {
            for (String p : DIRECT_LAMP_PATTERNS) {
                if (compact.contains(p)) {
                    return Role.DIRECT_STOP_LAMP;
                }
            }
        }

        for (String p : DIAGNOSTIC_PATTERNS) {
            if (compact.contains(p)) {
                // Exclude generic non-brake "stop" getters like getAutoStopTimer unless lamp/sw/brk present
                if ("stop".equals(p) && !(compact.contains("lamp") || compact.contains("lmp")
                        || compact.contains("light") || compact.contains("sw") || compact.contains("brk")
                        || compact.contains("brake"))) {
                    continue;
                }
                return Role.DIAGNOSTIC_CANDIDATE;
            }
        }

        return Role.NONE;
    }

    /**
     * Interpret a DIRECT_STOP_LAMP return value as active/inactive, filtering out
     * standard HKMC CAN invalid sentinels (0xFF, 0xFFFF, negative error codes).
     */
    public static Boolean interpretBinaryLampValue(Object raw) {
        if (raw == null) {
            return null;
        }
        if (raw instanceof Boolean) {
            return (Boolean) raw;
        }
        if (raw instanceof Number) {
            double v = ((Number) raw).doubleValue();
            if (Double.isNaN(v) || Double.isInfinite(v) || v < 0) {
                return null;
            }
            long iv = ((Number) raw).longValue();
            // On HKMC HSCAN 2-bit/4-bit/8-bit status signals:
            // 0 = Off, 1 = On, 2/3 (on 2-bit fault/invalid) or 0xFF / 0xFFFF = Invalid
            if (iv == 0xFF || iv == 0xFFFF || iv == 0x7FFFFFFF) {
                return null;
            }
            if (iv == 0) {
                return Boolean.FALSE;
            }
            if (iv == 1) {
                return Boolean.TRUE;
            }
            // Many 2-bit CAN status fields use 0=Off, 1=On, 2=Reserved/Error, 3=Invalid
            if (iv == 2 || iv == 3) {
                return null;
            }
            return Boolean.TRUE;
        }
        return null;
    }
}
