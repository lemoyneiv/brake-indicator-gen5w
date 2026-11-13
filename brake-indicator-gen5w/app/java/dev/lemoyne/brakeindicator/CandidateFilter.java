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
        // Per-car identity, location, credentials, telecom (unambiguous multi-char stems)
        "gps", "coord", "nmea", "latitude", "longitude", "location",
        "imei", "msisdn", "imsi", "usim", "iccid", "serial",
        "btaddr", "ssid", "bssid", "passwd", "password",
        "token", "cert", "secret", "crypt", "auth", "login", "account",
        "phone", "modem",
        // Mutating / callback / system-control substrings
        "write", "register", "callback", "listener",
        "dispatch", "trigger", "exec", "command",
        "reboot", "shutdown", "delete", "remove", "flash",
        // Known side-effecting getter on HKMC Gen5W BCM (triggers OSD popup)
        "drivemode"
    };

    private static final String[] BLOCKED_TOKENS = {
        // Short acronyms/verbs matched on CamelCase / underscore token boundaries
        // so e.g. "vin" blocks getVIN / getVINStatus without blocking IsCarMoving
        "vin", "lat", "lon", "pos", "eid", "pcid", "mac", "tel", "nad",
        "send", "put", "post", "req", "request", "reset", "clear",
        "init", "open", "close", "notify", "cmd", "diag", "ota"
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
        "brake", "brk", "stop", "stplmp", "lamp", "lmp", "cyl", "prsr",
        "pedal", "pdl", "regen", "decel", "longaccel", "ieb", "esc",
        "autohold", "avh", "epb", "hac", "hill",
        "scc", "cruise", "adas", "fca", "vcu", "mcu", "hcu", "ems", "xev"
    };

    private static final String[] NOISY_CONTINUOUS_PATTERNS = {
        "accelpdl", "carspeed", "whlspd", "wheelspeed", "sasangle", "steer",
        "odometer", "odo", "batsoc", "soh", "temp", "volt", "curr",
        "clock", "timer", "alive", "aliv", "crc"
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

    /**
     * Returns true if a zero-argument method is strictly read-only, returns a primitive
     * boolean/number, and cannot expose per-car identity/location or trigger side effects.
     */
    public static boolean isSafeReadOnlyGetter(String methodName, int paramCount, Class<?> returnType) {
        return extractSafeCompactStem(methodName, paramCount, returnType) != null;
    }

    /**
     * Returns true for high-rate analog telemetry (speed, steering angle, throttle, temperatures)
     * that should appear in the getter inventory snapshot without flooding the transition log.
     */
    public static boolean isNoisyContinuousGetter(String methodName) {
        if (methodName == null) {
            return false;
        }
        String compact = methodName.toLowerCase(Locale.US).replace("_", "");
        for (String p : NOISY_CONTINUOUS_PATTERNS) {
            if (compact.contains(p)) {
                return true;
            }
        }
        return false;
    }

    private static String extractSafeCompactStem(String methodName, int paramCount, Class<?> returnType) {
        if (methodName == null || paramCount != 0 || !isAllowedReturnType(returnType)) {
            return null;
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
            return null;
        }
        String compact = stem.replace("_", "");
        if (compact.isEmpty()) {
            return null;
        }
        // Reject Object/Binder plumbing
        if ("class".equals(compact) || "hashcode".equals(compact) || "interfacedescriptor".equals(compact)) {
            return null;
        }
        for (String blocked : BLOCKED_SUBSTRINGS) {
            if (lower.contains(blocked) || compact.contains(blocked)) {
                return null;
            }
        }
        java.util.List<String> tokens = tokenizeIdentifier(methodName);
        for (String tok : tokens) {
            for (String blockedTok : BLOCKED_TOKENS) {
                if (blockedTok.equals(tok)) {
                    return null;
                }
            }
        }
        // Guard against "start..." methods disguised behind get/is
        if (compact.startsWith("start")) {
            return null;
        }
        return compact;
    }

    private static java.util.List<String> tokenizeIdentifier(String name) {
        java.util.List<String> tokens = new java.util.ArrayList<String>(8);
        String[] parts = name.split("_");
        for (String part : parts) {
            if (part.isEmpty()) {
                continue;
            }
            int start = 0;
            int len = part.length();
            for (int i = 1; i < len; i++) {
                char c = part.charAt(i);
                char prev = part.charAt(i - 1);
                boolean nextLower = (i + 1 < len) && Character.isLowerCase(part.charAt(i + 1));
                if (Character.isUpperCase(c) && (Character.isLowerCase(prev) || nextLower)) {
                    tokens.add(part.substring(start, i).toLowerCase(Locale.US));
                    start = i;
                } else if (Character.isDigit(c) && !Character.isDigit(prev)) {
                    tokens.add(part.substring(start, i).toLowerCase(Locale.US));
                    start = i;
                } else if (!Character.isDigit(c) && Character.isDigit(prev)) {
                    tokens.add(part.substring(start, i).toLowerCase(Locale.US));
                    start = i;
                }
            }
            tokens.add(part.substring(start).toLowerCase(Locale.US));
        }
        return tokens;
    }

    public static Role classify(String methodName, int paramCount, Class<?> returnType) {
        String compact = extractSafeCompactStem(methodName, paramCount, returnType);
        if (compact == null) {
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
