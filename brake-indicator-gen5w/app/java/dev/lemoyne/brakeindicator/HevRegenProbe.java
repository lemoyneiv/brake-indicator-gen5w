package dev.lemoyne.brakeindicator;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;

/**
 * Read-only Gen5W OEM service & {@code HEVManager$HEVFlag} snapshot/diff probe for discovering
 * whether the head unit receives live steering-wheel paddle / recuperation level changes.
 *
 * Inspects numeric/boolean getters only across:
 *  - {@code com.hkmc.hev.HEVManager$HEVFlag} via {@code IHEVService.getHEVStatus(int)}
 *  - {@code com.hkmc.hev.IHEVService} zero-arg primitive getters
 *  - {@code com.hkmc.usm.IUSMService} zero-arg primitive getters (EV / Smart Recuperation settings)
 *  - {@code com.hkmc.cluster.IClusterService} zero-arg primitive getters
 *  - Selected regen/eco/accel getters on {@code IBCMService} and {@code ITM_HSCANService}
 *
 * Hard privacy/safety invariants:
 *  - Never invokes mutators ({@code set*}, {@code request*}, {@code send*}, {@code register*}).
 *  - Never reads {@code String}, {@code byte[]}, or object getters (primitive numbers/booleans only).
 *  - Explicitly blocks any flag/method name resembling VIN, serials, phone/account/GPS identifiers.
 */
public final class HevRegenProbe {

    public interface ChangeListener {
        void onHevSignalChanged(String key, String oldVal, String newVal, boolean regenKeywordMatch);
    }

    public interface SignalSource {
        String serviceStatusSummary();
        Map<String, String> readSignals(boolean fullScan);
    }

    private static final int CHANGE_LOG_CAP = 100;

    private static final String[] BLOCKED_NAME_TOKENS = {
        "vin", "serial", "sn_", "_sn", "imei", "iccid", "imsi", "msisdn",
        "phone", "tel", "number", "account", "user", "token", "secret",
        "pass", "key", "cert", "mac", "addr", "ssid", "bssid", "uuid",
        "lat", "lon", "gps", "coord", "dest", "bluetooth"
    };

    private static final String[] REGEN_PRIORITY_TOKENS = {
        "regen", "recup", "paddle", "ipal", "ipedal", "i_pedal", "pedal",
        "coast", "smart", "eco", "drive", "mode", "gear", " shift",
        "decel", "longaccel", "brk", "brake", "scc", "cruise", "hda"
    };

    private static final String[] DIRECT_REGEN_TOKENS = {
        "regen", "recup", "paddle", "ipedal", "i_pedal", "coast"
    };

    private static final String[] NOISY_COUNTER_TOKENS = {
        "odometer", "drivetime", "idletime", "runtime", "runningtime",
        "carspeederror", "sasangle", "cylprsr", "accelpdl", "car_speed", "carspeed"
    };

    private final SignalSource source;
    private ChangeListener listener;

    private final Map<String, String> initialValues = new TreeMap<String, String>();
    private final Map<String, String> baselineValues = new TreeMap<String, String>();
    private final Map<String, String> latestValues = new TreeMap<String, String>();
    private final Map<String, Integer> changeCounts = new TreeMap<String, Integer>();

    private final String[] changeLog = new String[CHANGE_LOG_CAP];
    private int changeHead;
    private int changeSize;

    private long baselineWallMs;
    private int markCounter;
    private volatile String latestDirectRegenKey = "";
    private volatile int latestDirectRegenRaw = -1;
    private volatile String lastDiffSummary = "none";
    private volatile int baselineChangedCount;

    public HevRegenProbe() {
        this(new ReflectionGen5wSignalSource());
    }

    public HevRegenProbe(SignalSource source) {
        this.source = source == null ? new ReflectionGen5wSignalSource() : source;
    }

    public synchronized void setChangeListener(ChangeListener l) {
        this.listener = l;
    }

    public String getLatestDirectRegenKey() {
        return latestDirectRegenKey;
    }

    public int getLatestDirectRegenRaw() {
        return latestDirectRegenRaw;
    }

    public String getLastDiffSummary() {
        return lastDiffSummary;
    }

    public int getBaselineChangedCount() {
        return baselineChangedCount;
    }

    /**
     * Polls Gen5W HEV/USM/CLUSTER signals (`fullScan=true` reads every safe numeric getter;
     * `fullScan=false` reads all `HEVFlag` constants plus regen/eco/mode candidate getters).
     */
    public void poll(boolean fullScan) {
        Map<String, String> snap;
        try {
            snap = source.readSignals(fullScan);
        } catch (Throwable t) {
            return;
        }
        if (snap == null || snap.isEmpty()) {
            return;
        }

        List<PendingChange> notifyList = null;
        synchronized (this) {
            long wallNow = System.currentTimeMillis();
            boolean firstEver = initialValues.isEmpty();

            for (Map.Entry<String, String> e : snap.entrySet()) {
                String key = e.getKey();
                String val = e.getValue();
                if (key == null || val == null || !isSafeSignalName(key)) {
                    continue;
                }
                if (!initialValues.containsKey(key)) {
                    initialValues.put(key, val);
                }
                if (baselineWallMs > 0L && !baselineValues.containsKey(key)) {
                    baselineValues.put(key, val);
                }

                String prev = latestValues.put(key, val);
                if (!firstEver && prev != null && !prev.equals(val)) {
                    Integer c = changeCounts.get(key);
                    changeCounts.put(key, Integer.valueOf(c == null ? 1 : c.intValue() + 1));
                    if (!isNoisyCounter(key)) {
                        boolean regenMatch = matchesAnyToken(key, REGEN_PRIORITY_TOKENS);
                        String ts = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date(wallNow));
                        pushChangeLine("[" + ts + "] " + key + ": " + prev + " -> " + val
                                + (regenMatch ? "  [CANDIDATE]" : ""));
                        lastDiffSummary = shortSignalName(key) + ": " + prev + "->" + val;
                        if (notifyList == null) {
                            notifyList = new ArrayList<PendingChange>(4);
                        }
                        notifyList.add(new PendingChange(key, prev, val, regenMatch));
                    }
                }
            }

            recomputeBaselineAndDirectCandidateLocked();
        }

        if (notifyList != null) {
            ChangeListener cb;
            synchronized (this) {
                cb = this.listener;
            }
            if (cb != null) {
                for (int i = 0; i < notifyList.size(); i++) {
                    PendingChange pc = notifyList.get(i);
                    try {
                        cb.onHevSignalChanged(pc.key, pc.oldVal, pc.newVal, pc.regenMatch);
                    } catch (Throwable ignored) {
                    }
                }
            }
        }
    }

    /**
     * Captures a fresh full baseline across all safe HEVFlag / USM / CLUSTER numeric signals
     * right before the driver clicks the steering-wheel regen paddle.
     */
    public String markBaseline() {
        Map<String, String> snap;
        try {
            snap = source.readSignals(true);
        } catch (Throwable t) {
            snap = Collections.emptyMap();
        }
        synchronized (this) {
            markCounter++;
            baselineWallMs = System.currentTimeMillis();
            if (snap != null) {
                for (Map.Entry<String, String> e : snap.entrySet()) {
                    if (e.getKey() != null && e.getValue() != null && isSafeSignalName(e.getKey())) {
                        latestValues.put(e.getKey(), e.getValue());
                        if (!initialValues.containsKey(e.getKey())) {
                            initialValues.put(e.getKey(), e.getValue());
                        }
                    }
                }
            }
            baselineValues.clear();
            baselineValues.putAll(latestValues);
            baselineChangedCount = 0;
            lastDiffSummary = "baseline #" + markCounter + " (" + baselineValues.size() + " signals)";
            String ts = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date(baselineWallMs));
            pushChangeLine("[" + ts + "] === MARKED BASELINE #" + markCounter
                    + " (" + baselineValues.size() + " signals) ===");
            recomputeBaselineAndDirectCandidateLocked();
            return "MARK #" + markCounter + " (" + baselineValues.size() + " HEV/USM/CLU signals)";
        }
    }

    public synchronized String compactUiSummary() {
        int total = latestValues.size();
        if (total == 0) {
            return "scanning Gen5W HEV/USM...";
        }
        StringBuilder sb = new StringBuilder(96);
        if (!latestDirectRegenKey.isEmpty() && latestDirectRegenRaw >= 0) {
            sb.append(shortSignalName(latestDirectRegenKey)).append('=').append(latestDirectRegenRaw).append(" | ");
        }
        if (baselineWallMs > 0L) {
            sb.append("diff=").append(baselineChangedCount).append(" (").append(lastDiffSummary).append(')');
        } else {
            sb.append(total).append(" sigs (last: ").append(lastDiffSummary).append(')');
        }
        return sb.toString();
    }

    public String buildReport(boolean refreshFullScan) {
        if (refreshFullScan) {
            poll(true);
        }
        synchronized (this) {
            StringBuilder sb = new StringBuilder(4096);
            sb.append("=== Gen5W HEVFlag / USM / CLUSTER Regen Signal Probe ===\n");
            sb.append("Generated       : ")
              .append(new SimpleDateFormat("yyyy-MM-dd HH:mm:ss Z", Locale.US).format(new Date()))
              .append('\n');
            sb.append("Services        : ").append(source.serviceStatusSummary()).append('\n');
            sb.append("Total Signals   : ").append(latestValues.size()).append('\n');
            sb.append("Direct Candidate: ")
              .append(latestDirectRegenKey.isEmpty()
                      ? "none"
                      : (latestDirectRegenKey + " = " + latestDirectRegenRaw))
              .append('\n');
            if (baselineWallMs > 0L) {
                sb.append("Active Baseline : #").append(markCounter).append(" at ")
                  .append(new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date(baselineWallMs)))
                  .append(" (changed since baseline: ").append(baselineChangedCount).append(")\n");
            } else {
                sb.append("Active Baseline : none (comparing against session startup)\n");
            }

            // 1. Signals that changed since baseline (or since startup if no baseline marked)
            Map<String, String> ref = baselineWallMs > 0L ? baselineValues : initialValues;
            sb.append("\n--- Signals Changed Since ")
              .append(baselineWallMs > 0L ? ("Baseline #" + markCounter) : "Session Start")
              .append(" ---\n");
            int diffPrinted = 0;
            for (Map.Entry<String, String> e : latestValues.entrySet()) {
                String k = e.getKey();
                String cur = e.getValue();
                String base = ref.get(k);
                if (base != null && !base.equals(cur) && !isNoisyCounter(k)) {
                    diffPrinted++;
                    Integer chg = changeCounts.get(k);
                    sb.append("  ").append(k).append(" : ").append(base).append(" -> ").append(cur)
                      .append("  [changes=").append(chg == null ? 1 : chg.intValue()).append(']')
                      .append(matchesAnyToken(k, REGEN_PRIORITY_TOKENS) ? "  <-- REGEN/MODE CANDIDATE" : "")
                      .append('\n');
                }
            }
            if (diffPrinted == 0) {
                sb.append("  (none)\n");
            }

            // 2. Timestamped live change log
            sb.append("\n--- Live Signal Change Log (newest first) ---\n");
            if (changeSize == 0) {
                sb.append("  (no discrete HEV/USM/CLUSTER changes observed yet)\n");
            } else {
                for (int i = changeSize - 1; i >= 0; i--) {
                    int idx = (changeHead + i) % CHANGE_LOG_CAP;
                    sb.append("  ").append(changeLog[idx]).append('\n');
                }
            }

            // 3. All HEVFlag constants & priority regen/mode getters
            sb.append("\n--- All Safe HEVFlag & Regen/Mode Signals (")
              .append(latestValues.size()).append(") ---\n");
            for (Map.Entry<String, String> e : latestValues.entrySet()) {
                String k = e.getKey();
                String cur = e.getValue();
                Integer chg = changeCounts.get(k);
                sb.append("  ").append(k).append(" = ").append(cur);
                if ("65535".equals(cur)) {
                    sb.append(" (0xFFFF invalid)");
                }
                if (chg != null && chg.intValue() > 0) {
                    sb.append("  [changes=").append(chg.intValue()).append(']');
                }
                sb.append('\n');
            }
            return sb.toString();
        }
    }

    private void recomputeBaselineAndDirectCandidateLocked() {
        Map<String, String> ref = baselineWallMs > 0L ? baselineValues : initialValues;
        int diffs = 0;
        String changedDirectKey = "";
        int changedDirectVal = -1;
        String staticDirectKey = "";
        int staticDirectVal = -1;

        for (Map.Entry<String, String> e : latestValues.entrySet()) {
            String k = e.getKey();
            String cur = e.getValue();
            if (isNoisyCounter(k)) {
                continue;
            }
            String base = ref.get(k);
            boolean changed = (base != null && !base.equals(cur));
            if (changed) {
                diffs++;
            }
            if (matchesAnyToken(k, DIRECT_REGEN_TOKENS)) {
                Integer parsed = parseValidSmallInt(cur);
                if (parsed != null) {
                    if (changed && changedDirectKey.isEmpty()) {
                        changedDirectKey = k;
                        changedDirectVal = parsed.intValue();
                    } else if (staticDirectKey.isEmpty()) {
                        staticDirectKey = k;
                        staticDirectVal = parsed.intValue();
                    }
                }
            }
        }
        baselineChangedCount = diffs;
        if (!changedDirectKey.isEmpty()) {
            latestDirectRegenKey = changedDirectKey;
            latestDirectRegenRaw = changedDirectVal;
        } else if (!staticDirectKey.isEmpty()) {
            latestDirectRegenKey = staticDirectKey;
            latestDirectRegenRaw = staticDirectVal;
        }
    }

    private static Integer parseValidSmallInt(String s) {
        if (s == null || s.isEmpty()) return null;
        try {
            int v = Integer.parseInt(s.trim());
            if (v < 0 || v == 250 || v == 255 || v == 65535 || v > 32000) {
                return null;
            }
            return Integer.valueOf(v);
        } catch (NumberFormatException e) {
            return null;
        }
    }

    private void pushChangeLine(String line) {
        if (changeSize < CHANGE_LOG_CAP) {
            int idx = (changeHead + changeSize) % CHANGE_LOG_CAP;
            changeLog[idx] = line;
            changeSize++;
        } else {
            changeLog[changeHead] = line;
            changeHead = (changeHead + 1) % CHANGE_LOG_CAP;
        }
    }

    public static boolean isSafeSignalName(String name) {
        if (name == null || name.trim().isEmpty()) {
            return false;
        }
        String lower = name.toLowerCase(Locale.US);
        for (int i = 0; i < BLOCKED_NAME_TOKENS.length; i++) {
            if (lower.contains(BLOCKED_NAME_TOKENS[i])) {
                return false;
            }
        }
        return true;
    }

    public static boolean isSafeGetterMethod(Method m) {
        if (m == null || m.getParameterTypes().length != 0) {
            return false;
        }
        String name = m.getName();
        if (!(name.startsWith("get") && name.length() > 3)
                && !(name.startsWith("is") && name.length() > 2)) {
            return false;
        }
        if ("getClass".equals(name) || "asBinder".equals(name) || "getInterfaceDescriptor".equals(name)) {
            return false;
        }
        if (!isSafeSignalName(name)) {
            return false;
        }
        Class<?> rt = m.getReturnType();
        return rt == int.class
                || rt == boolean.class
                || rt == byte.class
                || rt == short.class
                || rt == long.class
                || rt == float.class
                || rt == double.class;
    }

    static boolean matchesAnyToken(String text, String[] tokens) {
        if (text == null) return false;
        String lower = text.toLowerCase(Locale.US);
        for (int i = 0; i < tokens.length; i++) {
            if (lower.contains(tokens[i])) {
                return true;
            }
        }
        return false;
    }

    private static boolean isNoisyCounter(String key) {
        return matchesAnyToken(key, NOISY_COUNTER_TOKENS);
    }

    private static String shortSignalName(String key) {
        int dot = key.lastIndexOf('.');
        return dot >= 0 && dot + 1 < key.length() ? key.substring(dot + 1) : key;
    }

    private static final class PendingChange {
        final String key;
        final String oldVal;
        final String newVal;
        final boolean regenMatch;

        PendingChange(String key, String oldVal, String newVal, boolean regenMatch) {
            this.key = key;
            this.oldVal = oldVal;
            this.newVal = newVal;
            this.regenMatch = regenMatch;
        }
    }

    /**
     * Reflection-backed Gen5W service reader (loads {@code com.hkmc.*} and {@code com.hkmc.ext.hcan.jar}
     * lazily on the head unit, and no-ops cleanly on off-car host JVMs).
     */
    static final class ReflectionGen5wSignalSource implements SignalSource {

        private static final String HCAN_JAR = "/system/framework/com.hkmc.ext.hcan.jar";

        private boolean initialized;
        private Object hevService;
        private Method getHevStatusMethod;
        private final Map<String, Integer> hevFlags = new LinkedHashMap<String, Integer>();

        private final List<BoundGetter> fastGetters = new ArrayList<BoundGetter>();
        private final List<BoundGetter> allExtraGetters = new ArrayList<BoundGetter>();
        private String statusText = "uninitialized";

        @Override
        public synchronized String serviceStatusSummary() {
            ensureInitialized();
            return statusText;
        }

        @Override
        public synchronized Map<String, String> readSignals(boolean fullScan) {
            ensureInitialized();
            Map<String, String> out = new LinkedHashMap<String, String>();

            if (hevService != null && getHevStatusMethod != null) {
                for (Map.Entry<String, Integer> f : hevFlags.entrySet()) {
                    try {
                        Object v = getHevStatusMethod.invoke(hevService, f.getValue());
                        if (v != null) {
                            out.put(f.getKey(), String.valueOf(v));
                        }
                    } catch (Throwable ignored) {
                    }
                }
            }

            List<BoundGetter> list = fullScan ? allExtraGetters : fastGetters;
            for (int i = 0; i < list.size(); i++) {
                BoundGetter bg = list.get(i);
                try {
                    Object v = bg.method.invoke(bg.target);
                    if (v != null) {
                        out.put(bg.key, formatPrimitive(v));
                    }
                } catch (Throwable ignored) {
                }
            }
            return out;
        }

        private void ensureInitialized() {
            if (initialized) {
                return;
            }
            initialized = true;
            try {
                Class<?> smClass = Class.forName("android.os.ServiceManager");
                Method getService = smClass.getMethod("getService", String.class);
                Class<?> ibinderClass = Class.forName("android.os.IBinder");

                Class<?> carSvc = null;
                try {
                    carSvc = Class.forName("com.hkmc.Service");
                } catch (Throwable ignored) {}

                // 1. HEV service + HEVManager$HEVFlag constants
                hevService = bindStub(getService, ibinderClass,
                        resolveServiceName(carSvc, "HEV_SERVICE", "com.hkmc.micom.hev"),
                        "com.hkmc.hev.IHEVService$Stub", null);
                if (hevService != null) {
                    try {
                        Class<?> hevIface = Class.forName("com.hkmc.hev.IHEVService");
                        getHevStatusMethod = hevIface.getMethod("getHEVStatus", int.class);
                        registerGetters("HEV", hevService, hevIface);
                    } catch (Throwable ignored) {}

                    try {
                        Class<?> flagCls = Class.forName("com.hkmc.hev.HEVManager$HEVFlag");
                        Field[] fields = flagCls.getFields();
                        Map<String, Integer> sorted = new TreeMap<String, Integer>();
                        for (int i = 0; i < fields.length; i++) {
                            Field f = fields[i];
                            int mod = f.getModifiers();
                            if (Modifier.isStatic(mod) && f.getType() == int.class && isSafeSignalName(f.getName())) {
                                sorted.put("HEVFlag." + f.getName() + "(" + f.getInt(null) + ")",
                                        Integer.valueOf(f.getInt(null)));
                            }
                        }
                        hevFlags.putAll(sorted);
                    } catch (Throwable ignored) {}
                }

                // 2. USM service (EV / Smart Recuperation user settings)
                Object usm = bindStub(getService, ibinderClass,
                        resolveServiceName(carSvc, "USM_SERVICE", "com.hkmc.micom.usm"),
                        "com.hkmc.usm.IUSMService$Stub", null);
                if (usm != null) {
                    try {
                        registerGetters("USM", usm, Class.forName("com.hkmc.usm.IUSMService"));
                    } catch (Throwable ignored) {}
                }

                // 3. CLUSTER service
                Object clu = bindStub(getService, ibinderClass,
                        resolveServiceName(carSvc, "CLUSTER_SERVICE", "com.hkmc.micom.cluster"),
                        "com.hkmc.cluster.IClusterService$Stub", null);
                if (clu != null) {
                    try {
                        registerGetters("CLU", clu, Class.forName("com.hkmc.cluster.IClusterService"));
                    } catch (Throwable ignored) {}
                }

                // 4. BCM service
                Object bcm = bindStub(getService, ibinderClass,
                        resolveServiceName(carSvc, "BCM_SERVICE", "com.hkmc.micom.bcm"),
                        "com.hkmc.bcm.IBCMService$Stub", null);
                if (bcm != null) {
                    try {
                        registerGetters("BCM", bcm, Class.forName("com.hkmc.bcm.IBCMService"));
                    } catch (Throwable ignored) {}
                }

                // 5. HSCAN service (for getEcoLevel, getLongAccelVal, getGearPosition, etc.)
                Object hscan = null;
                try {
                    ClassLoader hcanLoader = HevRegenProbe.class.getClassLoader();
                    java.io.File jar = new java.io.File(HCAN_JAR);
                    if (jar.isFile()) {
                        Class<?> pcl = Class.forName("dalvik.system.PathClassLoader");
                        hcanLoader = (ClassLoader) pcl.getConstructor(String.class, ClassLoader.class)
                                .newInstance(HCAN_JAR, hcanLoader);
                    }
                    Class<?> adapter = Class.forName("com.hkmc.telematics.hscan.tm_interface.TM_HSCANAdapter", true, hcanLoader);
                    String hscanName = resolveServiceName(adapter, "TM_HSCAN_SERVICE", "tm_hscan_service");
                    hscan = bindStub(getService, ibinderClass, hscanName,
                            "com.hkmc.telematics.hscan.tm_interface.ITM_HSCANService$Stub", hcanLoader);
                    if (hscan != null) {
                        registerGetters("HSCAN", hscan,
                                Class.forName("com.hkmc.telematics.hscan.tm_interface.ITM_HSCANService", true, hcanLoader));
                    }
                } catch (Throwable ignored) {}

                statusText = "HEV=" + (hevService != null ? ("YES(" + hevFlags.size() + " flags)") : "NO")
                        + " USM=" + (usm != null ? "YES" : "NO")
                        + " CLU=" + (clu != null ? "YES" : "NO")
                        + " BCM=" + (bcm != null ? "YES" : "NO")
                        + " HSCAN=" + (hscan != null ? "YES" : "NO");
            } catch (Throwable t) {
                statusText = "off-car / unavailable (" + t.getClass().getSimpleName() + ")";
            }
        }

        private void registerGetters(String prefix, Object serviceInstance, Class<?> iface) {
            if (serviceInstance == null || iface == null) return;
            Method[] methods = iface.getMethods();
            Map<String, Method> sorted = new TreeMap<String, Method>();
            for (int i = 0; i < methods.length; i++) {
                Method m = methods[i];
                if (isSafeGetterMethod(m)) {
                    sorted.put(m.getName(), m);
                }
            }
            for (Map.Entry<String, Method> e : sorted.entrySet()) {
                String key = prefix + "." + e.getKey();
                BoundGetter bg = new BoundGetter(key, serviceInstance, e.getValue());
                allExtraGetters.add(bg);
                if ("HEV".equals(prefix)
                        || "CLU".equals(prefix)
                        || matchesAnyToken(key, REGEN_PRIORITY_TOKENS)) {
                    fastGetters.add(bg);
                }
            }
        }

        private static String resolveServiceName(Class<?> holder, String fieldName, String fallback) {
            if (holder != null) {
                try {
                    Object v = holder.getField(fieldName).get(null);
                    if (v instanceof String && !((String) v).isEmpty()) {
                        return (String) v;
                    }
                } catch (Throwable ignored) {}
            }
            return fallback;
        }

        private static Object bindStub(
                Method getService,
                Class<?> ibinderClass,
                String serviceName,
                String stubClassName,
                ClassLoader loader) {
            try {
                Object binder = getService.invoke(null, serviceName);
                if (binder == null) return null;
                Class<?> stub = loader == null
                        ? Class.forName(stubClassName)
                        : Class.forName(stubClassName, true, loader);
                Method asInterface = stub.getMethod("asInterface", ibinderClass);
                return asInterface.invoke(null, binder);
            } catch (Throwable t) {
                return null;
            }
        }

        private static String formatPrimitive(Object v) {
            if (v instanceof Float) {
                float f = ((Float) v).floatValue();
                if (Float.isNaN(f) || Float.isInfinite(f)) return "NaN";
                return String.format(Locale.US, "%.2f", Float.valueOf(f));
            }
            if (v instanceof Double) {
                double d = ((Double) v).doubleValue();
                if (Double.isNaN(d) || Double.isInfinite(d)) return "NaN";
                return String.format(Locale.US, "%.2f", Double.valueOf(d));
            }
            return String.valueOf(v);
        }

        private static final class BoundGetter {
            final String key;
            final Object target;
            final Method method;

            BoundGetter(String key, Object target, Method method) {
                this.key = key;
                this.target = target;
                this.method = method;
            }
        }
    }
}
