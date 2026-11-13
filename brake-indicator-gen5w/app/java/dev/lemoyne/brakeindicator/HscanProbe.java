package dev.lemoyne.brakeindicator;

import android.os.IBinder;

import java.io.File;
import java.io.FileOutputStream;
import java.lang.reflect.Method;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Option 1 read-only OEM signal probe for Gen5W's telematics CAN service
 * (tm_hscan_service in /system/framework/com.hkmc.ext.hcan.jar).
 *
 * Discovers zero-argument primitive getters filtered by CandidateFilter,
 * polls direct stop-lamp / brake-pressure getters every 100 ms, and optionally
 * records candidate transitions for on-car verification and USB export.
 */
public final class HscanProbe {

    private static final String HCAN_JAR = "/system/framework/com.hkmc.ext.hcan.jar";
    private static final String SERVICE_NAME = "tm_hscan_service";
    private static final String STUB_CLASS = "com.hkmc.telematics.hscan.tm_interface.ITM_HSCANService$Stub";
    private static final String IFACE_CLASS = "com.hkmc.telematics.hscan.tm_interface.ITM_HSCANService";
    private static final int MAX_LOG_LINES = 200;

    public static final class PollResult {
        public final boolean connected;
        public final Boolean directStopLamp;
        public final String directStopLampSource;
        public final Float directBrakePressure;
        public final int directCandidateCount;
        public final int diagnosticCandidateCount;
        public final String summary;

        PollResult(
                boolean connected,
                Boolean directStopLamp,
                String directStopLampSource,
                Float directBrakePressure,
                int directCandidateCount,
                int diagnosticCandidateCount,
                String summary) {
            this.connected = connected;
            this.directStopLamp = directStopLamp;
            this.directStopLampSource = directStopLampSource == null ? "" : directStopLampSource;
            this.directBrakePressure = directBrakePressure;
            this.directCandidateCount = directCandidateCount;
            this.diagnosticCandidateCount = diagnosticCandidateCount;
            this.summary = summary == null ? "" : summary;
        }
    }

    private static final class CandidateMethod {
        final Method method;
        final String name;
        final CandidateFilter.Role role;
        String lastValue = "?";

        CandidateMethod(Method method, CandidateFilter.Role role) {
            this.method = method;
            this.name = method.getName();
            this.role = role;
        }
    }

    private final File dataDir;
    private ClassLoader hcanLoader;
    private Object hscanProxy;
    private long lastBindAttemptMs;
    private final List<CandidateMethod> directLampMethods = new ArrayList<CandidateMethod>();
    private final List<CandidateMethod> pressureMethods = new ArrayList<CandidateMethod>();
    private final List<CandidateMethod> diagnosticMethods = new ArrayList<CandidateMethod>();
    private final List<String> transitionLog = new ArrayList<String>();
    private int totalInterfaceMethods;
    private int tickCounter;
    private String lastSummary = "not connected";

    public HscanProbe(File dataDir) {
        this.dataDir = dataDir;
    }

    public synchronized PollResult poll(long nowMs, boolean pollDiagnostics, Float speedKmh, Float brakePrsrFromHub) {
        ensureBound(nowMs);
        if (hscanProxy == null) {
            return new PollResult(
                    false,
                    null,
                    "",
                    null,
                    directLampMethods.size(),
                    diagnosticMethods.size(),
                    lastSummary);
        }

        tickCounter++;
        Boolean anyDirectTrue = null;
        String activeSource = "";
        Float directPressure = null;
        StringBuilder summaryBuilder = new StringBuilder();

        try {
            // 1. Poll direct stop-lamp / brake-switch getters every tick (100 ms)
            for (CandidateMethod cm : directLampMethods) {
                Object val = cm.method.invoke(hscanProxy);
                String text = formatValue(val);
                recordIfChanged(cm, text, pollDiagnostics, speedKmh, brakePrsrFromHub);
                Boolean interpreted = CandidateFilter.interpretBinaryLampValue(val);
                if (Boolean.TRUE.equals(interpreted)) {
                    anyDirectTrue = Boolean.TRUE;
                    if (activeSource.isEmpty()) {
                        activeSource = cm.name + "=" + text;
                    }
                } else if (Boolean.FALSE.equals(interpreted) && anyDirectTrue == null) {
                    anyDirectTrue = Boolean.FALSE;
                }
                appendSummaryItem(summaryBuilder, cm.name, text);
            }

            // 2. Poll cylinder pressure getter(s) every tick so fallback works even on unmapped firmware
            for (CandidateMethod cm : pressureMethods) {
                Object val = cm.method.invoke(hscanProxy);
                String text = formatValue(val);
                recordIfChanged(cm, text, false, speedKmh, brakePrsrFromHub);
                if (val instanceof Number) {
                    float f = ((Number) val).floatValue();
                    if (!Float.isNaN(f) && f >= 0.0f && f <= BrakeEvaluator.MAX_VALID_PRESSURE) {
                        if (directPressure == null || f > directPressure) {
                            directPressure = f;
                        }
                    }
                }
                appendSummaryItem(summaryBuilder, cm.name, text);
            }

            // 3. Poll broader diagnostic candidates every 5th tick (500 ms) when probe logging is enabled
            if (pollDiagnostics && (tickCounter % 5 == 1)) {
                for (CandidateMethod cm : diagnosticMethods) {
                    Object val = cm.method.invoke(hscanProxy);
                    String text = formatValue(val);
                    recordIfChanged(cm, text, true, speedKmh, brakePrsrFromHub);
                    appendSummaryItem(summaryBuilder, cm.name, text);
                }
            } else {
                for (CandidateMethod cm : diagnosticMethods) {
                    if (!"?".equals(cm.lastValue)) {
                        appendSummaryItem(summaryBuilder, cm.name, cm.lastValue);
                    }
                }
            }

            lastSummary = summaryBuilder.length() == 0
                    ? "connected (0 brake/lamp getters matched)"
                    : summaryBuilder.toString();
        } catch (Throwable t) {
            // Binder died or service restarted; rebind on next cycle
            appendLog("tm_hscan_service poll error: " + t.getClass().getSimpleName() + ": " + t.getMessage());
            hscanProxy = null;
            lastSummary = "reconnecting (" + t.getClass().getSimpleName() + ")";
            return new PollResult(
                    false,
                    null,
                    "",
                    null,
                    directLampMethods.size(),
                    diagnosticMethods.size(),
                    lastSummary);
        }

        return new PollResult(
                true,
                anyDirectTrue,
                activeSource,
                directPressure,
                directLampMethods.size(),
                diagnosticMethods.size(),
                lastSummary);
    }

    public synchronized String buildReport(IndicatorConfig cfg, BrakeEvaluator.Snapshot latest) {
        StringBuilder sb = new StringBuilder(2048);
        sb.append("=== Brake Light Indicator (Gen5W) - Option 1 HSCAN Probe Report ===\n");
        sb.append("Generated: ")
                .append(new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()))
                .append('\n');
        sb.append("tm_hscan_service bound: ").append(hscanProxy != null ? "YES" : "NO").append('\n');
        sb.append("ITM_HSCANService total methods scanned: ").append(totalInterfaceMethods).append('\n');
        sb.append("Signal mode: ").append(cfg.signalMode.name()).append('\n');
        if (latest != null) {
            sb.append("Latest state: illuminated=").append(latest.illuminated)
                    .append(" source=").append(latest.source)
                    .append(" pressure=").append(latest.brakePressure)
                    .append(" speedKmh=").append(latest.speedKmh)
                    .append(" decelMps2=").append(String.format(Locale.US, "%.2f", latest.decelMps2))
                    .append('\n');
        }
        sb.append("\n--- Direct Stop-Lamp / Brake-Switch Getters (")
                .append(directLampMethods.size()).append(") ---\n");
        if (directLampMethods.isEmpty()) {
            sb.append("  (none matched on this firmware)\n");
        } else {
            for (CandidateMethod cm : directLampMethods) {
                sb.append("  ").append(cm.name).append("() : ")
                        .append(cm.method.getReturnType().getSimpleName())
                        .append(" = ").append(cm.lastValue).append('\n');
            }
        }

        sb.append("\n--- Brake Pressure Getters (")
                .append(pressureMethods.size()).append(") ---\n");
        if (pressureMethods.isEmpty()) {
            sb.append("  (none matched on this firmware)\n");
        } else {
            for (CandidateMethod cm : pressureMethods) {
                sb.append("  ").append(cm.name).append("() : ")
                        .append(cm.method.getReturnType().getSimpleName())
                        .append(" = ").append(cm.lastValue).append('\n');
            }
        }

        sb.append("\n--- Diagnostic Brake / Regen / Chassis Candidates (")
                .append(diagnosticMethods.size()).append(") ---\n");
        if (diagnosticMethods.isEmpty()) {
            sb.append("  (none matched on this firmware)\n");
        } else {
            for (CandidateMethod cm : diagnosticMethods) {
                sb.append("  ").append(cm.name).append("() : ")
                        .append(cm.method.getReturnType().getSimpleName())
                        .append(" = ").append(cm.lastValue).append('\n');
            }
        }

        sb.append("\n--- Recent Candidate Transitions (")
                .append(transitionLog.size()).append(") ---\n");
        if (transitionLog.isEmpty()) {
            sb.append("  (no transitions recorded yet)\n");
        } else {
            for (String line : transitionLog) {
                sb.append("  ").append(line).append('\n');
            }
        }
        return sb.toString();
    }

    private void ensureBound(long nowMs) {
        if (hscanProxy != null) {
            return;
        }
        if (nowMs - lastBindAttemptMs < 3000L && lastBindAttemptMs != 0L) {
            return;
        }
        lastBindAttemptMs = nowMs;

        try {
            File jarFile = new File(HCAN_JAR);
            if (!jarFile.isFile()) {
                lastSummary = "missing " + HCAN_JAR;
                return;
            }
            if (hcanLoader == null) {
                Class<?> pclClass = Class.forName("dalvik.system.PathClassLoader");
                hcanLoader = (ClassLoader) pclClass
                        .getConstructor(String.class, ClassLoader.class)
                        .newInstance(HCAN_JAR, HscanProbe.class.getClassLoader());
            }

            Class<?> smClass = Class.forName("android.os.ServiceManager");
            Method getService = smClass.getMethod("getService", String.class);
            IBinder binder = (IBinder) getService.invoke(null, SERVICE_NAME);
            if (binder == null) {
                lastSummary = SERVICE_NAME + " not registered yet";
                return;
            }

            Class<?> stubClass = Class.forName(STUB_CLASS, true, hcanLoader);
            Method asInterface = stubClass.getMethod("asInterface", IBinder.class);
            Object proxy = asInterface.invoke(null, binder);
            if (proxy == null) {
                lastSummary = "asInterface returned null";
                return;
            }

            if (directLampMethods.isEmpty() && pressureMethods.isEmpty() && diagnosticMethods.isEmpty()) {
                Class<?> ifaceClass = Class.forName(IFACE_CLASS, true, hcanLoader);
                scanInterfaceMethods(ifaceClass);
            }
            hscanProxy = proxy;
            appendLog("Bound " + SERVICE_NAME + " ("
                    + directLampMethods.size() + " lamp, "
                    + pressureMethods.size() + " pressure, "
                    + diagnosticMethods.size() + " diag getters out of "
                    + totalInterfaceMethods + " methods)");
        } catch (Throwable t) {
            lastSummary = "bind failed: " + t.getClass().getSimpleName();
            appendLog("Bind " + SERVICE_NAME + " failed: " + t);
        }
    }

    private void scanInterfaceMethods(Class<?> ifaceClass) {
        Method[] methods = ifaceClass.getDeclaredMethods();
        totalInterfaceMethods = methods.length;
        Map<String, CandidateMethod> byName = new LinkedHashMap<String, CandidateMethod>();

        for (Method m : methods) {
            CandidateFilter.Role role = CandidateFilter.classify(
                    m.getName(),
                    m.getParameterTypes().length,
                    m.getReturnType());
            if (role == CandidateFilter.Role.NONE) {
                continue;
            }
            m.setAccessible(true);
            byName.put(m.getName(), new CandidateMethod(m, role));
        }

        List<CandidateMethod> sorted = new ArrayList<CandidateMethod>(byName.values());
        Collections.sort(sorted, new Comparator<CandidateMethod>() {
            @Override
            public int compare(CandidateMethod a, CandidateMethod b) {
                return a.name.compareTo(b.name);
            }
        });

        for (CandidateMethod cm : sorted) {
            if (cm.role == CandidateFilter.Role.DIRECT_STOP_LAMP) {
                directLampMethods.add(cm);
            } else if (cm.role == CandidateFilter.Role.BRAKE_PRESSURE) {
                pressureMethods.add(cm);
            } else if (cm.role == CandidateFilter.Role.DIAGNOSTIC_CANDIDATE) {
                diagnosticMethods.add(cm);
            }
        }
    }

    private void recordIfChanged(
            CandidateMethod cm,
            String newValue,
            boolean logTransition,
            Float speedKmh,
            Float brakePressure) {
        String prev = cm.lastValue;
        if (newValue.equals(prev)) {
            return;
        }
        cm.lastValue = newValue;
        if (logTransition) {
            String ctx = String.format(
                    Locale.US,
                    "%s: %s -> %s (spd=%s, prsr=%s)",
                    cm.name,
                    prev,
                    newValue,
                    speedKmh == null ? "-" : String.format(Locale.US, "%.1f", speedKmh),
                    brakePressure == null ? "-" : String.format(Locale.US, "%.1f", brakePressure));
            appendLog(ctx);
        }
    }

    private void appendLog(String msg) {
        String ts = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date());
        String line = ts + "  " + msg;
        System.out.println("brake-indicator: " + msg);
        transitionLog.add(line);
        while (transitionLog.size() > MAX_LOG_LINES) {
            transitionLog.remove(0);
        }
        persistLogSnapshot();
    }

    private void persistLogSnapshot() {
        if (dataDir == null) {
            return;
        }
        try {
            if (!dataDir.isDirectory()) {
                dataDir.mkdirs();
            }
            File f = new File(dataDir, "transitions.log");
            StringBuilder sb = new StringBuilder();
            for (String l : transitionLog) {
                sb.append(l).append('\n');
            }
            FileOutputStream out = new FileOutputStream(f);
            try {
                out.write(sb.toString().getBytes("UTF-8"));
            } finally {
                out.close();
            }
        } catch (Exception ignored) {
            // Non-fatal
        }
    }

    private static void appendSummaryItem(StringBuilder sb, String name, String val) {
        if (sb.length() > 220) {
            return;
        }
        if (sb.length() > 0) {
            sb.append(", ");
        }
        sb.append(name).append('=').append(val);
    }

    private static String formatValue(Object val) {
        if (val == null) {
            return "null";
        }
        if (val instanceof Float || val instanceof Double) {
            double d = ((Number) val).doubleValue();
            if (Double.isNaN(d) || Double.isInfinite(d)) {
                return "NaN";
            }
            return String.format(Locale.US, "%.1f", d);
        }
        return String.valueOf(val);
    }
}
