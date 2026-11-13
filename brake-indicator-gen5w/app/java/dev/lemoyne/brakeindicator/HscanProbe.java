package dev.lemoyne.brakeindicator;

import android.os.IBinder;

import java.io.File;
import java.io.FileOutputStream;
import java.lang.reflect.Field;
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
 * Direct low-latency OEM Binder reader and signal discovery probe for Gen5W:
 *  - Binds `tm_hscan_service` (`ITM_HSCANService` in `/system/framework/com.hkmc.ext.hcan.jar`)
 *    for zero-hop 25 Hz reads of `getCylPrsrVal()`, `getCarSpeed((byte) 0)`,
 *    `getAccelPdlVal()`, `IsCarMoving()`, and any direct stop-lamp getters.
 *  - Binds `ISystemService` (`com.hkmc.Service.SYSTEM_SERVICE`) for fast `getGearPStatus()`
 *    and `getEPBStatus()`, plus `IBCMService` (`com.hkmc.Service.BCM_SERVICE`) for safe
 *    read-only lamp/chassis getters.
 *  - Inventories all safe zero-argument primitive getters on `ITM_HSCANService` and records
 *    live state transitions during pedal, regenerative, and Smart Cruise Control braking.
 */
public final class HscanProbe {

    private static final String HCAN_JAR = "/system/framework/com.hkmc.ext.hcan.jar";
    private static final String DEFAULT_HSCAN_SERVICE = "tm_hscan_service";
    private static final String HSCAN_ADAPTER_CLASS =
            "com.hkmc.telematics.hscan.tm_interface.TM_HSCANAdapter";
    private static final String HSCAN_STUB_CLASS =
            "com.hkmc.telematics.hscan.tm_interface.ITM_HSCANService$Stub";
    private static final String HSCAN_IFACE_CLASS =
            "com.hkmc.telematics.hscan.tm_interface.ITM_HSCANService";

    private static final String HKMC_SERVICE_CONST_CLASS = "com.hkmc.Service";
    private static final String SYSTEM_STUB_CLASS = "com.hkmc.micom.ISystemService$Stub";
    private static final String SYSTEM_IFACE_CLASS = "com.hkmc.micom.ISystemService";
    private static final String BCM_STUB_CLASS = "com.hkmc.bcm.IBCMService$Stub";
    private static final String BCM_IFACE_CLASS = "com.hkmc.bcm.IBCMService";

    private static final int MAX_LOG_LINES = 500;
    private static final long REBIND_COOLDOWN_MS = 3000L;
    private static final long DIAG_POLL_INTERVAL_MS = 500L;
    private static final long LOG_PERSIST_INTERVAL_MS = 10000L;

    public static final class PollResult {
        public final boolean connected;
        public final Boolean directStopLamp;
        public final String directStopLampSource;
        public final Float directBrakePressure;
        public final Float directSpeedKmh;
        public final Float directAccelPedal;
        public final Boolean directMoving;
        public final Boolean directGearPark;
        public final Boolean directParkingBrake;
        public final int directCandidateCount;
        public final int diagnosticCandidateCount;
        public final String summary;

        PollResult(
                boolean connected,
                Boolean directStopLamp,
                String directStopLampSource,
                Float directBrakePressure,
                Float directSpeedKmh,
                Float directAccelPedal,
                Boolean directMoving,
                Boolean directGearPark,
                Boolean directParkingBrake,
                int directCandidateCount,
                int diagnosticCandidateCount,
                String summary) {
            this.connected = connected;
            this.directStopLamp = directStopLamp;
            this.directStopLampSource = directStopLampSource == null ? "" : directStopLampSource;
            this.directBrakePressure = directBrakePressure;
            this.directSpeedKmh = directSpeedKmh;
            this.directAccelPedal = directAccelPedal;
            this.directMoving = directMoving;
            this.directGearPark = directGearPark;
            this.directParkingBrake = directParkingBrake;
            this.directCandidateCount = directCandidateCount;
            this.diagnosticCandidateCount = diagnosticCandidateCount;
            this.summary = summary == null ? "" : summary;
        }
    }

    private enum OwnerService {
        HSCAN("HSCAN"),
        SYSTEM("SYSTEM"),
        BCM("BCM");

        final String tag;

        OwnerService(String tag) {
            this.tag = tag;
        }
    }

    private static final class CandidateMethod {
        final OwnerService owner;
        final Method method;
        final String name;
        final String qualifiedName;
        final CandidateFilter.Role role;
        final boolean noisyContinuous;
        final boolean floatingPoint;
        String lastValue = "?";
        double lastNumeric = Double.NaN;
        int changeCount;

        CandidateMethod(OwnerService owner, Method method, CandidateFilter.Role role) {
            this.owner = owner;
            this.method = method;
            this.name = method.getName();
            this.qualifiedName = owner == OwnerService.HSCAN ? name : (owner.tag + "." + name);
            this.role = role;
            this.noisyContinuous = CandidateFilter.isNoisyContinuousGetter(name);
            Class<?> rt = method.getReturnType();
            this.floatingPoint = (rt == Float.TYPE || rt == Double.TYPE);
        }
    }

    private final File dataDir;
    private ClassLoader hcanLoader;
    private Object hscanProxy;
    private Object systemProxy;
    private Object bcmProxy;

    private Method carSpeedMethod;
    private Method accelPedalMethod;
    private Method carMovingMethod;
    private Method gearParkMethod;
    private Method epbMethod;

    private boolean hscanScanned;
    private boolean systemScanned;
    private boolean bcmScanned;

    private long lastBindAttemptMs;
    private long lastDiagPollMs;
    private long lastPersistMs;
    private boolean logDirty;

    private final List<CandidateMethod> directLampMethods = new ArrayList<CandidateMethod>();
    private final List<CandidateMethod> pressureMethods = new ArrayList<CandidateMethod>();
    private final List<CandidateMethod> diagnosticMethods = new ArrayList<CandidateMethod>();
    private final List<CandidateMethod> otherSafeHscanMethods = new ArrayList<CandidateMethod>();
    private final List<String> transitionLog = new ArrayList<String>();

    private int totalHscanMethods;
    private String lastSummary = "not connected";

    public HscanProbe(File dataDir) {
        this.dataDir = dataDir;
    }

    /**
     * Fast 25 Hz in-process Binder poll (< 0.5 ms). Reads brake pressure, vehicle speed,
     * accelerator pedal, gear Park, EPB, and any direct binary stop-lamp getters.
     */
    public synchronized PollResult pollFast(long nowMs) {
        ensureBound(nowMs);
        int diagTotal = diagnosticMethods.size() + otherSafeHscanMethods.size();
        if (hscanProxy == null && systemProxy == null) {
            return new PollResult(
                    false,
                    null,
                    "",
                    null,
                    null,
                    null,
                    null,
                    null,
                    null,
                    directLampMethods.size(),
                    diagTotal,
                    lastSummary);
        }

        Boolean anyDirectTrue = null;
        String activeSource = "";
        Float directPressure = null;
        Float directSpeed = null;
        Float directAccel = null;
        Boolean directMoving = null;
        Boolean directPark = null;
        Boolean directEpb = null;
        StringBuilder summaryBuilder = new StringBuilder();

        // 1. Poll ISystemService (Gear P & EPB) if bound
        if (systemProxy != null) {
            try {
                if (gearParkMethod != null) {
                    Object gp = gearParkMethod.invoke(systemProxy);
                    if (gp instanceof Number) {
                        int v = ((Number) gp).intValue();
                        if (v >= 0) {
                            directPark = Boolean.valueOf(v == 1);
                        }
                    }
                }
                if (epbMethod != null) {
                    Object epb = epbMethod.invoke(systemProxy);
                    if (epb instanceof Number) {
                        int st = ((Number) epb).intValue();
                        if (st == 2) {
                            directEpb = Boolean.TRUE;
                        } else if (st == 1) {
                            directEpb = Boolean.FALSE;
                        }
                    }
                }
            } catch (Throwable t) {
                systemProxy = null;
            }
        }

        // 2. Poll direct stop-lamp getters across bound services
        for (CandidateMethod cm : directLampMethods) {
            Object target = proxyFor(cm.owner);
            if (target == null) {
                continue;
            }
            try {
                Object val = cm.method.invoke(target);
                String text = formatValue(val);
                Boolean interpreted = CandidateFilter.interpretBinaryLampValue(val);
                if (Boolean.TRUE.equals(interpreted)) {
                    anyDirectTrue = Boolean.TRUE;
                    if (activeSource.isEmpty()) {
                        activeSource = cm.qualifiedName + "=" + text;
                    }
                } else if (Boolean.FALSE.equals(interpreted) && anyDirectTrue == null) {
                    anyDirectTrue = Boolean.FALSE;
                }
                updateCandidateValue(cm, val, text, true, directSpeed, directPressure, directAccel, 0.0f, activeSource);
                appendSummaryItem(summaryBuilder, cm.qualifiedName, text);
            } catch (Throwable t) {
                clearProxy(cm.owner, t);
            }
        }

        // 3. Poll ITM_HSCANService core fast telemetry
        if (hscanProxy != null) {
            try {
                if (carSpeedMethod != null) {
                    Object sp = carSpeedMethod.invoke(hscanProxy, Byte.valueOf((byte) 0));
                    if (sp instanceof Number) {
                        float f = ((Number) sp).floatValue();
                        if (!Float.isNaN(f) && !Float.isInfinite(f) && f >= 0.0f && f <= BrakeEvaluator.MAX_VALID_SPEED_KMH) {
                            directSpeed = Float.valueOf(f);
                        }
                    }
                }
                if (accelPedalMethod != null) {
                    Object ap = accelPedalMethod.invoke(hscanProxy);
                    if (ap instanceof Number) {
                        float f = ((Number) ap).floatValue();
                        if (!Float.isNaN(f) && !Float.isInfinite(f) && f >= 0.0f && f <= BrakeEvaluator.MAX_VALID_ACCEL_PEDAL) {
                            directAccel = Float.valueOf(f);
                        }
                    }
                }
                if (carMovingMethod != null) {
                    Object mv = carMovingMethod.invoke(hscanProxy);
                    if (mv instanceof Boolean) {
                        directMoving = (Boolean) mv;
                    }
                }
                for (CandidateMethod cm : pressureMethods) {
                    Object val = cm.method.invoke(hscanProxy);
                    String text = formatValue(val);
                    updateCandidateValue(cm, val, text, false, directSpeed, directPressure, directAccel, 0.0f, "");
                    if (val instanceof Number) {
                        float f = ((Number) val).floatValue();
                        if (!Float.isNaN(f) && f >= 0.0f && f <= BrakeEvaluator.MAX_VALID_PRESSURE) {
                            if (directPressure == null || f > directPressure.floatValue()) {
                                directPressure = Float.valueOf(f);
                            }
                        }
                    }
                    appendSummaryItem(summaryBuilder, cm.qualifiedName, text);
                }
            } catch (Throwable t) {
                clearProxy(OwnerService.HSCAN, t);
            }
        }

        for (CandidateMethod cm : diagnosticMethods) {
            if (!"?".equals(cm.lastValue)) {
                appendSummaryItem(summaryBuilder, cm.qualifiedName, cm.lastValue);
            }
        }

        lastSummary = summaryBuilder.length() == 0
                ? (hscanProxy != null ? "connected" : "waiting for tm_hscan_service")
                : summaryBuilder.toString();

        return new PollResult(
                hscanProxy != null,
                anyDirectTrue,
                activeSource,
                directPressure,
                directSpeed,
                directAccel,
                directMoving,
                directPark,
                directEpb,
                directLampMethods.size(),
                diagTotal,
                lastSummary);
    }

    /**
     * Runs at 2 Hz AFTER fast brake evaluation & overlay broadcast so diagnostic reflection
     * adds zero latency to brake-lamp illumination. Scans all safe OEM getters to capture any
     * signal that changes during pedal, regenerative, or Smart Cruise Control braking.
     */
    public synchronized void pollDiagnosticsIfDue(
            long nowMs,
            boolean probeLogging,
            BrakeEvaluator.Snapshot snap) {
        if (!probeLogging || nowMs - lastDiagPollMs < DIAG_POLL_INTERVAL_MS) {
            return;
        }
        lastDiagPollMs = nowMs;

        Float spd = snap != null ? snap.speedKmh : null;
        Float prsr = snap != null ? snap.brakePressure : null;
        Float accel = snap != null ? snap.accelPedal : null;
        float decel = snap != null ? snap.decelMps2 : 0.0f;
        String src = snap != null ? snap.source : "OFF";

        for (CandidateMethod cm : diagnosticMethods) {
            Object target = proxyFor(cm.owner);
            if (target == null) {
                continue;
            }
            try {
                Object val = cm.method.invoke(target);
                String text = formatValue(val);
                updateCandidateValue(cm, val, text, true, spd, prsr, accel, decel, src);
            } catch (Throwable t) {
                clearProxy(cm.owner, t);
            }
        }

        if (hscanProxy != null) {
            for (CandidateMethod cm : otherSafeHscanMethods) {
                try {
                    Object val = cm.method.invoke(hscanProxy);
                    String text = formatValue(val);
                    updateCandidateValue(cm, val, text, true, spd, prsr, accel, decel, src);
                } catch (Throwable t) {
                    clearProxy(OwnerService.HSCAN, t);
                    break;
                }
            }
        }

        if (logDirty && (lastPersistMs == 0L || nowMs - lastPersistMs >= LOG_PERSIST_INTERVAL_MS)) {
            persistLogSnapshot();
            lastPersistMs = nowMs;
            logDirty = false;
        }
    }

    public synchronized String buildReport(IndicatorConfig cfg, BrakeEvaluator.Snapshot latest) {
        StringBuilder sb = new StringBuilder(4096);
        sb.append("=== Brake Light Indicator v0.2.0 (Gen5W) - OEM CAN Signal Probe Report ===\n");
        sb.append("Generated: ")
                .append(new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date()))
                .append('\n');
        sb.append("Services bound: tm_hscan_service=").append(hscanProxy != null ? "YES" : "NO")
                .append("  ISystemService=").append(systemProxy != null ? "YES" : "NO")
                .append("  IBCMService=").append(bcmProxy != null ? "YES" : "NO")
                .append('\n');
        sb.append("ITM_HSCANService total methods scanned: ").append(totalHscanMethods)
                .append(" (direct=").append(directLampMethods.size())
                .append(", pressure=").append(pressureMethods.size())
                .append(", diag=").append(diagnosticMethods.size())
                .append(", otherSafe=").append(otherSafeHscanMethods.size())
                .append(")\n");
        sb.append("Config: mode=").append(cfg.signalMode.name())
                .append(" pressureThreshold=").append(String.format(Locale.US, "%.2f", cfg.pressureThreshold))
                .append(" decelThreshold=").append(String.format(Locale.US, "%.2f", cfg.decelThresholdMps2))
                .append(" standstillHold=").append(cfg.standstillHold)
                .append('\n');
        if (latest != null) {
            sb.append("Latest state: illuminated=").append(latest.illuminated)
                    .append(" source=").append(latest.source)
                    .append(" pressure=").append(latest.brakePressure)
                    .append(" speedKmh=").append(latest.speedKmh)
                    .append(" decelMps2=").append(String.format(Locale.US, "%.2f", latest.decelMps2))
                    .append(" accelPedal=").append(latest.accelPedal)
                    .append(" gearPark=").append(latest.gearPark)
                    .append(" epb=").append(latest.parkingBrake)
                    .append('\n');
        }

        appendMethodSection(sb, "Direct Stop-Lamp / Brake-Switch Getters", directLampMethods);
        appendMethodSection(sb, "Brake Pressure Getters", pressureMethods);
        appendMethodSection(sb, "Diagnostic Brake / Regen / SCC / Chassis Getters", diagnosticMethods);
        appendMethodSection(sb, "All Other Safe Read-Only ITM_HSCANService Getters", otherSafeHscanMethods);

        sb.append("\n--- Recorded Getter Transitions During Drive (")
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

    private static void appendMethodSection(StringBuilder sb, String title, List<CandidateMethod> methods) {
        sb.append("\n--- ").append(title).append(" (").append(methods.size()).append(") ---\n");
        if (methods.isEmpty()) {
            sb.append("  (none matched on this firmware)\n");
            return;
        }
        for (CandidateMethod cm : methods) {
            sb.append("  ").append(cm.qualifiedName).append("() : ")
                    .append(cm.method.getReturnType().getSimpleName())
                    .append(" = ").append(cm.lastValue);
            if (cm.changeCount > 0) {
                sb.append("  [changes=").append(cm.changeCount).append(']');
            }
            sb.append('\n');
        }
    }

    private Object proxyFor(OwnerService owner) {
        if (owner == OwnerService.SYSTEM) {
            return systemProxy;
        }
        if (owner == OwnerService.BCM) {
            return bcmProxy;
        }
        return hscanProxy;
    }

    private void clearProxy(OwnerService owner, Throwable t) {
        appendLog(owner.tag + " poll error: " + t.getClass().getSimpleName());
        if (owner == OwnerService.SYSTEM) {
            systemProxy = null;
        } else if (owner == OwnerService.BCM) {
            bcmProxy = null;
        } else {
            hscanProxy = null;
            lastSummary = "reconnecting (" + t.getClass().getSimpleName() + ")";
        }
    }

    private void ensureBound(long nowMs) {
        if (hscanProxy != null && systemProxy != null && bcmProxy != null) {
            return;
        }
        if (lastBindAttemptMs != 0L && nowMs - lastBindAttemptMs < REBIND_COOLDOWN_MS) {
            return;
        }
        lastBindAttemptMs = nowMs;

        try {
            Class<?> smClass = Class.forName("android.os.ServiceManager");
            Method getService = smClass.getMethod("getService", String.class);

            if (hscanProxy == null) {
                bindHscan(getService);
            }
            if (systemProxy == null) {
                bindSystemService(getService);
            }
            if (bcmProxy == null) {
                bindBcmService(getService);
            }
        } catch (Throwable t) {
            lastSummary = "bind error: " + t.getClass().getSimpleName();
        }
    }

    private void bindHscan(Method getService) {
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

            String serviceName = DEFAULT_HSCAN_SERVICE;
            try {
                Class<?> adapterClass = Class.forName(HSCAN_ADAPTER_CLASS, true, hcanLoader);
                Field f = adapterClass.getField("TM_HSCAN_SERVICE");
                Object val = f.get(null);
                if (val instanceof String && !((String) val).isEmpty()) {
                    serviceName = (String) val;
                }
            } catch (Throwable ignored) {
            }

            IBinder binder = (IBinder) getService.invoke(null, serviceName);
            if (binder == null) {
                lastSummary = serviceName + " not registered yet";
                return;
            }

            Class<?> stubClass = Class.forName(HSCAN_STUB_CLASS, true, hcanLoader);
            Object proxy = stubClass.getMethod("asInterface", IBinder.class).invoke(null, binder);
            if (proxy == null) {
                lastSummary = "HSCAN asInterface null";
                return;
            }

            Class<?> ifaceClass = Class.forName(HSCAN_IFACE_CLASS, true, hcanLoader);
            if (!hscanScanned) {
                scanHscanInterface(ifaceClass);
                hscanScanned = true;
            }
            hscanProxy = proxy;
            appendLog("Bound " + serviceName + " ("
                    + directLampMethods.size() + " lamp, "
                    + pressureMethods.size() + " pressure, "
                    + diagnosticMethods.size() + " diag, "
                    + otherSafeHscanMethods.size() + " other safe getters out of "
                    + totalHscanMethods + " HSCAN methods)");
        } catch (Throwable t) {
            lastSummary = "HSCAN bind failed: " + t.getClass().getSimpleName();
            appendLog("Bind HSCAN failed: " + t);
        }
    }

    private void bindSystemService(Method getService) {
        try {
            String svcName = readHkmcServiceConstant("SYSTEM_SERVICE", "com.hkmc.micom.system");
            IBinder binder = (IBinder) getService.invoke(null, svcName);
            if (binder == null) {
                return;
            }
            Class<?> stubClass = Class.forName(SYSTEM_STUB_CLASS);
            Object proxy = stubClass.getMethod("asInterface", IBinder.class).invoke(null, binder);
            if (proxy == null) {
                return;
            }
            Class<?> ifaceClass = Class.forName(SYSTEM_IFACE_CLASS);
            try {
                gearParkMethod = ifaceClass.getMethod("getGearPStatus");
            } catch (NoSuchMethodException ignored) {
            }
            try {
                epbMethod = ifaceClass.getMethod("getEPBStatus");
            } catch (NoSuchMethodException ignored) {
            }
            if (!systemScanned) {
                scanAuxiliaryInterface(OwnerService.SYSTEM, ifaceClass);
                systemScanned = true;
            }
            systemProxy = proxy;
        } catch (Throwable ignored) {
        }
    }

    private void bindBcmService(Method getService) {
        try {
            String svcName = readHkmcServiceConstant("BCM_SERVICE", "com.hkmc.bcm");
            IBinder binder = (IBinder) getService.invoke(null, svcName);
            if (binder == null) {
                return;
            }
            Class<?> stubClass = Class.forName(BCM_STUB_CLASS);
            Object proxy = stubClass.getMethod("asInterface", IBinder.class).invoke(null, binder);
            if (proxy == null) {
                return;
            }
            Class<?> ifaceClass = Class.forName(BCM_IFACE_CLASS);
            if (!bcmScanned) {
                scanAuxiliaryInterface(OwnerService.BCM, ifaceClass);
                bcmScanned = true;
            }
            bcmProxy = proxy;
        } catch (Throwable ignored) {
        }
    }

    private static String readHkmcServiceConstant(String fieldName, String fallback) {
        try {
            Class<?> svcConsts = Class.forName(HKMC_SERVICE_CONST_CLASS);
            Object v = svcConsts.getField(fieldName).get(null);
            if (v instanceof String && !((String) v).isEmpty()) {
                return (String) v;
            }
        } catch (Throwable ignored) {
        }
        return fallback;
    }

    private void scanHscanInterface(Class<?> ifaceClass) {
        Method[] methods = ifaceClass.getDeclaredMethods();
        totalHscanMethods = methods.length;

        try {
            carSpeedMethod = ifaceClass.getMethod("getCarSpeed", Byte.TYPE);
        } catch (NoSuchMethodException ignored) {
        }
        try {
            accelPedalMethod = ifaceClass.getMethod("getAccelPdlVal");
        } catch (NoSuchMethodException ignored) {
        }
        try {
            carMovingMethod = ifaceClass.getMethod("IsCarMoving");
        } catch (NoSuchMethodException ignored) {
        }

        Map<String, CandidateMethod> classified = new LinkedHashMap<String, CandidateMethod>();
        Map<String, CandidateMethod> otherSafe = new LinkedHashMap<String, CandidateMethod>();

        for (Method m : methods) {
            int params = m.getParameterTypes().length;
            Class<?> rt = m.getReturnType();
            CandidateFilter.Role role = CandidateFilter.classify(m.getName(), params, rt);
            if (role != CandidateFilter.Role.NONE) {
                m.setAccessible(true);
                classified.put(m.getName(), new CandidateMethod(OwnerService.HSCAN, m, role));
            } else if (CandidateFilter.isSafeReadOnlyGetter(m.getName(), params, rt)) {
                m.setAccessible(true);
                otherSafe.put(m.getName(), new CandidateMethod(OwnerService.HSCAN, m, CandidateFilter.Role.NONE));
            }
        }

        for (CandidateMethod cm : sortByName(classified)) {
            if (cm.role == CandidateFilter.Role.DIRECT_STOP_LAMP) {
                directLampMethods.add(cm);
            } else if (cm.role == CandidateFilter.Role.BRAKE_PRESSURE) {
                pressureMethods.add(cm);
            } else if (cm.role == CandidateFilter.Role.DIAGNOSTIC_CANDIDATE) {
                diagnosticMethods.add(cm);
            }
        }
        otherSafeHscanMethods.addAll(sortByName(otherSafe));
    }

    private void scanAuxiliaryInterface(OwnerService owner, Class<?> ifaceClass) {
        Method[] methods = ifaceClass.getDeclaredMethods();
        Map<String, CandidateMethod> matched = new LinkedHashMap<String, CandidateMethod>();
        for (Method m : methods) {
            CandidateFilter.Role role = CandidateFilter.classify(
                    m.getName(),
                    m.getParameterTypes().length,
                    m.getReturnType());
            if (role == CandidateFilter.Role.NONE) {
                continue;
            }
            m.setAccessible(true);
            matched.put(m.getName(), new CandidateMethod(owner, m, role));
        }
        for (CandidateMethod cm : sortByName(matched)) {
            if (cm.role == CandidateFilter.Role.DIRECT_STOP_LAMP) {
                directLampMethods.add(cm);
            } else if (cm.role == CandidateFilter.Role.DIAGNOSTIC_CANDIDATE) {
                diagnosticMethods.add(cm);
            }
        }
    }

    private static List<CandidateMethod> sortByName(Map<String, CandidateMethod> map) {
        List<CandidateMethod> list = new ArrayList<CandidateMethod>(map.values());
        Collections.sort(list, new Comparator<CandidateMethod>() {
            @Override
            public int compare(CandidateMethod a, CandidateMethod b) {
                return a.qualifiedName.compareTo(b.qualifiedName);
            }
        });
        return list;
    }

    private void updateCandidateValue(
            CandidateMethod cm,
            Object rawVal,
            String newValue,
            boolean allowLog,
            Float speedKmh,
            Float brakePressure,
            Float accelPedal,
            float decelMps2,
            String activeSrc) {
        String prev = cm.lastValue;
        if (newValue.equals(prev)) {
            return;
        }

        double curNum = (rawVal instanceof Number) ? ((Number) rawVal).doubleValue() : Double.NaN;
        double prevNum = cm.lastNumeric;
        cm.lastValue = newValue;
        cm.lastNumeric = curNum;

        // First observation only seeds baseline; do not flood transition log on boot
        if ("?".equals(prev)) {
            return;
        }
        cm.changeCount++;

        if (!allowLog || cm.noisyContinuous) {
            return;
        }

        // For non-brake float getters in otherSafeHscanMethods, log zero-crossings or >= 1.0 steps (up to 40 entries)
        if (cm.role == CandidateFilter.Role.NONE && cm.floatingPoint) {
            if (cm.changeCount > 40) {
                return;
            }
            boolean zeroCross = !Double.isNaN(prevNum) && !Double.isNaN(curNum)
                    && ((Math.abs(prevNum) < 0.05 && Math.abs(curNum) >= 0.05)
                    || (Math.abs(prevNum) >= 0.05 && Math.abs(curNum) < 0.05));
            boolean step = !Double.isNaN(prevNum) && !Double.isNaN(curNum)
                    && Math.abs(curNum - prevNum) >= 1.0;
            if (!zeroCross && !step) {
                return;
            }
        }

        String ctx = String.format(
                Locale.US,
                "%s: %s -> %s (spd=%s, decel=%.2f, prsr=%s, accel=%s, src=%s)",
                cm.qualifiedName,
                prev,
                newValue,
                speedKmh == null ? "-" : String.format(Locale.US, "%.1f", speedKmh),
                decelMps2,
                brakePressure == null ? "-" : String.format(Locale.US, "%.1f", brakePressure),
                accelPedal == null ? "-" : String.format(Locale.US, "%.1f", accelPedal),
                activeSrc == null || activeSrc.isEmpty() ? "OFF" : activeSrc);
        appendLog(ctx);
    }

    private void appendLog(String msg) {
        String ts = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date());
        String line = ts + "  " + msg;
        System.out.println("brake-indicator: " + msg);
        transitionLog.add(line);
        while (transitionLog.size() > MAX_LOG_LINES) {
            transitionLog.remove(0);
        }
        logDirty = true;
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
        if (val instanceof Byte) {
            return String.valueOf(((Byte) val).intValue() & 0xFF);
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
