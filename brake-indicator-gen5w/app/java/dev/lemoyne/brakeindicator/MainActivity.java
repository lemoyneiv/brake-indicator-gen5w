package dev.lemoyne.brakeindicator;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.Drawable;
import android.graphics.drawable.GradientDrawable;
import android.graphics.drawable.StateListDrawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.TextUtils;
import android.util.DisplayMetrics;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import hmgapi.client.HmgApi;

import java.io.File;
import java.util.List;
import java.util.Locale;

/**
 * Material 3 widescreen dashboard for Brake Light Indicator v0.7.14 on Gen5W (1920x720):
 *  - Left Control Sidebar: Live brake-lamp & OBD status header card, stadium-pill Material 3
 *    controls (`Mark Log`, `Mode`, `Test (3s)`, `Export USB`, `Scan BLE`, `Reconnect`,
 *    `Clear`, `Unpair`, and `Close`), and inline action feedback right beside the driver.
 *  - Center Status Monitor Card: Full-vertical-height, high-contrast driving-legible rows
 *    (`Real OBD (22BC06)` through `RAM Dataset Buffer`) with semantic M3 tonal status pills.
 *  - Right OBD-II & Transition Log Card: `@hmgapi-ble` scanner discovery list and full-height
 *    scrollable discrete transition log.
 */
public final class MainActivity extends Activity
        implements BrakeEngine.StateListener, ObdBrakeClient.Listener {

    // Material 3 Dark automotive tonal palette
    private static final int M3_SURFACE_DIM = Color.rgb(16, 19, 24);
    private static final int M3_SURFACE_CONTAINER_LOW = Color.rgb(24, 28, 35);
    private static final int M3_SURFACE_CONTAINER = Color.rgb(30, 35, 44);
    private static final int M3_SURFACE_CONTAINER_HIGH = Color.rgb(40, 46, 57);
    private static final int M3_SURFACE_CONTAINER_HIGHEST = Color.rgb(55, 63, 78);

    private static final int M3_OUTLINE_VARIANT = Color.rgb(56, 64, 78);
    private static final int M3_OUTLINE = Color.rgb(86, 96, 112);
    private static final int M3_LOG_OUTLINE = Color.rgb(42, 49, 61);

    private static final int M3_ON_SURFACE = Color.rgb(236, 240, 247);
    private static final int M3_ON_SURFACE_VARIANT = Color.rgb(184, 194, 209);
    private static final int M3_ON_SURFACE_MUTED = Color.rgb(148, 158, 174);

    private static final int M3_PRIMARY = Color.rgb(90, 228, 204);
    private static final int M3_PRIMARY_BORDER = Color.rgb(45, 128, 115);
    private static final int M3_PRIMARY_CONTAINER = Color.rgb(20, 77, 69);
    private static final int M3_PRIMARY_CONTAINER_PRESSED = Color.rgb(30, 104, 93);
    private static final int M3_ON_PRIMARY_CONTAINER = Color.rgb(172, 248, 234);

    private static final int M3_SECONDARY_BORDER = Color.rgb(58, 88, 120);
    private static final int M3_SECONDARY_CONTAINER = Color.rgb(36, 58, 82);
    private static final int M3_SECONDARY_CONTAINER_PRESSED = Color.rgb(49, 77, 108);
    private static final int M3_ON_SECONDARY_CONTAINER = Color.rgb(208, 228, 255);

    private static final int M3_TERTIARY_CONTAINER = Color.rgb(27, 65, 74);
    private static final int M3_TERTIARY_CONTAINER_PRESSED = Color.rgb(39, 88, 99);

    private static final int M3_BRAKE_LIT_CONTAINER = Color.rgb(122, 22, 31);
    private static final int M3_BRAKE_LIT_BORDER = Color.rgb(255, 86, 96);
    private static final int M3_ON_BRAKE_LIT = Color.rgb(255, 222, 222);

    private static final int M3_WARNING_AMBER = Color.rgb(255, 186, 92);
    private static final int M3_WARNING_CONTAINER = Color.rgb(66, 48, 22);

    private static final int BADGE_NONE = 0;
    private static final int BADGE_OFF = 1;
    private static final int BADGE_LIT = 2;
    private static final int BADGE_PRIMARY = 3;

    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private BrakeEngine engine;

    // Left sidebar views & cached state
    private LinearLayout lampStatusCard;
    private GradientDrawable lampCardOffBg;
    private GradientDrawable lampCardLitBg;
    private int lastLampCardState = -1;

    private BrakeLampView previewLamp;
    private TextView statusBadge;
    private TextView obdHeaderBadge;
    private TextView configSummaryText;
    private TextView actionNoticeText;
    private Button modeToggleBtn;
    private Button scanBleBtn;
    private boolean lastScanActiveState;

    // Center full-height Status Monitor rows
    private StatusRow rowRealObd;
    private StatusRow rowHmgModel;
    private StatusRow rowSpeedDecel;
    private StatusRow rowBrakeAccel;
    private StatusRow rowRegenMode;
    private StatusRow rowSteerGearSoc;
    private StatusRow rowAccuracy;
    private StatusRow rowBuffer;

    // Right OBD-II & Transition Log views
    private TextView obdStatusLineText;
    private LinearLayout deviceListContainer;
    private TextView transitionLogText;

    private long lastRightRefreshMs;
    private int lastObservedAutoExportSeq = -1;

    private static final class StatusRow {
        final LinearLayout container;
        final TextView labelView;
        final TextView badgeView;
        final TextView valueView;
        final GradientDrawable offPillBg;
        final GradientDrawable litPillBg;
        final GradientDrawable primaryPillBg;
        int badgeState = -1;
        int valueColor = 0;

        StatusRow(
                LinearLayout container,
                TextView labelView,
                TextView badgeView,
                TextView valueView) {
            this.container = container;
            this.labelView = labelView;
            this.badgeView = badgeView;
            this.valueView = valueView;
            this.offPillBg = roundedRect(M3_SURFACE_CONTAINER_HIGH, 999f, 1, M3_OUTLINE_VARIANT);
            this.litPillBg = roundedRect(M3_BRAKE_LIT_CONTAINER, 999f, 1, M3_BRAKE_LIT_BORDER);
            this.primaryPillBg = roundedRect(M3_PRIMARY_CONTAINER, 999f, 1, M3_PRIMARY_BORDER);
        }

        void updateBadge(int state, String text) {
            if (state == BADGE_NONE) {
                if (badgeState != BADGE_NONE) {
                    badgeView.setVisibility(View.GONE);
                    badgeState = BADGE_NONE;
                }
                return;
            }
            if (badgeState == BADGE_NONE || badgeView.getVisibility() != View.VISIBLE) {
                badgeView.setVisibility(View.VISIBLE);
            }
            if (badgeState != state) {
                badgeState = state;
                if (state == BADGE_LIT) {
                    badgeView.setBackground(litPillBg);
                    badgeView.setTextColor(M3_ON_BRAKE_LIT);
                } else if (state == BADGE_PRIMARY) {
                    badgeView.setBackground(primaryPillBg);
                    badgeView.setTextColor(M3_ON_PRIMARY_CONTAINER);
                } else {
                    badgeView.setBackground(offPillBg);
                    badgeView.setTextColor(M3_ON_SURFACE_VARIANT);
                }
            }
            setTextIfChanged(badgeView, text);
        }

        void updateValue(String text, int color) {
            if (valueColor != color) {
                valueColor = color;
                valueView.setTextColor(color);
            }
            setTextIfChanged(valueView, text);
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        startService(new Intent(this, OverlayService.class));
        engine = BrakeEngine.getInstance(this);
        engine.ensureStarted();
        setContentView(buildContentView());
    }

    @Override
    protected void onResume() {
        super.onResume();
        engine.addListener(this);
        engine.getObdClient().addListener(this);
        engine.requestSettingsReload();
        refreshObdPanel();
    }

    @Override
    protected void onPause() {
        engine.removeListener(this);
        engine.getObdClient().removeListener(this);
        super.onPause();
    }

    @Override
    public void onEngineState(
            final IndicatorConfig config,
            final BrakeEvaluator.Snapshot snapshot,
            final boolean hmgConnected) {
        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                renderTelemetry(config, snapshot, hmgConnected);
            }
        });
    }

    @Override
    public void onObdStateChanged() {
        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                refreshObdPanel();
            }
        });
    }

    private View buildContentView() {
        DisplayMetrics dm = getResources().getDisplayMetrics();
        boolean widescreen = dm == null || dm.widthPixels <= 0 || dm.widthPixels >= 1600;

        int outerPadH = widescreen ? 18 : 12;
        int outerPadV = widescreen ? 14 : 10;
        int colGap = widescreen ? 14 : 10;
        int sidebarWidth = widescreen ? 332 : 252;
        int labelColWidth = widescreen ? 236 : 172;
        float labelFontPx = widescreen ? 20.0f : 16.5f;
        float badgeFontPx = widescreen ? 20.0f : 16.5f;
        float valueFontPx = widescreen ? 27.0f : 20.5f;
        float logFontPx = widescreen ? 17.0f : 14.5f;

        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.HORIZONTAL);
        root.setBackgroundColor(M3_SURFACE_DIM);
        root.setPadding(outerPadH, outerPadV, outerPadH, outerPadV);

        // ================= 1. LEFT MATERIAL 3 CONTROL SIDEBAR =================
        LinearLayout sidebar = surfaceCard(M3_SURFACE_CONTAINER_LOW, 24f, 16, 14, 16, 14);
        LinearLayout.LayoutParams sidebarLp = new LinearLayout.LayoutParams(
                sidebarWidth, ViewGroup.LayoutParams.MATCH_PARENT);
        sidebarLp.rightMargin = colGap;

        // Top live brake-lamp + OBD status tonal card
        lampCardOffBg = roundedRect(M3_SURFACE_CONTAINER_HIGH, 18f, 1, M3_OUTLINE_VARIANT);
        lampCardLitBg = roundedRect(M3_BRAKE_LIT_CONTAINER, 18f, 2, M3_BRAKE_LIT_BORDER);

        lampStatusCard = new LinearLayout(this);
        lampStatusCard.setOrientation(LinearLayout.HORIZONTAL);
        lampStatusCard.setGravity(Gravity.CENTER_VERTICAL);
        lampStatusCard.setPadding(12, 10, 12, 10);
        lampStatusCard.setBackground(lampCardOffBg);
        lastLampCardState = 0;

        previewLamp = new BrakeLampView(this);
        previewLamp.setPreviewBackdrop(true);
        LinearLayout.LayoutParams lampLp = new LinearLayout.LayoutParams(
                widescreen ? 74 : 60, widescreen ? 52 : 44);
        lampLp.rightMargin = 12;
        lampStatusCard.addView(previewLamp, lampLp);

        LinearLayout lampMeta = new LinearLayout(this);
        lampMeta.setOrientation(LinearLayout.VERTICAL);

        statusBadge = new TextView(this);
        statusBadge.setText("BRAKE OFF");
        statusBadge.setTextColor(M3_ON_SURFACE);
        statusBadge.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 19.0f : 16.5f);
        statusBadge.setTypeface(Typeface.DEFAULT_BOLD);
        statusBadge.setSingleLine(true);
        lampMeta.addView(statusBadge);

        obdHeaderBadge = new TextView(this);
        obdHeaderBadge.setText("OBD STANDBY");
        obdHeaderBadge.setTextColor(M3_ON_SURFACE_VARIANT);
        obdHeaderBadge.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 15.0f : 13.5f);
        obdHeaderBadge.setTypeface(Typeface.DEFAULT_BOLD);
        obdHeaderBadge.setSingleLine(true);
        lampMeta.addView(obdHeaderBadge);

        lampStatusCard.addView(lampMeta, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));
        sidebar.addView(lampStatusCard, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        // Primary full-width driving action: Mark Log
        Button markLogBtn = m3Button(
                "Mark Log",
                widescreen ? 18.5f : 16.0f,
                M3_PRIMARY_CONTAINER,
                M3_PRIMARY_CONTAINER_PRESSED,
                M3_ON_PRIMARY_CONTAINER,
                1,
                M3_PRIMARY_BORDER,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        String msg = engine.markDriveEvent();
                        actionNoticeText.setText("Bookmarked: " + msg);
                        transitionLogText.setText(engine.getRecorder().recentTransitionsText(35));
                    }
                });
        LinearLayout.LayoutParams markLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, widescreen ? 52 : 46);
        markLp.topMargin = 12;
        sidebar.addView(markLogBtn, markLp);

        // Secondary full-width mode selector pill
        modeToggleBtn = m3Button(
                "Mode: Real OBD \u2192 Model",
                widescreen ? 15.5f : 13.5f,
                M3_SECONDARY_CONTAINER,
                M3_SECONDARY_CONTAINER_PRESSED,
                M3_ON_SECONDARY_CONTAINER,
                1,
                M3_SECONDARY_BORDER,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        cycleObdMode();
                    }
                });
        LinearLayout.LayoutParams modeLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, widescreen ? 48 : 42);
        modeLp.topMargin = 8;
        sidebar.addView(modeToggleBtn, modeLp);

        int pairRowHeight = widescreen ? 46 : 42;
        float pairBtnFont = widescreen ? 15.5f : 13.5f;

        // 2-column row 1: Test (3s) | Export USB
        Button testBtn = m3Button(
                "Test (3s)",
                pairBtnFont,
                M3_SURFACE_CONTAINER_HIGH,
                M3_SURFACE_CONTAINER_HIGHEST,
                M3_ON_SURFACE,
                1,
                M3_OUTLINE_VARIANT,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        startService(new Intent(MainActivity.this, OverlayService.class));
                        engine.triggerManualTest(3000L);
                        actionNoticeText.setText("Illuminating top-bar indicator for 3 seconds.");
                    }
                });
        Button exportBtn = m3Button(
                "Export USB",
                pairBtnFont,
                M3_TERTIARY_CONTAINER,
                M3_TERTIARY_CONTAINER_PRESSED,
                M3_ON_PRIMARY_CONTAINER,
                1,
                M3_PRIMARY_BORDER,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        exportDatasetToUsb();
                    }
                });
        sidebar.addView(twoButtonRow(testBtn, exportBtn, pairRowHeight), rowPairLayoutParams());

        // 2-column row 2: Scan BLE | Reconnect
        scanBleBtn = m3Button(
                "Scan BLE",
                pairBtnFont,
                M3_SURFACE_CONTAINER_HIGH,
                M3_SURFACE_CONTAINER_HIGHEST,
                M3_ON_SURFACE,
                1,
                M3_OUTLINE_VARIANT,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        ObdBrakeClient obd = engine.getObdClient();
                        if (obd.isBleScanning()) {
                            obd.stopBleScan();
                        } else {
                            obd.startBleScan();
                        }
                    }
                });
        Button reconnBtn = m3Button(
                "Reconnect",
                pairBtnFont,
                M3_SURFACE_CONTAINER_HIGH,
                M3_SURFACE_CONTAINER_HIGHEST,
                M3_ON_SURFACE,
                1,
                M3_OUTLINE_VARIANT,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        engine.getRecorder().recordExternalEvent(
                                engine.getObdClient().monotonicMs(), "OBD Reconnect tapped");
                        engine.getObdClient().requestReconnect();
                        refreshObdPanel();
                        actionNoticeText.setText("Retrying 127.0.0.1 bridge / @hmgapi-ble...");
                    }
                });
        sidebar.addView(twoButtonRow(scanBleBtn, reconnBtn, pairRowHeight), rowPairLayoutParams());

        // 2-column row 3: Clear | Unpair
        Button clearBtn = m3Button(
                "Clear",
                pairBtnFont,
                M3_SURFACE_CONTAINER_HIGH,
                M3_SURFACE_CONTAINER_HIGHEST,
                M3_ON_SURFACE_VARIANT,
                1,
                M3_OUTLINE_VARIANT,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        engine.clearDataset();
                        refreshObdPanel();
                        actionNoticeText.setText("Cleared in-RAM dataset and scorecard.");
                    }
                });
        Button unpairBtn = m3Button(
                "Unpair",
                pairBtnFont,
                M3_SURFACE_CONTAINER_HIGH,
                M3_SURFACE_CONTAINER_HIGHEST,
                M3_ON_SURFACE_VARIANT,
                1,
                M3_OUTLINE_VARIANT,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        engine.getObdClient().unpairAdapter();
                        actionNoticeText.setText("Cleared paired BLE adapter (bridge still active).");
                    }
                });
        sidebar.addView(twoButtonRow(clearBtn, unpairBtn, pairRowHeight), rowPairLayoutParams());

        // Bottom-aligned feedback & app version/HMG-API caption
        LinearLayout sidebarFooterInfo = new LinearLayout(this);
        sidebarFooterInfo.setOrientation(LinearLayout.VERTICAL);
        sidebarFooterInfo.setGravity(Gravity.BOTTOM);
        sidebarFooterInfo.setPadding(4, 8, 4, 2);

        actionNoticeText = new TextView(this);
        actionNoticeText.setTextColor(M3_PRIMARY);
        actionNoticeText.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 14.5f : 13.0f);
        actionNoticeText.setLineSpacing(2.0f, 1.0f);
        actionNoticeText.setMaxLines(3);
        actionNoticeText.setEllipsize(TextUtils.TruncateAt.END);
        actionNoticeText.setText("Auto-exports CSV + summary to <usb>/BrakeIndicator/ on Park or 95% buffer.");
        sidebarFooterInfo.addView(actionNoticeText);

        configSummaryText = new TextView(this);
        configSummaryText.setTextColor(M3_ON_SURFACE_MUTED);
        configSummaryText.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 13.5f : 12.0f);
        configSummaryText.setSingleLine(true);
        configSummaryText.setEllipsize(TextUtils.TruncateAt.END);
        configSummaryText.setPadding(0, 4, 0, 0);
        configSummaryText.setText("Brake Indicator v0.7.14");
        sidebarFooterInfo.addView(configSummaryText);

        sidebar.addView(sidebarFooterInfo, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f));

        // Outlined Material 3 Close pill at bottom of sidebar
        Button closeBtn = m3Button(
                "Close",
                widescreen ? 16.5f : 14.5f,
                M3_SURFACE_CONTAINER,
                M3_SURFACE_CONTAINER_HIGHEST,
                M3_ON_SURFACE,
                1,
                M3_OUTLINE,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        finish();
                    }
                });
        LinearLayout.LayoutParams closeLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, pairRowHeight);
        closeLp.topMargin = 8;
        sidebar.addView(closeBtn, closeLp);

        root.addView(sidebar, sidebarLp);

        // ================= 2. CENTER FULL-HEIGHT STATUS MONITOR =================
        LinearLayout monitorCard = surfaceCard(M3_SURFACE_CONTAINER, 24f, 22, 8, 22, 8);
        LinearLayout.LayoutParams monitorLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 0.62f);
        monitorLp.rightMargin = colGap;

        rowRealObd = createStatusRow("Real OBD (22BC06)", "Starting...", labelColWidth, labelFontPx, badgeFontPx, valueFontPx);
        rowHmgModel = createStatusRow("HMG-API Model", "Starting...", labelColWidth, labelFontPx, badgeFontPx, valueFontPx);
        rowSpeedDecel = createStatusRow("Speed & Decel", "-", labelColWidth, labelFontPx, badgeFontPx, valueFontPx);
        rowBrakeAccel = createStatusRow("Brake & Accel", "-", labelColWidth, labelFontPx, badgeFontPx, valueFontPx);
        rowRegenMode = createStatusRow("Detected Regen", "Learning...", labelColWidth, labelFontPx, badgeFontPx, valueFontPx);
        rowSteerGearSoc = createStatusRow("Steer \u00B7 Gear \u00B7 SOC", "-", labelColWidth, labelFontPx, badgeFontPx, valueFontPx);
        rowAccuracy = createStatusRow("Model vs OBD", "Awaiting OBD samples", labelColWidth, labelFontPx, badgeFontPx, valueFontPx);
        rowBuffer = createStatusRow("RAM Dataset Buffer", "0 / " + TelemetryRecorder.CAPACITY + " rows", labelColWidth, labelFontPx, badgeFontPx, valueFontPx);

        StatusRow[] rows = {
            rowRealObd,
            rowHmgModel,
            rowSpeedDecel,
            rowBrakeAccel,
            rowRegenMode,
            rowSteerGearSoc,
            rowAccuracy,
            rowBuffer
        };
        for (int i = 0; i < rows.length; i++) {
            monitorCard.addView(rows[i].container, new LinearLayout.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f));
            if (i < rows.length - 1) {
                View divider = new View(this);
                divider.setBackgroundColor(M3_OUTLINE_VARIANT);
                monitorCard.addView(divider, new LinearLayout.LayoutParams(
                        ViewGroup.LayoutParams.MATCH_PARENT, 1));
            }
        }

        root.addView(monitorCard, monitorLp);

        // ================= 3. RIGHT FULL-HEIGHT OBD & TRANSITION LOG =================
        LinearLayout rightCard = surfaceCard(M3_SURFACE_CONTAINER_LOW, 24f, 18, 14, 18, 14);
        LinearLayout.LayoutParams rightLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 0.38f);

        TextView obdTitle = new TextView(this);
        obdTitle.setText("OBD-II & Transition Log");
        obdTitle.setTextColor(M3_PRIMARY);
        obdTitle.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 19.5f : 16.5f);
        obdTitle.setTypeface(Typeface.DEFAULT_BOLD);
        obdTitle.setSingleLine(true);
        rightCard.addView(obdTitle);

        obdStatusLineText = new TextView(this);
        obdStatusLineText.setTextColor(M3_ON_SURFACE_VARIANT);
        obdStatusLineText.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 15.0f : 13.5f);
        obdStatusLineText.setPadding(0, 4, 0, 8);
        obdStatusLineText.setMaxLines(2);
        obdStatusLineText.setEllipsize(TextUtils.TruncateAt.END);
        obdStatusLineText.setText("127.0.0.1:35000 bridge coexistence + @hmgapi-ble direct BLE");
        rightCard.addView(obdStatusLineText);

        deviceListContainer = new LinearLayout(this);
        deviceListContainer.setOrientation(LinearLayout.VERTICAL);
        rightCard.addView(deviceListContainer, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        ScrollView scroll = new ScrollView(this);
        scroll.setBackground(roundedRect(M3_SURFACE_DIM, 16f, 1, M3_LOG_OUTLINE));
        scroll.setPadding(14, 12, 14, 12);
        LinearLayout.LayoutParams scrollLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f);
        scrollLp.topMargin = 4;

        transitionLogText = new TextView(this);
        transitionLogText.setTypeface(Typeface.MONOSPACE);
        transitionLogText.setTextColor(M3_ON_SURFACE);
        transitionLogText.setTextSize(TypedValue.COMPLEX_UNIT_PX, logFontPx);
        transitionLogText.setLineSpacing(5.0f, 1.0f);
        transitionLogText.setText("No brake-lamp, regen-level, or HMG-API transitions recorded yet.");
        scroll.addView(transitionLogText);
        rightCard.addView(scroll, scrollLp);

        root.addView(rightCard, rightLp);
        return root;
    }

    private StatusRow createStatusRow(
            String label,
            String initialValue,
            int labelColWidth,
            float labelFontPx,
            float badgeFontPx,
            float valueFontPx) {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);

        TextView labelView = new TextView(this);
        labelView.setText(label);
        labelView.setTextColor(M3_ON_SURFACE_VARIANT);
        labelView.setTextSize(TypedValue.COMPLEX_UNIT_PX, labelFontPx);
        labelView.setTypeface(Typeface.DEFAULT);
        labelView.setSingleLine(true);
        labelView.setEllipsize(TextUtils.TruncateAt.END);
        LinearLayout.LayoutParams labelLp = new LinearLayout.LayoutParams(
                labelColWidth, ViewGroup.LayoutParams.WRAP_CONTENT);
        labelLp.rightMargin = 10;
        row.addView(labelView, labelLp);

        TextView badgeView = new TextView(this);
        badgeView.setTextSize(TypedValue.COMPLEX_UNIT_PX, badgeFontPx);
        badgeView.setTypeface(Typeface.DEFAULT_BOLD);
        badgeView.setPadding(14, 4, 14, 4);
        badgeView.setSingleLine(true);
        badgeView.setVisibility(View.GONE);
        LinearLayout.LayoutParams badgeLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        badgeLp.rightMargin = 12;
        row.addView(badgeView, badgeLp);

        TextView valueView = new TextView(this);
        valueView.setText(initialValue);
        valueView.setTextColor(M3_ON_SURFACE);
        valueView.setTextSize(TypedValue.COMPLEX_UNIT_PX, valueFontPx);
        valueView.setTypeface(Typeface.DEFAULT_BOLD);
        valueView.setSingleLine(true);
        valueView.setEllipsize(TextUtils.TruncateAt.END);
        row.addView(valueView, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));

        return new StatusRow(row, labelView, badgeView, valueView);
    }

    private void renderTelemetry(
            IndicatorConfig cfg,
            BrakeEvaluator.Snapshot snap,
            boolean hmgConnected) {
        if (cfg == null || snap == null) {
            return;
        }
        previewLamp.updateState(snap.illuminated, IndicatorConfig.OffState.DIM);

        int nextCardState = snap.illuminated ? 1 : 0;
        if (lastLampCardState != nextCardState) {
            lastLampCardState = nextCardState;
            lampStatusCard.setBackground(snap.illuminated ? lampCardLitBg : lampCardOffBg);
            statusBadge.setTextColor(snap.illuminated ? M3_ON_BRAKE_LIT : M3_ON_SURFACE);
        }
        setTextIfChanged(statusBadge, snap.illuminated ? "BRAKE ON" : "BRAKE OFF");

        ObdBrakeClient obdClient = engine.getObdClient();
        float hz = obdClient.getPollHz();
        if (snap.obdFresh) {
            setTextIfChanged(obdHeaderBadge, String.format(Locale.US, "OBD %s \u00B7 %.1f Hz",
                    snap.obd.transport, Float.valueOf(hz)));
            obdHeaderBadge.setTextColor(snap.illuminated ? M3_ON_BRAKE_LIT : M3_PRIMARY);
        } else {
            setTextIfChanged(obdHeaderBadge,
                    cfg.obdMode == IndicatorConfig.ObdMode.DISABLED ? "OBD DISABLED" : "OBD STANDBY");
            obdHeaderBadge.setTextColor(snap.illuminated ? M3_ON_BRAKE_LIT : M3_ON_SURFACE_VARIANT);
        }

        setTextIfChanged(configSummaryText,
                "v0.7.14 \u00B7 " + cfg.iconSizePx + "px \u00B7 HMG-API: " + engine.getHmgStatusSummary());
        setTextIfChanged(modeToggleBtn, sidebarModeLabel(cfg.obdMode));

        // 1. Real OBD (770 / 22BC06)
        if (snap.obdFresh) {
            if (snap.obd.brakeLampLit) {
                rowRealObd.updateBadge(BADGE_LIT, snap.obd.pedalBit ? "ON \u00B7 PEDAL" : "ON \u00B7 REGEN");
            } else {
                rowRealObd.updateBadge(BADGE_OFF, "OFF");
            }
            rowRealObd.updateValue(
                    String.format(Locale.US, "raw %d  \u00B7  %s  \u00B7  %d ms",
                            Integer.valueOf(snap.obd.rawCode),
                            snap.obd.transport,
                            Long.valueOf(snap.obd.ageMsAt(snap.timestampMs))),
                    M3_ON_SURFACE);
        } else {
            rowRealObd.updateBadge(
                    BADGE_OFF,
                    cfg.obdMode == IndicatorConfig.ObdMode.DISABLED ? "DISABLED" : "STANDBY");
            rowRealObd.updateValue(obdClient.getStatusSummary(), M3_ON_SURFACE_VARIANT);
        }

        // 2. HMG-API Model & Top-Bar Icon Trigger
        if (snap.modelIlluminated) {
            rowHmgModel.updateBadge(BADGE_LIT, "ON");
            rowHmgModel.updateValue(
                    snap.modelSource + "   \u00B7   Icon: " + snap.source,
                    M3_ON_SURFACE);
        } else {
            rowHmgModel.updateBadge(BADGE_OFF, "OFF");
            rowHmgModel.updateValue("Icon: " + snap.source, M3_ON_SURFACE);
        }

        // 3. Speed / Decel (-dv/dt)
        rowSpeedDecel.updateBadge(BADGE_NONE, "");
        rowSpeedDecel.updateValue(
                fmtObjFloat(snap.speedKmh, "%.1f") + " km/h   \u00B7   "
                        + String.format(Locale.US, "%.2f m/s\u00B2", Float.valueOf(snap.decelMps2)),
                M3_ON_SURFACE);

        // 4. Brake Prsr / Accel
        rowBrakeAccel.updateBadge(BADGE_NONE, "");
        rowBrakeAccel.updateValue(
                "Prsr " + fmtObjFloat(snap.brakePressure, "%.1f") + " bar   \u00B7   Accel "
                        + fmtObjFloat(snap.accelPedal, "%.1f") + " %",
                M3_ON_SURFACE);

        // 5. Detected Regen Mode
        boolean regenLearned = snap.inferredRegenLevel >= 0 || snap.sccActive || snap.autoRegenActive;
        if (regenLearned) {
            rowRegenMode.updateBadge(
                    BADGE_PRIMARY,
                    BrakeEvaluator.regenLevelLabel(snap.inferredRegenLevel, snap.sccActive, snap.autoRegenActive));
            String modeDesc = snap.sccActive ? "Smart Cruise Control"
                    : (snap.autoRegenActive ? "Smart Recuperation"
                    : (snap.inferredRegenLevel == BrakeEvaluator.REGEN_LEVEL_IPEDAL
                            ? "One-Pedal Driving" : "Manual Paddle"));
            rowRegenMode.updateValue(modeDesc, M3_ON_SURFACE_VARIANT);
        } else {
            rowRegenMode.updateBadge(BADGE_OFF, "LEARNING");
            rowRegenMode.updateValue("Coast \u2265 21 km/h off pedals", M3_ON_SURFACE_VARIANT);
        }

        // 6. Steer / Park / HV SOC
        rowSteerGearSoc.updateBadge(BADGE_NONE, "");
        rowSteerGearSoc.updateValue(
                fmtObjFloat(snap.steeringAngleDeg, "%+.1f") + "\u00B0   \u00B7   P="
                        + fmtFlag(snap.gearPark) + "  EPB=" + fmtFlag(snap.parkingBrake)
                        + "   \u00B7   SOC " + fmtObjFloat(snap.hvSocPct, "%.1f") + "%",
                M3_ON_SURFACE);

        // 7. Model vs OBD Accuracy
        TelemetryRecorder.ScoreSnapshot sc = engine.getRecorder().scoreSnapshot();
        rowAccuracy.updateBadge(BADGE_NONE, "");
        if (sc.scoredSamples > 0L) {
            rowAccuracy.updateValue(
                    String.format(Locale.US, "%.1f%% agree  \u00B7  Regen %s  \u00B7  Lag %s",
                            Float.valueOf(sc.agreementPct()),
                            Float.isNaN(sc.regenRecallPct()) ? "n/a"
                                    : String.format(Locale.US, "%.1f%%", Float.valueOf(sc.regenRecallPct())),
                            sc.onsetCount > 0 ? (sc.meanOnsetLagMs + " ms") : "n/a"),
                    M3_ON_SURFACE);
        } else {
            rowAccuracy.updateValue("Awaiting live OBD 22BC06 samples", M3_ON_SURFACE_VARIANT);
        }

        // 8. RAM Dataset Buffer & Auto-Export Status
        AutoExportCoordinator.StatusSnapshot autoSnap = engine.getAutoExportStatus();
        if (engine.isExportInFlight()) {
            rowBuffer.updateBadge(BADGE_PRIMARY, "WRITING USB");
        } else if (!cfg.autoExportUsb) {
            rowBuffer.updateBadge(BADGE_OFF, "MANUAL");
        } else if (autoSnap.waitingForUsb) {
            rowBuffer.updateBadge(BADGE_OFF, "WAIT USB");
        } else if (autoSnap.autoExportCount > 0) {
            rowBuffer.updateBadge(BADGE_PRIMARY, "AUTO \u00B7 " + autoSnap.autoExportCount);
        } else {
            rowBuffer.updateBadge(BADGE_PRIMARY, "AUTO USB");
        }

        String shortSaved = autoSnap.shortLastSavedLabel();
        if (autoSnap.waitingForUsb && autoSnap.waitingReason != null) {
            rowBuffer.updateValue(
                    String.format(Locale.US, "%,d / %,d rows  \u00B7  Insert USB (%s)",
                            Integer.valueOf(sc.bufferedRows),
                            Integer.valueOf(TelemetryRecorder.CAPACITY),
                            autoSnap.waitingReason),
                    M3_WARNING_AMBER);
        } else if (shortSaved != null) {
            rowBuffer.updateValue(
                    String.format(Locale.US, "%,d / %,d rows  \u00B7  Saved %s",
                            Integer.valueOf(sc.bufferedRows),
                            Integer.valueOf(TelemetryRecorder.CAPACITY),
                            shortSaved),
                    M3_ON_SURFACE);
        } else {
            rowBuffer.updateValue(
                    String.format(Locale.US, "%,d / %,d rows  (%,d scored)",
                            Integer.valueOf(sc.bufferedRows),
                            Integer.valueOf(TelemetryRecorder.CAPACITY),
                            Long.valueOf(sc.scoredSamples)),
                    M3_ON_SURFACE);
        }

        if (lastObservedAutoExportSeq < 0) {
            lastObservedAutoExportSeq = autoSnap.exportSeq;
            if (autoSnap.lastNotice != null) {
                setTextIfChanged(actionNoticeText, autoSnap.lastNotice);
            }
        } else if (autoSnap.exportSeq != lastObservedAutoExportSeq) {
            lastObservedAutoExportSeq = autoSnap.exportSeq;
            if (autoSnap.lastTrigger != AutoExportCoordinator.Trigger.MANUAL
                    && autoSnap.lastNotice != null) {
                setTextIfChanged(actionNoticeText, autoSnap.lastNotice);
            }
        }

        long now = System.currentTimeMillis();
        if (now - lastRightRefreshMs >= 450L) {
            lastRightRefreshMs = now;
            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
        }
    }

    private void refreshObdPanel() {
        ObdBrakeClient obd = engine.getObdClient();
        boolean scanning = obd.isBleScanning();
        setTextIfChanged(scanBleBtn, scanning ? "Stop Scan" : "Scan BLE");
        if (scanning != lastScanActiveState) {
            lastScanActiveState = scanning;
            if (scanning) {
                scanBleBtn.setBackground(m3PillDrawable(
                        M3_PRIMARY_CONTAINER, M3_PRIMARY_CONTAINER_PRESSED, 1, M3_PRIMARY));
                scanBleBtn.setTextColor(M3_ON_PRIMARY_CONTAINER);
            } else {
                scanBleBtn.setBackground(m3PillDrawable(
                        M3_SURFACE_CONTAINER_HIGH, M3_SURFACE_CONTAINER_HIGHEST, 1, M3_OUTLINE_VARIANT));
                scanBleBtn.setTextColor(M3_ON_SURFACE);
            }
        }

        String pairedMac = obd.getPairedMac();
        String pairedInfo = pairedMac.isEmpty()
                ? "Paired BLE: none"
                : ("Paired: " + obd.getPairedName() + " [" + pairedMac + "]");
        setTextIfChanged(obdStatusLineText, pairedInfo + "  \u00B7  " + obd.getStatusSummary());

        deviceListContainer.removeAllViews();
        List<ObdBrakeClient.DiscoveredDevice> devs = obd.getDiscoveredDevices();
        int shown = 0;
        for (int i = 0; i < devs.size() && shown < 6; i++) {
            final ObdBrakeClient.DiscoveredDevice d = devs.get(i);
            shown++;
            String prefix = d.likelyBvm02 ? "[BVM02 - Ignored] " : (d.likelyObd ? "[OBD] " : "");
            String label = prefix + d.name + "  (" + d.mac + ", " + d.rssi + " dBm)";
            int fill = d.likelyObd ? M3_PRIMARY_CONTAINER
                    : (d.likelyBvm02 ? M3_WARNING_CONTAINER : M3_SURFACE_CONTAINER_HIGH);
            int pressed = d.likelyObd ? M3_PRIMARY_CONTAINER_PRESSED : M3_SURFACE_CONTAINER_HIGHEST;
            int textCol = d.likelyObd ? M3_ON_PRIMARY_CONTAINER
                    : (d.likelyBvm02 ? M3_WARNING_AMBER : M3_ON_SURFACE);
            int strokeCol = d.likelyObd ? M3_PRIMARY
                    : (d.likelyBvm02 ? M3_WARNING_AMBER : M3_OUTLINE_VARIANT);

            Button rowBtn = m3Button(label, 15.5f, fill, pressed, textCol, 1, strokeCol,
                    new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            if (d.likelyBvm02) {
                                actionNoticeText.setText("Refused BVM02 battery sensor (hardware safety guard).");
                                return;
                            }
                            boolean ok = engine.getObdClient().pairAdapter(d.mac, d.name, d.addrType);
                            if (ok) {
                                actionNoticeText.setText("Paired BLE OBD adapter " + d.name + " (" + d.mac + ").");
                            }
                        }
                    });
            rowBtn.setGravity(Gravity.LEFT | Gravity.CENTER_VERTICAL);
            rowBtn.setPadding(16, 8, 16, 8);
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
            lp.bottomMargin = 6;
            deviceListContainer.addView(rowBtn, lp);
        }

        setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
    }

    private void cycleObdMode() {
        Thread worker = new Thread("BrakeModeCycle") {
            @Override
            public void run() {
                final IndicatorConfig.ObdMode next = engine.cycleObdModeAndSave();
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        setTextIfChanged(modeToggleBtn, sidebarModeLabel(next));
                        actionNoticeText.setText("Indicator mode: " + next.shortLabel());
                    }
                });
            }
        };
        worker.setDaemon(true);
        worker.start();
    }

    private void exportDatasetToUsb() {
        actionNoticeText.setText("Exporting CSV + calibration summary to USB...");
        Thread worker = new Thread("BrakeUsbExport") {
            @Override
            public void run() {
                final UsbExporter.ExportResult res = engine.exportDatasetToUsb();
                if (res.ok && res.csvPath != null) {
                    try {
                        HmgApi.toast(MainActivity.this, "Exported " + new File(res.csvPath).getName());
                    } catch (Exception ignored) {}
                }
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        if (res.ok) {
                            actionNoticeText.setText("Saved & cleared buffer: " + res.csvPath);
                            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                        } else {
                            actionNoticeText.setText(res.errorMessage);
                        }
                    }
                });
            }
        };
        worker.setDaemon(true);
        worker.start();
    }

    private static String sidebarModeLabel(IndicatorConfig.ObdMode mode) {
        if (mode == IndicatorConfig.ObdMode.MODEL_ONLY_ICON) {
            return "Mode: Model Icon (Score OBD)";
        }
        if (mode == IndicatorConfig.ObdMode.DISABLED) {
            return "Mode: HMG-API Only (No OBD)";
        }
        return "Mode: Real OBD \u2192 Model";
    }

    private LinearLayout surfaceCard(int bgColor, float radiusPx, int padL, int padT, int padR, int padB) {
        LinearLayout card = new LinearLayout(this);
        card.setOrientation(LinearLayout.VERTICAL);
        card.setPadding(padL, padT, padR, padB);
        card.setBackground(roundedRect(bgColor, radiusPx, 1, M3_OUTLINE_VARIANT));
        return card;
    }

    private LinearLayout twoButtonRow(Button left, Button right, int heightPx) {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        LinearLayout.LayoutParams lpLeft = new LinearLayout.LayoutParams(0, heightPx, 1.0f);
        lpLeft.rightMargin = 8;
        LinearLayout.LayoutParams lpRight = new LinearLayout.LayoutParams(0, heightPx, 1.0f);
        row.addView(left, lpLeft);
        row.addView(right, lpRight);
        return row;
    }

    private static LinearLayout.LayoutParams rowPairLayoutParams() {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        lp.topMargin = 8;
        return lp;
    }

    private Button m3Button(
            String label,
            float textSizePx,
            int normalFill,
            int pressedFill,
            int textColor,
            int strokeWidth,
            int strokeColor,
            View.OnClickListener listener) {
        Button b = new Button(this);
        b.setText(label);
        b.setAllCaps(false);
        b.setSingleLine(true);
        b.setEllipsize(TextUtils.TruncateAt.END);
        b.setGravity(Gravity.CENTER);
        b.setMinHeight(0);
        b.setMinimumHeight(0);
        b.setMinWidth(0);
        b.setMinimumWidth(0);
        b.setPadding(12, 0, 12, 0);
        b.setTextColor(textColor);
        b.setTextSize(TypedValue.COMPLEX_UNIT_PX, textSizePx);
        b.setTypeface(Typeface.DEFAULT_BOLD);
        b.setBackground(m3PillDrawable(normalFill, pressedFill, strokeWidth, strokeColor));
        b.setOnClickListener(listener);
        return b;
    }

    private static Drawable m3PillDrawable(int normalFill, int pressedFill, int strokeWidth, int strokeColor) {
        StateListDrawable sld = new StateListDrawable();
        sld.addState(
                new int[]{android.R.attr.state_pressed},
                roundedRect(pressedFill, 999f, strokeWidth, strokeColor));
        sld.addState(
                new int[]{android.R.attr.state_focused},
                roundedRect(pressedFill, 999f, strokeWidth, strokeColor));
        sld.addState(
                new int[0],
                roundedRect(normalFill, 999f, strokeWidth, strokeColor));
        return sld;
    }

    private static GradientDrawable roundedRect(int fill, float radiusPx, int strokeWidth, int strokeColor) {
        GradientDrawable d = new GradientDrawable();
        d.setColor(fill);
        d.setCornerRadius(radiusPx);
        if (strokeWidth > 0) {
            d.setStroke(strokeWidth, strokeColor);
        }
        return d;
    }

    private static void setTextIfChanged(TextView tv, String next) {
        String safe = next == null ? "" : next;
        CharSequence cur = tv.getText();
        if (cur == null || !safe.contentEquals(cur)) {
            tv.setText(safe);
        }
    }

    private static String fmtObjFloat(Float v, String fmt) {
        return v == null ? "-" : String.format(Locale.US, fmt, v);
    }

    private static String fmtFlag(Boolean b) {
        if (b == null) return "-";
        return b.booleanValue() ? "1" : "0";
    }
}
