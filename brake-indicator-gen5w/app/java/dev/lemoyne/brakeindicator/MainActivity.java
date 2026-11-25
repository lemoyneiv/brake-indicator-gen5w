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
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import hmgapi.client.HmgApi;

import java.io.File;
import java.util.List;
import java.util.Locale;

/**
 * Material 3 widescreen dashboard for Brake Light Indicator v0.10.4 on Gen5W (1920x720):
 *  - Left Control Sidebar: Live brake-lamp & OBD status header card, stadium-pill Material 3
 *    controls (`Mark Log`, `Mode`, `Test (3s)`, `Export USB`, `Upload Cloud`, `Test Cloud`,
 *    `Update Model`, `Clear`, `BLE & Log`, and `Close`), and inline action feedback.
 *  - Center Regen Ground-Truth Training Card (driver-side): Large, zero-scroll Material 3 buttons
 *    (`i-Pedal`, `L3`, `L2`, `L1`, `L0`, `Auto Regen`, `SCC Cruise`, `Unset`, and compact
 *    `Auto-Hold: ON / OFF / ?` pills) with fresh (`<= 90s` focused) vs carry-forward visual states
 *    and live `Driver vs Model` match feedback.
 *  - Right Full-Height Slot: Full-vertical-height driving-legible Status Monitor Card by default,
 *    plus a hidden toggleable OBD-II BLE & Transition Log panel (`Scan BLE`, `Reconnect`, `Unpair`,
 *    discovered adapters, and scrollable transition log).
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

    private static final int GT_STYLE_INACTIVE = 0;
    private static final int GT_STYLE_CONFIRMED = 1;
    private static final int GT_STYLE_CARRY_FWD = 2;
    private static final int GT_STYLE_SUBTLE_ACTIVE = 3;

    private static final int GT_BANNER_UNSET = 0;
    private static final int GT_BANNER_CONFIRMED_OK = 1;
    private static final int GT_BANNER_CONFIRMED_MISMATCH = 2;
    private static final int GT_BANNER_CARRY_FWD = 3;

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
    private Button toggleBleLogBtn;
    private int lastBleToggleVisualState = -1;

    // Center Ground-Truth Training Card views & cached drawables
    private LinearLayout gtBannerCard;
    private TextView gtBannerTitle;
    private TextView gtBannerSub;
    private GradientDrawable gtBannerUnsetBg;
    private GradientDrawable gtBannerOkBg;
    private GradientDrawable gtBannerMismatchBg;
    private GradientDrawable gtBannerCarryBg;
    private int lastGtBannerState = -1;

    private GtToggleButton gtBtnIpedal;
    private GtToggleButton gtBtnL3;
    private GtToggleButton gtBtnL2;
    private GtToggleButton gtBtnL1;
    private GtToggleButton gtBtnL0;
    private GtToggleButton gtBtnAuto;
    private GtToggleButton gtBtnScc;
    private GtToggleButton gtBtnAhOn;
    private GtToggleButton gtBtnAhOff;
    private GtToggleButton gtBtnAhUnset;

    // Right slot: Status Monitor Card (default) + Hidden OBD-II BLE & Log Card
    private LinearLayout monitorCard;
    private LinearLayout bleLogCard;
    private boolean bleLogPanelVisible;

    private StatusRow rowRealObd;
    private StatusRow rowHmgModel;
    private StatusRow rowSpeedDecel;
    private StatusRow rowBrakeAccel;
    private StatusRow rowRegenMode;
    private StatusRow rowSteerGearSoc;
    private StatusRow rowAccuracy;
    private StatusRow rowBuffer;

    // Hidden OBD-II BLE & Transition Log views
    private Button scanBleBtn;
    private boolean lastScanActiveState;
    private TextView obdStatusLineText;
    private LinearLayout deviceListContainer;
    private TextView transitionLogText;

    private long lastRightRefreshMs;
    private int lastObservedAutoExportSeq = -1;

    private static final class GtToggleButton {
        final Button button;
        final Drawable inactiveBg;
        final Drawable confirmedBg;
        final Drawable carryBg;
        final Drawable subtleActiveBg;
        int currentStyle = -1;

        GtToggleButton(Button button, float cornerRadiusPx) {
            this.button = button;
            this.inactiveBg = m3RoundedStateDrawable(
                    M3_SURFACE_CONTAINER_HIGH, M3_SURFACE_CONTAINER_HIGHEST, cornerRadiusPx, 1, M3_OUTLINE_VARIANT);
            this.confirmedBg = m3RoundedStateDrawable(
                    M3_PRIMARY_CONTAINER, M3_PRIMARY_CONTAINER_PRESSED, cornerRadiusPx, 3, M3_PRIMARY);
            this.carryBg = m3RoundedStateDrawable(
                    M3_WARNING_CONTAINER, M3_SURFACE_CONTAINER_HIGHEST, cornerRadiusPx, 2, M3_WARNING_AMBER);
            this.subtleActiveBg = m3RoundedStateDrawable(
                    M3_SECONDARY_CONTAINER, M3_SECONDARY_CONTAINER_PRESSED, cornerRadiusPx, 1, M3_SECONDARY_BORDER);
            applyStyle(GT_STYLE_INACTIVE);
        }

        void applyStyle(int style) {
            if (currentStyle == style) {
                return;
            }
            currentStyle = style;
            if (style == GT_STYLE_CONFIRMED) {
                button.setBackground(confirmedBg);
                button.setTextColor(M3_ON_PRIMARY_CONTAINER);
            } else if (style == GT_STYLE_CARRY_FWD) {
                button.setBackground(carryBg);
                button.setTextColor(M3_WARNING_AMBER);
            } else if (style == GT_STYLE_SUBTLE_ACTIVE) {
                button.setBackground(subtleActiveBg);
                button.setTextColor(M3_ON_SECONDARY_CONTAINER);
            } else {
                button.setBackground(inactiveBg);
                button.setTextColor(M3_ON_SURFACE);
            }
        }

        void setLabel(String text) {
            setTextIfChanged(button, text);
        }
    }

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
        engine.setUiFocused(true);
        engine.addListener(this);
        engine.getObdClient().addListener(this);
        engine.requestSettingsReload();
        refreshGroundTruthControls(engine.getLatestSnapshot());
        refreshObdPanel();
    }

    @Override
    protected void onPause() {
        engine.setUiFocused(false);
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
                widescreen ? 18.0f : 16.0f,
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
                        if (bleLogPanelVisible) {
                            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                        }
                    }
                });
        LinearLayout.LayoutParams markLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, widescreen ? 48 : 42);
        markLp.topMargin = 10;
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
                ViewGroup.LayoutParams.MATCH_PARENT, widescreen ? 42 : 38);
        modeLp.topMargin = 6;
        sidebar.addView(modeToggleBtn, modeLp);

        int pairRowHeight = widescreen ? 41 : 37;
        float pairBtnFont = widescreen ? 15.0f : 13.0f;

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

        // 2-column row 2: Upload Cloud | Test Cloud
        Button uploadCloudBtn = m3Button(
                "Upload Cloud",
                pairBtnFont,
                M3_PRIMARY_CONTAINER,
                M3_PRIMARY_CONTAINER_PRESSED,
                M3_ON_PRIMARY_CONTAINER,
                1,
                M3_PRIMARY_BORDER,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        uploadDatasetToCloud();
                    }
                });
        Button testCloudBtn = m3Button(
                "Test Cloud",
                pairBtnFont,
                M3_SECONDARY_CONTAINER,
                M3_SECONDARY_CONTAINER_PRESSED,
                M3_ON_SECONDARY_CONTAINER,
                1,
                M3_SECONDARY_BORDER,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        testCloudConnectivity();
                    }
                });
        sidebar.addView(twoButtonRow(uploadCloudBtn, testCloudBtn, pairRowHeight), rowPairLayoutParams());

        // Full-width OTA Brake Model action: tap to fetch signed OTA model (or USB .jar), long-press to revert to built-in
        Button updateModelBtn = m3Button(
                "Update Model",
                widescreen ? 15.5f : 13.5f,
                M3_TERTIARY_CONTAINER,
                M3_TERTIARY_CONTAINER_PRESSED,
                M3_ON_PRIMARY_CONTAINER,
                1,
                M3_PRIMARY_BORDER,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        updateBrakeModelNow();
                    }
                });
        updateModelBtn.setOnLongClickListener(new View.OnLongClickListener() {
            @Override
            public boolean onLongClick(View v) {
                resetBrakeModelToBuiltIn();
                return true;
            }
        });
        LinearLayout.LayoutParams updateModelLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, widescreen ? 42 : 38);
        updateModelLp.topMargin = 6;
        sidebar.addView(updateModelBtn, updateModelLp);

        // 2-column row 3: Clear | BLE & Log (toggles hidden OBD-II BLE & Transition Log panel)
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
        toggleBleLogBtn = m3Button(
                "BLE & Log",
                pairBtnFont,
                M3_SURFACE_CONTAINER_HIGH,
                M3_SURFACE_CONTAINER_HIGHEST,
                M3_ON_SURFACE,
                1,
                M3_OUTLINE_VARIANT,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        setBleLogPanelVisible(!bleLogPanelVisible);
                    }
                });
        sidebar.addView(twoButtonRow(clearBtn, toggleBleLogBtn, pairRowHeight), rowPairLayoutParams());

        // Bottom-aligned feedback & app version/HMG-API caption
        LinearLayout sidebarFooterInfo = new LinearLayout(this);
        sidebarFooterInfo.setOrientation(LinearLayout.VERTICAL);
        sidebarFooterInfo.setGravity(Gravity.BOTTOM);
        sidebarFooterInfo.setPadding(4, 6, 4, 2);

        actionNoticeText = new TextView(this);
        actionNoticeText.setTextColor(M3_PRIMARY);
        actionNoticeText.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 14.0f : 12.5f);
        actionNoticeText.setLineSpacing(2.0f, 1.0f);
        actionNoticeText.setMaxLines(4);
        actionNoticeText.setEllipsize(TextUtils.TruncateAt.END);
        CloudUploadConfig initialCloud = engine.getCloudUploadConfig();
        if (initialCloud.hasCredentials()) {
            actionNoticeText.setText("Cloud ready (" + initialCloud.repo
                    + ") \u2014 Park auto-uploads log & checks OTA model.");
        } else {
            actionNoticeText.setText("Tap a regen button to log ground truth; Park auto-checks OTA model.");
        }
        sidebarFooterInfo.addView(actionNoticeText);

        configSummaryText = new TextView(this);
        configSummaryText.setTextColor(M3_ON_SURFACE_MUTED);
        configSummaryText.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 13.0f : 11.5f);
        configSummaryText.setSingleLine(true);
        configSummaryText.setEllipsize(TextUtils.TruncateAt.END);
        configSummaryText.setPadding(0, 4, 0, 0);
        configSummaryText.setText("v0.10.4 \u00B7 Model " + engine.getActiveModelBadge());
        sidebarFooterInfo.addView(configSummaryText);

        sidebar.addView(sidebarFooterInfo, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f));

        Button closeBtn = m3Button(
                "Close",
                widescreen ? 16.0f : 14.0f,
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
        closeLp.topMargin = 6;
        sidebar.addView(closeBtn, closeLp);

        root.addView(sidebar, sidebarLp);

        // ================= 2. CENTER REGEN GROUND-TRUTH TRAINING CARD =================
        LinearLayout gtCard = surfaceCard(M3_SURFACE_CONTAINER_LOW, 24f, 18, 14, 18, 14);
        LinearLayout.LayoutParams gtCardLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 0.38f);
        gtCardLp.rightMargin = colGap;

        LinearLayout gtHeaderRow = new LinearLayout(this);
        gtHeaderRow.setOrientation(LinearLayout.HORIZONTAL);
        gtHeaderRow.setGravity(Gravity.CENTER_VERTICAL);

        TextView gtTitle = new TextView(this);
        gtTitle.setText("Regen Ground Truth");
        gtTitle.setTextColor(M3_PRIMARY);
        gtTitle.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 20.5f : 17.0f);
        gtTitle.setTypeface(Typeface.DEFAULT_BOLD);
        gtTitle.setSingleLine(true);
        gtTitle.setEllipsize(TextUtils.TruncateAt.END);
        gtHeaderRow.addView(gtTitle, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));

        Button unsetGtBtn = m3Button(
                "Unset",
                widescreen ? 15.0f : 13.0f,
                M3_SURFACE_CONTAINER_HIGH,
                M3_SURFACE_CONTAINER_HIGHEST,
                M3_ON_SURFACE_VARIANT,
                1,
                M3_OUTLINE_VARIANT,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        engine.setUiFocused(true);
                        engine.clearDriverGroundTruth();
                        actionNoticeText.setText("Cleared driver ground-truth regen label.");
                        refreshGroundTruthControls(engine.getLatestSnapshot());
                        if (bleLogPanelVisible) {
                            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                        }
                    }
                });
        LinearLayout.LayoutParams unsetLp = new LinearLayout.LayoutParams(
                widescreen ? 96 : 78, widescreen ? 36 : 32);
        unsetLp.leftMargin = 8;
        gtHeaderRow.addView(unsetGtBtn, unsetLp);
        gtCard.addView(gtHeaderRow, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        // Live Tonal Ground-Truth Status & Match Banner
        gtBannerUnsetBg = roundedRect(M3_SURFACE_CONTAINER, 16f, 1, M3_OUTLINE_VARIANT);
        gtBannerOkBg = roundedRect(M3_PRIMARY_CONTAINER, 16f, 2, M3_PRIMARY);
        gtBannerMismatchBg = roundedRect(M3_BRAKE_LIT_CONTAINER, 16f, 2, M3_BRAKE_LIT_BORDER);
        gtBannerCarryBg = roundedRect(M3_WARNING_CONTAINER, 16f, 2, M3_WARNING_AMBER);

        gtBannerCard = new LinearLayout(this);
        gtBannerCard.setOrientation(LinearLayout.VERTICAL);
        gtBannerCard.setPadding(14, 8, 14, 8);
        gtBannerCard.setBackground(gtBannerUnsetBg);
        lastGtBannerState = GT_BANNER_UNSET;

        gtBannerTitle = new TextView(this);
        gtBannerTitle.setText("UNSET \u00B7 Unlabeled");
        gtBannerTitle.setTextColor(M3_ON_SURFACE_VARIANT);
        gtBannerTitle.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 18.5f : 15.5f);
        gtBannerTitle.setTypeface(Typeface.DEFAULT_BOLD);
        gtBannerTitle.setSingleLine(true);
        gtBannerTitle.setEllipsize(TextUtils.TruncateAt.END);
        gtBannerCard.addView(gtBannerTitle);

        gtBannerSub = new TextView(this);
        gtBannerSub.setText("Tap active regen mode whenever convenient");
        gtBannerSub.setTextColor(M3_ON_SURFACE_MUTED);
        gtBannerSub.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 14.5f : 12.5f);
        gtBannerSub.setSingleLine(true);
        gtBannerSub.setEllipsize(TextUtils.TruncateAt.END);
        gtBannerSub.setPadding(0, 2, 0, 0);
        gtBannerCard.addView(gtBannerSub);

        LinearLayout.LayoutParams bannerLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        bannerLp.topMargin = 8;
        gtCard.addView(gtBannerCard, bannerLp);

        float heroModeFont = widescreen ? 25.0f : 19.5f;
        float paddleModeFont = widescreen ? 26.0f : 20.0f;
        float modifierFont = widescreen ? 19.5f : 15.5f;
        float gtTileRadius = widescreen ? 20f : 16f;
        int gtRowGap = widescreen ? 8 : 6;

        // Hero One-Pedal button: i-Pedal
        gtBtnIpedal = createGtButton(
                "i-Pedal  \u00B7  One-Pedal",
                heroModeFont,
                gtTileRadius,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        onConfirmRegenLevelTapped(BrakeEvaluator.REGEN_LEVEL_IPEDAL);
                    }
                });
        LinearLayout.LayoutParams ipedalLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.05f);
        ipedalLp.topMargin = gtRowGap + 2;
        gtCard.addView(gtBtnIpedal.button, ipedalLp);

        // Paddle Row 1 (stronger left -> weaker right, matching E-GMP steering paddles): L3 | L2
        gtBtnL3 = createGtButton("L3", paddleModeFont, gtTileRadius, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onConfirmRegenLevelTapped(BrakeEvaluator.REGEN_LEVEL_3);
            }
        });
        gtBtnL2 = createGtButton("L2", paddleModeFont, gtTileRadius, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onConfirmRegenLevelTapped(BrakeEvaluator.REGEN_LEVEL_2);
            }
        });
        gtCard.addView(weightedTwoButtonRow(gtBtnL3.button, gtBtnL2.button, gtRowGap),
                weightedRowParams(1.15f, gtRowGap));

        // Paddle Row 2: L1 | L0
        gtBtnL1 = createGtButton("L1", paddleModeFont, gtTileRadius, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onConfirmRegenLevelTapped(BrakeEvaluator.REGEN_LEVEL_1);
            }
        });
        gtBtnL0 = createGtButton("L0", paddleModeFont, gtTileRadius, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onConfirmRegenLevelTapped(BrakeEvaluator.REGEN_LEVEL_0);
            }
        });
        gtCard.addView(weightedTwoButtonRow(gtBtnL1.button, gtBtnL0.button, gtRowGap),
                weightedRowParams(1.15f, gtRowGap));

        // Dynamic Modifier Row: Auto Regen | SCC Cruise
        gtBtnAuto = createGtButton("Auto Regen", modifierFont, gtTileRadius, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                engine.setUiFocused(true);
                TelemetryRecorder.DriverGroundTruth gt = engine.toggleDriverAutoRegen();
                actionNoticeText.setText("Driver GT: " + gt.formatRegenLabel());
                refreshGroundTruthControls(engine.getLatestSnapshot());
                if (bleLogPanelVisible) {
                    setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                }
            }
        });
        gtBtnScc = createGtButton("SCC Cruise", modifierFont, gtTileRadius, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                engine.setUiFocused(true);
                TelemetryRecorder.DriverGroundTruth gt = engine.toggleDriverScc();
                actionNoticeText.setText("Driver GT: " + gt.formatRegenLabel());
                refreshGroundTruthControls(engine.getLatestSnapshot());
                if (bleLogPanelVisible) {
                    setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                }
            }
        });
        gtCard.addView(weightedTwoButtonRow(gtBtnAuto.button, gtBtnScc.button, gtRowGap),
                weightedRowParams(0.85f, gtRowGap));

        // Compact Auto-Hold Ground-Truth Bar: Auto-Hold  [ ON ] [ OFF ] [ ? ]
        LinearLayout autoHoldBar = new LinearLayout(this);
        autoHoldBar.setOrientation(LinearLayout.HORIZONTAL);
        autoHoldBar.setGravity(Gravity.CENTER_VERTICAL);
        autoHoldBar.setPadding(12, 5, 8, 5);
        autoHoldBar.setBackground(roundedRect(M3_SURFACE_CONTAINER, 16f, 1, M3_OUTLINE_VARIANT));

        TextView ahLabel = new TextView(this);
        ahLabel.setText("Auto-Hold");
        ahLabel.setTextColor(M3_ON_SURFACE_VARIANT);
        ahLabel.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 16.5f : 14.0f);
        ahLabel.setTypeface(Typeface.DEFAULT_BOLD);
        ahLabel.setSingleLine(true);
        autoHoldBar.addView(ahLabel, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));

        float ahFont = widescreen ? 15.5f : 13.5f;
        int ahPillH = widescreen ? 34 : 30;
        int ahPillW = widescreen ? 78 : 60;

        gtBtnAhOn = createGtButton("ON", ahFont, 999f, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onAutoHoldTapped(TelemetryRecorder.AUTOHOLD_ON);
            }
        });
        gtBtnAhOff = createGtButton("OFF", ahFont, 999f, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onAutoHoldTapped(TelemetryRecorder.AUTOHOLD_OFF);
            }
        });
        gtBtnAhUnset = createGtButton("?", ahFont, 999f, new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                onAutoHoldTapped(TelemetryRecorder.AUTOHOLD_UNSET);
            }
        });

        LinearLayout.LayoutParams ahOnLp = new LinearLayout.LayoutParams(ahPillW, ahPillH);
        ahOnLp.rightMargin = 6;
        LinearLayout.LayoutParams ahOffLp = new LinearLayout.LayoutParams(ahPillW, ahPillH);
        ahOffLp.rightMargin = 6;
        LinearLayout.LayoutParams ahUnsetLp = new LinearLayout.LayoutParams(
                widescreen ? 56 : 46, ahPillH);
        autoHoldBar.addView(gtBtnAhOn.button, ahOnLp);
        autoHoldBar.addView(gtBtnAhOff.button, ahOffLp);
        autoHoldBar.addView(gtBtnAhUnset.button, ahUnsetLp);

        LinearLayout.LayoutParams ahBarLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        ahBarLp.topMargin = gtRowGap + 2;
        gtCard.addView(autoHoldBar, ahBarLp);

        root.addView(gtCard, gtCardLp);

        // ================= 3. RIGHT FULL-HEIGHT SLOT (MONITOR + HIDDEN BLE/LOG) =================
        FrameLayout rightSlot = new FrameLayout(this);
        LinearLayout.LayoutParams rightSlotLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 0.62f);

        // 3A. Default Visible: Full-Height Status Monitor Card
        monitorCard = surfaceCard(M3_SURFACE_CONTAINER, 24f, 22, 8, 22, 8);

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
        rightSlot.addView(monitorCard, new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));

        // 3B. Hidden Panel: OBD-II BLE & Transition Log Card (toggled via sidebar `BLE & Log`)
        bleLogCard = surfaceCard(M3_SURFACE_CONTAINER_LOW, 24f, 18, 14, 18, 14);
        bleLogCard.setVisibility(View.GONE);

        LinearLayout bleHeaderRow = new LinearLayout(this);
        bleHeaderRow.setOrientation(LinearLayout.HORIZONTAL);
        bleHeaderRow.setGravity(Gravity.CENTER_VERTICAL);

        TextView obdTitle = new TextView(this);
        obdTitle.setText("OBD-II BLE & Transition Log");
        obdTitle.setTextColor(M3_PRIMARY);
        obdTitle.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 19.5f : 16.0f);
        obdTitle.setTypeface(Typeface.DEFAULT_BOLD);
        obdTitle.setSingleLine(true);
        obdTitle.setEllipsize(TextUtils.TruncateAt.END);
        bleHeaderRow.addView(obdTitle, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));

        int bleBtnH = widescreen ? 36 : 32;
        float bleBtnFont = widescreen ? 14.5f : 12.5f;

        scanBleBtn = m3Button(
                "Scan BLE",
                bleBtnFont,
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
                bleBtnFont,
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
        Button unpairBtn = m3Button(
                "Unpair",
                bleBtnFont,
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
        Button hideBleBtn = m3Button(
                "Hide",
                bleBtnFont,
                M3_SECONDARY_CONTAINER,
                M3_SECONDARY_CONTAINER_PRESSED,
                M3_ON_SECONDARY_CONTAINER,
                1,
                M3_SECONDARY_BORDER,
                new View.OnClickListener() {
                    @Override
                    public void onClick(View v) {
                        setBleLogPanelVisible(false);
                    }
                });

        bleHeaderRow.addView(scanBleBtn, headerPillParams(bleBtnH));
        bleHeaderRow.addView(reconnBtn, headerPillParams(bleBtnH));
        bleHeaderRow.addView(unpairBtn, headerPillParams(bleBtnH));
        bleHeaderRow.addView(hideBleBtn, headerPillParams(bleBtnH));
        bleLogCard.addView(bleHeaderRow, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        obdStatusLineText = new TextView(this);
        obdStatusLineText.setTextColor(M3_ON_SURFACE_VARIANT);
        obdStatusLineText.setTextSize(TypedValue.COMPLEX_UNIT_PX, widescreen ? 15.0f : 13.5f);
        obdStatusLineText.setPadding(0, 6, 0, 8);
        obdStatusLineText.setMaxLines(2);
        obdStatusLineText.setEllipsize(TextUtils.TruncateAt.END);
        obdStatusLineText.setText("127.0.0.1:35000 bridge coexistence + @hmgapi-ble direct BLE");
        bleLogCard.addView(obdStatusLineText);

        deviceListContainer = new LinearLayout(this);
        deviceListContainer.setOrientation(LinearLayout.VERTICAL);
        bleLogCard.addView(deviceListContainer, new LinearLayout.LayoutParams(
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
        bleLogCard.addView(scroll, scrollLp);

        rightSlot.addView(bleLogCard, new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));

        root.addView(rightSlot, rightSlotLp);
        return root;
    }

    private void onConfirmRegenLevelTapped(int level) {
        engine.setUiFocused(true);
        TelemetryRecorder.DriverGroundTruth gt = engine.confirmDriverRegenLevel(level);
        actionNoticeText.setText("Confirmed ground truth: " + gt.formatRegenLabel());
        refreshGroundTruthControls(engine.getLatestSnapshot());
        if (bleLogPanelVisible) {
            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
        }
    }

    private void onAutoHoldTapped(int state) {
        engine.setUiFocused(true);
        engine.setDriverAutoHold(state);
        actionNoticeText.setText("Auto-Hold ground truth: "
                + TelemetryRecorder.DriverGroundTruth.formatAutoHoldLabel(state));
        refreshGroundTruthControls(engine.getLatestSnapshot());
        if (bleLogPanelVisible) {
            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
        }
    }

    private void setBleLogPanelVisible(boolean visible) {
        bleLogPanelVisible = visible;
        monitorCard.setVisibility(visible ? View.GONE : View.VISIBLE);
        bleLogCard.setVisibility(visible ? View.VISIBLE : View.GONE);
        updateBleToggleButtonVisual();
        if (visible) {
            refreshObdPanel();
            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
        }
    }

    private void updateBleToggleButtonVisual() {
        boolean scanning = engine.getObdClient().isBleScanning();
        int nextState = bleLogPanelVisible ? 2 : (scanning ? 1 : 0);
        if (nextState == 2) {
            setTextIfChanged(toggleBleLogBtn, "Hide BLE/Log");
        } else if (nextState == 1) {
            setTextIfChanged(toggleBleLogBtn, "BLE Scan \u25CF");
        } else {
            setTextIfChanged(toggleBleLogBtn, "BLE & Log");
        }
        if (lastBleToggleVisualState != nextState) {
            lastBleToggleVisualState = nextState;
            if (nextState > 0) {
                toggleBleLogBtn.setBackground(m3PillDrawable(
                        M3_PRIMARY_CONTAINER, M3_PRIMARY_CONTAINER_PRESSED, 1, M3_PRIMARY));
                toggleBleLogBtn.setTextColor(M3_ON_PRIMARY_CONTAINER);
            } else {
                toggleBleLogBtn.setBackground(m3PillDrawable(
                        M3_SURFACE_CONTAINER_HIGH, M3_SURFACE_CONTAINER_HIGHEST, 1, M3_OUTLINE_VARIANT));
                toggleBleLogBtn.setTextColor(M3_ON_SURFACE);
            }
        }
    }

    private void refreshGroundTruthControls(BrakeEvaluator.Snapshot snap) {
        TelemetryRecorder.DriverGroundTruth gt = engine.getDriverGroundTruth();
        int activeStyle = gt.carryForward ? GT_STYLE_CARRY_FWD : GT_STYLE_CONFIRMED;

        gtBtnIpedal.applyStyle(
                gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_IPEDAL ? activeStyle : GT_STYLE_INACTIVE);
        gtBtnL3.applyStyle(
                gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_3 ? activeStyle : GT_STYLE_INACTIVE);
        gtBtnL2.applyStyle(
                gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_2 ? activeStyle : GT_STYLE_INACTIVE);
        gtBtnL1.applyStyle(
                gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_1 ? activeStyle : GT_STYLE_INACTIVE);
        gtBtnL0.applyStyle(
                gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_0 ? activeStyle : GT_STYLE_INACTIVE);
        gtBtnAuto.applyStyle(gt.autoRegen ? activeStyle : GT_STYLE_INACTIVE);
        gtBtnScc.applyStyle(gt.scc ? activeStyle : GT_STYLE_INACTIVE);

        gtBtnL3.setLabel(gt.autoRegen && gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_3 ? "L3 \u00B7 AUTO" : "L3");
        gtBtnL2.setLabel(gt.autoRegen && gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_2 ? "L2 \u00B7 AUTO" : "L2");
        gtBtnL1.setLabel(gt.autoRegen && gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_1 ? "L1 \u00B7 AUTO" : "L1");
        gtBtnL0.setLabel(gt.autoRegen && gt.regenLevel == BrakeEvaluator.REGEN_LEVEL_0 ? "L0 \u00B7 AUTO" : "L0");

        gtBtnAhOn.applyStyle(
                gt.autoHold == TelemetryRecorder.AUTOHOLD_ON ? GT_STYLE_CONFIRMED : GT_STYLE_INACTIVE);
        gtBtnAhOff.applyStyle(
                gt.autoHold == TelemetryRecorder.AUTOHOLD_OFF ? GT_STYLE_CONFIRMED : GT_STYLE_INACTIVE);
        gtBtnAhUnset.applyStyle(
                gt.autoHold == TelemetryRecorder.AUTOHOLD_UNSET ? GT_STYLE_SUBTLE_ACTIVE : GT_STYLE_INACTIVE);

        if (!gt.hasRegenConfirmation()) {
            applyGtBannerStyle(GT_BANNER_UNSET);
            setTextIfChanged(gtBannerTitle, "UNSET \u00B7 Unlabeled");
            setTextIfChanged(gtBannerSub, "Tap active regen mode whenever convenient");
            return;
        }

        boolean inPark = snap != null && Boolean.TRUE.equals(snap.gearPark);
        boolean modelKnown = snap != null
                && (snap.inferredRegenLevel >= 0 || snap.sccActive || snap.autoRegenActive);
        String modelLbl = modelKnown
                ? BrakeEvaluator.regenLevelLabel(snap.inferredRegenLevel, snap.sccActive, snap.autoRegenActive)
                : "Learning...";
        boolean matched = modelKnown && TelemetryRecorder.doesModelMatchDriverGt(snap, gt);
        String ageText = fmtAgeSeconds(gt.ageSeconds);

        if (gt.carryForward) {
            applyGtBannerStyle(GT_BANNER_CARRY_FWD);
            setTextIfChanged(gtBannerTitle,
                    "CARRY-FWD \u00B7 " + gt.formatRegenLabel() + " (" + ageText + ")");
            String sub = modelKnown
                    ? ("Tap button to re-confirm \u00B7 Model: " + modelLbl + (matched ? " \u2713" : " \u2717"))
                    : "Tap active button to re-confirm fresh age";
            setTextIfChanged(gtBannerSub, sub);
        } else if (modelKnown && !inPark && !matched) {
            applyGtBannerStyle(GT_BANNER_CONFIRMED_MISMATCH);
            setTextIfChanged(gtBannerTitle, "CONFIRMED \u00B7 " + gt.formatRegenLabel());
            setTextIfChanged(gtBannerSub,
                    "Fresh (" + ageText + ") \u00B7 Model: " + modelLbl + " \u2717 Mismatch");
        } else {
            applyGtBannerStyle(GT_BANNER_CONFIRMED_OK);
            setTextIfChanged(gtBannerTitle, "CONFIRMED \u00B7 " + gt.formatRegenLabel());
            String sub;
            if (inPark) {
                sub = "Latched in Park (" + ageText + ") \u00B7 clears after next drive";
            } else if (modelKnown) {
                sub = "Fresh (" + ageText + ") \u00B7 Model: " + modelLbl + " \u2713 Match";
            } else {
                sub = "Fresh (" + ageText + ") \u00B7 Model: Learning...";
            }
            setTextIfChanged(gtBannerSub, sub);
        }
    }

    private void applyGtBannerStyle(int bannerState) {
        if (lastGtBannerState == bannerState) {
            return;
        }
        lastGtBannerState = bannerState;
        if (bannerState == GT_BANNER_CONFIRMED_OK) {
            gtBannerCard.setBackground(gtBannerOkBg);
            gtBannerTitle.setTextColor(M3_ON_PRIMARY_CONTAINER);
            gtBannerSub.setTextColor(M3_ON_PRIMARY_CONTAINER);
        } else if (bannerState == GT_BANNER_CONFIRMED_MISMATCH) {
            gtBannerCard.setBackground(gtBannerMismatchBg);
            gtBannerTitle.setTextColor(M3_ON_BRAKE_LIT);
            gtBannerSub.setTextColor(M3_ON_BRAKE_LIT);
        } else if (bannerState == GT_BANNER_CARRY_FWD) {
            gtBannerCard.setBackground(gtBannerCarryBg);
            gtBannerTitle.setTextColor(M3_WARNING_AMBER);
            gtBannerSub.setTextColor(M3_ON_SURFACE_VARIANT);
        } else {
            gtBannerCard.setBackground(gtBannerUnsetBg);
            gtBannerTitle.setTextColor(M3_ON_SURFACE_VARIANT);
            gtBannerSub.setTextColor(M3_ON_SURFACE_MUTED);
        }
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
                "v0.10.4 \u00B7 Model " + engine.getActiveModelBadge()
                        + " \u00B7 HMG: " + engine.getHmgStatusSummary());
        setTextIfChanged(modeToggleBtn, sidebarModeLabel(cfg.obdMode));

        // Update Center Ground-Truth card live age & match banner
        refreshGroundTruthControls(snap);
        TelemetryRecorder.DriverGroundTruth gt = engine.getDriverGroundTruth();

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

        // 5. Detected Regen Mode (+ live Driver GT comparison if confirmed)
        boolean regenLearned = snap.inferredRegenLevel >= 0 || snap.sccActive || snap.autoRegenActive;
        if (regenLearned) {
            rowRegenMode.updateBadge(
                    BADGE_PRIMARY,
                    BrakeEvaluator.regenLevelLabel(snap.inferredRegenLevel, snap.sccActive, snap.autoRegenActive));
            String modeDesc = snap.sccActive ? "Smart Cruise Control"
                    : (snap.autoRegenActive ? "Smart Recuperation"
                    : (snap.inferredRegenLevel == BrakeEvaluator.REGEN_LEVEL_IPEDAL
                            ? "One-Pedal Driving" : "Manual Paddle"));
            if (gt.hasRegenConfirmation() && !Boolean.TRUE.equals(snap.gearPark)) {
                boolean match = TelemetryRecorder.doesModelMatchDriverGt(snap, gt);
                rowRegenMode.updateValue(
                        modeDesc + (match
                                ? "  \u00B7  GT \u2713"
                                : ("  \u00B7  GT=" + gt.formatRegenLabel() + " \u2717")),
                        match ? M3_ON_SURFACE_VARIANT : M3_WARNING_AMBER);
            } else {
                rowRegenMode.updateValue(modeDesc, M3_ON_SURFACE_VARIANT);
            }
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

        // 7. Model vs OBD & Driver Ground-Truth Accuracy
        TelemetryRecorder.ScoreSnapshot sc = engine.getRecorder().scoreSnapshot();
        rowAccuracy.updateBadge(BADGE_NONE, "");
        if (sc.scoredSamples > 0L) {
            String gtSuffix = !Float.isNaN(sc.gtHighConfAccuracyPct())
                    ? String.format(Locale.US, "  \u00B7  GT %.0f%%", Float.valueOf(sc.gtHighConfAccuracyPct()))
                    : "";
            rowAccuracy.updateValue(
                    String.format(Locale.US, "%.1f%% agree  \u00B7  Regen %s  \u00B7  Lag %s%s",
                            Float.valueOf(sc.agreementPct()),
                            Float.isNaN(sc.regenRecallPct()) ? "n/a"
                                    : String.format(Locale.US, "%.1f%%", Float.valueOf(sc.regenRecallPct())),
                            sc.onsetCount > 0 ? (sc.meanOnsetLagMs + " ms") : "n/a",
                            gtSuffix),
                    M3_ON_SURFACE);
        } else if (sc.gtHighConfEvalSamples > 0L) {
            rowAccuracy.updateValue(
                    String.format(Locale.US, "Regen GT %.1f%% (%d/%d ticks)  \u00B7  No OBD",
                            Float.valueOf(sc.gtHighConfAccuracyPct()),
                            Long.valueOf(sc.gtHighConfMatchedSamples),
                            Long.valueOf(sc.gtHighConfEvalSamples)),
                    M3_ON_SURFACE);
        } else {
            rowAccuracy.updateValue("Awaiting live OBD 22BC06 samples", M3_ON_SURFACE_VARIANT);
        }

        // 8. RAM Dataset Buffer & Cloud / USB Auto-Export Status
        AutoExportCoordinator.StatusSnapshot autoSnap = engine.getAutoExportStatus();
        CloudUploadConfig cloudCfg = engine.getCloudUploadConfig();
        boolean cloudReady = cloudCfg.isReadyForAutoUpload(cfg);
        if (engine.isExportInFlight()) {
            rowBuffer.updateBadge(BADGE_PRIMARY, cloudReady ? "SYNCING" : "WRITING USB");
        } else if (autoSnap.queuedCloudCount > 0) {
            rowBuffer.updateBadge(BADGE_OFF, "QUEUED \u00B7 " + autoSnap.queuedCloudCount);
        } else if (cloudReady) {
            rowBuffer.updateBadge(
                    BADGE_PRIMARY,
                    autoSnap.cloudUploadCount > 0
                            ? ("CLOUD \u00B7 " + autoSnap.cloudUploadCount)
                            : "AUTO CLOUD");
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
        if (autoSnap.queuedCloudCount > 0) {
            int qKb = Math.max(1, (autoSnap.queuedCloudBytes + 512) / 1024);
            rowBuffer.updateValue(
                    String.format(Locale.US, "%,d / %,d rows  \u00B7  %d queued (%d KB gz)",
                            Integer.valueOf(sc.bufferedRows),
                            Integer.valueOf(TelemetryRecorder.CAPACITY),
                            Integer.valueOf(autoSnap.queuedCloudCount),
                            Integer.valueOf(qKb)),
                    M3_WARNING_AMBER);
        } else if (autoSnap.waitingForUsb && autoSnap.waitingReason != null) {
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

        if (bleLogPanelVisible) {
            long now = System.currentTimeMillis();
            if (now - lastRightRefreshMs >= 450L) {
                lastRightRefreshMs = now;
                setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
            }
        }
    }

    private void refreshObdPanel() {
        ObdBrakeClient obd = engine.getObdClient();
        boolean scanning = obd.isBleScanning();
        updateBleToggleButtonVisual();
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

        if (bleLogPanelVisible) {
            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
        }
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
                            if (bleLogPanelVisible) {
                                setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                            }
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

    private void uploadDatasetToCloud() {
        actionNoticeText.setText("Compressing & uploading .tar.gz to GitHub over LTE/Wi-Fi...");
        Thread worker = new Thread("BrakeCloudUpload") {
            @Override
            public void run() {
                final AutoExportCoordinator.ActionOutcome out = engine.uploadDatasetToCloud();
                if (out.ok && out.uploadedToCloud && out.primaryTarget != null) {
                    try {
                        String name = out.primaryTarget.substring(out.primaryTarget.lastIndexOf('/') + 1);
                        HmgApi.toast(MainActivity.this, "Uploaded " + name);
                    } catch (Exception ignored) {}
                }
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        actionNoticeText.setText(out.message);
                        if (bleLogPanelVisible) {
                            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                        }
                    }
                });
            }
        };
        worker.setDaemon(true);
        worker.start();
    }

    private void testCloudConnectivity() {
        actionNoticeText.setText("Checking USB cloud-upload.json + TLS 1.2 -> api.github.com...");
        Thread worker = new Thread("BrakeCloudProbe") {
            @Override
            public void run() {
                final String msg = engine.testCloudAndImportFromUsb();
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        actionNoticeText.setText(msg);
                        if (bleLogPanelVisible) {
                            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                        }
                    }
                });
            }
        };
        worker.setDaemon(true);
        worker.start();
    }

    private void updateBrakeModelNow() {
        actionNoticeText.setText("Checking GitHub ota channel for signed brake model update...");
        Thread worker = new Thread("BrakeModelUpdate") {
            @Override
            public void run() {
                final ModelManager.UpdateOutcome out = engine.updateModelNow();
                if (out.updated) {
                    try {
                        HmgApi.toast(MainActivity.this, out.message);
                    } catch (Exception ignored) {}
                }
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        String suffix = (!out.updated && out.activeIsOta)
                                ? " (hold button to reset to built-in)"
                                : "";
                        actionNoticeText.setText(out.message + suffix);
                        setTextIfChanged(configSummaryText,
                                "v0.10.4 \u00B7 Model " + engine.getActiveModelBadge()
                                        + " \u00B7 HMG: " + engine.getHmgStatusSummary());
                        if (bleLogPanelVisible) {
                            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
                        }
                    }
                });
            }
        };
        worker.setDaemon(true);
        worker.start();
    }

    private void resetBrakeModelToBuiltIn() {
        Thread worker = new Thread("BrakeModelReset") {
            @Override
            public void run() {
                final ModelManager.UpdateOutcome out = engine.resetToBuiltInModel();
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        actionNoticeText.setText(out.message);
                        setTextIfChanged(configSummaryText,
                                "v0.10.4 \u00B7 Model " + engine.getActiveModelBadge()
                                        + " \u00B7 HMG: " + engine.getHmgStatusSummary());
                        if (bleLogPanelVisible) {
                            setTextIfChanged(transitionLogText, engine.getRecorder().recentTransitionsText(35));
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

    private GtToggleButton createGtButton(
            String label,
            float textSizePx,
            float cornerRadiusPx,
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
        b.setTextSize(TypedValue.COMPLEX_UNIT_PX, textSizePx);
        b.setTypeface(Typeface.DEFAULT_BOLD);
        b.setOnClickListener(listener);
        return new GtToggleButton(b, cornerRadiusPx);
    }

    private LinearLayout weightedTwoButtonRow(Button left, Button right, int gapPx) {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        LinearLayout.LayoutParams lpLeft = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 1.0f);
        lpLeft.rightMargin = gapPx;
        LinearLayout.LayoutParams lpRight = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 1.0f);
        row.addView(left, lpLeft);
        row.addView(right, lpRight);
        return row;
    }

    private static LinearLayout.LayoutParams weightedRowParams(float weight, int topMarginPx) {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, weight);
        lp.topMargin = topMarginPx;
        return lp;
    }

    private static LinearLayout.LayoutParams headerPillParams(int heightPx) {
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, heightPx);
        lp.leftMargin = 6;
        return lp;
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
        lp.topMargin = 6;
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
        return m3RoundedStateDrawable(normalFill, pressedFill, 999f, strokeWidth, strokeColor);
    }

    private static Drawable m3RoundedStateDrawable(
            int normalFill,
            int pressedFill,
            float radiusPx,
            int strokeWidth,
            int strokeColor) {
        StateListDrawable sld = new StateListDrawable();
        sld.addState(
                new int[]{android.R.attr.state_pressed},
                roundedRect(pressedFill, radiusPx, strokeWidth, strokeColor));
        sld.addState(
                new int[]{android.R.attr.state_focused},
                roundedRect(pressedFill, radiusPx, strokeWidth, strokeColor));
        sld.addState(
                new int[0],
                roundedRect(normalFill, radiusPx, strokeWidth, strokeColor));
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

    private static String fmtAgeSeconds(int ageSec) {
        if (ageSec < 0) {
            return "0s";
        }
        if (ageSec < 60) {
            return ageSec + "s";
        }
        int mins = ageSec / 60;
        int secs = ageSec % 60;
        if (mins < 60) {
            return secs > 0 ? (mins + "m " + secs + "s") : (mins + "m");
        }
        int hrs = mins / 60;
        int remM = mins % 60;
        return hrs + "h " + remM + "m";
    }

    private static String fmtObjFloat(Float v, String fmt) {
        return v == null ? "-" : String.format(Locale.US, fmt, v);
    }

    private static String fmtFlag(Boolean b) {
        if (b == null) return "-";
        return b.booleanValue() ? "1" : "0";
    }
}
