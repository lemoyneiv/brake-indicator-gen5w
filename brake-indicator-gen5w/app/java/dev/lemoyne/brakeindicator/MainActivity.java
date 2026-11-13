package dev.lemoyne.brakeindicator;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
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
 * Gen5W landscape dashboard for Brake Light Indicator v0.3:
 *  - Left card: Live brake-lamp preview, configurable OBD vs HMG-API icon mode toggle,
 *    built-in HMG-API readings vs real OBD (`770`/`22BC06`) ground truth, live accuracy/lag
 *    scorecard, 3-second test, dataset reset, and atomic USB CSV export.
 *  - Right card: `@gen5w_ble` direct BLE scanner pairing (with automatic `127.0.0.1:35000`
 *    loopback fallback) and live discrete transition log.
 */
public final class MainActivity extends Activity
        implements BrakeEngine.StateListener, ObdBrakeClient.Listener {

    private static final int BG_DARK = Color.rgb(20, 21, 24);
    private static final int CARD_BG = Color.rgb(31, 34, 41);
    private static final int CARD_BORDER = Color.rgb(48, 53, 64);
    private static final int TEXT_PRIMARY = Color.rgb(230, 234, 240);
    private static final int TEXT_MUTED = Color.rgb(146, 155, 168);
    private static final int ACCENT_TEAL = Color.rgb(35, 228, 197);
    private static final int ACCENT_AMBER = Color.rgb(245, 179, 66);
    private static final int BADGE_LIT = Color.rgb(214, 38, 38);
    private static final int BADGE_OFF = Color.rgb(45, 50, 60);
    private static final int BADGE_OBD_OK = Color.rgb(23, 98, 84);
    private static final int BTN_BG = Color.rgb(43, 48, 58);
    private static final int BTN_OBD_PAIR = Color.rgb(28, 72, 68);

    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private BrakeEngine engine;

    private BrakeLampView previewLamp;
    private TextView obdHeaderBadge;
    private TextView statusBadge;
    private TextView sourceLabel;
    private TextView configSummaryText;
    private Button modeToggleBtn;
    private TextView telemetryText;
    private TextView actionNoticeText;

    private TextView obdStatusLineText;
    private Button scanBleBtn;
    private LinearLayout deviceListContainer;
    private TextView transitionLogText;

    private long lastRightRefreshMs;

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
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(BG_DARK);
        root.setPadding(26, 18, 26, 18);

        // Header bar
        LinearLayout header = new LinearLayout(this);
        header.setOrientation(LinearLayout.HORIZONTAL);
        header.setGravity(Gravity.CENTER_VERTICAL);

        TextView title = new TextView(this);
        title.setText("Brake Light Indicator v0.3");
        title.setTextColor(TEXT_PRIMARY);
        title.setTextSize(TypedValue.COMPLEX_UNIT_PX, 28);
        title.setTypeface(Typeface.DEFAULT_BOLD);
        header.addView(title, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));

        obdHeaderBadge = new TextView(this);
        obdHeaderBadge.setText("OBD: IDLE");
        obdHeaderBadge.setTextColor(TEXT_PRIMARY);
        obdHeaderBadge.setTextSize(TypedValue.COMPLEX_UNIT_PX, 16);
        obdHeaderBadge.setTypeface(Typeface.DEFAULT_BOLD);
        obdHeaderBadge.setPadding(16, 9, 16, 9);
        obdHeaderBadge.setBackground(roundedRect(BADGE_OFF, 16, 0, 0));
        LinearLayout.LayoutParams obdBadgeLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        obdBadgeLp.rightMargin = 12;
        header.addView(obdHeaderBadge, obdBadgeLp);

        statusBadge = new TextView(this);
        statusBadge.setText("BRAKE LIGHTS OFF");
        statusBadge.setTextColor(TEXT_PRIMARY);
        statusBadge.setTextSize(TypedValue.COMPLEX_UNIT_PX, 17);
        statusBadge.setTypeface(Typeface.DEFAULT_BOLD);
        statusBadge.setPadding(20, 9, 20, 9);
        statusBadge.setBackground(roundedRect(BADGE_OFF, 16, 0, 0));
        LinearLayout.LayoutParams badgeLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        badgeLp.rightMargin = 14;
        header.addView(statusBadge, badgeLp);

        Button closeBtn = styledButton("Close", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                finish();
            }
        });
        header.addView(closeBtn, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        root.addView(header, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        // Two-column landscape body
        LinearLayout columns = new LinearLayout(this);
        columns.setOrientation(LinearLayout.HORIZONTAL);
        LinearLayout.LayoutParams colsLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f);
        colsLp.topMargin = 14;

        // ================= LEFT CARD =================
        LinearLayout leftCard = cardLayout();
        LinearLayout.LayoutParams leftLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 0.50f);
        leftLp.rightMargin = 14;

        LinearLayout previewRow = new LinearLayout(this);
        previewRow.setOrientation(LinearLayout.HORIZONTAL);
        previewRow.setGravity(Gravity.CENTER_VERTICAL);

        previewLamp = new BrakeLampView(this);
        previewLamp.setPreviewBackdrop(true);
        LinearLayout.LayoutParams lampLp = new LinearLayout.LayoutParams(116, 82);
        lampLp.rightMargin = 16;
        previewRow.addView(previewLamp, lampLp);

        LinearLayout previewMeta = new LinearLayout(this);
        previewMeta.setOrientation(LinearLayout.VERTICAL);

        sourceLabel = new TextView(this);
        sourceLabel.setText("Icon Trigger: OFF");
        sourceLabel.setTextColor(TEXT_PRIMARY);
        sourceLabel.setTextSize(TypedValue.COMPLEX_UNIT_PX, 20);
        sourceLabel.setTypeface(Typeface.DEFAULT_BOLD);
        previewMeta.addView(sourceLabel);

        configSummaryText = new TextView(this);
        configSummaryText.setText("HMG-API built-in readings + OBD calibration");
        configSummaryText.setTextColor(TEXT_MUTED);
        configSummaryText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 15);
        previewMeta.addView(configSummaryText);

        modeToggleBtn = styledButton("Mode: Real OBD -> Model Fallback", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                cycleObdMode();
            }
        });
        LinearLayout.LayoutParams modeLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        modeLp.topMargin = 6;
        previewMeta.addView(modeToggleBtn, modeLp);

        previewRow.addView(previewMeta, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));
        leftCard.addView(previewRow);

        telemetryText = new TextView(this);
        telemetryText.setTypeface(Typeface.MONOSPACE);
        telemetryText.setTextColor(TEXT_PRIMARY);
        telemetryText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 16);
        telemetryText.setLineSpacing(5.0f, 1.0f);
        telemetryText.setText("Starting HMG-API vehicle polling...");
        LinearLayout.LayoutParams teleLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f);
        teleLp.topMargin = 12;
        leftCard.addView(telemetryText, teleLp);

        // Left action buttons
        LinearLayout actions = new LinearLayout(this);
        actions.setOrientation(LinearLayout.HORIZONTAL);

        Button testBtn = styledButton("Test Icon (3s)", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startService(new Intent(MainActivity.this, OverlayService.class));
                engine.triggerManualTest(3000L);
                actionNoticeText.setText("Illuminating top-bar brake indicator for 3 seconds.");
            }
        });
        LinearLayout.LayoutParams b1 = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f);
        b1.rightMargin = 8;
        actions.addView(testBtn, b1);

        Button clearBtn = styledButton("Clear Dataset", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                engine.getRecorder().clear();
                refreshObdPanel();
                actionNoticeText.setText("Cleared in-RAM calibration dataset and scorecard.");
            }
        });
        LinearLayout.LayoutParams b2 = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f);
        b2.rightMargin = 8;
        actions.addView(clearBtn, b2);

        Button exportBtn = styledButton("Export CSV to USB", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                exportDatasetToUsb();
            }
        });
        actions.addView(exportBtn, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.15f));

        leftCard.addView(actions, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        actionNoticeText = new TextView(this);
        actionNoticeText.setTextColor(ACCENT_TEAL);
        actionNoticeText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 14);
        actionNoticeText.setPadding(2, 6, 2, 0);
        actionNoticeText.setText("Exports synchronized HMG-API + OBD 22BC06 CSV & summary to <usb>/BrakeIndicator/.");
        leftCard.addView(actionNoticeText);

        columns.addView(leftCard, leftLp);

        // ================= RIGHT CARD =================
        LinearLayout rightCard = cardLayout();
        LinearLayout.LayoutParams rightLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 0.50f);

        LinearLayout rightHeader = new LinearLayout(this);
        rightHeader.setOrientation(LinearLayout.HORIZONTAL);
        rightHeader.setGravity(Gravity.CENTER_VERTICAL);

        TextView obdTitle = new TextView(this);
        obdTitle.setText("OBD-II Ground Truth (@gen5w_ble + Loopback)");
        obdTitle.setTextColor(ACCENT_TEAL);
        obdTitle.setTextSize(TypedValue.COMPLEX_UNIT_PX, 19);
        obdTitle.setTypeface(Typeface.DEFAULT_BOLD);
        rightHeader.addView(obdTitle, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));

        scanBleBtn = styledButton("Scan BLE", new View.OnClickListener() {
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
        LinearLayout.LayoutParams scanLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        scanLp.rightMargin = 8;
        rightHeader.addView(scanBleBtn, scanLp);

        Button reconnBtn = styledButton("Reconnect", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                engine.getObdClient().requestReconnect();
                actionNoticeText.setText("Retrying @gen5w_ble direct -> 127.0.0.1 fallback...");
            }
        });
        LinearLayout.LayoutParams recLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        recLp.rightMargin = 8;
        rightHeader.addView(reconnBtn, recLp);

        Button unpairBtn = styledButton("Unpair", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                engine.getObdClient().unpairAdapter();
                actionNoticeText.setText("Cleared paired BLE adapter (127.0.0.1 loopback fallback still active).");
            }
        });
        rightHeader.addView(unpairBtn);
        rightCard.addView(rightHeader);

        obdStatusLineText = new TextView(this);
        obdStatusLineText.setTextColor(TEXT_MUTED);
        obdStatusLineText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 15);
        obdStatusLineText.setPadding(0, 6, 0, 6);
        obdStatusLineText.setText("Priority: 1) Direct BLE via @gen5w_ble  ->  2) 127.0.0.1:35000 loopback");
        rightCard.addView(obdStatusLineText);

        deviceListContainer = new LinearLayout(this);
        deviceListContainer.setOrientation(LinearLayout.VERTICAL);
        rightCard.addView(deviceListContainer, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        ScrollView scroll = new ScrollView(this);
        LinearLayout.LayoutParams scrollLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f);
        scrollLp.topMargin = 8;

        transitionLogText = new TextView(this);
        transitionLogText.setTypeface(Typeface.MONOSPACE);
        transitionLogText.setTextColor(TEXT_PRIMARY);
        transitionLogText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 14);
        transitionLogText.setLineSpacing(4.0f, 1.0f);
        transitionLogText.setText("No brake-lamp or HMG-API model transitions recorded yet.");
        scroll.addView(transitionLogText);
        rightCard.addView(scroll, scrollLp);

        columns.addView(rightCard, rightLp);
        root.addView(columns, colsLp);
        return root;
    }

    private void renderTelemetry(
            IndicatorConfig cfg,
            BrakeEvaluator.Snapshot snap,
            boolean hmgConnected) {
        if (cfg == null || snap == null) {
            return;
        }
        previewLamp.updateState(snap.illuminated, IndicatorConfig.OffState.DIM);

        statusBadge.setText(snap.illuminated ? "BRAKE LIGHTS ON" : "BRAKE LIGHTS OFF");
        statusBadge.setBackground(roundedRect(snap.illuminated ? BADGE_LIT : BADGE_OFF, 16, 0, 0));

        ObdBrakeClient obdClient = engine.getObdClient();
        float hz = obdClient.getPollHz();
        if (snap.obdFresh) {
            obdHeaderBadge.setText(String.format(Locale.US, "OBD %s %.1fHz",
                    snap.obd.transport, Float.valueOf(hz)));
            obdHeaderBadge.setBackground(roundedRect(BADGE_OBD_OK, 16, 0, 0));
        } else {
            obdHeaderBadge.setText(cfg.obdMode == IndicatorConfig.ObdMode.DISABLED ? "OBD: OFF" : "OBD: STANDBY");
            obdHeaderBadge.setBackground(roundedRect(BADGE_OFF, 16, 0, 0));
        }

        sourceLabel.setText("Icon Trigger: " + snap.source);
        configSummaryText.setText(cfg.anchor + " (" + cfg.xOffsetPx + ", " + cfg.yOffsetPx + ") "
                + cfg.iconSizePx + "px  |  HMG-API: " + (hmgConnected ? "OK" : "WAITING"));
        modeToggleBtn.setText("Mode: " + cfg.obdMode.shortLabel() + " (tap to switch)");

        TelemetryRecorder.ScoreSnapshot sc = engine.getRecorder().scoreSnapshot();

        StringBuilder sb = new StringBuilder(512);
        sb.append("Real OBD (770/22BC06) : ");
        if (snap.obdFresh) {
            sb.append(snap.obd.brakeLampLit ? "ON " : "OFF")
              .append(" [raw=").append(snap.obd.rawCode);
            if (snap.obd.brakeLampLit) {
                sb.append(snap.obd.pedalBit ? " PEDAL" : " REGEN/SCC");
            }
            sb.append(", ").append(snap.obd.transport)
              .append(", age=").append(snap.obd.ageMsAt(snap.timestampMs)).append("ms]\n");
        } else {
            sb.append(obdClient.getStatusSummary()).append('\n');
        }

        sb.append("HMG-API Model         : ")
          .append(snap.modelIlluminated ? ("ON  [" + snap.modelSource + "]") : "OFF").append('\n');
        sb.append("Speed / Decel (-dv/dt): ")
          .append(fmtObjFloat(snap.speedKmh, "%.1f")).append(" km/h  |  ")
          .append(String.format(Locale.US, "%.2f m/s^2", Float.valueOf(snap.decelMps2))).append('\n');
        sb.append("Brake Prsr / Accel    : prsr=")
          .append(fmtObjFloat(snap.brakePressure, "%.1f")).append("  |  accel=")
          .append(fmtObjFloat(snap.accelPedal, "%.1f")).append("%\n");
        sb.append("Steer / Park / HV SOC : ")
          .append(fmtObjFloat(snap.steeringAngleDeg, "%+.1f")).append(" deg  |  P=")
          .append(fmtFlag(snap.gearPark)).append(" EPB=").append(fmtFlag(snap.parkingBrake))
          .append("  |  SOC=").append(fmtObjFloat(snap.hvSocPct, "%.1f")).append("%\n");

        sb.append("Model vs OBD Accuracy : ");
        if (sc.scoredSamples > 0L) {
            sb.append(String.format(Locale.US, "Agree %.1f%% | RegenRec %s | Lag %s\n",
                    Float.valueOf(sc.agreementPct()),
                    Float.isNaN(sc.regenRecallPct()) ? "n/a"
                            : String.format(Locale.US, "%.1f%%", Float.valueOf(sc.regenRecallPct())),
                    sc.onsetCount > 0 ? (sc.meanOnsetLagMs + "ms") : "n/a"));
        } else {
            sb.append("awaiting live OBD 22BC06 samples\n");
        }
        sb.append("RAM Dataset Buffer    : ")
          .append(sc.bufferedRows).append(" / ").append(TelemetryRecorder.CAPACITY)
          .append(" rows (").append(sc.scoredSamples).append(" scored)");

        telemetryText.setText(sb.toString());

        long now = System.currentTimeMillis();
        if (now - lastRightRefreshMs >= 450L) {
            lastRightRefreshMs = now;
            transitionLogText.setText(engine.getRecorder().recentTransitionsText(35));
        }
    }

    private void refreshObdPanel() {
        ObdBrakeClient obd = engine.getObdClient();
        scanBleBtn.setText(obd.isBleScanning() ? "Stop Scan" : "Scan BLE");

        String pairedMac = obd.getPairedMac();
        String pairedInfo = pairedMac.isEmpty()
                ? "Paired BLE: none"
                : ("Paired BLE: " + obd.getPairedName() + " [" + pairedMac + "]");
        obdStatusLineText.setText(pairedInfo + "  |  " + obd.getStatusSummary());

        deviceListContainer.removeAllViews();
        List<ObdBrakeClient.DiscoveredDevice> devs = obd.getDiscoveredDevices();
        int shown = 0;
        for (int i = 0; i < devs.size() && shown < 4; i++) {
            final ObdBrakeClient.DiscoveredDevice d = devs.get(i);
            shown++;
            String prefix = d.likelyBvm02 ? "[BVM02 - Ignored] " : (d.likelyObd ? "[OBD] " : "");
            String label = prefix + d.name + "  (" + d.mac + ", " + d.rssi + " dBm)";
            Button rowBtn = styledButton(label, new View.OnClickListener() {
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
            if (d.likelyObd) {
                rowBtn.setBackground(roundedRect(BTN_OBD_PAIR, 10, 1, ACCENT_TEAL));
            } else if (d.likelyBvm02) {
                rowBtn.setTextColor(ACCENT_AMBER);
            }
            LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
            lp.bottomMargin = 6;
            deviceListContainer.addView(rowBtn, lp);
        }

        transitionLogText.setText(engine.getRecorder().recentTransitionsText(35));
    }

    private void cycleObdMode() {
        Thread worker = new Thread("BrakeModeCycle") {
            @Override
            public void run() {
                final IndicatorConfig.ObdMode next = engine.cycleObdModeAndSave();
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        modeToggleBtn.setText("Mode: " + next.shortLabel() + " (tap to switch)");
                        actionNoticeText.setText("Switched indicator mode to: " + next.shortLabel());
                    }
                });
            }
        };
        worker.setDaemon(true);
        worker.start();
    }

    private void exportDatasetToUsb() {
        actionNoticeText.setText("Exporting synchronized CSV + summary to USB...");
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
                            actionNoticeText.setText("Saved to USB: " + res.csvPath);
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

    private LinearLayout cardLayout() {
        LinearLayout card = new LinearLayout(this);
        card.setOrientation(LinearLayout.VERTICAL);
        card.setPadding(20, 16, 20, 16);
        card.setBackground(roundedRect(CARD_BG, 16, 2, CARD_BORDER));
        return card;
    }

    private Button styledButton(String label, View.OnClickListener listener) {
        Button b = new Button(this);
        b.setText(label);
        b.setAllCaps(false);
        b.setTextColor(TEXT_PRIMARY);
        b.setTextSize(TypedValue.COMPLEX_UNIT_PX, 16);
        b.setPadding(16, 9, 16, 9);
        b.setBackground(roundedRect(BTN_BG, 10, 1, CARD_BORDER));
        b.setOnClickListener(listener);
        return b;
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

    private static String fmtObjFloat(Float v, String fmt) {
        return v == null ? "-" : String.format(Locale.US, fmt, v);
    }

    private static String fmtFlag(Boolean b) {
        if (b == null) return "-";
        return b.booleanValue() ? "1" : "0";
    }
}
