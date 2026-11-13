package dev.lemoyne.brakeindicator;

import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.net.LocalSocket;
import android.net.LocalSocketAddress;
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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Gen5W landscape status & Option 1 OEM CAN signal probe screen.
 * Displays a live brake-lamp preview, vehicle telemetry, discovered
 * ITM_HSCANService getters, a 3-second overlay test button, and USB export.
 */
public final class MainActivity extends Activity {

    private static final int BG_DARK = Color.rgb(20, 21, 24);
    private static final int CARD_BG = Color.rgb(31, 34, 41);
    private static final int CARD_BORDER = Color.rgb(48, 53, 64);
    private static final int TEXT_PRIMARY = Color.rgb(230, 234, 240);
    private static final int TEXT_MUTED = Color.rgb(146, 155, 168);
    private static final int ACCENT_TEAL = Color.rgb(35, 228, 197);
    private static final int BADGE_LIT = Color.rgb(214, 38, 38);
    private static final int BADGE_OFF = Color.rgb(45, 50, 60);
    private static final int BTN_BG = Color.rgb(43, 48, 58);

    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private BrakeLampView previewLamp;
    private TextView statusBadge;
    private TextView sourceLabel;
    private TextView telemetryText;
    private TextView configSummaryText;
    private TextView probeHeaderText;
    private TextView probeReportText;
    private TextView actionNoticeText;

    private volatile boolean streaming;
    private volatile LocalSocket streamSocket;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // Ensure top-bar overlay service is running whenever app is opened
        startService(new Intent(this, OverlayService.class));
        setContentView(buildContentView());
    }

    @Override
    protected void onResume() {
        super.onResume();
        startStream();
        refreshProbeReport();
    }

    @Override
    protected void onPause() {
        stopStream();
        super.onPause();
    }

    private View buildContentView() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(BG_DARK);
        root.setPadding(28, 20, 28, 20);

        // Header bar
        LinearLayout header = new LinearLayout(this);
        header.setOrientation(LinearLayout.HORIZONTAL);
        header.setGravity(Gravity.CENTER_VERTICAL);

        TextView title = new TextView(this);
        title.setText("Brake Light Indicator");
        title.setTextColor(TEXT_PRIMARY);
        title.setTextSize(TypedValue.COMPLEX_UNIT_PX, 30);
        title.setTypeface(Typeface.DEFAULT_BOLD);
        LinearLayout.LayoutParams titleLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f);
        header.addView(title, titleLp);

        statusBadge = new TextView(this);
        statusBadge.setText("WAITING FOR MONITOR");
        statusBadge.setTextColor(TEXT_PRIMARY);
        statusBadge.setTextSize(TypedValue.COMPLEX_UNIT_PX, 18);
        statusBadge.setTypeface(Typeface.DEFAULT_BOLD);
        statusBadge.setPadding(22, 10, 22, 10);
        statusBadge.setBackground(roundedRect(BADGE_OFF, 18, 0, 0));
        LinearLayout.LayoutParams badgeLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        badgeLp.rightMargin = 18;
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

        // Main two-column body
        LinearLayout columns = new LinearLayout(this);
        columns.setOrientation(LinearLayout.HORIZONTAL);
        LinearLayout.LayoutParams colsLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f);
        colsLp.topMargin = 18;

        // Left column: Live lamp preview + telemetry + controls
        LinearLayout leftCard = cardLayout();
        LinearLayout.LayoutParams leftLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 0.46f);
        leftLp.rightMargin = 14;

        LinearLayout previewRow = new LinearLayout(this);
        previewRow.setOrientation(LinearLayout.HORIZONTAL);
        previewRow.setGravity(Gravity.CENTER_VERTICAL);

        previewLamp = new BrakeLampView(this);
        previewLamp.setPreviewBackdrop(true);
        LinearLayout.LayoutParams lampLp = new LinearLayout.LayoutParams(128, 90);
        lampLp.rightMargin = 20;
        previewRow.addView(previewLamp, lampLp);

        LinearLayout previewMeta = new LinearLayout(this);
        previewMeta.setOrientation(LinearLayout.VERTICAL);

        sourceLabel = new TextView(this);
        sourceLabel.setText("Trigger: OFF");
        sourceLabel.setTextColor(TEXT_PRIMARY);
        sourceLabel.setTextSize(TypedValue.COMPLEX_UNIT_PX, 22);
        sourceLabel.setTypeface(Typeface.DEFAULT_BOLD);
        previewMeta.addView(sourceLabel);

        configSummaryText = new TextView(this);
        configSummaryText.setText("Mode: NATIVE  |  Anchor: TOP_RIGHT (260, 6)");
        configSummaryText.setTextColor(TEXT_MUTED);
        configSummaryText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 16);
        previewMeta.addView(configSummaryText);

        previewRow.addView(previewMeta, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));
        leftCard.addView(previewRow);

        telemetryText = new TextView(this);
        telemetryText.setTypeface(Typeface.MONOSPACE);
        telemetryText.setTextColor(TEXT_PRIMARY);
        telemetryText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 18);
        telemetryText.setLineSpacing(6.0f, 1.0f);
        telemetryText.setText("Connecting to monitor service...");
        LinearLayout.LayoutParams teleLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f);
        teleLp.topMargin = 16;
        leftCard.addView(telemetryText, teleLp);

        // Action row
        LinearLayout actions = new LinearLayout(this);
        actions.setOrientation(LinearLayout.HORIZONTAL);

        Button testBtn = styledButton("Test Icon (3s)", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                triggerTestOverlay();
            }
        });
        LinearLayout.LayoutParams b1 = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f);
        b1.rightMargin = 10;
        actions.addView(testBtn, b1);

        Button reloadBtn = styledButton("Reload Settings", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                triggerReload();
            }
        });
        LinearLayout.LayoutParams b2 = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f);
        b2.rightMargin = 10;
        actions.addView(reloadBtn, b2);

        Button exportBtn = styledButton("Export USB", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                exportReportToUsb();
            }
        });
        actions.addView(exportBtn, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));

        leftCard.addView(actions, new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        actionNoticeText = new TextView(this);
        actionNoticeText.setTextColor(ACCENT_TEAL);
        actionNoticeText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 15);
        actionNoticeText.setPadding(2, 8, 2, 0);
        actionNoticeText.setText("Configure placement & thresholds in HMG-API -> Mods -> Brake Light Indicator.");
        leftCard.addView(actionNoticeText);

        columns.addView(leftCard, leftLp);

        // Right column: Option 1 ITM_HSCANService probe inspector
        LinearLayout rightCard = cardLayout();
        LinearLayout.LayoutParams rightLp = new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.MATCH_PARENT, 0.54f);

        LinearLayout rightHeader = new LinearLayout(this);
        rightHeader.setOrientation(LinearLayout.HORIZONTAL);
        rightHeader.setGravity(Gravity.CENTER_VERTICAL);

        probeHeaderText = new TextView(this);
        probeHeaderText.setText("Option 1 OEM Signal Probe (ITM_HSCANService)");
        probeHeaderText.setTextColor(ACCENT_TEAL);
        probeHeaderText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 20);
        probeHeaderText.setTypeface(Typeface.DEFAULT_BOLD);
        rightHeader.addView(probeHeaderText, new LinearLayout.LayoutParams(
                0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.0f));

        Button refreshProbeBtn = styledButton("Refresh Probe", new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                refreshProbeReport();
            }
        });
        rightHeader.addView(refreshProbeBtn);
        rightCard.addView(rightHeader);

        ScrollView scroll = new ScrollView(this);
        LinearLayout.LayoutParams scrollLp = new LinearLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, 0, 1.0f);
        scrollLp.topMargin = 12;

        probeReportText = new TextView(this);
        probeReportText.setTypeface(Typeface.MONOSPACE);
        probeReportText.setTextColor(TEXT_PRIMARY);
        probeReportText.setTextSize(TypedValue.COMPLEX_UNIT_PX, 15);
        probeReportText.setText("Loading ITM_HSCANService getter report...");
        scroll.addView(probeReportText);
        rightCard.addView(scroll, scrollLp);

        columns.addView(rightCard, rightLp);
        root.addView(columns, colsLp);
        return root;
    }

    private LinearLayout cardLayout() {
        LinearLayout card = new LinearLayout(this);
        card.setOrientation(LinearLayout.VERTICAL);
        card.setPadding(22, 18, 22, 18);
        card.setBackground(roundedRect(CARD_BG, 16, 2, CARD_BORDER));
        return card;
    }

    private Button styledButton(String label, View.OnClickListener listener) {
        Button b = new Button(this);
        b.setText(label);
        b.setAllCaps(false);
        b.setTextColor(TEXT_PRIMARY);
        b.setTextSize(TypedValue.COMPLEX_UNIT_PX, 17);
        b.setPadding(18, 10, 18, 10);
        b.setBackground(roundedRect(BTN_BG, 12, 1, CARD_BORDER));
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

    private void startStream() {
        if (streaming) {
            return;
        }
        streaming = true;
        Thread t = new Thread("brake-ui-stream") {
            @Override
            public void run() {
                while (streaming) {
                    LocalSocket s = new LocalSocket();
                    try {
                        s.connect(new LocalSocketAddress(
                                IpcProtocol.SOCKET_NAME, LocalSocketAddress.Namespace.ABSTRACT));
                        streamSocket = s;
                        OutputStream out = s.getOutputStream();
                        out.write("STREAM\n".getBytes("UTF-8"));
                        out.flush();
                        BufferedReader in = new BufferedReader(
                                new InputStreamReader(s.getInputStream(), "UTF-8"), 512);
                        String line;
                        while (streaming && (line = in.readLine()) != null) {
                            final Map<String, String> msg = IpcProtocol.decodeLine(line);
                            if ("STATE".equals(msg.get("_type"))) {
                                uiHandler.post(new Runnable() {
                                    @Override
                                    public void run() {
                                        renderState(msg);
                                    }
                                });
                            }
                        }
                    } catch (Exception ignored) {
                    } finally {
                        streamSocket = null;
                        try {
                            s.close();
                        } catch (Exception ignored) {
                        }
                    }
                    if (streaming) {
                        try {
                            Thread.sleep(1000L);
                        } catch (InterruptedException ie) {
                            return;
                        }
                    }
                }
            }
        };
        t.setDaemon(true);
        t.start();
    }

    private void stopStream() {
        streaming = false;
        LocalSocket s = streamSocket;
        streamSocket = null;
        if (s != null) {
            try {
                s.close();
            } catch (Exception ignored) {
            }
        }
    }

    private void renderState(Map<String, String> m) {
        boolean lit = "1".equals(m.get("lit"));
        previewLamp.updateState(lit, IndicatorConfig.OffState.DIM);

        statusBadge.setText(lit ? "BRAKE LIGHTS ON" : "BRAKE LIGHTS OFF");
        statusBadge.setBackground(roundedRect(lit ? BADGE_LIT : BADGE_OFF, 18, 0, 0));

        sourceLabel.setText("Trigger: " + val(m, "src", "OFF"));
        configSummaryText.setText("Mode: " + val(m, "mode", "NATIVE")
                + "  |  Off: " + val(m, "off", "HIDDEN")
                + "  |  " + val(m, "anchor", "TOP_RIGHT")
                + " (" + val(m, "x", "260") + ", " + val(m, "y", "6") + ") "
                + val(m, "size", "36") + "px");

        StringBuilder sb = new StringBuilder();
        sb.append("Brake Pressure (HSCAN) : ").append(val(m, "prsr", "-")).append('\n');
        sb.append("Direct OEM Lamp Getter : ").append(val(m, "lamp", "-"));
        String lampName = val(m, "lamp_name", "");
        if (!lampName.isEmpty()) {
            sb.append(" (").append(lampName).append(')');
        }
        sb.append('\n');
        sb.append("Vehicle Speed          : ").append(val(m, "spd", "-")).append(" km/h\n");
        sb.append("Deceleration (-dv/dt)  : ").append(val(m, "decel", "0.00")).append(" m/s^2\n");
        sb.append("Accelerator Pedal      : ").append(val(m, "accel", "-")).append('\n');
        sb.append("Gear Park / EPB        : P=").append(val(m, "park", "-"))
                .append("  EPB=").append(val(m, "epb", "-")).append('\n');
        sb.append("tm_hscan_service       : ")
                .append("1".equals(m.get("hscan")) ? "CONNECTED" : "DISCONNECTED")
                .append(" (lamp=").append(val(m, "n_lamp", "0"))
                .append(", diag=").append(val(m, "n_diag", "0")).append(")\n");
        String cands = val(m, "cands", "");
        if (!cands.isEmpty()) {
            sb.append("Live HSCAN Getters     : ").append(cands);
        }
        telemetryText.setText(sb.toString());
    }

    private void triggerTestOverlay() {
        startService(new Intent(this, OverlayService.class));
        Thread t = new Thread("brake-ui-test") {
            @Override
            public void run() {
                final String res = OverlayService.sendMonitorCommand("TEST 3000");
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        actionNoticeText.setText(res != null
                                ? "Illuminating top-bar brake indicator for 3 seconds."
                                : "Monitor service not reachable yet (enable mod in HMG-API).");
                    }
                });
            }
        };
        t.setDaemon(true);
        t.start();
    }

    private void triggerReload() {
        Thread t = new Thread("brake-ui-reload") {
            @Override
            public void run() {
                final String res = OverlayService.sendMonitorCommand("RELOAD");
                final String rep = OverlayService.sendMonitorCommand("REPORT");
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        if (rep != null && !rep.isEmpty()) {
                            probeReportText.setText(rep);
                        }
                        actionNoticeText.setText(res != null
                                ? "Settings reloaded from HMG-API."
                                : "Monitor service not reachable.");
                    }
                });
            }
        };
        t.setDaemon(true);
        t.start();
    }

    private void refreshProbeReport() {
        Thread t = new Thread("brake-ui-report") {
            @Override
            public void run() {
                final String rep = OverlayService.sendMonitorCommand("REPORT");
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        if (rep != null && !rep.isEmpty()) {
                            probeReportText.setText(rep);
                        } else {
                            probeReportText.setText(
                                    "Monitor service is not connected yet.\n"
                                    + "Once enabled in HMG-API on a Gen5W unit, this panel lists every\n"
                                    + "read-only brake/stop-lamp/regen getter discovered in\n"
                                    + "/system/framework/com.hkmc.ext.hcan.jar (ITM_HSCANService)\n"
                                    + "and logs live value transitions during regenerative braking.");
                        }
                    }
                });
            }
        };
        t.setDaemon(true);
        t.start();
    }

    private void exportReportToUsb() {
        Thread t = new Thread("brake-usb-export") {
            @Override
            public void run() {
                String msg;
                try {
                    String report = OverlayService.sendMonitorCommand("REPORT");
                    if (report == null || report.isEmpty()) {
                        msg = "Monitor service not running; nothing to export.";
                    } else {
                        HmgApi api = HmgApi.connect().timeout(3000);
                        List<String> folders = api.usbFolders();
                        if (folders.isEmpty()) {
                            msg = "No USB drive detected (insert FAT32/exFAT USB stick).";
                        } else {
                            File dir = new File(folders.get(0));
                            dir.mkdirs();
                            String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
                            File out = new File(dir, "hscan-probe-" + stamp + ".txt");
                            FileOutputStream fos = new FileOutputStream(out);
                            try {
                                fos.write(report.getBytes("UTF-8"));
                            } finally {
                                fos.close();
                            }
                            msg = "Saved " + out.getAbsolutePath();
                            HmgApi.toast(MainActivity.this, "Exported " + out.getName());
                        }
                    }
                } catch (Exception e) {
                    msg = "USB export failed: " + e.getMessage();
                }
                final String finalMsg = msg;
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        actionNoticeText.setText(finalMsg);
                    }
                });
            }
        };
        t.setDaemon(true);
        t.start();
    }

    private static String val(Map<String, String> m, String key, String fallback) {
        String v = m.get(key);
        return v == null || v.isEmpty() ? fallback : v;
    }
}
