package dev.lemoyne.brakeindicator;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.PixelFormat;
import android.net.LocalSocket;
import android.net.LocalSocketAddress;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.view.Gravity;
import android.view.WindowManager;

import hmgapi.client.HmgApi;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.Map;

/**
 * Maintains the non-touchable top-row brake light indicator overlay on the
 * Gen5W WindowManager (TYPE_SYSTEM_OVERLAY sits above TYPE_STATUS_BAR on API 19
 * without intercepting touches).
 */
public final class OverlayService extends Service {

    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private WindowManager windowManager;
    private BrakeLampView lampView;
    private WindowManager.LayoutParams layoutParams;

    private IndicatorConfig.Anchor currentAnchor = IndicatorConfig.Anchor.TOP_RIGHT;
    private int currentX = 260;
    private int currentY = 6;
    private int currentSize = 36;

    private volatile boolean running;
    private volatile LocalSocket activeSocket;

    private final BroadcastReceiver stateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            checkModEnabledAndReload();
        }
    };

    @Override
    public void onCreate() {
        super.onCreate();
        running = true;
        windowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
        attachOverlayView();

        IntentFilter filter = new IntentFilter(HmgApi.ACTION_STATE_CHANGED);
        registerReceiver(stateReceiver, filter);

        Thread reader = new Thread("brake-overlay-ipc") {
            @Override
            public void run() {
                streamLoop();
            }
        };
        reader.setDaemon(true);
        reader.start();
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        running = false;
        try {
            unregisterReceiver(stateReceiver);
        } catch (Exception ignored) {
        }
        LocalSocket s = activeSocket;
        activeSocket = null;
        if (s != null) {
            try {
                s.close();
            } catch (Exception ignored) {
            }
        }
        removeOverlayView();
        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    private void attachOverlayView() {
        if (lampView != null || windowManager == null) {
            return;
        }
        IndicatorConfig defaults = IndicatorConfig.defaults();
        currentAnchor = defaults.anchor;
        currentX = defaults.xOffsetPx;
        currentY = defaults.yOffsetPx;
        currentSize = defaults.iconSizePx;

        lampView = new BrakeLampView(this);
        layoutParams = buildLayoutParams(currentAnchor, currentX, currentY, currentSize);
        try {
            windowManager.addView(lampView, layoutParams);
        } catch (Throwable t) {
            lampView = null;
        }
    }

    private void removeOverlayView() {
        if (lampView != null && windowManager != null) {
            try {
                windowManager.removeView(lampView);
            } catch (Throwable ignored) {
            }
            lampView = null;
        }
    }

    private static WindowManager.LayoutParams buildLayoutParams(
            IndicatorConfig.Anchor anchor,
            int xOffset,
            int yOffset,
            int iconSize) {
        int width = Math.max(28, Math.round(iconSize * 1.42f));
        WindowManager.LayoutParams lp = new WindowManager.LayoutParams(
                width,
                iconSize,
                WindowManager.LayoutParams.TYPE_SYSTEM_OVERLAY,
                WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE
                        | WindowManager.LayoutParams.FLAG_NOT_TOUCHABLE
                        | WindowManager.LayoutParams.FLAG_LAYOUT_IN_SCREEN
                        | WindowManager.LayoutParams.FLAG_LAYOUT_NO_LIMITS,
                PixelFormat.TRANSLUCENT);
        lp.gravity = Gravity.TOP | anchorGravity(anchor);
        lp.x = xOffset;
        lp.y = yOffset;
        lp.setTitle("BrakeLightIndicator");
        return lp;
    }

    private static int anchorGravity(IndicatorConfig.Anchor anchor) {
        if (anchor == IndicatorConfig.Anchor.TOP_LEFT) {
            return Gravity.LEFT;
        }
        if (anchor == IndicatorConfig.Anchor.TOP_CENTER) {
            return Gravity.CENTER_HORIZONTAL;
        }
        return Gravity.RIGHT;
    }

    private void streamLoop() {
        while (running) {
            LocalSocket s = new LocalSocket();
            try {
                s.connect(new LocalSocketAddress(IpcProtocol.SOCKET_NAME, LocalSocketAddress.Namespace.ABSTRACT));
                activeSocket = s;
                OutputStream out = s.getOutputStream();
                out.write("STREAM\n".getBytes("UTF-8"));
                out.flush();

                BufferedReader in = new BufferedReader(new InputStreamReader(s.getInputStream(), "UTF-8"), 512);
                String line;
                while (running && (line = in.readLine()) != null) {
                    final Map<String, String> msg = IpcProtocol.decodeLine(line);
                    if ("STATE".equals(msg.get("_type"))) {
                        uiHandler.post(new Runnable() {
                            @Override
                            public void run() {
                                applyState(msg);
                            }
                        });
                    }
                }
            } catch (Exception ignored) {
                // MonitorMain may be starting or restarting; clear lit state so icon never sticks ON
                uiHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        if (lampView != null) {
                            lampView.updateState(false, IndicatorConfig.OffState.HIDDEN);
                        }
                    }
                });
            } finally {
                activeSocket = null;
                try {
                    s.close();
                } catch (Exception ignored) {
                }
            }

            if (running) {
                try {
                    Thread.sleep(1000L);
                } catch (InterruptedException ie) {
                    return;
                }
            }
        }
    }

    private void applyState(Map<String, String> msg) {
        if (lampView == null) {
            attachOverlayView();
            if (lampView == null) {
                return;
            }
        }

        boolean lit = "1".equals(msg.get("lit"));
        IndicatorConfig.OffState off = IndicatorConfig.OffState.fromString(msg.get("off"));
        IndicatorConfig.Anchor anchor = IndicatorConfig.Anchor.fromString(msg.get("anchor"));
        int x = parseInt(msg.get("x"), currentX);
        int y = parseInt(msg.get("y"), currentY);
        int size = Math.max(20, Math.min(96, parseInt(msg.get("size"), currentSize)));

        lampView.updateState(lit, off);

        if (anchor != currentAnchor || x != currentX || y != currentY || size != currentSize) {
            currentAnchor = anchor;
            currentX = x;
            currentY = y;
            currentSize = size;
            layoutParams = buildLayoutParams(anchor, x, y, size);
            try {
                windowManager.updateViewLayout(lampView, layoutParams);
            } catch (Throwable ignored) {
            }
        }
    }

    private void checkModEnabledAndReload() {
        Thread worker = new Thread("brake-state-check") {
            @Override
            public void run() {
                try {
                    HmgApi api = HmgApi.connect().timeout(2000);
                    if (api.available() && api.mod() == null) {
                        // Mod was disabled or removed in HMG-API
                        uiHandler.post(new Runnable() {
                            @Override
                            public void run() {
                                stopSelf();
                            }
                        });
                        return;
                    }
                } catch (Exception ignored) {
                }
                sendMonitorCommand("RELOAD");
            }
        };
        worker.setDaemon(true);
        worker.start();
    }

    static String sendMonitorCommand(String command) {
        LocalSocket s = new LocalSocket();
        try {
            s.connect(new LocalSocketAddress(IpcProtocol.SOCKET_NAME, LocalSocketAddress.Namespace.ABSTRACT));
            s.setSoTimeout(6000);
            OutputStream out = s.getOutputStream();
            out.write((command + "\n").getBytes("UTF-8"));
            out.flush();
            BufferedReader in = new BufferedReader(new InputStreamReader(s.getInputStream(), "UTF-8"), 1024);
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = in.readLine()) != null) {
                if ("END_REPORT".equals(line)) {
                    break;
                }
                if (sb.length() > 0) {
                    sb.append('\n');
                }
                sb.append(line);
            }
            return sb.toString();
        } catch (Exception e) {
            return null;
        } finally {
            try {
                s.close();
            } catch (Exception ignored) {
            }
        }
    }

    private static int parseInt(String s, int fallback) {
        if (s == null) {
            return fallback;
        }
        try {
            return Integer.parseInt(s.trim());
        } catch (NumberFormatException e) {
            return fallback;
        }
    }
}
