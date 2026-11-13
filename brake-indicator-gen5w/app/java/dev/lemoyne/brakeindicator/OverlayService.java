package dev.lemoyne.brakeindicator;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.PixelFormat;
import android.os.Handler;
import android.os.IBinder;
import android.os.Looper;
import android.view.Gravity;
import android.view.WindowManager;

import hmgapi.client.HmgApi;

/**
 * Maintains the non-touchable top-row brake light indicator overlay on the
 * Gen5W WindowManager (`TYPE_SYSTEM_OVERLAY` sits above `TYPE_STATUS_BAR` on API 19
 * without intercepting touches) and hosts {@link BrakeEngine}.
 */
public final class OverlayService extends Service implements BrakeEngine.StateListener {

    private final Handler uiHandler = new Handler(Looper.getMainLooper());
    private WindowManager windowManager;
    private BrakeLampView lampView;
    private WindowManager.LayoutParams layoutParams;
    private BrakeEngine engine;

    private IndicatorConfig.Anchor currentAnchor = IndicatorConfig.Anchor.TOP_RIGHT;
    private int currentX = 260;
    private int currentY = 6;
    private int currentSize = 36;

    private final BroadcastReceiver stateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            checkModEnabledAndReload();
        }
    };

    @Override
    public void onCreate() {
        super.onCreate();
        windowManager = (WindowManager) getSystemService(WINDOW_SERVICE);
        attachOverlayView();

        engine = BrakeEngine.getInstance(this);
        engine.ensureStarted();
        engine.addListener(this);

        IntentFilter filter = new IntentFilter(HmgApi.ACTION_STATE_CHANGED);
        registerReceiver(stateReceiver, filter);
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        if (engine != null) {
            engine.ensureStarted();
            engine.requestSettingsReload();
        }
        return START_STICKY;
    }

    @Override
    public void onDestroy() {
        if (engine != null) {
            engine.removeListener(this);
        }
        try {
            unregisterReceiver(stateReceiver);
        } catch (Exception ignored) {
        }
        removeOverlayView();
        super.onDestroy();
    }

    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    @Override
    public void onEngineState(
            final IndicatorConfig config,
            final BrakeEvaluator.Snapshot snapshot,
            boolean hmgConnected) {
        uiHandler.post(new Runnable() {
            @Override
            public void run() {
                applyState(config, snapshot);
            }
        });
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

    private void applyState(IndicatorConfig cfg, BrakeEvaluator.Snapshot snap) {
        if (cfg == null || snap == null) {
            return;
        }
        if (lampView == null) {
            attachOverlayView();
            if (lampView == null) {
                return;
            }
        }

        lampView.updateState(snap.illuminated, cfg.offState);

        if (cfg.anchor != currentAnchor
                || cfg.xOffsetPx != currentX
                || cfg.yOffsetPx != currentY
                || cfg.iconSizePx != currentSize) {
            currentAnchor = cfg.anchor;
            currentX = cfg.xOffsetPx;
            currentY = cfg.yOffsetPx;
            currentSize = cfg.iconSizePx;
            layoutParams = buildLayoutParams(currentAnchor, currentX, currentY, currentSize);
            try {
                windowManager.updateViewLayout(lampView, layoutParams);
            } catch (Throwable ignored) {
            }
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

    private void checkModEnabledAndReload() {
        Thread worker = new Thread("BrakeStateCheck") {
            @Override
            public void run() {
                try {
                    HmgApi api = HmgApi.connect().timeout(2000);
                    if (api.available() && api.mod() == null) {
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
                if (engine != null) {
                    engine.requestSettingsReload();
                }
            }
        };
        worker.setDaemon(true);
        worker.start();
    }
}
