package dev.lemoyne.brakeindicator;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;

import hmgapi.client.HmgApi;

/**
 * Ensures {@link OverlayService} starts automatically on boot or live mod enablement
 * without requiring a root daemon process.
 */
public final class BootReceiver extends BroadcastReceiver {

    @Override
    public void onReceive(final Context context, Intent intent) {
        if (context == null) return;
        final PendingResult pending = goAsync();
        Thread worker = new Thread("BrakeBootCheck") {
            @Override
            public void run() {
                try {
                    HmgApi api = HmgApi.connect().timeout(2000);
                    if (!api.available() || api.mod() != null) {
                        context.startService(new Intent(context, OverlayService.class));
                    }
                } catch (Throwable ignored) {
                } finally {
                    try {
                        pending.finish();
                    } catch (Throwable ignored) {}
                }
            }
        };
        worker.setDaemon(true);
        worker.start();
    }
}
