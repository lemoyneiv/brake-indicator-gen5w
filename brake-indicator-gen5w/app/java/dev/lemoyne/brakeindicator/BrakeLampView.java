package dev.lemoyne.brakeindicator;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.view.View;

/**
 * Custom anti-aliased automotive brake-lamp indicator view (( ● )).
 * Used both in the top status bar overlay (OverlayService) and as a live
 * preview inside MainActivity.
 */
public final class BrakeLampView extends View {

    private static final int COLOR_LIT_RED = Color.rgb(255, 46, 46);
    private static final int COLOR_LIT_CORE = Color.rgb(255, 96, 96);
    private static final int COLOR_LIT_HALO = Color.argb(92, 255, 30, 30);
    private static final int COLOR_LIT_PILL = Color.argb(145, 22, 10, 12);
    private static final int COLOR_DIM_OUTLINE = Color.argb(72, 148, 160, 176);

    private final Paint pillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint haloPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint strokePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint corePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final RectF pillRect = new RectF();
    private final RectF arcRect = new RectF();

    private boolean illuminated;
    private IndicatorConfig.OffState offState = IndicatorConfig.OffState.HIDDEN;
    private boolean previewBackdrop;

    public BrakeLampView(Context context) {
        super(context);
        pillPaint.setStyle(Paint.Style.FILL);
        haloPaint.setStyle(Paint.Style.FILL);
        strokePaint.setStyle(Paint.Style.STROKE);
        strokePaint.setStrokeCap(Paint.Cap.ROUND);
        corePaint.setStyle(Paint.Style.FILL);
    }

    public void setPreviewBackdrop(boolean previewBackdrop) {
        this.previewBackdrop = previewBackdrop;
        invalidate();
    }

    public void updateState(boolean illuminated, IndicatorConfig.OffState offState) {
        IndicatorConfig.OffState nextOff = offState == null ? IndicatorConfig.OffState.HIDDEN : offState;
        if (this.illuminated == illuminated && this.offState == nextOff) {
            return;
        }
        this.illuminated = illuminated;
        this.offState = nextOff;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        int w = getWidth();
        int h = getHeight();
        if (w <= 0 || h <= 0) {
            return;
        }

        if (!illuminated && offState == IndicatorConfig.OffState.HIDDEN && !previewBackdrop) {
            return;
        }

        float cx = w * 0.5f;
        float cy = h * 0.5f;
        float unit = Math.min(w / 1.42f, (float) h);
        float stroke = Math.max(2.0f, unit * 0.095f);

        if (illuminated) {
            // Subtle dark contrast pill so the red icon stays crisp over bright daytime CarPlay/maps
            float pw = unit * 1.28f;
            float ph = unit * 0.88f;
            pillRect.set(cx - pw * 0.5f, cy - ph * 0.5f, cx + pw * 0.5f, cy + ph * 0.5f);
            pillPaint.setColor(COLOR_LIT_PILL);
            canvas.drawRoundRect(pillRect, ph * 0.45f, ph * 0.45f, pillPaint);

            // Soft red glow behind central lamp
            haloPaint.setColor(COLOR_LIT_HALO);
            canvas.drawCircle(cx, cy, unit * 0.36f, haloPaint);

            // Central ring + inner illuminated core
            strokePaint.setStrokeWidth(stroke);
            strokePaint.setColor(COLOR_LIT_RED);
            canvas.drawCircle(cx, cy, unit * 0.25f, strokePaint);

            corePaint.setColor(COLOR_LIT_CORE);
            canvas.drawCircle(cx, cy, unit * 0.135f, corePaint);

            // Left & right curved brake-shoe arcs
            float arcRadius = unit * 0.41f;
            arcRect.set(cx - arcRadius, cy - arcRadius, cx + arcRadius, cy + arcRadius);
            canvas.drawArc(arcRect, -42.0f, 84.0f, false, strokePaint);
            canvas.drawArc(arcRect, 138.0f, 84.0f, false, strokePaint);
        } else {
            // Dim outline (either OffState.DIM on status bar, or unlit preview in MainActivity)
            strokePaint.setStrokeWidth(Math.max(1.6f, stroke * 0.85f));
            strokePaint.setColor(COLOR_DIM_OUTLINE);
            canvas.drawCircle(cx, cy, unit * 0.25f, strokePaint);

            float arcRadius = unit * 0.41f;
            arcRect.set(cx - arcRadius, cy - arcRadius, cx + arcRadius, cy + arcRadius);
            canvas.drawArc(arcRect, -40.0f, 80.0f, false, strokePaint);
            canvas.drawArc(arcRect, 140.0f, 80.0f, false, strokePaint);
        }
    }
}
