package dev.lemoyne.brakeindicator;

import java.util.Map;

/**
 * Stable modular ABI between the Brake Light Indicator host APK ({@code dev.lemoyne.brakeindicator})
 * and ECDSA-signed OTA brake-lamp &amp; regen-inference model plugins
 * ({@code dev.lemoyne.brakeindicator.model.*}).
 *
 * <p>Because Android 4.4.2 (API 19) Dalvik {@code DexClassLoader} uses parent-first delegation,
 * the host APK defines this interface and its immutable DTOs ({@link IndicatorConfig},
 * {@link BrakeEvaluator.Input}, {@link BrakeEvaluator.ObdSample}, {@link BrakeEvaluator.Snapshot}),
 * while downloadable {@code brake-model-<ver>.jar} bundles implement {@code BrakeModel} in the
 * isolated {@code dev.lemoyne.brakeindicator.model} subpackage with zero reflection on the 20 Hz
 * evaluation loop.
 */
public interface BrakeModel {

    /** Host <-> model binary compatibility version supported by this APK. */
    int HOST_MODEL_API_VERSION = 1;

    /** Human-readable model release version (for example {@code "2026.11.18.1"}). */
    String modelVersion();

    /**
     * Strictly increasing monotonic sequence number (for example {@code 2026111801L}) used to
     * prevent replay/downgrade swaps and to retire older cached OTA plugins automatically when a
     * newer full APK is installed.
     */
    long modelSeq();

    /** Returns the {@link #HOST_MODEL_API_VERSION} targeted by this model implementation. */
    int modelApiVersion();

    /**
     * Evaluates a single poll tick using both typed {@link BrakeEvaluator.Input} fields and the
     * raw HMG-API {@code Vehicle} reading strings so future OTA models can inspect raw signal
     * formatting or extra readings without changing the host method signature.
     */
    BrakeEvaluator.Snapshot evaluate(
            IndicatorConfig config,
            BrakeEvaluator.Input input,
            Map<String, String> rawHmgReadings);

    /** Resets all internal longitudinal deceleration, hysteresis, and mode-inference state. */
    void reset();
}
