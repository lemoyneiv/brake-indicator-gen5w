package dev.lemoyne.brakeindicator;

import org.json.JSONObject;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.security.KeyFactory;
import java.security.MessageDigest;
import java.security.PrivateKey;
import java.security.PublicKey;
import java.security.Signature;
import java.security.spec.PKCS8EncodedKeySpec;
import java.security.spec.X509EncodedKeySpec;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import java.util.zip.ZipOutputStream;

/**
 * Pure-Java (Android 4.4.2 / API 19 + OpenJDK compatible) ECDSA P-256 ({@code SHA256withECDSA})
 * signer and verifier for downloadable OTA {@code brake-model-<ver>.jar} plugins.
 *
 * <p>Uses the exact same {@code secp256r1} author key &amp; version-8 SHA-256 UUID derivation as
 * HMG-API's {@code Signing.java}, pinning {@link #PINNED_AUTHOR_UUID} and
 * {@link #PINNED_AUTHOR_PUBKEY_B64} inside the APK so no untrusted or modified {@code classes.dex}
 * can ever be written to disk or passed to {@code DexClassLoader}.
 */
public final class ModelBundleVerifier {

    public static final String MODEL_BUNDLE_ID = "brake-indicator-model";
    public static final String META_DIR_PREFIX = "META-MODEL/";
    public static final String MANIFEST_ENTRY = "META-MODEL/manifest.json";
    public static final String SIGNATURE_ENTRY = "META-MODEL/signature";
    public static final String REQUIRED_MODEL_PKG_PREFIX = "dev.lemoyne.brakeindicator.model.";

    /** Author ECDSA P-256 identity matching the mod's HMG-API signing key (MOD_KEY). */
    public static final String PINNED_AUTHOR_UUID = "28ad71e0-07c0-8e61-8bf3-380cc71f9280";
    public static final String PINNED_AUTHOR_PUBKEY_B64 =
            "MFkwEwYHKoZIzj0CAQYIKoZIzj0DAQcDQgAEGtp1xjP0nZnDLZkeViOZC4TAuwR3"
            + "22bqowfNyxd6Xc9j6a6l+RBy2CNL6CPYJEaROBUm1CbToOKiSfudu5wQXg==";

    /** Strict size bounds (~16 KB typical bundle) to prevent zip bombs or RAM exhaustion. */
    public static final int MAX_BUNDLE_BYTES = 512 * 1024;
    public static final int MAX_ENTRY_BYTES = 384 * 1024;
    public static final int MAX_TOTAL_UNCOMPRESSED_BYTES = 1024 * 1024;
    public static final int MAX_ZIP_ENTRIES = 64;

    private static final long DETERMINISTIC_ZIP_TIME_MS = 1704067200000L; // 2024-01-01T00:00:00Z
    private static final char[] B64 =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/".toCharArray();

    private ModelBundleVerifier() {}

    public static final class VerifiedBundle {
        public final boolean ok;
        public final String modelVersion;
        public final long modelSeq;
        public final int minHostApi;
        public final int maxHostApi;
        public final String entryClass;
        public final String authorUuid;
        public final String bundleSha256;
        public final String dexSha256;
        public final String error;

        VerifiedBundle(
                boolean ok,
                String modelVersion,
                long modelSeq,
                int minHostApi,
                int maxHostApi,
                String entryClass,
                String authorUuid,
                String bundleSha256,
                String dexSha256,
                String error) {
            this.ok = ok;
            this.modelVersion = modelVersion;
            this.modelSeq = modelSeq;
            this.minHostApi = minHostApi;
            this.maxHostApi = maxHostApi;
            this.entryClass = entryClass;
            this.authorUuid = authorUuid;
            this.bundleSha256 = bundleSha256;
            this.dexSha256 = dexSha256;
            this.error = error;
        }

        static VerifiedBundle failure(String why) {
            return new VerifiedBundle(false, null, 0L, 0, 0, null, null, null, null, why);
        }
    }

    public static VerifiedBundle verifyBundle(byte[] jarBytes, int hostApiVersion) {
        return verifyBundle(jarBytes, hostApiVersion, PINNED_AUTHOR_UUID, PINNED_AUTHOR_PUBKEY_B64);
    }

    public static VerifiedBundle verifyBundle(
            byte[] jarBytes,
            int hostApiVersion,
            String expectedAuthorUuid,
            String expectedAuthorPubKeyB64) {
        if (jarBytes == null || jarBytes.length == 0) {
            return VerifiedBundle.failure("Empty model bundle.");
        }
        if (jarBytes.length > MAX_BUNDLE_BYTES) {
            return VerifiedBundle.failure("Model bundle exceeds " + MAX_BUNDLE_BYTES + " byte limit.");
        }

        Map<String, byte[]> entries = new LinkedHashMap<String, byte[]>();
        int totalUncompressed = 0;
        int entryCount = 0;

        try {
            ZipInputStream zis = new ZipInputStream(new ByteArrayInputStream(jarBytes));
            try {
                ZipEntry ze;
                byte[] buf = new byte[4096];
                while ((ze = zis.getNextEntry()) != null) {
                    entryCount++;
                    if (entryCount > MAX_ZIP_ENTRIES) {
                        return VerifiedBundle.failure("Too many ZIP entries in model bundle.");
                    }
                    String name = ze.getName();
                    if (!isSafeZipEntryName(name)) {
                        return VerifiedBundle.failure("Unsafe ZIP entry path: " + name);
                    }
                    if (ze.isDirectory()) {
                        continue;
                    }
                    if (entries.containsKey(name)) {
                        return VerifiedBundle.failure("Duplicate ZIP entry in model bundle: " + name);
                    }
                    ByteArrayOutputStream baos = new ByteArrayOutputStream(4096);
                    int n;
                    while ((n = zis.read(buf)) > 0) {
                        if (baos.size() + n > MAX_ENTRY_BYTES) {
                            return VerifiedBundle.failure("ZIP entry exceeds size cap: " + name);
                        }
                        totalUncompressed += n;
                        if (totalUncompressed > MAX_TOTAL_UNCOMPRESSED_BYTES) {
                            return VerifiedBundle.failure("Uncompressed model bundle exceeds safety cap.");
                        }
                        baos.write(buf, 0, n);
                    }
                    entries.put(name, baos.toByteArray());
                }
            } finally {
                zis.close();
            }
        } catch (Exception e) {
            return VerifiedBundle.failure("Corrupt model ZIP archive: " + e.getMessage());
        }

        byte[] manifestBytes = entries.get(MANIFEST_ENTRY);
        byte[] sigFileBytes = entries.get(SIGNATURE_ENTRY);
        if (manifestBytes == null || sigFileBytes == null) {
            return VerifiedBundle.failure("Unsigned model bundle (missing META-MODEL/manifest.json or signature).");
        }

        // Reject any unexpected file inside META-MODEL/
        for (String path : entries.keySet()) {
            if (path.startsWith(META_DIR_PREFIX)
                    && !MANIFEST_ENTRY.equals(path)
                    && !SIGNATURE_ENTRY.equals(path)) {
                return VerifiedBundle.failure("Unexpected metadata entry: " + path);
            }
        }

        try {
            JSONObject man = new JSONObject(new String(manifestBytes, "UTF-8"));
            int format = man.optInt("format", -1);
            if (format != 1) {
                return VerifiedBundle.failure("Unsupported model manifest format: " + format);
            }
            String id = man.optString("id", "");
            if (!MODEL_BUNDLE_ID.equals(id)) {
                return VerifiedBundle.failure("Unexpected model bundle id: " + id);
            }

            String modelVer = man.optString("model_version", "").trim();
            if (!isSafeModelVersion(modelVer)) {
                return VerifiedBundle.failure("Invalid model_version in manifest: " + modelVer);
            }
            long modelSeq = man.optLong("model_seq", -1L);
            if (modelSeq <= 0L) {
                return VerifiedBundle.failure("Invalid model_seq in manifest: " + modelSeq);
            }

            int minHostApi = man.optInt("min_host_api", 1);
            int maxHostApi = man.optInt("max_host_api", minHostApi);
            if (hostApiVersion < minHostApi || (maxHostApi > 0 && hostApiVersion > maxHostApi)) {
                return VerifiedBundle.failure("Model " + modelVer + " requires host API ["
                        + minHostApi + ".." + maxHostApi + "], host is " + hostApiVersion + ".");
            }

            String entryClass = man.optString("entry_class", "").trim();
            if (!isValidEntryClassName(entryClass)) {
                return VerifiedBundle.failure("Invalid entry_class (must reside in "
                        + REQUIRED_MODEL_PKG_PREFIX + "*): " + entryClass);
            }

            String keyB64 = man.optString("key", "").trim();
            String manifestUuid = man.optString("uuid", "").trim();
            if (keyB64.isEmpty() || manifestUuid.isEmpty()) {
                return VerifiedBundle.failure("Missing author public key or uuid in manifest.");
            }

            byte[] keyBytes = decodeBase64(keyB64);
            String derivedUuid = deriveAuthorUuid(keyBytes);
            if (!derivedUuid.equals(manifestUuid)) {
                return VerifiedBundle.failure("Manifest uuid does not match embedded public key.");
            }
            if (expectedAuthorUuid != null && !expectedAuthorUuid.equals(derivedUuid)) {
                return VerifiedBundle.failure("Untrusted model author uuid " + derivedUuid
                        + " (expected " + expectedAuthorUuid + ").");
            }
            if (expectedAuthorPubKeyB64 != null && !expectedAuthorPubKeyB64.isEmpty()) {
                byte[] pinnedKeyBytes = decodeBase64(expectedAuthorPubKeyB64);
                if (!Arrays.equals(pinnedKeyBytes, keyBytes)) {
                    return VerifiedBundle.failure("Public key mismatch against pinned MOD_KEY.");
                }
            }

            PublicKey pubKey = KeyFactory.getInstance("EC").generatePublic(new X509EncodedKeySpec(keyBytes));
            Signature sig = Signature.getInstance("SHA256withECDSA");
            sig.initVerify(pubKey);
            sig.update(manifestBytes);
            byte[] sigDer = decodeBase64(new String(sigFileBytes, "UTF-8").trim());
            if (sigDer.length == 0 || !sig.verify(sigDer)) {
                return VerifiedBundle.failure("ECDSA P-256 signature verification failed.");
            }

            JSONObject filesObj = man.optJSONObject("files");
            if (filesObj == null) {
                return VerifiedBundle.failure("Missing files digest map in manifest.");
            }

            // Every non-META-MODEL entry in the ZIP must be covered by manifest.files
            for (Map.Entry<String, byte[]> ze : entries.entrySet()) {
                String path = ze.getKey();
                if (path.startsWith(META_DIR_PREFIX)) {
                    continue;
                }
                String expectedSha = filesObj.optString(path, "");
                if (expectedSha.isEmpty()) {
                    return VerifiedBundle.failure("Unsigned extra file in model bundle: " + path);
                }
            }

            // Every entry declared in manifest.files must exist in the ZIP with matching SHA-256
            @SuppressWarnings("unchecked")
            Iterator<String> keys = filesObj.keys();
            int verifiedFileCount = 0;
            while (keys.hasNext()) {
                String path = keys.next();
                String expectedSha = filesObj.optString(path, "").toLowerCase(Locale.US);
                byte[] data = entries.get(path);
                if (data == null) {
                    return VerifiedBundle.failure("Missing signed file from bundle: " + path);
                }
                String actualSha = sha256Hex(data);
                if (!actualSha.equals(expectedSha)) {
                    return VerifiedBundle.failure("SHA-256 mismatch on signed file: " + path);
                }
                verifiedFileCount++;
            }
            if (verifiedFileCount == 0) {
                return VerifiedBundle.failure("Manifest files map is empty.");
            }

            byte[] dexBytes = entries.get("classes.dex");
            if (dexBytes == null || !hasDalvikDexMagic(dexBytes)) {
                return VerifiedBundle.failure("Bundle missing valid Dalvik classes.dex bytecode.");
            }

            String bundleSha = sha256Hex(jarBytes);
            String dexSha = filesObj.optString("classes.dex", "").toLowerCase(Locale.US);
            return new VerifiedBundle(
                    true,
                    modelVer,
                    modelSeq,
                    minHostApi,
                    maxHostApi,
                    entryClass,
                    derivedUuid,
                    bundleSha,
                    dexSha,
                    null);
        } catch (Exception e) {
            return VerifiedBundle.failure("Model bundle verification error: " + e.getMessage());
        }
    }

    /**
     * Builds a deterministic, ECDSA-signed {@code brake-model-<ver>.jar} in memory from payload
     * entries (must include {@code classes.dex}) and PKCS#8 / X.509 EC P-256 key bytes.
     */
    public static byte[] buildSignedModelJar(
            Map<String, byte[]> payloadEntries,
            String modelVersion,
            long modelSeq,
            int minHostApi,
            int maxHostApi,
            String entryClass,
            byte[] pkcs8PrivateKey,
            byte[] x509PublicKey) throws Exception {
        if (payloadEntries == null || !payloadEntries.containsKey("classes.dex")) {
            throw new IllegalArgumentException("payloadEntries must contain classes.dex");
        }
        TreeMap<String, byte[]> sortedPayload = new TreeMap<String, byte[]>(payloadEntries);
        JSONObject filesJson = new JSONObject();
        for (Map.Entry<String, byte[]> e : sortedPayload.entrySet()) {
            String path = e.getKey();
            if (path.startsWith(META_DIR_PREFIX) || !isSafeZipEntryName(path)) {
                throw new IllegalArgumentException("Invalid payload entry path: " + path);
            }
            filesJson.put(path, sha256Hex(e.getValue()));
        }

        String authorUuid = deriveAuthorUuid(x509PublicKey);
        JSONObject manifest = new JSONObject();
        manifest.put("format", 1);
        manifest.put("id", MODEL_BUNDLE_ID);
        manifest.put("model_version", modelVersion);
        manifest.put("model_seq", modelSeq);
        manifest.put("min_host_api", minHostApi);
        manifest.put("max_host_api", maxHostApi);
        manifest.put("entry_class", entryClass);
        manifest.put("uuid", authorUuid);
        manifest.put("key", encodeBase64(x509PublicKey));
        manifest.put("files", filesJson);

        byte[] manifestBytes = (manifest.toString(2) + "\n").getBytes("UTF-8");
        KeyFactory kf = KeyFactory.getInstance("EC");
        PrivateKey privKey = kf.generatePrivate(new PKCS8EncodedKeySpec(pkcs8PrivateKey));
        Signature signer = Signature.getInstance("SHA256withECDSA");
        signer.initSign(privKey);
        signer.update(manifestBytes);
        byte[] sigBytes = (encodeBase64(signer.sign()) + "\n").getBytes("UTF-8");

        ByteArrayOutputStream zipOut = new ByteArrayOutputStream(32768);
        ZipOutputStream zos = new ZipOutputStream(zipOut);
        try {
            for (Map.Entry<String, byte[]> e : sortedPayload.entrySet()) {
                writeZipEntry(zos, e.getKey(), e.getValue());
            }
            writeZipEntry(zos, MANIFEST_ENTRY, manifestBytes);
            writeZipEntry(zos, SIGNATURE_ENTRY, sigBytes);
        } finally {
            zos.close();
        }
        return zipOut.toByteArray();
    }

    /** Derives HMG-API's custom version-8 SHA-256 UUID from an X.509-encoded EC public key. */
    public static String deriveAuthorUuid(byte[] x509PublicKey) throws Exception {
        byte[] h = MessageDigest.getInstance("SHA-256").digest(x509PublicKey);
        h[6] = (byte) ((h[6] & 0x0f) | 0x80); // version 8
        h[8] = (byte) ((h[8] & 0x3f) | 0x80); // RFC 4122 variant
        StringBuilder sb = new StringBuilder(36);
        for (int i = 0; i < 16; i++) {
            if (i == 4 || i == 6 || i == 8 || i == 10) {
                sb.append('-');
            }
            sb.append(String.format(Locale.US, "%02x", Integer.valueOf(h[i] & 0xff)));
        }
        return sb.toString();
    }

    public static String sha256Hex(byte[] data) {
        try {
            MessageDigest md = MessageDigest.getInstance("SHA-256");
            byte[] dig = md.digest(data == null ? new byte[0] : data);
            StringBuilder sb = new StringBuilder(64);
            for (int i = 0; i < dig.length; i++) {
                sb.append(String.format(Locale.US, "%02x", Integer.valueOf(dig[i] & 0xff)));
            }
            return sb.toString();
        } catch (Exception e) {
            throw new IllegalStateException("SHA-256 unavailable", e);
        }
    }

    public static String encodeBase64(byte[] b) {
        if (b == null || b.length == 0) {
            return "";
        }
        StringBuilder sb = new StringBuilder(((b.length + 2) / 3) * 4);
        for (int i = 0; i < b.length; i += 3) {
            int n = ((b[i] & 0xff) << 16)
                    | (i + 1 < b.length ? ((b[i + 1] & 0xff) << 8) : 0)
                    | (i + 2 < b.length ? (b[i + 2] & 0xff) : 0);
            sb.append(B64[(n >>> 18) & 63]);
            sb.append(B64[(n >>> 12) & 63]);
            sb.append(i + 1 < b.length ? B64[(n >>> 6) & 63] : '=');
            sb.append(i + 2 < b.length ? B64[n & 63] : '=');
        }
        return sb.toString();
    }

    public static byte[] decodeBase64(String s) {
        if (s == null || s.isEmpty()) {
            return new byte[0];
        }
        ByteArrayOutputStream out = new ByteArrayOutputStream((s.length() * 3) / 4);
        int buf = 0;
        int bits = 0;
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            int v;
            if (c >= 'A' && c <= 'Z') v = c - 'A';
            else if (c >= 'a' && c <= 'z') v = c - 'a' + 26;
            else if (c >= '0' && c <= '9') v = c - '0' + 52;
            else if (c == '+') v = 62;
            else if (c == '/') v = 63;
            else continue;
            buf = (buf << 6) | v;
            bits += 6;
            if (bits >= 8) {
                bits -= 8;
                out.write((buf >>> bits) & 0xff);
            }
        }
        return out.toByteArray();
    }

    public static boolean isSafeModelVersion(String v) {
        if (v == null || v.isEmpty() || v.length() > 48) {
            return false;
        }
        if (v.startsWith(".") || v.endsWith(".") || v.contains("..")) {
            return false;
        }
        for (int i = 0; i < v.length(); i++) {
            char c = v.charAt(i);
            boolean ok = (c >= '0' && c <= '9')
                    || (c >= 'a' && c <= 'z')
                    || (c >= 'A' && c <= 'Z')
                    || c == '.' || c == '_' || c == '-';
            if (!ok) {
                return false;
            }
        }
        return true;
    }

    private static boolean isSafeZipEntryName(String name) {
        if (name == null || name.isEmpty() || name.length() > 180) {
            return false;
        }
        if (name.startsWith("/") || name.contains("\\") || name.contains("..")) {
            return false;
        }
        for (int i = 0; i < name.length(); i++) {
            char c = name.charAt(i);
            if (c < 0x20 || c >= 0x7f) {
                return false;
            }
        }
        return true;
    }

    private static boolean isValidEntryClassName(String cls) {
        if (cls == null || !cls.startsWith(REQUIRED_MODEL_PKG_PREFIX)) {
            return false;
        }
        String tail = cls.substring(REQUIRED_MODEL_PKG_PREFIX.length());
        if (tail.isEmpty() || tail.length() > 80) {
            return false;
        }
        for (int i = 0; i < tail.length(); i++) {
            char c = tail.charAt(i);
            boolean ok = (c >= 'a' && c <= 'z')
                    || (c >= 'A' && c <= 'Z')
                    || (c >= '0' && c <= '9')
                    || c == '_' || c == '$';
            if (!ok) {
                return false;
            }
        }
        return true;
    }

    private static boolean hasDalvikDexMagic(byte[] b) {
        // Dalvik DEX header starts with "dex\n035\0" (or 036..039)
        return b != null
                && b.length >= 40
                && b[0] == 'd'
                && b[1] == 'e'
                && b[2] == 'x'
                && b[3] == '\n';
    }

    private static void writeZipEntry(ZipOutputStream zos, String name, byte[] data) throws IOException {
        ZipEntry ze = new ZipEntry(name);
        ze.setTime(DETERMINISTIC_ZIP_TIME_MS);
        zos.putNextEntry(ze);
        zos.write(data);
        zos.closeEntry();
    }

    static byte[] readFileBytes(File f) throws IOException {
        InputStream in = new FileInputStream(f);
        try {
            ByteArrayOutputStream out = new ByteArrayOutputStream((int) Math.max(256L, f.length()));
            byte[] buf = new byte[8192];
            int n;
            while ((n = in.read(buf)) > 0) {
                out.write(buf, 0, n);
            }
            return out.toByteArray();
        } finally {
            in.close();
        }
    }

    private static void walkDir(File root, String relPrefix, Map<String, byte[]> out) throws IOException {
        File[] children = root.listFiles();
        if (children == null) {
            return;
        }
        List<File> sorted = new ArrayList<File>(Arrays.asList(children));
        Collections.sort(sorted);
        for (int i = 0; i < sorted.size(); i++) {
            File f = sorted.get(i);
            String rel = relPrefix.isEmpty() ? f.getName() : (relPrefix + "/" + f.getName());
            if (rel.startsWith("META-MODEL")) {
                continue;
            }
            if (f.isDirectory()) {
                walkDir(f, rel, out);
            } else if (f.isFile()) {
                out.put(rel, readFileBytes(f));
            }
        }
    }

    /**
     * Build-time CLI invoked by {@code ./build-model.sh} and {@code publish-model.yml}:
     * <ul>
     *   <li>{@code sign <payloadDir> <outJar> <otaModelJson> <entryClass> <authorKeyFile>}</li>
     *   <li>{@code verify <jarFile> [hostApiVersion]}</li>
     * </ul>
     */
    public static void main(String[] args) throws Exception {
        if (args.length >= 6 && "sign".equals(args[0])) {
            File payloadDir = new File(args[1]);
            File outJar = new File(args[2]);
            File metaJsonFile = new File(args[3]);
            String entryClass = args[4];
            File privKeyFile = new File(args[5]);
            File pubKeyFile = new File(privKeyFile.getPath() + ".pub");
            if (!pubKeyFile.isFile()) {
                throw new IOException("Missing public key file: " + pubKeyFile);
            }

            JSONObject cfg = new JSONObject(new String(readFileBytes(metaJsonFile), "UTF-8"));
            String modelVer = cfg.optString("model_version", "").trim();
            long modelSeq = cfg.optLong("model_seq", -1L);
            int minApi = cfg.optInt("min_host_api", 1);
            int maxApi = cfg.optInt("max_host_api", minApi);

            Map<String, byte[]> payload = new TreeMap<String, byte[]>();
            walkDir(payloadDir, "", payload);

            byte[] privBytes = decodeBase64(new String(readFileBytes(privKeyFile), "UTF-8").trim());
            byte[] pubBytes = decodeBase64(new String(readFileBytes(pubKeyFile), "UTF-8").trim());
            byte[] signedJar = buildSignedModelJar(
                    payload, modelVer, modelSeq, minApi, maxApi, entryClass, privBytes, pubBytes);

            // Immediate round-trip verification against pinned MOD_KEY UUID (if using production key)
            String derivedUuid = deriveAuthorUuid(pubBytes);
            String pubB64 = encodeBase64(pubBytes);
            VerifiedBundle check = verifyBundle(signedJar, minApi, derivedUuid, pubB64);
            if (!check.ok) {
                throw new IllegalStateException("Self-verification of signed model jar failed: " + check.error);
            }

            File parent = outJar.getParentFile();
            if (parent != null && !parent.exists()) {
                parent.mkdirs();
            }
            FileOutputStream fos = new FileOutputStream(outJar);
            try {
                fos.write(signedJar);
                fos.getFD().sync();
            } finally {
                fos.close();
            }
            System.out.println(check.bundleSha256);
            return;
        }

        if (args.length >= 2 && "verify".equals(args[0])) {
            File jarFile = new File(args[1]);
            int hostApi = args.length >= 3 ? Integer.parseInt(args[2]) : BrakeModel.HOST_MODEL_API_VERSION;
            VerifiedBundle res = verifyBundle(readFileBytes(jarFile), hostApi);
            if (res.ok) {
                System.out.println("ok: model " + res.modelVersion + " (seq=" + res.modelSeq
                        + ", uuid=" + res.authorUuid + ", sha256=" + res.bundleSha256 + ")");
                return;
            }
            System.err.println("FAILED: " + res.error);
            System.exit(1);
        }

        System.err.println("usage: ModelBundleVerifier sign <dir> <out.jar> <ota-model.json> <entryClass> <key>"
                + " | verify <jar> [hostApi]");
        System.exit(2);
    }
}
