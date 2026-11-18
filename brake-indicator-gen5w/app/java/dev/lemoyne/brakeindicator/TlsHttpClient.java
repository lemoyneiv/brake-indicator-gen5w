package dev.lemoyne.brakeindicator;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.math.BigInteger;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.security.GeneralSecurityException;
import java.security.MessageDigest;
import java.security.PublicKey;
import java.security.SecureRandom;
import java.security.Signature;
import java.security.cert.CertificateExpiredException;
import java.security.cert.CertificateFactory;
import java.security.cert.CertificateNotYetValidException;
import java.security.cert.X509Certificate;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import javax.crypto.Cipher;
import javax.crypto.Mac;
import javax.crypto.spec.SecretKeySpec;

/**
 * Self-contained pure-Java TLS 1.2 + HTTP/1.1 client engineered specifically for
 * Android 4.4.2 (API 19, KitKat) Gen5W head units.
 *
 * Why this exists:
 *  - Android 4.4.2's native {@code SSLSocket} (OpenSSL 1.0.1e) only exposes legacy SHA-1 CBC
 *    cipher suites ({@code TLS_ECDHE_*_CBC_SHA}); {@code AES_128_GCM_SHA256} and
 *    {@code AES_256_GCM_SHA384} were not added to Android's SSLSocket until API 20+.
 *  - Modern endpoints such as {@code api.github.com} reject TLS 1.0/1.1 and all SHA-1 CBC
 *    ciphers, requiring TLS 1.2+ with {@code ECDHE_{ECDSA,RSA}_WITH_AES_{128,256}_GCM_SHA{256,384}}
 *    and modern root CAs ({@link PinnedRoots}).
 *  - Stock US EV6 Gen5W firmware has no {@code curl} or {@code wget} binary.
 *
 * Implements RFC 5246 (TLS 1.2), RFC 4492 / RFC 8422 (ECDHE secp256r1 & secp384r1),
 * RFC 5288 / RFC 5289 (AES-GCM cipher suites over {@code AES/ECB/NoPadding} + NIST SP 800-38D
 * GHASH), and RFC 5280 / RFC 6125 certificate chain + SAN hostname verification.
 */
public final class TlsHttpClient {

    public static final int CIPHER_ECDHE_ECDSA_AES128_GCM_SHA256 = 0xC02B;
    public static final int CIPHER_ECDHE_RSA_AES128_GCM_SHA256   = 0xC02F;
    public static final int CIPHER_ECDHE_ECDSA_AES256_GCM_SHA384 = 0xC02C;
    public static final int CIPHER_ECDHE_RSA_AES256_GCM_SHA384   = 0xC030;

    private static final int CONTENT_CHANGE_CIPHER_SPEC = 20;
    private static final int CONTENT_ALERT              = 21;
    private static final int CONTENT_HANDSHAKE          = 22;
    private static final int CONTENT_APPLICATION_DATA   = 23;

    private static final int HS_CLIENT_HELLO       = 1;
    private static final int HS_SERVER_HELLO       = 2;
    private static final int HS_CERTIFICATE        = 11;
    private static final int HS_SERVER_KEY_EXCH    = 12;
    private static final int HS_CERTIFICATE_REQ    = 13;
    private static final int HS_SERVER_HELLO_DONE  = 14;
    private static final int HS_CLIENT_KEY_EXCH    = 16;
    private static final int HS_FINISHED           = 20;

    private static final int NAMED_CURVE_SECP256R1 = 0x0017;
    private static final int NAMED_CURVE_SECP384R1 = 0x0018;

    private static final int MAX_PLAINTEXT_CHUNK   = 8192;
    private static final int MAX_TLS_RECORD_LEN    = 18432;
    private static final int MAX_HTTP_BODY_BYTES   = 4 * 1024 * 1024;

    private static final SecureRandom RNG = new SecureRandom();

    public static final class HttpResponse {
        public final int statusCode;
        public final String statusMessage;
        public final Map<String, String> headers;
        public final byte[] body;
        public final String negotiatedCipher;
        public final String peerSubject;

        HttpResponse(
                int statusCode,
                String statusMessage,
                Map<String, String> headers,
                byte[] body,
                String negotiatedCipher,
                String peerSubject) {
            this.statusCode = statusCode;
            this.statusMessage = statusMessage == null ? "" : statusMessage;
            this.headers = headers;
            this.body = body == null ? new byte[0] : body;
            this.negotiatedCipher = negotiatedCipher == null ? "" : negotiatedCipher;
            this.peerSubject = peerSubject == null ? "" : peerSubject;
        }

        public String bodyUtf8() {
            try {
                return new String(body, "UTF-8");
            } catch (Exception e) {
                return "";
            }
        }
    }

    public static final class ProbeResult {
        public final boolean ok;
        public final String stage;
        public final String resolvedIp;
        public final long dnsMs;
        public final long tcpMs;
        public final long tlsMs;
        public final String negotiatedCipher;
        public final int httpStatus;
        public final String summary;

        ProbeResult(
                boolean ok,
                String stage,
                String resolvedIp,
                long dnsMs,
                long tcpMs,
                long tlsMs,
                String negotiatedCipher,
                int httpStatus,
                String summary) {
            this.ok = ok;
            this.stage = stage;
            this.resolvedIp = resolvedIp;
            this.dnsMs = dnsMs;
            this.tcpMs = tcpMs;
            this.tlsMs = tlsMs;
            this.negotiatedCipher = negotiatedCipher;
            this.httpStatus = httpStatus;
            this.summary = summary;
        }
    }

    private final int connectTimeoutMs;
    private final int readTimeoutMs;
    private final List<X509Certificate> extraTrustAnchors;
    private final boolean includeDefaultRoots;

    public TlsHttpClient() {
        this(10000, 25000, Collections.<X509Certificate>emptyList(), true);
    }

    public TlsHttpClient(
            int connectTimeoutMs,
            int readTimeoutMs,
            List<X509Certificate> extraTrustAnchors,
            boolean includeDefaultRoots) {
        this.connectTimeoutMs = Math.max(1000, connectTimeoutMs);
        this.readTimeoutMs = Math.max(2000, readTimeoutMs);
        this.extraTrustAnchors = extraTrustAnchors == null
                ? Collections.<X509Certificate>emptyList()
                : new ArrayList<X509Certificate>(extraTrustAnchors);
        this.includeDefaultRoots = includeDefaultRoots;
    }

    public static String cipherName(int suite) {
        switch (suite) {
            case CIPHER_ECDHE_ECDSA_AES128_GCM_SHA256:
                return "TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256";
            case CIPHER_ECDHE_RSA_AES128_GCM_SHA256:
                return "TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256";
            case CIPHER_ECDHE_ECDSA_AES256_GCM_SHA384:
                return "TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384";
            case CIPHER_ECDHE_RSA_AES256_GCM_SHA384:
                return "TLS_ECDHE_RSA_WITH_AES_256_GCM_SHA384";
            default:
                return String.format(Locale.US, "0x%04X", Integer.valueOf(suite));
        }
    }

    public HttpResponse execute(
            String method,
            String host,
            int port,
            String sniAndVerifyHost,
            String path,
            Map<String, String> headers,
            byte[] body) throws IOException {
        String verifyHost = (sniAndVerifyHost == null || sniAndVerifyHost.isEmpty()) ? host : sniAndVerifyHost;
        Socket raw = new Socket();
        TlsSession tls = null;
        try {
            raw.connect(new InetSocketAddress(host, port), connectTimeoutMs);
            raw.setSoTimeout(readTimeoutMs);
            raw.setTcpNoDelay(true);

            tls = handshake(raw, verifyHost);
            writeHttpRequest(tls, method, verifyHost, port, path, headers, body);
            return readHttpResponse(tls);
        } finally {
            if (tls != null) {
                tls.closeQuietly();
            } else {
                try {
                    raw.close();
                } catch (Throwable ignored) {}
            }
        }
    }

    public ProbeResult probe(
            String connectHost,
            int port,
            String verifyHost,
            String path,
            Map<String, String> headers) {
        long t0 = System.nanoTime();
        String ip;
        try {
            InetAddress addr = InetAddress.getByName(connectHost);
            ip = addr.getHostAddress();
        } catch (Exception e) {
            long dnsMs = elapsedMsSince(t0);
            return new ProbeResult(
                    false, "DNS", "", dnsMs, 0L, 0L, "", 0,
                    "DNS lookup failed for " + connectHost + " (" + cleanErr(e) + ")");
        }
        long dnsMs = elapsedMsSince(t0);

        Socket raw = new Socket();
        TlsSession tls = null;
        long tcpMs = 0L;
        long tlsMs = 0L;
        try {
            long tTcp = System.nanoTime();
            raw.connect(new InetSocketAddress(ip, port), connectTimeoutMs);
            raw.setSoTimeout(readTimeoutMs);
            raw.setTcpNoDelay(true);
            tcpMs = elapsedMsSince(tTcp);

            long tTls = System.nanoTime();
            tls = handshake(raw, verifyHost);
            tlsMs = elapsedMsSince(tTls);

            writeHttpRequest(tls, "GET", verifyHost, port, path, headers, null);
            HttpResponse resp = readHttpResponse(tls);
            String okMsg = String.format(
                    Locale.US,
                    "DNS %dms (%s) · TCP %dms · TLS 1.2 %dms (%s) · HTTP %d",
                    Long.valueOf(dnsMs), ip, Long.valueOf(tcpMs), Long.valueOf(tlsMs),
                    shortCipher(tls.cipherSuite), Integer.valueOf(resp.statusCode));
            return new ProbeResult(
                    resp.statusCode >= 200 && resp.statusCode < 300,
                    "HTTP",
                    ip,
                    dnsMs,
                    tcpMs,
                    tlsMs,
                    cipherName(tls.cipherSuite),
                    resp.statusCode,
                    okMsg);
        } catch (Exception e) {
            String stage = tcpMs == 0L ? "TCP" : (tlsMs == 0L ? "TLS" : "HTTP");
            String msg = stage + " failed (" + connectHost + ":" + port + "): " + cleanErr(e);
            return new ProbeResult(false, stage, ip, dnsMs, tcpMs, tlsMs, "", 0, msg);
        } finally {
            if (tls != null) {
                tls.closeQuietly();
            } else {
                try {
                    raw.close();
                } catch (Throwable ignored) {}
            }
        }
    }

    private static String shortCipher(int suite) {
        if (suite == CIPHER_ECDHE_ECDSA_AES128_GCM_SHA256) return "ECDHE-ECDSA-AES128-GCM";
        if (suite == CIPHER_ECDHE_RSA_AES128_GCM_SHA256) return "ECDHE-RSA-AES128-GCM";
        if (suite == CIPHER_ECDHE_ECDSA_AES256_GCM_SHA384) return "ECDHE-ECDSA-AES256-GCM";
        if (suite == CIPHER_ECDHE_RSA_AES256_GCM_SHA384) return "ECDHE-RSA-AES256-GCM";
        return cipherName(suite);
    }

    private static long elapsedMsSince(long startNs) {
        return Math.max(0L, (System.nanoTime() - startNs) / 1000000L);
    }

    private static String cleanErr(Throwable t) {
        if (t == null) return "error";
        String m = t.getMessage();
        return (m == null || m.trim().isEmpty()) ? t.getClass().getSimpleName() : m.trim();
    }

    // ===================== TLS 1.2 HANDSHAKE & RECORD LAYER =====================

    private TlsSession handshake(Socket socket, String verifyHost) throws IOException {
        InputStream in = socket.getInputStream();
        OutputStream out = socket.getOutputStream();
        ByteArrayOutputStream transcript = new ByteArrayOutputStream(4096);

        byte[] clientRandom = new byte[32];
        RNG.nextBytes(clientRandom);

        byte[] clientHelloMsg = buildClientHello(verifyHost, clientRandom);
        transcript.write(clientHelloMsg);
        writePlainRecord(out, CONTENT_HANDSHAKE, clientHelloMsg);
        out.flush();

        HandshakeReader hsReader = new HandshakeReader(in);

        // 1. ServerHello
        HandshakeMsg sh = hsReader.nextMessage();
        if (sh.type != HS_SERVER_HELLO) {
            throw new IOException("Expected ServerHello, got handshake type " + sh.type);
        }
        transcript.write(sh.rawWithHeader);
        ServerHelloParsed shp = parseServerHello(sh.body);

        boolean sha384 = shp.cipherSuite == CIPHER_ECDHE_ECDSA_AES256_GCM_SHA384
                || shp.cipherSuite == CIPHER_ECDHE_RSA_AES256_GCM_SHA384;
        String hashAlg = sha384 ? "SHA-384" : "SHA-256";
        String hmacAlg = sha384 ? "HmacSHA384" : "HmacSHA256";
        int keyBytes = sha384 ? 32 : 16;

        // 2. Certificate
        HandshakeMsg certMsg = hsReader.nextMessage();
        if (certMsg.type != HS_CERTIFICATE) {
            throw new IOException("Expected Certificate, got handshake type " + certMsg.type);
        }
        transcript.write(certMsg.rawWithHeader);
        List<X509Certificate> chain = parseCertificateList(certMsg.body);
        verifyCertificateChainAndHost(chain, verifyHost, getTrustAnchors());
        X509Certificate leafCert = chain.get(0);

        // 3. ServerKeyExchange
        HandshakeMsg skeMsg = hsReader.nextMessage();
        if (skeMsg.type != HS_SERVER_KEY_EXCH) {
            throw new IOException("Expected ServerKeyExchange, got handshake type " + skeMsg.type);
        }
        transcript.write(skeMsg.rawWithHeader);
        ServerKeyExchParsed ske = parseAndVerifyServerKeyExchange(
                skeMsg.body, clientRandom, shp.serverRandom, leafCert.getPublicKey());

        // 4. Optional CertificateRequest (if sent, we will decline / expect ServerHelloDone)
        HandshakeMsg nextMsg = hsReader.nextMessage();
        boolean requestedClientCert = false;
        if (nextMsg.type == HS_CERTIFICATE_REQ) {
            requestedClientCert = true;
            transcript.write(nextMsg.rawWithHeader);
            nextMsg = hsReader.nextMessage();
        }
        if (nextMsg.type != HS_SERVER_HELLO_DONE) {
            throw new IOException("Expected ServerHelloDone, got handshake type " + nextMsg.type);
        }
        transcript.write(nextMsg.rawWithHeader);

        if (requestedClientCert) {
            // Empty Certificate message (3 zero bytes length)
            byte[] emptyCert = buildHandshakeMessage(HS_CERTIFICATE, new byte[]{0, 0, 0});
            transcript.write(emptyCert);
            writePlainRecord(out, CONTENT_HANDSHAKE, emptyCert);
        }

        // 5. Ephemeral ECDH & ClientKeyExchange
        EcCurve curve = EcCurve.forNamedGroup(ske.namedCurve);
        EcdhResult ecdh = curve.computeEcdh(ske.serverPointUncompressed, RNG);

        byte[] ckeBody = new byte[1 + ecdh.clientPointUncompressed.length];
        ckeBody[0] = (byte) ecdh.clientPointUncompressed.length;
        System.arraycopy(
                ecdh.clientPointUncompressed, 0, ckeBody, 1, ecdh.clientPointUncompressed.length);
        byte[] ckeMsg = buildHandshakeMessage(HS_CLIENT_KEY_EXCH, ckeBody);
        transcript.write(ckeMsg);
        writePlainRecord(out, CONTENT_HANDSHAKE, ckeMsg);

        // 6. Derive Master Secret & AES-GCM Key Block (RFC 5246 §6.3, §8.1)
        byte[] randomClientServer = concat(clientRandom, shp.serverRandom);
        byte[] randomServerClient = concat(shp.serverRandom, clientRandom);
        byte[] masterSecret = tls12Prf(
                hmacAlg, ecdh.sharedSecretZ, "master secret", randomClientServer, 48);
        zeroize(ecdh.sharedSecretZ);

        // key_block = client_write_key + server_write_key + client_write_IV(4) + server_write_IV(4)
        byte[] keyBlock = tls12Prf(
                hmacAlg, masterSecret, "key expansion", randomServerClient, 2 * keyBytes + 8);
        byte[] clientWriteKey = slice(keyBlock, 0, keyBytes);
        byte[] serverWriteKey = slice(keyBlock, keyBytes, keyBytes);
        byte[] clientWriteIv  = slice(keyBlock, 2 * keyBytes, 4);
        byte[] serverWriteIv  = slice(keyBlock, 2 * keyBytes + 4, 4);
        zeroize(keyBlock);

        AesGcm clientGcm = new AesGcm(clientWriteKey);
        AesGcm serverGcm = new AesGcm(serverWriteKey);
        zeroize(clientWriteKey);
        zeroize(serverWriteKey);

        // 7. Client ChangeCipherSpec + Encrypted Client Finished
        writePlainRecord(out, CONTENT_CHANGE_CIPHER_SPEC, new byte[]{0x01});

        byte[] hsHashBeforeClientFin = digest(hashAlg, transcript.toByteArray());
        byte[] clientVerifyData = tls12Prf(
                hmacAlg, masterSecret, "client finished", hsHashBeforeClientFin, 12);
        byte[] clientFinishedPlain = buildHandshakeMessage(HS_FINISHED, clientVerifyData);
        transcript.write(clientFinishedPlain);

        TlsSession session = new TlsSession(
                socket, in, out, clientGcm, serverGcm, clientWriteIv, serverWriteIv,
                shp.cipherSuite, leafCert.getSubjectX500Principal().getName());
        session.writeEncryptedRecord(CONTENT_HANDSHAKE, clientFinishedPlain, 0, clientFinishedPlain.length);
        out.flush();

        // 8. Server ChangeCipherSpec + Encrypted Server Finished
        hsReader.expectChangeCipherSpec();
        RecordFrame serverFinRec = session.readEncryptedRecord();
        if (serverFinRec.type != CONTENT_HANDSHAKE) {
            throw new IOException("Expected encrypted Server Finished, got record type " + serverFinRec.type);
        }
        byte[] expectedServerVerify = tls12Prf(
                hmacAlg,
                masterSecret,
                "server finished",
                digest(hashAlg, transcript.toByteArray()),
                12);
        zeroize(masterSecret);

        if (serverFinRec.plaintext.length != 16
                || (serverFinRec.plaintext[0] & 0xFF) != HS_FINISHED
                || serverFinRec.plaintext[1] != 0
                || serverFinRec.plaintext[2] != 0
                || serverFinRec.plaintext[3] != 12) {
            throw new IOException("Malformed Server Finished message");
        }
        byte[] actualServerVerify = slice(serverFinRec.plaintext, 4, 12);
        if (!constantTimeEquals(expectedServerVerify, actualServerVerify)) {
            throw new IOException("Server Finished verify_data mismatch");
        }

        return session;
    }

    private List<X509Certificate> getTrustAnchors() {
        if (!includeDefaultRoots) {
            return extraTrustAnchors;
        }
        List<X509Certificate> defaults = PinnedRoots.trustedRoots();
        if (extraTrustAnchors.isEmpty()) {
            return defaults;
        }
        List<X509Certificate> merged = new ArrayList<X509Certificate>(
                defaults.size() + extraTrustAnchors.size());
        merged.addAll(extraTrustAnchors);
        merged.addAll(defaults);
        return merged;
    }

    // ===================== CLIENT/SERVER HELLO & EXTENSIONS =====================

    private static byte[] buildClientHello(String serverName, byte[] clientRandom) throws IOException {
        ByteArrayOutputStream body = new ByteArrayOutputStream(256);
        // client_version = TLS 1.2 (0x0303)
        body.write(0x03);
        body.write(0x03);
        body.write(clientRandom);
        // session_id length = 0
        body.write(0x00);

        // Cipher suites (4 AEAD GCM suites = 8 bytes)
        int[] suites = {
            CIPHER_ECDHE_ECDSA_AES128_GCM_SHA256,
            CIPHER_ECDHE_RSA_AES128_GCM_SHA256,
            CIPHER_ECDHE_ECDSA_AES256_GCM_SHA384,
            CIPHER_ECDHE_RSA_AES256_GCM_SHA384
        };
        writeU16(body, suites.length * 2);
        for (int i = 0; i < suites.length; i++) {
            writeU16(body, suites[i]);
        }

        // Compression methods: 1 method (null = 0x00)
        body.write(0x01);
        body.write(0x00);

        // Extensions
        ByteArrayOutputStream exts = new ByteArrayOutputStream(128);

        // 1. SNI (0x0000) — omit for raw IPv4/IPv6 literals per RFC 6066 §3
        if (serverName != null && !serverName.isEmpty() && !isIpLiteral(serverName)) {
            byte[] hostBytes = serverName.getBytes("US-ASCII");
            writeU16(exts, 0x0000);
            writeU16(exts, hostBytes.length + 5);
            writeU16(exts, hostBytes.length + 3);
            exts.write(0x00); // host_name
            writeU16(exts, hostBytes.length);
            exts.write(hostBytes);
        }

        // 2. supported_groups / elliptic_curves (0x000A): secp256r1 (0x0017), secp384r1 (0x0018)
        writeU16(exts, 0x000A);
        writeU16(exts, 6);
        writeU16(exts, 4);
        writeU16(exts, NAMED_CURVE_SECP256R1);
        writeU16(exts, NAMED_CURVE_SECP384R1);

        // 3. ec_point_formats (0x000B): uncompressed (0x00)
        writeU16(exts, 0x000B);
        writeU16(exts, 2);
        exts.write(0x01);
        exts.write(0x00);

        // 4. signature_algorithms (0x000D)
        int[] sigAlgs = {
            0x0403, // ecdsa_secp256r1_sha256
            0x0503, // ecdsa_secp384r1_sha384
            0x0603, // ecdsa_secp521r1_sha512
            0x0401, // rsa_pkcs1_sha256
            0x0501, // rsa_pkcs1_sha384
            0x0601  // rsa_pkcs1_sha512
        };
        writeU16(exts, 0x000D);
        writeU16(exts, 2 + sigAlgs.length * 2);
        writeU16(exts, sigAlgs.length * 2);
        for (int i = 0; i < sigAlgs.length; i++) {
            writeU16(exts, sigAlgs[i]);
        }

        // 5. renegotiation_info (0xFF01, empty = RFC 5746 secure initial handshake)
        writeU16(exts, 0xFF01);
        writeU16(exts, 1);
        exts.write(0x00);

        byte[] extBytes = exts.toByteArray();
        writeU16(body, extBytes.length);
        body.write(extBytes);

        return buildHandshakeMessage(HS_CLIENT_HELLO, body.toByteArray());
    }

    private static boolean isIpLiteral(String host) {
        if (host.indexOf(':') >= 0) {
            return true;
        }
        String[] parts = host.split("\\.");
        if (parts.length != 4) {
            return false;
        }
        for (int i = 0; i < 4; i++) {
            if (parts[i].isEmpty() || parts[i].length() > 3) return false;
            for (int j = 0; j < parts[i].length(); j++) {
                char c = parts[i].charAt(j);
                if (c < '0' || c > '9') return false;
            }
        }
        return true;
    }

    private static final class ServerHelloParsed {
        final byte[] serverRandom;
        final int cipherSuite;

        ServerHelloParsed(byte[] serverRandom, int cipherSuite) {
            this.serverRandom = serverRandom;
            this.cipherSuite = cipherSuite;
        }
    }

    private static ServerHelloParsed parseServerHello(byte[] b) throws IOException {
        if (b.length < 38) {
            throw new IOException("ServerHello too short");
        }
        int ver = readU16(b, 0);
        if (ver != 0x0303) {
            throw new IOException(String.format(Locale.US, "Unsupported TLS server version 0x%04X", Integer.valueOf(ver)));
        }
        byte[] serverRandom = slice(b, 2, 32);
        int sidLen = b[34] & 0xFF;
        int pos = 35 + sidLen;
        if (pos + 3 > b.length) {
            throw new IOException("Truncated ServerHello");
        }
        int suite = readU16(b, pos);
        int comp = b[pos + 2] & 0xFF;
        if (comp != 0) {
            throw new IOException("Unsupported TLS compression method " + comp);
        }
        if (suite != CIPHER_ECDHE_ECDSA_AES128_GCM_SHA256
                && suite != CIPHER_ECDHE_RSA_AES128_GCM_SHA256
                && suite != CIPHER_ECDHE_ECDSA_AES256_GCM_SHA384
                && suite != CIPHER_ECDHE_RSA_AES256_GCM_SHA384) {
            throw new IOException("Server selected unsupported cipher suite " + cipherName(suite));
        }
        return new ServerHelloParsed(serverRandom, suite);
    }

    // ===================== X.509 CERTIFICATE & HOSTNAME VERIFICATION =====================

    private static List<X509Certificate> parseCertificateList(byte[] b) throws IOException {
        if (b.length < 3) {
            throw new IOException("Empty Certificate message");
        }
        int totalLen = readU24(b, 0);
        if (3 + totalLen > b.length || totalLen == 0) {
            throw new IOException("Malformed Certificate list length");
        }
        List<X509Certificate> certs = new ArrayList<X509Certificate>(4);
        try {
            CertificateFactory cf = CertificateFactory.getInstance("X.509");
            int pos = 3;
            int end = 3 + totalLen;
            while (pos < end) {
                if (pos + 3 > end) {
                    throw new IOException("Truncated certificate entry");
                }
                int certLen = readU24(b, pos);
                pos += 3;
                if (certLen <= 0 || pos + certLen > end) {
                    throw new IOException("Invalid certificate entry length");
                }
                X509Certificate cert = (X509Certificate) cf.generateCertificate(
                        new ByteArrayInputStream(b, pos, certLen));
                certs.add(cert);
                pos += certLen;
            }
        } catch (IOException ioe) {
            throw ioe;
        } catch (Exception e) {
            throw new IOException("Failed to parse server X.509 certificate: " + cleanErr(e), e);
        }
        if (certs.isEmpty()) {
            throw new IOException("Server presented zero X.509 certificates");
        }
        return certs;
    }

    static void verifyCertificateChainAndHost(
            List<X509Certificate> chain,
            String expectedHost,
            List<X509Certificate> trustedRoots) throws IOException {
        if (chain == null || chain.isEmpty()) {
            throw new IOException("Empty certificate chain");
        }
        X509Certificate leaf = chain.get(0);
        checkCertValidityDate(leaf, "Leaf");

        if (!matchesHostname(leaf, expectedHost)) {
            throw new IOException("Certificate hostname mismatch for " + expectedHost
                    + " (subject: " + leaf.getSubjectX500Principal().getName() + ")");
        }

        // Walk up chain until signed by a trusted anchor (or leaf itself is a pinned anchor)
        X509Certificate current = leaf;
        for (int depth = 0; depth < 8; depth++) {
            if (isTrustedAnchor(current, trustedRoots)) {
                return;
            }
            if (isSignedByAnyAnchor(current, trustedRoots)) {
                return;
            }
            X509Certificate issuer = findIssuerInChain(current, chain);
            if (issuer == null || issuer.equals(current)) {
                throw new IOException("Untrusted certificate issuer: "
                        + current.getIssuerX500Principal().getName());
            }
            if (issuer.getBasicConstraints() < 0) {
                throw new IOException("Intermediate certificate is not a CA: "
                        + issuer.getSubjectX500Principal().getName());
            }
            checkCertValidityDate(issuer, "Intermediate CA");
            try {
                current.verify(issuer.getPublicKey());
            } catch (Exception e) {
                throw new IOException("Certificate signature verification failed at "
                        + current.getSubjectX500Principal().getName(), e);
            }
            current = issuer;
        }
        throw new IOException("Certificate chain exceeded maximum depth");
    }

    private static void checkCertValidityDate(X509Certificate cert, String role) throws IOException {
        try {
            cert.checkValidity();
        } catch (CertificateNotYetValidException e) {
            throw new IOException(role + " certificate not yet valid (check head-unit GPS/LTE clock)");
        } catch (CertificateExpiredException e) {
            throw new IOException(role + " certificate has expired");
        }
    }

    private static boolean isTrustedAnchor(X509Certificate cert, List<X509Certificate> roots) {
        for (int i = 0; i < roots.size(); i++) {
            if (cert.equals(roots.get(i))) {
                return true;
            }
        }
        return false;
    }

    private static boolean isSignedByAnyAnchor(X509Certificate cert, List<X509Certificate> roots) {
        for (int i = 0; i < roots.size(); i++) {
            X509Certificate root = roots.get(i);
            if (!cert.getIssuerX500Principal().equals(root.getSubjectX500Principal())) {
                continue;
            }
            try {
                cert.verify(root.getPublicKey());
                return true;
            } catch (Exception ignored) {
                // Continue searching in case multiple cross-signed roots share the subject DN
            }
        }
        return false;
    }

    private static X509Certificate findIssuerInChain(
            X509Certificate child,
            List<X509Certificate> chain) {
        for (int i = 0; i < chain.size(); i++) {
            X509Certificate cand = chain.get(i);
            if (cand == child) {
                continue;
            }
            if (child.getIssuerX500Principal().equals(cand.getSubjectX500Principal())) {
                return cand;
            }
        }
        return null;
    }

    static boolean matchesHostname(X509Certificate cert, String host) {
        if (host == null || host.isEmpty()) {
            return false;
        }
        String target = host.trim().toLowerCase(Locale.US);
        boolean ipTarget = isIpLiteral(target);
        boolean sawDnsSan = false;

        try {
            Collection<List<?>> sans = cert.getSubjectAlternativeNames();
            if (sans != null) {
                for (List<?> entry : sans) {
                    if (entry == null || entry.size() < 2) continue;
                    Object typeObj = entry.get(0);
                    Object valObj = entry.get(1);
                    if (!(typeObj instanceof Integer) || !(valObj instanceof String)) continue;
                    int type = ((Integer) typeObj).intValue();
                    String val = ((String) valObj).trim().toLowerCase(Locale.US);
                    if (type == 7 && ipTarget) {
                        if (target.equals(val)) return true;
                    } else if (type == 2) {
                        sawDnsSan = true;
                        if (!ipTarget && matchesDnsPattern(target, val)) {
                            return true;
                        }
                    }
                }
            }
        } catch (Exception ignored) {
        }

        if (sawDnsSan || ipTarget) {
            return false;
        }
        String cn = extractCommonName(cert.getSubjectX500Principal().getName());
        return cn != null && matchesDnsPattern(target, cn.toLowerCase(Locale.US));
    }

    static boolean matchesDnsPattern(String hostLower, String patternLower) {
        if (hostLower.equals(patternLower)) {
            return true;
        }
        if (patternLower.startsWith("*.") && patternLower.length() > 2) {
            String suffix = patternLower.substring(1); // ".github.com"
            // Require at least two labels in suffix (never match "*.com")
            if (suffix.indexOf('.', 1) < 0) {
                return false;
            }
            if (!hostLower.endsWith(suffix)) {
                return false;
            }
            String leftLabel = hostLower.substring(0, hostLower.length() - suffix.length());
            return !leftLabel.isEmpty() && leftLabel.indexOf('.') < 0;
        }
        return false;
    }

    private static String extractCommonName(String dn) {
        if (dn == null) return null;
        String[] parts = dn.split(",");
        for (int i = 0; i < parts.length; i++) {
            String p = parts[i].trim();
            if (p.regionMatches(true, 0, "CN=", 0, 3)) {
                return p.substring(3).trim();
            }
        }
        return null;
    }

    // ===================== SERVER KEY EXCHANGE & SIGNATURE =====================

    private static final class ServerKeyExchParsed {
        final int namedCurve;
        final byte[] serverPointUncompressed;

        ServerKeyExchParsed(int namedCurve, byte[] serverPointUncompressed) {
            this.namedCurve = namedCurve;
            this.serverPointUncompressed = serverPointUncompressed;
        }
    }

    private static ServerKeyExchParsed parseAndVerifyServerKeyExchange(
            byte[] b,
            byte[] clientRandom,
            byte[] serverRandom,
            PublicKey serverPubKey) throws IOException {
        if (b.length < 8) {
            throw new IOException("ServerKeyExchange too short");
        }
        int curveType = b[0] & 0xFF;
        if (curveType != 3) {
            throw new IOException("Expected named_curve (3), got " + curveType);
        }
        int namedCurve = readU16(b, 1);
        int ptLen = b[3] & 0xFF;
        int paramsLen = 4 + ptLen;
        if (ptLen < 33 || paramsLen + 4 > b.length) {
            throw new IOException("Malformed ServerECDHParams");
        }
        byte[] serverPoint = slice(b, 4, ptLen);
        int hashId = b[paramsLen] & 0xFF;
        int sigId = b[paramsLen + 1] & 0xFF;
        int sigLen = readU16(b, paramsLen + 2);
        if (paramsLen + 4 + sigLen != b.length) {
            throw new IOException("Malformed ServerKeyExchange signature length");
        }
        byte[] sigBytes = slice(b, paramsLen + 4, sigLen);

        String jcaSigAlg = jcaSignatureAlgorithm(hashId, sigId);
        try {
            Signature sig = Signature.getInstance(jcaSigAlg);
            sig.initVerify(serverPubKey);
            sig.update(clientRandom);
            sig.update(serverRandom);
            sig.update(b, 0, paramsLen);
            if (!sig.verify(sigBytes)) {
                throw new IOException("ServerKeyExchange cryptographic signature invalid");
            }
        } catch (IOException ioe) {
            throw ioe;
        } catch (GeneralSecurityException gse) {
            throw new IOException("ServerKeyExchange verification failed (" + jcaSigAlg + "): " + cleanErr(gse), gse);
        }

        return new ServerKeyExchParsed(namedCurve, serverPoint);
    }

    private static String jcaSignatureAlgorithm(int hashId, int sigId) throws IOException {
        String hashPart;
        switch (hashId) {
            case 2: hashPart = "SHA1"; break;
            case 4: hashPart = "SHA256"; break;
            case 5: hashPart = "SHA384"; break;
            case 6: hashPart = "SHA512"; break;
            default: throw new IOException("Unsupported TLS hash id " + hashId);
        }
        String sigPart;
        switch (sigId) {
            case 1: sigPart = "RSA"; break;
            case 3: sigPart = "ECDSA"; break;
            default: throw new IOException("Unsupported TLS signature id " + sigId);
        }
        return hashPart + "with" + sigPart;
    }

    // ===================== PURE-JAVA FIPS 186-4 ECDHE (P-256 & P-384) =====================

    static final class EcdhResult {
        final byte[] clientPointUncompressed;
        final byte[] sharedSecretZ;

        EcdhResult(byte[] clientPointUncompressed, byte[] sharedSecretZ) {
            this.clientPointUncompressed = clientPointUncompressed;
            this.sharedSecretZ = sharedSecretZ;
        }
    }

    static final class EcCurve {
        static final EcCurve SECP256R1 = new EcCurve(
                32,
                "FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF",
                "FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFC",
                "5AC635D8AA3A93E7B3EBBD55769886BC651D06B0CC53B0F63BCE3C3E27D2604B",
                "6B17D1F2E12C4247F8BCE6E563A440F277037D812DEB33A0F4A13945D898C296",
                "4FE342E2FE1A7F9B8EE7EB4A7C0F9E162BCE33576B315ECECBB6406837BF51F5",
                "FFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551");

        static final EcCurve SECP384R1 = new EcCurve(
                48,
                "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFF0000000000000000FFFFFFFF",
                "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFEFFFFFFFF0000000000000000FFFFFFFC",
                "B3312FA7E23EE7E4988E056BE3F82D19181D9C6EFE8141120314088F5013875AC656398D8A2ED19D2A85C8EDD3EC2AEF",
                "AA87CA22BE8B05378EB1C71EF320AD746E1D3B628BA79B9859F741E082542A385502F25DBF55296C3A545E3872760AB7",
                "3617DE4A96262C6F5D9E98BF9292DC29F8F41DBD289A147CE9DA3113B5F0B8C00A60B1CE1D7E819D7A431D7C90EA0E5F",
                "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC7634D81F4372DDF581A0DB248B0A77AECEC196ACCC52973");

        final int coordLen;
        final BigInteger p;
        final BigInteger a;
        final BigInteger b;
        final BigInteger gx;
        final BigInteger gy;
        final BigInteger n;

        private EcCurve(
                int coordLen,
                String pHex,
                String aHex,
                String bHex,
                String gxHex,
                String gyHex,
                String nHex) {
            this.coordLen = coordLen;
            this.p = new BigInteger(pHex, 16);
            this.a = new BigInteger(aHex, 16);
            this.b = new BigInteger(bHex, 16);
            this.gx = new BigInteger(gxHex, 16);
            this.gy = new BigInteger(gyHex, 16);
            this.n = new BigInteger(nHex, 16);
        }

        static EcCurve forNamedGroup(int id) throws IOException {
            if (id == NAMED_CURVE_SECP256R1) return SECP256R1;
            if (id == NAMED_CURVE_SECP384R1) return SECP384R1;
            throw new IOException(String.format(Locale.US, "Unsupported ECDHE named curve 0x%04X", Integer.valueOf(id)));
        }

        boolean isOnCurve(BigInteger x, BigInteger y) {
            if (x == null || y == null) return false;
            if (x.signum() < 0 || x.compareTo(p) >= 0 || y.signum() < 0 || y.compareTo(p) >= 0) {
                return false;
            }
            BigInteger lhs = y.multiply(y).mod(p);
            BigInteger rhs = x.modPow(BigInteger.valueOf(3L), p)
                    .add(a.multiply(x))
                    .add(b)
                    .mod(p);
            return lhs.equals(rhs);
        }

        EcdhResult computeEcdh(byte[] peerUncompressed, SecureRandom rng) throws IOException {
            if (peerUncompressed == null
                    || peerUncompressed.length != 1 + 2 * coordLen
                    || (peerUncompressed[0] & 0xFF) != 0x04) {
                throw new IOException("Invalid uncompressed EC point from server");
            }
            BigInteger px = new BigInteger(1, slice(peerUncompressed, 1, coordLen));
            BigInteger py = new BigInteger(1, slice(peerUncompressed, 1 + coordLen, coordLen));
            if (!isOnCurve(px, py)) {
                throw new IOException("Server ECDHE public point is not on curve");
            }

            BigInteger d;
            byte[] rawD = new byte[coordLen];
            do {
                rng.nextBytes(rawD);
                d = new BigInteger(1, rawD).mod(n);
            } while (d.compareTo(BigInteger.ONE) <= 0 || d.compareTo(n) >= 0);

            BigInteger[] clientPub = scalarMulAffine(d, gx, gy);
            BigInteger[] sharedPt = scalarMulAffine(d, px, py);
            if (clientPub == null || sharedPt == null) {
                throw new IOException("Invalid ECDHE point at infinity");
            }

            byte[] pubEncoded = new byte[1 + 2 * coordLen];
            pubEncoded[0] = 0x04;
            toFixedUnsigned(clientPub[0], pubEncoded, 1, coordLen);
            toFixedUnsigned(clientPub[1], pubEncoded, 1 + coordLen, coordLen);

            // RFC 4492 §5.10: shared secret Z preserves all leading zero octets (exact coordLen)
            byte[] secretZ = new byte[coordLen];
            toFixedUnsigned(sharedPt[0], secretZ, 0, coordLen);
            return new EcdhResult(pubEncoded, secretZ);
        }

        /** Returns affine {x, y} = k * (px, py), or null if point at infinity. */
        BigInteger[] scalarMulAffine(BigInteger k, BigInteger px, BigInteger py) {
            if (k.signum() == 0) {
                return null;
            }
            // Jacobian coordinates (X : Y : Z) representing affine (X/Z^2, Y/Z^3)
            BigInteger rx = BigInteger.ONE;
            BigInteger ry = BigInteger.ONE;
            BigInteger rz = BigInteger.ZERO; // Infinity when Z == 0

            int bits = k.bitLength();
            for (int i = bits - 1; i >= 0; i--) {
                BigInteger[] dbl = jacobianDouble(rx, ry, rz);
                rx = dbl[0];
                ry = dbl[1];
                rz = dbl[2];
                if (k.testBit(i)) {
                    BigInteger[] add = jacobianAddAffine(rx, ry, rz, px, py);
                    rx = add[0];
                    ry = add[1];
                    rz = add[2];
                }
            }
            if (rz.signum() == 0) {
                return null;
            }
            BigInteger zInv = rz.modInverse(p);
            BigInteger zInv2 = zInv.multiply(zInv).mod(p);
            BigInteger zInv3 = zInv2.multiply(zInv).mod(p);
            return new BigInteger[]{
                rx.multiply(zInv2).mod(p),
                ry.multiply(zInv3).mod(p)
            };
        }

        private BigInteger[] jacobianDouble(BigInteger x1, BigInteger y1, BigInteger z1) {
            if (z1.signum() == 0 || y1.signum() == 0) {
                return new BigInteger[]{BigInteger.ONE, BigInteger.ONE, BigInteger.ZERO};
            }
            // Note: both P-256 and P-384 have a = -3 mod p
            BigInteger z2 = z1.multiply(z1).mod(p);
            BigInteger m = BigInteger.valueOf(3L)
                    .multiply(x1.subtract(z2))
                    .multiply(x1.add(z2))
                    .mod(p);
            BigInteger y2 = y1.multiply(y1).mod(p);
            BigInteger s = BigInteger.valueOf(4L).multiply(x1).multiply(y2).mod(p);
            BigInteger x3 = m.multiply(m).subtract(s.shiftLeft(1)).mod(p);
            BigInteger y4 = y2.multiply(y2).mod(p);
            BigInteger y3 = m.multiply(s.subtract(x3)).subtract(y4.shiftLeft(3)).mod(p);
            BigInteger z3 = y1.multiply(z1).shiftLeft(1).mod(p);
            return new BigInteger[]{x3, y3, z3};
        }

        private BigInteger[] jacobianAddAffine(
                BigInteger x1, BigInteger y1, BigInteger z1,
                BigInteger x2, BigInteger y2) {
            if (z1.signum() == 0) {
                return new BigInteger[]{x2, y2, BigInteger.ONE};
            }
            BigInteger z1z1 = z1.multiply(z1).mod(p);
            BigInteger u2 = x2.multiply(z1z1).mod(p);
            BigInteger s2 = y2.multiply(z1z1).multiply(z1).mod(p);
            BigInteger h = u2.subtract(x1).mod(p);
            BigInteger r = s2.subtract(y1).shiftLeft(1).mod(p);
            if (h.signum() == 0) {
                if (r.signum() == 0) {
                    return jacobianDouble(x1, y1, z1);
                }
                return new BigInteger[]{BigInteger.ONE, BigInteger.ONE, BigInteger.ZERO};
            }
            BigInteger i = h.shiftLeft(1);
            i = i.multiply(i).mod(p);
            BigInteger j = h.multiply(i).mod(p);
            BigInteger v = x1.multiply(i).mod(p);
            BigInteger x3 = r.multiply(r).subtract(j).subtract(v.shiftLeft(1)).mod(p);
            BigInteger y3 = r.multiply(v.subtract(x3)).subtract(y1.multiply(j).shiftLeft(1)).mod(p);
            BigInteger z3 = z1.add(h).pow(2).subtract(z1z1).subtract(h.multiply(h)).mod(p);
            return new BigInteger[]{x3, y3, z3};
        }

        private static void toFixedUnsigned(BigInteger val, byte[] dst, int dstOff, int len) {
            byte[] raw = val.toByteArray();
            int srcOff = (raw.length > 1 && raw[0] == 0) ? 1 : 0;
            int copyLen = Math.min(len, raw.length - srcOff);
            for (int i = 0; i < len; i++) {
                dst[dstOff + i] = 0;
            }
            System.arraycopy(raw, raw.length - copyLen, dst, dstOff + len - copyLen, copyLen);
        }
    }

    // ===================== PURE-JAVA AES-GCM (NIST SP 800-38D) =====================

    static final class AesGcm {
        private final Cipher aesEcb;
        private final long hHi;
        private final long hLo;

        AesGcm(byte[] key) throws IOException {
            try {
                this.aesEcb = Cipher.getInstance("AES/ECB/NoPadding");
                this.aesEcb.init(Cipher.ENCRYPT_MODE, new SecretKeySpec(key, "AES"));
                byte[] hBytes = aesBlock(new byte[16]);
                this.hHi = getLongBE(hBytes, 0);
                this.hLo = getLongBE(hBytes, 8);
            } catch (GeneralSecurityException e) {
                throw new IOException("AES-GCM init failed: " + cleanErr(e), e);
            }
        }

        private synchronized byte[] aesBlock(byte[] block16) throws IOException {
            try {
                return aesEcb.doFinal(block16);
            } catch (GeneralSecurityException e) {
                throw new IOException("AES block cipher failure", e);
            }
        }

        /** Returns ciphertext || 16-byte tag. */
        byte[] seal(byte[] nonce12, byte[] aad, byte[] pt, int ptOff, int ptLen) throws IOException {
            if (nonce12.length != 12) {
                throw new IllegalArgumentException("GCM requires 12-byte nonce");
            }
            byte[] out = new byte[ptLen + 16];
            byte[] j0 = new byte[16];
            System.arraycopy(nonce12, 0, j0, 0, 12);
            j0[15] = 1;

            ctrCrypt(j0, pt, ptOff, ptLen, out, 0);
            byte[] tag = computeTag(j0, aad, out, 0, ptLen);
            System.arraycopy(tag, 0, out, ptLen, 16);
            return out;
        }

        /** Authenticates tag in constant time before returning plaintext. */
        byte[] open(byte[] nonce12, byte[] aad, byte[] ctAndTag, int off, int totalLen) throws IOException {
            if (nonce12.length != 12 || totalLen < 16) {
                throw new IOException("Truncated AES-GCM ciphertext");
            }
            int ctLen = totalLen - 16;
            byte[] j0 = new byte[16];
            System.arraycopy(nonce12, 0, j0, 0, 12);
            j0[15] = 1;

            byte[] expectedTag = computeTag(j0, aad, ctAndTag, off, ctLen);
            int diff = 0;
            for (int i = 0; i < 16; i++) {
                diff |= (expectedTag[i] ^ ctAndTag[off + ctLen + i]) & 0xFF;
            }
            if (diff != 0) {
                throw new IOException("TLS AES-GCM authentication tag mismatch (bad_record_mac)");
            }

            byte[] pt = new byte[ctLen];
            ctrCrypt(j0, ctAndTag, off, ctLen, pt, 0);
            return pt;
        }

        private void ctrCrypt(
                byte[] j0, byte[] src, int srcOff, int len, byte[] dst, int dstOff) throws IOException {
            byte[] cb = new byte[16];
            System.arraycopy(j0, 0, cb, 0, 16);
            int counter = 1;
            int pos = 0;
            while (pos < len) {
                counter++;
                putIntBE(cb, 12, counter);
                byte[] ks = aesBlock(cb);
                int chunk = Math.min(16, len - pos);
                for (int i = 0; i < chunk; i++) {
                    dst[dstOff + pos + i] = (byte) (src[srcOff + pos + i] ^ ks[i]);
                }
                pos += chunk;
            }
        }

        private byte[] computeTag(
                byte[] j0, byte[] aad, byte[] ct, int ctOff, int ctLen) throws IOException {
            long xHi = 0L;
            long xLo = 0L;
            byte[] pad = new byte[16];

            int aadLen = aad == null ? 0 : aad.length;
            int pos = 0;
            while (pos + 16 <= aadLen) {
                xHi ^= getLongBE(aad, pos);
                xLo ^= getLongBE(aad, pos + 8);
                long[] prod = gfMul(xHi, xLo, hHi, hLo);
                xHi = prod[0];
                xLo = prod[1];
                pos += 16;
            }
            if (pos < aadLen) {
                zeroize(pad);
                System.arraycopy(aad, pos, pad, 0, aadLen - pos);
                xHi ^= getLongBE(pad, 0);
                xLo ^= getLongBE(pad, 8);
                long[] prod = gfMul(xHi, xLo, hHi, hLo);
                xHi = prod[0];
                xLo = prod[1];
            }

            pos = 0;
            while (pos + 16 <= ctLen) {
                xHi ^= getLongBE(ct, ctOff + pos);
                xLo ^= getLongBE(ct, ctOff + pos + 8);
                long[] prod = gfMul(xHi, xLo, hHi, hLo);
                xHi = prod[0];
                xLo = prod[1];
                pos += 16;
            }
            if (pos < ctLen) {
                zeroize(pad);
                System.arraycopy(ct, ctOff + pos, pad, 0, ctLen - pos);
                xHi ^= getLongBE(pad, 0);
                xLo ^= getLongBE(pad, 8);
                long[] prod = gfMul(xHi, xLo, hHi, hLo);
                xHi = prod[0];
                xLo = prod[1];
            }

            xHi ^= ((long) aadLen) * 8L;
            xLo ^= ((long) ctLen) * 8L;
            long[] prod = gfMul(xHi, xLo, hHi, hLo);
            xHi = prod[0];
            xLo = prod[1];

            byte[] s = aesBlock(j0);
            putLongBE(pad, 0, xHi ^ getLongBE(s, 0));
            putLongBE(pad, 8, xLo ^ getLongBE(s, 8));
            return pad;
        }

        private static long[] gfMul(long xHi, long xLo, long vHi, long vLo) {
            long zHi = 0L;
            long zLo = 0L;
            for (int i = 0; i < 128; i++) {
                long bit = (i < 64) ? ((xHi >>> (63 - i)) & 1L) : ((xLo >>> (127 - i)) & 1L);
                long mask = -bit;
                zHi ^= (vHi & mask);
                zLo ^= (vLo & mask);

                long lsb = vLo & 1L;
                vLo = (vLo >>> 1) | (vHi << 63);
                vHi = (vHi >>> 1) ^ (0xe100000000000000L & (-lsb));
            }
            return new long[]{zHi, zLo};
        }
    }

    // ===================== TLS 1.2 PRF & RECORD STREAMING =====================

    static byte[] tls12Prf(String hmacAlg, byte[] secret, String label, byte[] seed, int outLen)
            throws IOException {
        try {
            byte[] labelBytes = label.getBytes("US-ASCII");
            byte[] ls = concat(labelBytes, seed);
            Mac mac = Mac.getInstance(hmacAlg);
            SecretKeySpec keySpec = new SecretKeySpec(secret, hmacAlg);
            mac.init(keySpec);

            byte[] out = new byte[outLen];
            byte[] a = ls;
            int filled = 0;
            while (filled < outLen) {
                a = mac.doFinal(a);
                mac.update(a);
                mac.update(ls);
                byte[] block = mac.doFinal();
                int take = Math.min(block.length, outLen - filled);
                System.arraycopy(block, 0, out, filled, take);
                filled += take;
            }
            return out;
        } catch (GeneralSecurityException e) {
            throw new IOException("TLS 1.2 PRF failure", e);
        }
    }

    private static byte[] digest(String alg, byte[] data) throws IOException {
        try {
            return MessageDigest.getInstance(alg).digest(data);
        } catch (GeneralSecurityException e) {
            throw new IOException("Digest failure: " + alg, e);
        }
    }

    private static final class HandshakeMsg {
        final int type;
        final byte[] body;
        final byte[] rawWithHeader;

        HandshakeMsg(int type, byte[] body, byte[] rawWithHeader) {
            this.type = type;
            this.body = body;
            this.rawWithHeader = rawWithHeader;
        }
    }

    private static final class HandshakeReader {
        private final InputStream in;
        private byte[] buf = new byte[0];
        private int off = 0;

        HandshakeReader(InputStream in) {
            this.in = in;
        }

        HandshakeMsg nextMessage() throws IOException {
            while (true) {
                int avail = buf.length - off;
                if (avail >= 4) {
                    int msgType = buf[off] & 0xFF;
                    int msgLen = readU24(buf, off + 1);
                    if (msgLen > 262144) {
                        throw new IOException("Oversized TLS handshake message: " + msgLen);
                    }
                    if (avail >= 4 + msgLen) {
                        byte[] raw = slice(buf, off, 4 + msgLen);
                        byte[] body = slice(buf, off + 4, msgLen);
                        off += 4 + msgLen;
                        if (off == buf.length) {
                            buf = new byte[0];
                            off = 0;
                        }
                        return new HandshakeMsg(msgType, body, raw);
                    }
                }
                byte[] nextRec = readNextHandshakeRecord();
                byte[] combined = new byte[avail + nextRec.length];
                if (avail > 0) {
                    System.arraycopy(buf, off, combined, 0, avail);
                }
                System.arraycopy(nextRec, 0, combined, avail, nextRec.length);
                buf = combined;
                off = 0;
            }
        }

        void expectChangeCipherSpec() throws IOException {
            if (off != buf.length) {
                throw new IOException("Unexpected trailing handshake bytes before ChangeCipherSpec");
            }
            byte[] hdr = readExact(in, 5);
            int type = hdr[0] & 0xFF;
            int len = readU16(hdr, 3);
            if (len > MAX_TLS_RECORD_LEN) {
                throw new IOException("Oversized TLS record: " + len);
            }
            byte[] payload = readExact(in, len);
            if (type == CONTENT_ALERT) {
                throw alertException(payload);
            }
            if (type != CONTENT_CHANGE_CIPHER_SPEC || len != 1 || payload[0] != 1) {
                throw new IOException("Expected ChangeCipherSpec, got type=" + type + " len=" + len);
            }
        }

        private byte[] readNextHandshakeRecord() throws IOException {
            while (true) {
                byte[] hdr = readExact(in, 5);
                int type = hdr[0] & 0xFF;
                int len = readU16(hdr, 3);
                if (len <= 0 || len > MAX_TLS_RECORD_LEN) {
                    throw new IOException("Invalid TLS handshake record length: " + len);
                }
                byte[] payload = readExact(in, len);
                if (type == CONTENT_ALERT) {
                    throw alertException(payload);
                }
                if (type != CONTENT_HANDSHAKE) {
                    throw new IOException("Expected TLS Handshake record, got type " + type);
                }
                return payload;
            }
        }
    }

    private static final class RecordFrame {
        final int type;
        final byte[] plaintext;

        RecordFrame(int type, byte[] plaintext) {
            this.type = type;
            this.plaintext = plaintext;
        }
    }

    private static final class TlsSession {
        final Socket socket;
        final InputStream in;
        final OutputStream out;
        final AesGcm clientGcm;
        final AesGcm serverGcm;
        final byte[] clientFixedIv;
        final byte[] serverFixedIv;
        final int cipherSuite;
        final String peerSubject;
        long clientSeq = 0L;
        long serverSeq = 0L;
        boolean receivedCloseNotify = false;

        TlsSession(
                Socket socket,
                InputStream in,
                OutputStream out,
                AesGcm clientGcm,
                AesGcm serverGcm,
                byte[] clientFixedIv,
                byte[] serverFixedIv,
                int cipherSuite,
                String peerSubject) {
            this.socket = socket;
            this.in = in;
            this.out = out;
            this.clientGcm = clientGcm;
            this.serverGcm = serverGcm;
            this.clientFixedIv = clientFixedIv;
            this.serverFixedIv = serverFixedIv;
            this.cipherSuite = cipherSuite;
            this.peerSubject = peerSubject;
        }

        void writeAppData(byte[] data, int off, int len) throws IOException {
            int pos = 0;
            while (pos < len) {
                int chunk = Math.min(MAX_PLAINTEXT_CHUNK, len - pos);
                writeEncryptedRecord(CONTENT_APPLICATION_DATA, data, off + pos, chunk);
                pos += chunk;
            }
        }

        void writeEncryptedRecord(int contentType, byte[] pt, int ptOff, int ptLen) throws IOException {
            long seq = clientSeq++;
            byte[] nonce = new byte[12];
            System.arraycopy(clientFixedIv, 0, nonce, 0, 4);
            putLongBE(nonce, 4, seq);

            byte[] aad = buildTlsAad(seq, contentType, ptLen);
            byte[] ctAndTag = clientGcm.seal(nonce, aad, pt, ptOff, ptLen);

            int recLen = 8 + ctAndTag.length;
            byte[] hdr = new byte[13];
            hdr[0] = (byte) contentType;
            hdr[1] = 0x03;
            hdr[2] = 0x03;
            hdr[3] = (byte) ((recLen >>> 8) & 0xFF);
            hdr[4] = (byte) (recLen & 0xFF);
            putLongBE(hdr, 5, seq); // explicit_nonce
            out.write(hdr);
            out.write(ctAndTag);
        }

        RecordFrame readEncryptedRecord() throws IOException {
            int b0 = in.read();
            if (b0 < 0) {
                receivedCloseNotify = true;
                return null;
            }
            byte[] hdrRest = readExact(in, 4);
            int type = b0 & 0xFF;
            int recLen = ((hdrRest[2] & 0xFF) << 8) | (hdrRest[3] & 0xFF);
            if (recLen < 24 || recLen > MAX_TLS_RECORD_LEN) {
                throw new IOException("Invalid encrypted TLS record length: " + recLen);
            }
            byte[] recPayload = readExact(in, recLen);

            long seq = serverSeq++;
            byte[] nonce = new byte[12];
            System.arraycopy(serverFixedIv, 0, nonce, 0, 4);
            System.arraycopy(recPayload, 0, nonce, 4, 8); // server explicit_nonce

            int ptLen = recLen - 24;
            byte[] aad = buildTlsAad(seq, type, ptLen);
            byte[] pt = serverGcm.open(nonce, aad, recPayload, 8, recLen - 8);
            if (type == CONTENT_ALERT) {
                if (pt.length >= 2 && (pt[1] & 0xFF) == 0) {
                    receivedCloseNotify = true;
                    return null;
                }
                throw alertException(pt);
            }
            return new RecordFrame(type, pt);
        }

        InputStream asAppInputStream() {
            return new InputStream() {
                private byte[] cur = new byte[0];
                private int pos = 0;

                @Override
                public int read() throws IOException {
                    byte[] one = new byte[1];
                    int n = read(one, 0, 1);
                    return n <= 0 ? -1 : (one[0] & 0xFF);
                }

                @Override
                public int read(byte[] dst, int dstOff, int dstLen) throws IOException {
                    if (dstLen == 0) return 0;
                    while (pos >= cur.length) {
                        if (receivedCloseNotify) return -1;
                        RecordFrame f = readEncryptedRecord();
                        if (f == null) return -1;
                        if (f.type != CONTENT_APPLICATION_DATA) continue;
                        cur = f.plaintext;
                        pos = 0;
                    }
                    int take = Math.min(dstLen, cur.length - pos);
                    System.arraycopy(cur, pos, dst, dstOff, take);
                    pos += take;
                    return take;
                }
            };
        }

        void closeQuietly() {
            try {
                writeEncryptedRecord(CONTENT_ALERT, new byte[]{0x01, 0x00}, 0, 2);
                out.flush();
            } catch (Throwable ignored) {}
            try {
                socket.close();
            } catch (Throwable ignored) {}
        }
    }

    private static byte[] buildTlsAad(long seqNum, int contentType, int plaintextLen) {
        byte[] aad = new byte[13];
        putLongBE(aad, 0, seqNum);
        aad[8] = (byte) contentType;
        aad[9] = 0x03;
        aad[10] = 0x03;
        aad[11] = (byte) ((plaintextLen >>> 8) & 0xFF);
        aad[12] = (byte) (plaintextLen & 0xFF);
        return aad;
    }

    private static IOException alertException(byte[] payload) {
        int level = payload.length > 0 ? (payload[0] & 0xFF) : -1;
        int desc = payload.length > 1 ? (payload[1] & 0xFF) : -1;
        return new IOException("TLS alert from server (level=" + level + ", desc=" + desc + " " + alertName(desc) + ")");
    }

    private static String alertName(int desc) {
        switch (desc) {
            case 20: return "bad_record_mac";
            case 40: return "handshake_failure";
            case 42: return "bad_certificate";
            case 48: return "unknown_ca";
            case 70: return "protocol_version";
            case 71: return "insufficient_security";
            case 80: return "internal_error";
            case 112: return "unrecognized_name";
            default: return "";
        }
    }

    private static byte[] buildHandshakeMessage(int hsType, byte[] body) {
        byte[] msg = new byte[4 + body.length];
        msg[0] = (byte) hsType;
        msg[1] = (byte) ((body.length >>> 16) & 0xFF);
        msg[2] = (byte) ((body.length >>> 8) & 0xFF);
        msg[3] = (byte) (body.length & 0xFF);
        System.arraycopy(body, 0, msg, 4, body.length);
        return msg;
    }

    private static void writePlainRecord(OutputStream out, int contentType, byte[] payload)
            throws IOException {
        byte[] hdr = new byte[5];
        hdr[0] = (byte) contentType;
        hdr[1] = 0x03;
        hdr[2] = 0x03;
        hdr[3] = (byte) ((payload.length >>> 8) & 0xFF);
        hdr[4] = (byte) (payload.length & 0xFF);
        out.write(hdr);
        out.write(payload);
    }

    // ===================== HTTP/1.1 OVER TLS SESSION =====================

    private static void writeHttpRequest(
            TlsSession tls,
            String method,
            String hostHeader,
            int port,
            String path,
            Map<String, String> headers,
            byte[] body) throws IOException {
        String reqPath = (path == null || path.isEmpty()) ? "/" : path;
        String hostVal = (port == 443) ? hostHeader : (hostHeader + ":" + port);
        int bodyLen = body == null ? 0 : body.length;

        StringBuilder sb = new StringBuilder(512);
        sb.append(method).append(' ').append(reqPath).append(" HTTP/1.1\r\n");
        sb.append("Host: ").append(hostVal).append("\r\n");
        sb.append("Connection: close\r\n");
        boolean hasUa = false;
        if (headers != null) {
            for (Map.Entry<String, String> e : headers.entrySet()) {
                if ("User-Agent".equalsIgnoreCase(e.getKey())) {
                    hasUa = true;
                }
                sb.append(e.getKey()).append(": ").append(e.getValue()).append("\r\n");
            }
        }
        if (!hasUa) {
            sb.append("User-Agent: brake-indicator-gen5w/0.8.1\r\n");
        }
        if (bodyLen > 0 || "PUT".equalsIgnoreCase(method) || "POST".equalsIgnoreCase(method)) {
            sb.append("Content-Length: ").append(bodyLen).append("\r\n");
        }
        sb.append("\r\n");

        byte[] headBytes = sb.toString().getBytes("UTF-8");
        tls.writeAppData(headBytes, 0, headBytes.length);
        if (bodyLen > 0) {
            tls.writeAppData(body, 0, bodyLen);
        }
        tls.out.flush();
    }

    private static HttpResponse readHttpResponse(TlsSession tls) throws IOException {
        InputStream appIn = tls.asAppInputStream();
        String statusLine = readCrLfLine(appIn);
        if (statusLine == null || !statusLine.startsWith("HTTP/1.")) {
            throw new IOException("Invalid HTTP status line: " + statusLine);
        }
        int sp1 = statusLine.indexOf(' ');
        if (sp1 < 0 || sp1 + 4 > statusLine.length()) {
            throw new IOException("Malformed HTTP status line: " + statusLine);
        }
        int sp2 = statusLine.indexOf(' ', sp1 + 1);
        String codeStr = sp2 > 0 ? statusLine.substring(sp1 + 1, sp2) : statusLine.substring(sp1 + 1);
        String statusMsg = sp2 > 0 ? statusLine.substring(sp2 + 1).trim() : "";
        int statusCode;
        try {
            statusCode = Integer.parseInt(codeStr.trim());
        } catch (NumberFormatException e) {
            throw new IOException("Invalid HTTP status code: " + codeStr);
        }

        Map<String, String> headers = new LinkedHashMap<String, String>();
        int contentLength = -1;
        boolean chunked = false;
        while (true) {
            String line = readCrLfLine(appIn);
            if (line == null || line.isEmpty()) {
                break;
            }
            int colon = line.indexOf(':');
            if (colon > 0) {
                String k = line.substring(0, colon).trim().toLowerCase(Locale.US);
                String v = line.substring(colon + 1).trim();
                headers.put(k, v);
                if ("content-length".equals(k)) {
                    try {
                        contentLength = Integer.parseInt(v);
                    } catch (NumberFormatException ignored) {}
                } else if ("transfer-encoding".equals(k) && v.toLowerCase(Locale.US).contains("chunked")) {
                    chunked = true;
                }
            }
        }

        byte[] body;
        if (statusCode == 204 || statusCode == 304) {
            body = new byte[0];
        } else if (chunked) {
            body = readChunkedBody(appIn);
        } else if (contentLength >= 0) {
            if (contentLength > MAX_HTTP_BODY_BYTES) {
                throw new IOException("HTTP response exceeds safety cap: " + contentLength);
            }
            body = readExact(appIn, contentLength);
        } else {
            body = readUntilEof(appIn);
        }

        return new HttpResponse(
                statusCode,
                statusMsg,
                headers,
                body,
                cipherName(tls.cipherSuite),
                tls.peerSubject);
    }

    private static String readCrLfLine(InputStream in) throws IOException {
        ByteArrayOutputStream line = new ByteArrayOutputStream(128);
        while (true) {
            int b = in.read();
            if (b < 0) {
                if (line.size() == 0) return null;
                break;
            }
            if (b == '\n') {
                break;
            }
            if (b != '\r') {
                line.write(b);
                if (line.size() > 16384) {
                    throw new IOException("HTTP header line too long");
                }
            }
        }
        return new String(line.toByteArray(), "US-ASCII");
    }

    private static byte[] readChunkedBody(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream(1024);
        while (true) {
            String sizeLine = readCrLfLine(in);
            if (sizeLine == null) {
                break;
            }
            int semi = sizeLine.indexOf(';');
            String hex = (semi >= 0 ? sizeLine.substring(0, semi) : sizeLine).trim();
            if (hex.isEmpty()) continue;
            int chunkSize;
            try {
                chunkSize = Integer.parseInt(hex, 16);
            } catch (NumberFormatException e) {
                throw new IOException("Invalid HTTP chunk size: " + hex);
            }
            if (chunkSize == 0) {
                // Consume trailing headers until blank line
                while (true) {
                    String trailer = readCrLfLine(in);
                    if (trailer == null || trailer.isEmpty()) break;
                }
                break;
            }
            if (out.size() + chunkSize > MAX_HTTP_BODY_BYTES) {
                throw new IOException("Chunked HTTP response exceeds safety cap");
            }
            byte[] chunk = readExact(in, chunkSize);
            out.write(chunk);
            readCrLfLine(in); // trailing CRLF after chunk data
        }
        return out.toByteArray();
    }

    private static byte[] readUntilEof(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream(1024);
        byte[] buf = new byte[4096];
        int n;
        while ((n = in.read(buf)) >= 0) {
            if (out.size() + n > MAX_HTTP_BODY_BYTES) {
                throw new IOException("HTTP response exceeds safety cap");
            }
            out.write(buf, 0, n);
        }
        return out.toByteArray();
    }

    // ===================== BYTE UTILITIES =====================

    private static byte[] readExact(InputStream in, int len) throws IOException {
        byte[] buf = new byte[len];
        int pos = 0;
        while (pos < len) {
            int n = in.read(buf, pos, len - pos);
            if (n < 0) {
                throw new EOFException("Unexpected EOF (wanted " + len + " bytes, got " + pos + ")");
            }
            pos += n;
        }
        return buf;
    }

    private static void writeU16(OutputStream out, int v) throws IOException {
        out.write((v >>> 8) & 0xFF);
        out.write(v & 0xFF);
    }

    private static int readU16(byte[] b, int off) {
        return ((b[off] & 0xFF) << 8) | (b[off + 1] & 0xFF);
    }

    private static int readU24(byte[] b, int off) {
        return ((b[off] & 0xFF) << 16) | ((b[off + 1] & 0xFF) << 8) | (b[off + 2] & 0xFF);
    }

    private static long getLongBE(byte[] b, int off) {
        return ((long) (b[off] & 0xFF) << 56)
                | ((long) (b[off + 1] & 0xFF) << 48)
                | ((long) (b[off + 2] & 0xFF) << 40)
                | ((long) (b[off + 3] & 0xFF) << 32)
                | ((long) (b[off + 4] & 0xFF) << 24)
                | ((long) (b[off + 5] & 0xFF) << 16)
                | ((long) (b[off + 6] & 0xFF) << 8)
                | ((long) (b[off + 7] & 0xFF));
    }

    private static void putIntBE(byte[] b, int off, int v) {
        b[off]     = (byte) ((v >>> 24) & 0xFF);
        b[off + 1] = (byte) ((v >>> 16) & 0xFF);
        b[off + 2] = (byte) ((v >>> 8) & 0xFF);
        b[off + 3] = (byte) (v & 0xFF);
    }

    private static void putLongBE(byte[] b, int off, long v) {
        b[off]     = (byte) ((v >>> 56) & 0xFF);
        b[off + 1] = (byte) ((v >>> 48) & 0xFF);
        b[off + 2] = (byte) ((v >>> 40) & 0xFF);
        b[off + 3] = (byte) ((v >>> 32) & 0xFF);
        b[off + 4] = (byte) ((v >>> 24) & 0xFF);
        b[off + 5] = (byte) ((v >>> 16) & 0xFF);
        b[off + 6] = (byte) ((v >>> 8) & 0xFF);
        b[off + 7] = (byte) (v & 0xFF);
    }

    private static byte[] concat(byte[] a, byte[] b) {
        byte[] r = new byte[a.length + b.length];
        System.arraycopy(a, 0, r, 0, a.length);
        System.arraycopy(b, 0, r, a.length, b.length);
        return r;
    }

    private static byte[] slice(byte[] b, int off, int len) {
        byte[] r = new byte[len];
        System.arraycopy(b, off, r, 0, len);
        return r;
    }

    private static void zeroize(byte[] b) {
        if (b != null) {
            for (int i = 0; i < b.length; i++) b[i] = 0;
        }
    }

    private static boolean constantTimeEquals(byte[] a, byte[] b) {
        if (a == null || b == null || a.length != b.length) return false;
        int diff = 0;
        for (int i = 0; i < a.length; i++) {
            diff |= (a[i] ^ b[i]) & 0xFF;
        }
        return diff == 0;
    }
}
