package dev.lemoyne.brakeindicator;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.zip.GZIPInputStream;
import java.util.zip.GZIPOutputStream;

/**
 * Pure in-RAM POSIX USTAR {@code .tar.gz} builder and extractor.
 *
 * Compresses repetitive 20 Hz numeric CSV + summary text ~10-12x (e.g. ~3.8 MB -> ~300 KB)
 * purely in memory with zero temporary files on internal eMMC flash, and allows unpacking back
 * to plain CSV + summary text if an offline queued bundle is later flushed to a USB stick.
 */
public final class TarGzArchive {

    private static final int BLOCK_SIZE = 512;

    private TarGzArchive() {}

    public static byte[] buildDatasetArchive(
            String stamp,
            String csvContent,
            String summaryContent,
            long mtimeEpochMillis) throws IOException {
        Map<String, byte[]> files = new LinkedHashMap<String, byte[]>();
        String safeStamp = (stamp == null || stamp.isEmpty()) ? "00000000-000000" : stamp;
        files.put("brake-dataset-" + safeStamp + ".csv",
                (csvContent == null ? "" : csvContent).getBytes("UTF-8"));
        files.put("brake-summary-" + safeStamp + ".txt",
                (summaryContent == null ? "" : summaryContent).getBytes("UTF-8"));
        return buildTarGz(files, mtimeEpochMillis);
    }

    public static byte[] buildTarGz(Map<String, byte[]> entries, long mtimeEpochMillis)
            throws IOException {
        long mtimeSec = mtimeEpochMillis > 0L ? (mtimeEpochMillis / 1000L) : (System.currentTimeMillis() / 1000L);
        ByteArrayOutputStream gzOut = new ByteArrayOutputStream(65536);
        GZIPOutputStream gzip = new GZIPOutputStream(gzOut);
        try {
            byte[] header = new byte[BLOCK_SIZE];
            byte[] zeroPad = new byte[BLOCK_SIZE];
            for (Map.Entry<String, byte[]> entry : entries.entrySet()) {
                String name = sanitizeEntryName(entry.getKey());
                byte[] content = entry.getValue() == null ? new byte[0] : entry.getValue();
                formatUstarHeader(header, name, content.length, mtimeSec);
                gzip.write(header);
                if (content.length > 0) {
                    gzip.write(content);
                    int rem = content.length % BLOCK_SIZE;
                    if (rem != 0) {
                        gzip.write(zeroPad, 0, BLOCK_SIZE - rem);
                    }
                }
            }
            // Two 512-byte zero blocks mark end-of-archive in POSIX tar
            gzip.write(zeroPad);
            gzip.write(zeroPad);
            gzip.finish();
        } finally {
            gzip.close();
        }
        return gzOut.toByteArray();
    }

    public static Map<String, byte[]> extractEntries(byte[] tarGzBytes) throws IOException {
        Map<String, byte[]> result = new LinkedHashMap<String, byte[]>();
        if (tarGzBytes == null || tarGzBytes.length == 0) {
            return result;
        }
        GZIPInputStream gis = new GZIPInputStream(new ByteArrayInputStream(tarGzBytes));
        try {
            byte[] header = new byte[BLOCK_SIZE];
            while (readBlockOrEof(gis, header)) {
                if (isAllZero(header)) {
                    break;
                }
                String name = readNullTerminatedAscii(header, 0, 100);
                long size = parseOctal(header, 124, 12);
                if (size < 0L || size > 64L * 1024L * 1024L) {
                    throw new IOException("Invalid tar entry size: " + size);
                }
                int len = (int) size;
                byte[] content = readExactBytes(gis, len);
                int rem = len % BLOCK_SIZE;
                if (rem != 0) {
                    readExactBytes(gis, BLOCK_SIZE - rem);
                }
                if (!name.isEmpty()) {
                    result.put(name, content);
                }
            }
        } finally {
            gis.close();
        }
        return result;
    }

    private static String sanitizeEntryName(String raw) {
        if (raw == null || raw.isEmpty()) return "entry.bin";
        String clean = raw.replace('\\', '/');
        while (clean.startsWith("/")) {
            clean = clean.substring(1);
        }
        if (clean.length() > 99) {
            clean = clean.substring(clean.length() - 99);
        }
        return clean;
    }

    private static void formatUstarHeader(byte[] hdr, String name, long size, long mtimeSec)
            throws IOException {
        for (int i = 0; i < BLOCK_SIZE; i++) {
            hdr[i] = 0;
        }
        writeAsciiField(hdr, 0, 100, name);
        writeOctalField(hdr, 100, 8, 0644L);
        writeOctalField(hdr, 108, 8, 0L);
        writeOctalField(hdr, 116, 8, 0L);
        writeOctalField(hdr, 124, 12, size);
        writeOctalField(hdr, 136, 12, mtimeSec);

        // Checksum field is treated as 8 ASCII spaces (0x20) during checksum computation
        for (int i = 148; i < 156; i++) {
            hdr[i] = ' ';
        }
        hdr[156] = '0'; // Normal file
        writeAsciiField(hdr, 257, 6, "ustar");
        hdr[263] = '0';
        hdr[264] = '0';

        int sum = 0;
        for (int i = 0; i < BLOCK_SIZE; i++) {
            sum += (hdr[i] & 0xFF);
        }
        String chk = String.format(Locale.US, "%06o", Integer.valueOf(sum));
        byte[] chkBytes = chk.getBytes("US-ASCII");
        System.arraycopy(chkBytes, 0, hdr, 148, Math.min(6, chkBytes.length));
        hdr[154] = 0;
        hdr[155] = ' ';
    }

    private static void writeAsciiField(byte[] dst, int off, int maxLen, String s)
            throws IOException {
        byte[] b = s.getBytes("US-ASCII");
        System.arraycopy(b, 0, dst, off, Math.min(maxLen - 1, b.length));
    }

    private static void writeOctalField(byte[] dst, int off, int fieldLen, long val)
            throws IOException {
        int digits = fieldLen - 1;
        String oct = String.format(Locale.US, "%0" + digits + "o", Long.valueOf(Math.max(0L, val)));
        byte[] b = oct.getBytes("US-ASCII");
        int copy = Math.min(digits, b.length);
        System.arraycopy(b, b.length - copy, dst, off + digits - copy, copy);
        dst[off + fieldLen - 1] = 0;
    }

    private static boolean readBlockOrEof(GZIPInputStream gis, byte[] block) throws IOException {
        int pos = 0;
        while (pos < block.length) {
            int n = gis.read(block, pos, block.length - pos);
            if (n < 0) {
                return pos > 0;
            }
            pos += n;
        }
        return true;
    }

    private static byte[] readExactBytes(GZIPInputStream gis, int len) throws IOException {
        byte[] out = new byte[len];
        int pos = 0;
        while (pos < len) {
            int n = gis.read(out, pos, len - pos);
            if (n < 0) {
                throw new IOException("Truncated tar entry payload");
            }
            pos += n;
        }
        return out;
    }

    private static boolean isAllZero(byte[] b) {
        for (int i = 0; i < b.length; i++) {
            if (b[i] != 0) return false;
        }
        return true;
    }

    private static String readNullTerminatedAscii(byte[] b, int off, int maxLen) throws IOException {
        int end = off;
        int limit = off + maxLen;
        while (end < limit && b[end] != 0) {
            end++;
        }
        return new String(b, off, end - off, "US-ASCII");
    }

    private static long parseOctal(byte[] b, int off, int len) {
        long v = 0L;
        int end = off + len;
        for (int i = off; i < end; i++) {
            byte ch = b[i];
            if (ch == 0 || ch == ' ') {
                if (v > 0L) break;
                continue;
            }
            if (ch >= '0' && ch <= '7') {
                v = (v << 3) | (ch - '0');
            }
        }
        return v;
    }
}
