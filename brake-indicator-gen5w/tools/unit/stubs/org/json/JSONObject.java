package org.json;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Host-JVM test implementation of Android's {@code org.json.JSONObject} so unit tests run
 * on standard OpenJDK without Android's {@code Stub!}-only {@code android.jar}.
 */
public class JSONObject {

    public static final Object NULL = new Object() {
        @Override
        public String toString() {
            return "null";
        }
    };

    private final Map<String, Object> values = new LinkedHashMap<String, Object>();

    public JSONObject() {}

    public JSONObject(String source) {
        if (source == null) {
            throw new IllegalArgumentException("Null JSON string");
        }
        Parser p = new Parser(source);
        p.skipWs();
        JSONObject parsed = p.readObject();
        p.skipWs();
        this.values.putAll(parsed.values);
    }

    public JSONObject put(String name, Object value) {
        if (name == null) {
            throw new IllegalArgumentException("Null key");
        }
        if (value == null) {
            values.remove(name);
        } else {
            values.put(name, value);
        }
        return this;
    }

    public JSONObject put(String name, boolean value) {
        return put(name, Boolean.valueOf(value));
    }

    public JSONObject put(String name, int value) {
        return put(name, Integer.valueOf(value));
    }

    public JSONObject put(String name, long value) {
        return put(name, Long.valueOf(value));
    }

    public Object opt(String name) {
        return name == null ? null : values.get(name);
    }

    public String optString(String name, String fallback) {
        Object v = opt(name);
        if (v == null || v == NULL) return fallback;
        return String.valueOf(v);
    }

    public boolean optBoolean(String name, boolean fallback) {
        Object v = opt(name);
        if (v instanceof Boolean) return ((Boolean) v).booleanValue();
        if (v instanceof String) {
            if ("true".equalsIgnoreCase((String) v)) return true;
            if ("false".equalsIgnoreCase((String) v)) return false;
        }
        return fallback;
    }

    public int optInt(String name, int fallback) {
        Object v = opt(name);
        if (v instanceof Number) return ((Number) v).intValue();
        if (v instanceof String) {
            try {
                return Integer.parseInt(((String) v).trim());
            } catch (NumberFormatException ignored) {}
        }
        return fallback;
    }

    public long optLong(String name, long fallback) {
        Object v = opt(name);
        if (v instanceof Number) return ((Number) v).longValue();
        if (v instanceof String) {
            try {
                return Long.parseLong(((String) v).trim());
            } catch (NumberFormatException ignored) {}
        }
        return fallback;
    }

    public JSONObject optJSONObject(String name) {
        Object v = opt(name);
        return (v instanceof JSONObject) ? (JSONObject) v : null;
    }

    public Iterator<String> keys() {
        return values.keySet().iterator();
    }

    public static String quote(String data) {
        if (data == null) {
            return "\"\"";
        }
        StringBuilder sb = new StringBuilder(data.length() + 4);
        sb.append('"');
        for (int i = 0; i < data.length(); i++) {
            char c = data.charAt(i);
            switch (c) {
                case '"': sb.append("\\\""); break;
                case '\\': sb.append("\\\\"); break;
                case '\b': sb.append("\\b"); break;
                case '\f': sb.append("\\f"); break;
                case '\n': sb.append("\\n"); break;
                case '\r': sb.append("\\r"); break;
                case '\t': sb.append("\\t"); break;
                default:
                    if (c < 0x20) {
                        sb.append(String.format("\\u%04x", Integer.valueOf(c)));
                    } else {
                        sb.append(c);
                    }
            }
        }
        sb.append('"');
        return sb.toString();
    }

    @Override
    public String toString() {
        return toString(0);
    }

    public String toString(int indentSpaces) {
        StringBuilder sb = new StringBuilder();
        writeJson(sb, this, indentSpaces, 0);
        return sb.toString();
    }

    private static void writeJson(StringBuilder sb, Object val, int indent, int depth) {
        if (val == null || val == NULL) {
            sb.append("null");
        } else if (val instanceof JSONObject) {
            Map<String, Object> map = ((JSONObject) val).values;
            if (map.isEmpty()) {
                sb.append("{}");
                return;
            }
            sb.append('{');
            boolean first = true;
            for (Map.Entry<String, Object> e : map.entrySet()) {
                if (!first) sb.append(',');
                first = false;
                if (indent > 0) {
                    sb.append('\n');
                    appendSpaces(sb, (depth + 1) * indent);
                }
                sb.append(quote(e.getKey()));
                sb.append(indent > 0 ? ": " : ":");
                writeJson(sb, e.getValue(), indent, depth + 1);
            }
            if (indent > 0) {
                sb.append('\n');
                appendSpaces(sb, depth * indent);
            }
            sb.append('}');
        } else if (val instanceof List) {
            List<?> list = (List<?>) val;
            sb.append('[');
            for (int i = 0; i < list.size(); i++) {
                if (i > 0) sb.append(',');
                writeJson(sb, list.get(i), indent, depth + 1);
            }
            sb.append(']');
        } else if (val instanceof Boolean || val instanceof Number) {
            sb.append(val.toString());
        } else {
            sb.append(quote(String.valueOf(val)));
        }
    }

    private static void appendSpaces(StringBuilder sb, int count) {
        for (int i = 0; i < count; i++) sb.append(' ');
    }

    private static final class Parser {
        private final String s;
        private int pos;

        Parser(String s) {
            this.s = s;
        }

        void skipWs() {
            while (pos < s.length()) {
                char c = s.charAt(pos);
                if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                    pos++;
                } else {
                    break;
                }
            }
        }

        JSONObject readObject() {
            expect('{');
            JSONObject obj = new JSONObject();
            skipWs();
            if (pos < s.length() && s.charAt(pos) == '}') {
                pos++;
                return obj;
            }
            while (true) {
                skipWs();
                String key = readString();
                skipWs();
                expect(':');
                skipWs();
                Object val = readValue();
                obj.values.put(key, val);
                skipWs();
                if (pos >= s.length()) {
                    throw new IllegalArgumentException("Unterminated JSON object");
                }
                char ch = s.charAt(pos++);
                if (ch == '}') {
                    return obj;
                }
                if (ch != ',') {
                    throw new IllegalArgumentException("Expected ',' or '}' at " + (pos - 1));
                }
            }
        }

        List<Object> readArray() {
            expect('[');
            List<Object> list = new ArrayList<Object>();
            skipWs();
            if (pos < s.length() && s.charAt(pos) == ']') {
                pos++;
                return list;
            }
            while (true) {
                skipWs();
                list.add(readValue());
                skipWs();
                if (pos >= s.length()) {
                    throw new IllegalArgumentException("Unterminated JSON array");
                }
                char ch = s.charAt(pos++);
                if (ch == ']') {
                    return list;
                }
                if (ch != ',') {
                    throw new IllegalArgumentException("Expected ',' or ']' at " + (pos - 1));
                }
            }
        }

        Object readValue() {
            skipWs();
            if (pos >= s.length()) {
                throw new IllegalArgumentException("Unexpected end of JSON");
            }
            char c = s.charAt(pos);
            if (c == '"') return readString();
            if (c == '{') return readObject();
            if (c == '[') return readArray();
            if (s.startsWith("true", pos)) {
                pos += 4;
                return Boolean.TRUE;
            }
            if (s.startsWith("false", pos)) {
                pos += 5;
                return Boolean.FALSE;
            }
            if (s.startsWith("null", pos)) {
                pos += 4;
                return NULL;
            }
            return readNumber();
        }

        String readString() {
            expect('"');
            StringBuilder sb = new StringBuilder();
            while (pos < s.length()) {
                char c = s.charAt(pos++);
                if (c == '"') {
                    return sb.toString();
                }
                if (c == '\\') {
                    if (pos >= s.length()) throw new IllegalArgumentException("Bad escape");
                    char esc = s.charAt(pos++);
                    switch (esc) {
                        case '"': sb.append('"'); break;
                        case '\\': sb.append('\\'); break;
                        case '/': sb.append('/'); break;
                        case 'b': sb.append('\b'); break;
                        case 'f': sb.append('\f'); break;
                        case 'n': sb.append('\n'); break;
                        case 'r': sb.append('\r'); break;
                        case 't': sb.append('\t'); break;
                        case 'u':
                            if (pos + 4 > s.length()) throw new IllegalArgumentException("Bad \\u escape");
                            sb.append((char) Integer.parseInt(s.substring(pos, pos + 4), 16));
                            pos += 4;
                            break;
                        default:
                            sb.append(esc);
                    }
                } else {
                    sb.append(c);
                }
            }
            throw new IllegalArgumentException("Unterminated string");
        }

        Number readNumber() {
            int start = pos;
            boolean isFloat = false;
            while (pos < s.length()) {
                char c = s.charAt(pos);
                if ((c >= '0' && c <= '9') || c == '-' || c == '+') {
                    pos++;
                } else if (c == '.' || c == 'e' || c == 'E') {
                    isFloat = true;
                    pos++;
                } else {
                    break;
                }
            }
            String num = s.substring(start, pos);
            if (isFloat) {
                return Double.valueOf(num);
            }
            long lv = Long.parseLong(num);
            if (lv >= Integer.MIN_VALUE && lv <= Integer.MAX_VALUE) {
                return Integer.valueOf((int) lv);
            }
            return Long.valueOf(lv);
        }

        void expect(char expected) {
            if (pos >= s.length() || s.charAt(pos) != expected) {
                throw new IllegalArgumentException("Expected '" + expected + "' at " + pos);
            }
            pos++;
        }
    }
}
