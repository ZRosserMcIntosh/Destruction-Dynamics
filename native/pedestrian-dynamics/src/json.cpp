// Pedestrian Dynamics — physically simulated people. Vendored from Ballistic Dynamics (same author, MIT).
// SPDX-License-Identifier: MIT
#include "ped/json.hpp"

#include <cmath>
#include <cstdlib>

namespace ped::json {

const Value* Value::find(const std::string& key) const {
    if (type_ != Type::Object) return nullptr;
    for (const auto& [k, v] : obj_)
        if (k == key) return &v;
    return nullptr;
}

namespace {

constexpr int kMaxDepth = 64;

class Parser {
public:
    explicit Parser(const std::string& t) : s_(t) {}

    ParseResult run() {
        ParseResult r;
        skipWs();
        if (!value(r.value, 0)) {
            r.error = error_;
            return r;
        }
        skipWs();
        if (i_ != s_.size()) {
            fail("unexpected trailing characters");
            r.error = error_;
        }
        return r;
    }

private:
    const std::string& s_;
    std::size_t i_ = 0;
    std::string error_;

    bool fail(const std::string& msg) {
        if (!error_.empty()) return false;
        int line = 1, col = 1;
        for (std::size_t k = 0; k < i_ && k < s_.size(); ++k) {
            if (s_[k] == '\n') {
                ++line;
                col = 1;
            } else {
                ++col;
            }
        }
        error_ = "line " + std::to_string(line) + ", column " + std::to_string(col) + ": " + msg;
        return false;
    }
    void skipWs() {
        while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) ++i_;
    }
    bool literal(const char* word) {
        std::size_t k = 0;
        while (word[k]) {
            if (i_ + k >= s_.size() || s_[i_ + k] != word[k]) return fail("invalid literal");
            ++k;
        }
        i_ += k;
        return true;
    }

    bool value(Value& out, int depth) {
        if (depth > kMaxDepth) return fail("nesting too deep");
        if (i_ >= s_.size()) return fail("unexpected end of input");
        const char c = s_[i_];
        if (c == '{') return object(out, depth);
        if (c == '[') return array(out, depth);
        if (c == '"') {
            std::string str;
            if (!string(str)) return false;
            out = Value::string(std::move(str));
            return true;
        }
        if (c == 't') { if (!literal("true")) return false; out = Value::boolean(true); return true; }
        if (c == 'f') { if (!literal("false")) return false; out = Value::boolean(false); return true; }
        if (c == 'n') { if (!literal("null")) return false; out = Value(); return true; }
        if (c == '-' || (c >= '0' && c <= '9')) return number(out);
        return fail(std::string("unexpected character '") + c + "'");
    }

    bool object(Value& out, int depth) {
        out = Value::object();
        ++i_;
        skipWs();
        if (i_ < s_.size() && s_[i_] == '}') {
            ++i_;
            return true;
        }
        for (;;) {
            skipWs();
            if (i_ >= s_.size() || s_[i_] != '"') return fail("expected a string key");
            std::string key;
            if (!string(key)) return false;
            for (const auto& m : out.members())
                if (m.first == key) return fail("duplicate key \"" + key + "\"");
            skipWs();
            if (i_ >= s_.size() || s_[i_] != ':') return fail("expected ':'");
            ++i_;
            skipWs();
            Value v;
            if (!value(v, depth + 1)) return false;
            out.members().emplace_back(std::move(key), std::move(v));
            skipWs();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == '}') {
                ++i_;
                return true;
            }
            return fail("expected ',' or '}'");
        }
    }

    bool array(Value& out, int depth) {
        out = Value::array();
        ++i_;
        skipWs();
        if (i_ < s_.size() && s_[i_] == ']') {
            ++i_;
            return true;
        }
        for (;;) {
            skipWs();
            Value v;
            if (!value(v, depth + 1)) return false;
            out.items().push_back(std::move(v));
            skipWs();
            if (i_ < s_.size() && s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (i_ < s_.size() && s_[i_] == ']') {
                ++i_;
                return true;
            }
            return fail("expected ',' or ']'");
        }
    }

    static int hex(char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    }
    static void utf8(std::string& o, unsigned cp) {
        if (cp < 0x80) {
            o += static_cast<char>(cp);
        } else if (cp < 0x800) {
            o += static_cast<char>(0xC0 | (cp >> 6));
            o += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            o += static_cast<char>(0xE0 | (cp >> 12));
            o += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            o += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            o += static_cast<char>(0xF0 | (cp >> 18));
            o += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            o += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            o += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }
    bool hex4(unsigned& cp) {
        if (i_ + 4 > s_.size()) return fail("truncated \\u escape");
        cp = 0;
        for (int k = 0; k < 4; ++k) {
            const int h = hex(s_[i_ + k]);
            if (h < 0) return fail("invalid \\u escape");
            cp = cp * 16 + static_cast<unsigned>(h);
        }
        i_ += 4;
        return true;
    }

    bool string(std::string& out) {
        ++i_;  // opening quote
        while (i_ < s_.size()) {
            const char c = s_[i_];
            if (c == '"') {
                ++i_;
                return true;
            }
            if (static_cast<unsigned char>(c) < 0x20) return fail("control character in string");
            if (c != '\\') {
                out += c;
                ++i_;
                continue;
            }
            ++i_;
            if (i_ >= s_.size()) break;
            const char e = s_[i_++];
            switch (e) {
                case '"': out += '"'; break;
                case '\\': out += '\\'; break;
                case '/': out += '/'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'n': out += '\n'; break;
                case 'r': out += '\r'; break;
                case 't': out += '\t'; break;
                case 'u': {
                    unsigned cp = 0;
                    if (!hex4(cp)) return false;
                    if (cp >= 0xD800 && cp < 0xDC00) {
                        unsigned lo = 0;
                        if (i_ + 2 > s_.size() || s_[i_] != '\\' || s_[i_ + 1] != 'u') return fail("lone surrogate");
                        i_ += 2;
                        if (!hex4(lo)) return false;
                        if (lo < 0xDC00 || lo >= 0xE000) return fail("invalid surrogate pair");
                        cp = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                    } else if (cp >= 0xDC00 && cp < 0xE000) {
                        return fail("lone surrogate");
                    }
                    utf8(out, cp);
                    break;
                }
                default: return fail("invalid escape");
            }
        }
        return fail("unterminated string");
    }

    bool number(Value& out) {
        const std::size_t start = i_;
        if (s_[i_] == '-') ++i_;
        if (i_ >= s_.size()) return fail("invalid number");
        if (s_[i_] == '0') {
            ++i_;
        } else if (s_[i_] >= '1' && s_[i_] <= '9') {
            while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
        } else {
            return fail("invalid number");
        }
        if (i_ < s_.size() && s_[i_] == '.') {
            ++i_;
            if (i_ >= s_.size() || s_[i_] < '0' || s_[i_] > '9') return fail("invalid number");
            while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
        }
        if (i_ < s_.size() && (s_[i_] == 'e' || s_[i_] == 'E')) {
            ++i_;
            if (i_ < s_.size() && (s_[i_] == '+' || s_[i_] == '-')) ++i_;
            if (i_ >= s_.size() || s_[i_] < '0' || s_[i_] > '9') return fail("invalid number");
            while (i_ < s_.size() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
        }
        // strtod is correctly rounded in glibc, musl/wasi-libc, Apple libc and
        // the MSVC CRT (since 2015), and the core never changes the C locale.
        const std::string tok = s_.substr(start, i_ - start);
        const double d = std::strtod(tok.c_str(), nullptr);
        if (!std::isfinite(d)) return fail("number out of range");
        out = Value::number(d);
        return true;
    }
};

}  // namespace

ParseResult parse(const std::string& text) { return Parser(text).run(); }

}  // namespace ped::json
