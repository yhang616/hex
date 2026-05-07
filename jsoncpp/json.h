#ifndef HEX_LOCAL_JSONCPP_JSON_H_
#define HEX_LOCAL_JSONCPP_JSON_H_

#include <cctype>
#include <cstdlib>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace Json {

class Value {
public:
    enum class Kind { Null, Int, Bool, String, Object, Array };

    Value() = default;
    Value(int value) : kind_(Kind::Int), int_value_(value) {}
    Value(bool value) : kind_(Kind::Bool), bool_value_(value) {}
    Value(const char* value) : kind_(Kind::String), string_value_(value) {}
    Value(const std::string& value) : kind_(Kind::String), string_value_(value) {}

    Value& operator=(int value) {
        clear();
        kind_ = Kind::Int;
        int_value_ = value;
        return *this;
    }

    Value& operator=(bool value) {
        clear();
        kind_ = Kind::Bool;
        bool_value_ = value;
        return *this;
    }

    Value& operator=(const std::string& value) {
        clear();
        kind_ = Kind::String;
        string_value_ = value;
        return *this;
    }

    Value& operator[](const std::string& key) {
        ensure_object();
        return object_value_[key];
    }

    const Value& operator[](const std::string& key) const {
        static const Value empty;
        if (kind_ != Kind::Object) return empty;
        auto it = object_value_.find(key);
        return it == object_value_.end() ? empty : it->second;
    }

    Value& operator[](int index) {
        ensure_array();
        if (index >= static_cast<int>(array_value_.size())) array_value_.resize(index + 1);
        return array_value_[index];
    }

    const Value& operator[](int index) const {
        static const Value empty;
        if (kind_ != Kind::Array || index < 0 || index >= static_cast<int>(array_value_.size())) return empty;
        return array_value_[index];
    }

    bool isMember(const std::string& key) const {
        return kind_ == Kind::Object && object_value_.find(key) != object_value_.end();
    }

    int asInt() const {
        if (kind_ == Kind::Int) return int_value_;
        if (kind_ == Kind::Bool) return bool_value_ ? 1 : 0;
        if (kind_ == Kind::String) return std::atoi(string_value_.c_str());
        return 0;
    }

    bool asBool() const {
        if (kind_ == Kind::Bool) return bool_value_;
        if (kind_ == Kind::Int) return int_value_ != 0;
        return false;
    }

    std::string asString() const {
        if (kind_ == Kind::String) return string_value_;
        if (kind_ == Kind::Int) return std::to_string(int_value_);
        if (kind_ == Kind::Bool) return bool_value_ ? "true" : "false";
        return "";
    }

    int size() const {
        if (kind_ == Kind::Array) return static_cast<int>(array_value_.size());
        if (kind_ == Kind::Object) return static_cast<int>(object_value_.size());
        return 0;
    }

    Kind kind() const { return kind_; }
    const std::map<std::string, Value>& object_items() const { return object_value_; }
    const std::vector<Value>& array_items() const { return array_value_; }

    void append(const Value& value) {
        ensure_array();
        array_value_.push_back(value);
    }

private:
    Kind kind_ = Kind::Null;
    int int_value_ = 0;
    bool bool_value_ = false;
    std::string string_value_;
    std::map<std::string, Value> object_value_;
    std::vector<Value> array_value_;

    void clear() {
        string_value_.clear();
        object_value_.clear();
        array_value_.clear();
        int_value_ = 0;
        bool_value_ = false;
        kind_ = Kind::Null;
    }

    void ensure_object() {
        if (kind_ != Kind::Object) {
            clear();
            kind_ = Kind::Object;
        }
    }

    void ensure_array() {
        if (kind_ != Kind::Array) {
            clear();
            kind_ = Kind::Array;
        }
    }
};

class CharReader {
public:
    bool parse(const char* begin, const char* end, Value* root, std::string* errors) {
        input_ = begin;
        end_ = end;
        skip_ws();
        if (!parse_value(*root)) {
            if (errors) *errors = "failed to parse JSON";
            return false;
        }
        skip_ws();
        if (input_ != end_) {
            if (errors) *errors = "trailing characters after JSON";
            return false;
        }
        return true;
    }

private:
    const char* input_ = nullptr;
    const char* end_ = nullptr;

    void skip_ws() {
        while (input_ != end_ && std::isspace(static_cast<unsigned char>(*input_))) ++input_;
    }

    bool consume(char ch) {
        skip_ws();
        if (input_ == end_ || *input_ != ch) return false;
        ++input_;
        return true;
    }

    bool parse_value(Value& out) {
        skip_ws();
        if (input_ == end_) return false;
        if (*input_ == '{') return parse_object(out);
        if (*input_ == '[') return parse_array(out);
        if (*input_ == '"') {
            std::string text;
            if (!parse_string(text)) return false;
            out = text;
            return true;
        }
        if (*input_ == '-' || std::isdigit(static_cast<unsigned char>(*input_))) return parse_number(out);
        if (match_literal("true")) {
            out = true;
            return true;
        }
        if (match_literal("false")) {
            out = false;
            return true;
        }
        if (match_literal("null")) {
            out = Value();
            return true;
        }
        return false;
    }

    bool parse_object(Value& out) {
        if (!consume('{')) return false;
        out = Value();
        if (consume('}')) return true;
        while (true) {
            std::string key;
            if (!parse_string(key)) return false;
            if (!consume(':')) return false;
            Value child;
            if (!parse_value(child)) return false;
            out[key] = child;
            if (consume('}')) return true;
            if (!consume(',')) return false;
        }
    }

    bool parse_array(Value& out) {
        if (!consume('[')) return false;
        out = Value();
        if (consume(']')) return true;
        while (true) {
            Value child;
            if (!parse_value(child)) return false;
            out.append(child);
            if (consume(']')) return true;
            if (!consume(',')) return false;
        }
    }

    bool parse_string(std::string& out) {
        if (!consume('"')) return false;
        out.clear();
        while (input_ != end_) {
            char ch = *input_++;
            if (ch == '"') return true;
            if (ch == '\\') {
                if (input_ == end_) return false;
                char escaped = *input_++;
                if (escaped == '"' || escaped == '\\' || escaped == '/') out.push_back(escaped);
                else if (escaped == 'b') out.push_back('\b');
                else if (escaped == 'f') out.push_back('\f');
                else if (escaped == 'n') out.push_back('\n');
                else if (escaped == 'r') out.push_back('\r');
                else if (escaped == 't') out.push_back('\t');
                else return false;
            } else {
                out.push_back(ch);
            }
        }
        return false;
    }

    bool parse_number(Value& out) {
        const char* start = input_;
        if (*input_ == '-') ++input_;
        if (input_ == end_ || !std::isdigit(static_cast<unsigned char>(*input_))) return false;
        while (input_ != end_ && std::isdigit(static_cast<unsigned char>(*input_))) ++input_;
        out = std::atoi(std::string(start, input_).c_str());
        return true;
    }

    bool match_literal(const char* literal) {
        const char* saved = input_;
        while (*literal != '\0') {
            if (input_ == end_ || *input_ != *literal) {
                input_ = saved;
                return false;
            }
            ++input_;
            ++literal;
        }
        return true;
    }
};

class CharReaderBuilder {
public:
    std::unique_ptr<CharReader> newCharReader() const {
        return std::unique_ptr<CharReader>(new CharReader());
    }
};

class StreamWriterBuilder {
public:
    std::string& operator[](const std::string& key) { return options_[key]; }
private:
    std::map<std::string, std::string> options_;
};

inline std::string escape_string(const std::string& input) {
    std::string out;
    for (char ch : input) {
        if (ch == '"' || ch == '\\') {
            out.push_back('\\');
            out.push_back(ch);
        } else if (ch == '\n') {
            out += "\\n";
        } else {
            out.push_back(ch);
        }
    }
    return out;
}

inline void write_value(std::ostringstream& out, const Value& value) {
    switch (value.kind()) {
        case Value::Kind::Null:
            out << "null";
            break;
        case Value::Kind::Int:
            out << value.asInt();
            break;
        case Value::Kind::Bool:
            out << (value.asBool() ? "true" : "false");
            break;
        case Value::Kind::String:
            out << '"' << escape_string(value.asString()) << '"';
            break;
        case Value::Kind::Array: {
            out << '[';
            bool first = true;
            for (const Value& child : value.array_items()) {
                if (!first) out << ',';
                first = false;
                write_value(out, child);
            }
            out << ']';
            break;
        }
        case Value::Kind::Object: {
            out << '{';
            bool first = true;
            for (const auto& item : value.object_items()) {
                if (!first) out << ',';
                first = false;
                out << '"' << escape_string(item.first) << "\":";
                write_value(out, item.second);
            }
            out << '}';
            break;
        }
    }
}

inline std::string writeString(const StreamWriterBuilder&, const Value& root) {
    std::ostringstream out;
    write_value(out, root);
    return out.str();
}

}  // namespace Json

#endif  // HEX_LOCAL_JSONCPP_JSON_H_
