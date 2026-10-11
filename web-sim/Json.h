#pragma once

//=====================================================================================
// Json.h
//
// 极简 JSON 生成/解析工具。后端只需要输出“状态对象”并解析“指令对象”，
// 因此这里不引入任何第三方 JSON 库，保持项目零额外依赖。
//=====================================================================================

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace json {

	/// <summary>
	/// JSON 写入器：按顺序拼出对象/数组
	/// </summary>
	class Writer {
	public:
		explicit Writer(int /*reserveHint*/ = 0) { buffer_.reserve(8192); }

		void beginObject() { separate(); buffer_ += '{'; stack_.push_back(Scope::Object); firstItem_ = true; }
		void endObject() { buffer_ += '}'; stack_.pop_back(); firstItem_ = false; }
		void beginArray() { separate(); buffer_ += '['; stack_.push_back(Scope::Array); firstItem_ = true; }
		void endArray() { buffer_ += ']'; stack_.pop_back(); firstItem_ = false; }

		void key(const std::string& name) {
			separate();
			writeString(name);
			buffer_ += ':';
			firstItem_ = true;		//键之后紧跟值，不插逗号
		}

		void value(double number) {
			separate();
			char text[64];
			if (!std::isfinite(number)) {
				buffer_ += "null";
			}
			else if (number == static_cast<double>(static_cast<long long>(number)) &&
				std::fabs(number) < 1e15) {
				std::snprintf(text, sizeof(text), "%lld", static_cast<long long>(number));
				buffer_ += text;
			}
			else {
				std::snprintf(text, sizeof(text), "%.6g", number);
				buffer_ += text;
			}
			firstItem_ = false;
		}

		void value(bool flag) { separate(); buffer_ += flag ? "true" : "false"; firstItem_ = false; }

		void value(const std::string& text) { separate(); writeString(text); firstItem_ = false; }

		void value(const char* text) { value(std::string(text ? text : "")); }

		template <typename T>
		void value(const std::vector<T>& list) {
			beginArray();
			for (const T& item : list) {
				value(item);
			}
			endArray();
		}

		/// <summary>直接写入一段已经构造好的 JSON 片段</summary>
		void rawValue(const std::string& raw) {
			separate();
			buffer_ += raw;
			firstItem_ = false;
		}

		const std::string& str() const { return buffer_; }

	private:
		enum class Scope { Object, Array };
		std::string buffer_;
		std::vector<Scope> stack_;
		bool firstItem_ = true;

		void separate() {
			if (firstItem_) {
				firstItem_ = false;
				return;
			}
			if (!stack_.empty() && stack_.back() == Scope::Array) {
				buffer_ += ',';
			}
			else if (!stack_.empty()) {
				buffer_ += ',';
			}
		}

		void writeString(const std::string& text) {
			buffer_ += '"';
			for (char c : text) {
				switch (c) {
				case '"': buffer_ += "\\\""; break;
				case '\\': buffer_ += "\\\\"; break;
				case '\n': buffer_ += "\\n"; break;
				case '\r': buffer_ += "\\r"; break;
				case '\t': buffer_ += "\\t"; break;
				default:
					if (static_cast<unsigned char>(c) < 0x20) {
						char escape[8];
						std::snprintf(escape, sizeof(escape), "\\u%04x", c);
						buffer_ += escape;
					}
					else {
						buffer_ += c;	//UTF-8 直接透传，前端按 UTF-8 解析
					}
					break;
				}
			}
			buffer_ += '"';
		}
	};

	/// <summary>
	/// 极简 JSON 解析器：只支持对象、数组、字符串、数字、布尔与 null。
	/// 解析结果放在 Value 中，通过 find() 按键取值。
	/// </summary>
	class Value {
	public:
		enum class Type { Null, Bool, Number, String, Array, Object };

		Type type = Type::Null;
		bool boolean = false;
		double number = 0.0;
		std::string text;
		std::vector<Value> items;					//数组元素
		std::map<std::string, Value> members;		//对象成员

		bool isObject() const { return type == Type::Object; }
		bool isArray() const { return type == Type::Array; }
		bool isNumber() const { return type == Type::Number; }

		const Value* find(const std::string& name) const {
			auto it = members.find(name);
			return (it == members.end()) ? nullptr : &it->second;
		}

		double num(const std::string& name, double fallback = 0.0) const {
			const Value* v = find(name);
			return (v && v->isNumber()) ? v->number : fallback;
		}

		bool flag(const std::string& name, bool fallback = false) const {
			const Value* v = find(name);
			return (v && v->type == Type::Bool) ? v->boolean : fallback;
		}

		std::string str(const std::string& name, const std::string& fallback = std::string()) const {
			const Value* v = find(name);
			return (v && v->type == Type::String) ? v->text : fallback;
		}

		/// <summary>读取对象中的数字数组</summary>
		std::vector<double> numbers(const std::string& name) const {
			std::vector<double> result;
			const Value* v = find(name);
			if (v && v->isArray()) {
				for (const Value& item : v->items) {
					if (item.isNumber()) {
						result.push_back(item.number);
					}
				}
			}
			return result;
		}
	};

	/// <summary>解析 JSON 文本；失败时返回 null 类型的 Value</summary>
	inline Value parse(const std::string& source) {
		size_t index = 0;

		struct Parser {
			const std::string& s;
			size_t& i;

			void skip() {
				while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) {
					i++;
				}
			}

			bool literal(const char* word) {
				size_t length = std::char_traits<char>::length(word);
				if (s.compare(i, length, word) == 0) {
					i += length;
					return true;
				}
				return false;
			}

			std::string parseString() {
				std::string out;
				if (i >= s.size() || s[i] != '"') {
					return out;
				}
				i++;
				while (i < s.size() && s[i] != '"') {
					if (s[i] == '\\' && i + 1 < s.size()) {
						i++;
						switch (s[i]) {
						case 'n': out += '\n'; break;
						case 'r': out += '\r'; break;
						case 't': out += '\t'; break;
						case 'b': out += '\b'; break;
						case 'f': out += '\f'; break;
						case 'u': {
							//仅处理基本的多字节转义：转成 UTF-8
							if (i + 4 < s.size()) {
								unsigned code = 0;
								for (int k = 1; k <= 4; k++) {
									char c = s[i + k];
									code <<= 4;
									if (c >= '0' && c <= '9') code |= (c - '0');
									else if (c >= 'a' && c <= 'f') code |= (c - 'a' + 10);
									else if (c >= 'A' && c <= 'F') code |= (c - 'A' + 10);
								}
								i += 4;
								if (code < 0x80) {
									out += static_cast<char>(code);
								}
								else if (code < 0x800) {
									out += static_cast<char>(0xC0 | (code >> 6));
									out += static_cast<char>(0x80 | (code & 0x3F));
								}
								else {
									out += static_cast<char>(0xE0 | (code >> 12));
									out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
									out += static_cast<char>(0x80 | (code & 0x3F));
								}
							}
							break;
						}
						default: out += s[i]; break;
						}
						i++;
					}
					else {
						out += s[i++];
					}
				}
				if (i < s.size()) {
					i++;	//跳过结束引号
				}
				return out;
			}

			Value parseValue() {
				skip();
				Value value;
				if (i >= s.size()) {
					return value;
				}
				char c = s[i];
				if (c == '{') {
					i++;
					value.type = Value::Type::Object;
					skip();
					if (i < s.size() && s[i] == '}') { i++; return value; }
					while (i < s.size()) {
						skip();
						std::string name = parseString();
						skip();
						if (i < s.size() && s[i] == ':') {
							i++;
						}
						value.members[name] = parseValue();
						skip();
						if (i < s.size() && s[i] == ',') { i++; continue; }
						if (i < s.size() && s[i] == '}') { i++; break; }
						break;
					}
					return value;
				}
				if (c == '[') {
					i++;
					value.type = Value::Type::Array;
					skip();
					if (i < s.size() && s[i] == ']') { i++; return value; }
					while (i < s.size()) {
						value.items.push_back(parseValue());
						skip();
						if (i < s.size() && s[i] == ',') { i++; continue; }
						if (i < s.size() && s[i] == ']') { i++; break; }
						break;
					}
					return value;
				}
				if (c == '"') {
					value.type = Value::Type::String;
					value.text = parseString();
					return value;
				}
				if (literal("true")) { value.type = Value::Type::Bool; value.boolean = true; return value; }
				if (literal("false")) { value.type = Value::Type::Bool; value.boolean = false; return value; }
				if (literal("null")) { return value; }

				//数字
				size_t start = i;
				while (i < s.size() &&
					(std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '-' ||
						s[i] == '+' || s[i] == '.' || s[i] == 'e' || s[i] == 'E')) {
					i++;
				}
				if (i > start) {
					value.type = Value::Type::Number;
					try {
						value.number = std::stod(s.substr(start, i - start));
					}
					catch (...) {
						value.number = 0.0;
					}
				}
				return value;
			}
		};

		Parser parser{ source, index };
		Value result = parser.parseValue();
		return result;
	}
}
