//=====================================================================================
// Console.cpp
//=====================================================================================

#include "Console.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

namespace console {

	namespace {
		bool g_stdoutIsConsole = false;
		bool g_stderrIsConsole = false;
		bool g_initialized = false;

		/// <summary>UTF-8 转 UTF-16</summary>
		std::wstring utf8ToWide(const std::string& text) {
			if (text.empty()) {
				return std::wstring();
			}
			int length = MultiByteToWideChar(CP_UTF8, 0, text.c_str(),
				static_cast<int>(text.size()), nullptr, 0);
			if (length <= 0) {
				return std::wstring();
			}
			std::wstring wide(static_cast<size_t>(length), L'\0');
			MultiByteToWideChar(CP_UTF8, 0, text.c_str(),
				static_cast<int>(text.size()), &wide[0], length);
			return wide;
		}

		/// <summary>把一段 UTF-8 文本写到指定句柄</summary>
		void output(HANDLE handle, bool isConsole, const std::string& utf8Text) {
			//把字符串切成“行”，每行单独处理，保证换行符也能正确输出
			size_t start = 0;
			while (start <= utf8Text.size()) {
				size_t end = utf8Text.find('\n', start);
				std::string line = (end == std::string::npos)
					? utf8Text.substr(start)
					: utf8Text.substr(start, end - start);
				bool last = (end == std::string::npos);

				if (isConsole) {
					//控制台：用宽字符 API，绕过代码页，中文一定正确
					std::wstring wide = utf8ToWide(line);
					wide += L"\r\n";
					DWORD written = 0;
					WriteConsoleW(handle, wide.c_str(),
						static_cast<DWORD>(wide.size()), &written, nullptr);
				}
				else {
					//重定向到文件/管道：写 UTF-8 字节
					std::string bytes = line;
					bytes += "\n";
					DWORD written = 0;
					WriteFile(handle, bytes.data(),
						static_cast<DWORD>(bytes.size()), &written, nullptr);
				}

				if (last) {
					break;
				}
				start = end + 1;
			}
		}

		std::string formatText(const char* utf8Format, va_list args) {
			va_list probe;
			va_copy(probe, args);
			int needed = std::vsnprintf(nullptr, 0, utf8Format, probe);
			va_end(probe);
			if (needed <= 0) {
				return std::string();
			}
			std::vector<char> buffer(static_cast<size_t>(needed) + 1);
			std::vsnprintf(buffer.data(), buffer.size(), utf8Format, args);
			return std::string(buffer.data(), static_cast<size_t>(needed));
		}
	}

	void init() {
		if (g_initialized) {
			return;
		}
		g_initialized = true;

		//把控制台的输入/输出代码页切到 UTF-8：
		//这样其它第三方库直接写 UTF-8 字节时也能正确显示
		SetConsoleOutputCP(CP_UTF8);
		SetConsoleCP(CP_UTF8);

		//记录输出目标是不是控制台：是则走 WriteConsoleW，否则按字节写
		HANDLE out = GetStdHandle(STD_OUTPUT_HANDLE);
		HANDLE err = GetStdHandle(STD_ERROR_HANDLE);
		DWORD mode = 0;
		g_stdoutIsConsole = (out && out != INVALID_HANDLE_VALUE &&
			GetConsoleMode(out, &mode) != 0);
		g_stderrIsConsole = (err && err != INVALID_HANDLE_VALUE &&
			GetConsoleMode(err, &mode) != 0);
	}

	void write(const std::string& utf8Text) {
		output(GetStdHandle(STD_OUTPUT_HANDLE), g_stdoutIsConsole, utf8Text);
	}

	void writeError(const std::string& utf8Text) {
		output(GetStdHandle(STD_ERROR_HANDLE), g_stderrIsConsole, utf8Text);
	}

	void writeFormat(const char* utf8Format, ...) {
		va_list args;
		va_start(args, utf8Format);
		std::string text = formatText(utf8Format, args);
		va_end(args);
		write(text);
	}

	void writeErrorFormat(const char* utf8Format, ...) {
		va_list args;
		va_start(args, utf8Format);
		std::string text = formatText(utf8Format, args);
		va_end(args);
		writeError(text);
	}
}
