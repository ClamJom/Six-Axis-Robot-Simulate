#pragma once

//=====================================================================================
// Console.h
//
// 控制台输出工具，用来解决中文乱码。
//
// 问题背景：源码中的中文字面量在内存里是 UTF-8 字节（工程用 /utf-8 编译），
// 但 Windows 控制台默认代码页是本地语言（简体中文为 936/GBK）。
// 直接用 printf 把 UTF-8 字节写进 GBK 控制台，就会显示成乱码。
//
// 处理方式：
//   * 输出目标是控制台时，用 WriteConsoleW 直接输出宽字符，完全绕过代码页；
//   * 输出被重定向（写入文件/管道）时，写 UTF-8 字节，符合现代工具的习惯；
//   * 同时把控制台代码页也切到 UTF-8，使其它库（如 VTK/日志）的输出也正常。
//=====================================================================================

#include <string>

namespace console {

	/// <summary>
	/// 初始化控制台：切换代码页到 UTF-8，并记录输出是否为控制台。
	/// 应在任何输出之前调用（通常放在 main 开头）。
	/// </summary>
	void init();

	/// <summary>向标准输出写一行（自动补换行），参数为 UTF-8 字符串</summary>
	void write(const std::string& utf8Text);

	/// <summary>向标准错误写一行，参数为 UTF-8 字符串</summary>
	void writeError(const std::string& utf8Text);

	/// <summary>
	/// 格式化输出（UTF-8），内部使用 vsnprintf，会自动换行。
	/// </summary>
	void writeFormat(const char* utf8Format, ...);

	/// <summary>格式化输出到标准错误（UTF-8），会自动换行</summary>
	void writeErrorFormat(const char* utf8Format, ...);
}
