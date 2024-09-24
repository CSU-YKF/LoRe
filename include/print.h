//
// Created by Rvosuke on 2024/9/19.
//
#ifndef LORE_PRINT_H
#define LORE_PRINT_H

#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>

// 获取当前时间戳
std::string getCurrentTimestamp();

// 辅助函数：将单个参数转换为字符串
template<typename T>
std::string toString(const T &value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

// 只打印一次时间戳
template<typename T, typename... Args>
void printt(const T &first, const Args&... args) {
    std::cout << "[" << getCurrentTimestamp() << "] ";
    printWithTimestamp(first, args...);  // 调用递归函数处理参数
}

// 基函数：处理最后一个参数
template<typename T>
void printWithTimestamp(const T &last) {
    std::cout << toString(last) << std::endl;
}

// 递归函数：处理变参模板
template<typename T, typename... Args>
void printWithTimestamp(const T &first, const Args&... args) {
    std::cout << toString(first);  // 输出当前参数
    printWithTimestamp(args...);  // 递归处理剩下的参数
}

#endif // LORE_PRINT_H