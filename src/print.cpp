//
// Created by Rvosuke on 2024/9/19.
//
#include "../include/print.h"

std::string getCurrentTimestamp() {
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    // 使用安全的 localtime_s 来获取本地时间
    struct tm timeInfo{};
    localtime_s(&timeInfo, &in_time_t);

    // 将时间格式化为字符串
    std::stringstream ss;
    ss << std::put_time(&timeInfo, "%Y-%m-%d %X");

    return ss.str();
}
