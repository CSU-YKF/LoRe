//
// Created by Rvosuke on 2024/9/19.
//

#ifndef LORE_VIEW_H
#define LORE_VIEW_H

#include "type.h"

#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>


// 获取当前时间戳
std::string getCurrentTimestamp();

// 辅助函数：将单个参数转换为字符串
template<typename T>
std::string toString(const T &value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
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

// 只打印一次时间戳
template<typename T, typename... Args>
void printt(const T &first, const Args&... args) {
    std::cout << "[" << getCurrentTimestamp() << "] ";
    printWithTimestamp(first, args...);  // 调用递归函数处理参数
}

void view_cloud(PointCloud &cloud, const std::string &window_name = "Cloud Viewer");

/**
 * @brief Visualizes the point cloud data with a fitted circle.
 * Create a PCL Viewer for visualization and display the original point cloud and the fitted circle
 */
void view_cloud(PointCloud &cloud, const std::vector<float>& circle_params);


#endif // LORE_VIEW_H
