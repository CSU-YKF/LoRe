cmake_minimum_required(VERSION 3.16)

# 设置项目名称和版本
project(LoRe VERSION 0.1 LANGUAGES CXX)

# 启用 Qt 的自动化功能
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 PCL 库
find_package(PCL 1.12.1 REQUIRED)
# 如果需要 Windows 上的 PCL 版本，使用以下行
# find_package(PCL 1.14.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 查找 Qt 库（支持 Qt5 和 Qt6）
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# 定义项目的源文件
set(PROJECT_SOURCES
        main.cpp
        src/filter.cpp
        src/fit.cpp
        src/optimize.cpp
        src/view.cpp
        src/segmentation.cpp
        src/lore.cpp
        src/trash/box_clipper.cpp
        src/trash/interactive_icp.cpp
        src/mainwindow.cpp
        include/filter.h
        include/lore.h
        include/segmentation.h
        include/type.h
        include/fit.h
        include/optimize.h
        include/view.h
        include/mainwindow.h
        ui/mainwindow.ui
)

# 判断 Qt 版本并创建可执行文件
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(LoRe
            MANUAL_FINALIZATION
            ${PROJECT_SOURCES}
            resources.qrc
    )
else()
    if(ANDROID)
        add_library(LoRe SHARED ${PROJECT_SOURCES})
    else()
        add_executable(LoRe ${PROJECT_SOURCES})
    endif()
endif()

# 链接 PCL 库
target_link_libraries(LoRe PRIVATE ${PCL_LIBRARIES})

# 链接 Qt Widgets 库
target_link_libraries(LoRe PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# 定义平台特定的设置
if(${QT_VERSION} VERSION_LESS 6.1.0)
    set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.LoRe)
endif()

set_target_properties(LoRe PROPERTIES
        ${BUNDLE_ID_OPTION}
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
        MACOSX_BUNDLE TRUE
        WIN32_EXECUTABLE TRUE
)

# 安装路径设置
include(GNUInstallDirs)
install(TARGETS LoRe
        BUNDLE DESTINATION .
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Qt6 的最终可执行文件处理
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(LoRe)
endif()
