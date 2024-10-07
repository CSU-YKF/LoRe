# cmake_policy(SET CMP0144 OLD)
# cmake_policy(SET CMP0074 OLD)
cmake_policy(VERSION 3.10)

cmake_minimum_required(VERSION 2.6)
project(LoRe)

find_package(PCL 1.12.1 REQUIRED)
# find_package(PCL 1.14.1 REQUIRED) # for windows
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(LoRe
        src/trash/box_clipper.cpp
        src/filter.cpp
        src/trash/interactive_icp.cpp
        src/lore.cpp
        src/segmentation.cpp
        src/view.cpp
        include/view.h
        include/filter.h
        include/lore.h
        main.cpp
        include/segmentation.h
        include/type.h
        include/fit.h
        src/fit.cpp
        src/optimize.cpp
        include/optimize.h
        include/mainwindow.h src/mainwindow.cpp ui/mainwindow.ui)
target_link_libraries (LoRe ${PCL_LIBRARIES} )

install(TARGETS LoRe RUNTIME DESTINATION bin)
