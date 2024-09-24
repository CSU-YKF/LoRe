cmake_policy(SET CMP0144 OLD)
cmake_policy(SET CMP0074 OLD)
cmake_policy(VERSION 3.10)

cmake_minimum_required(VERSION 2.6)
project(LoRe)
 
find_package(PCL 1.14.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable(LoRe
        src/box_clipper.cpp
        src/filter.cpp
        src/interactive_icp.cpp
        src/lore.cpp
        src/segmentation.cpp
        src/print.cpp
        include/print.h
        include/filter.h
        include/lore.h
        main.cpp
        include/segmentation.h
        include/type.h)
target_link_libraries (LoRe ${PCL_LIBRARIES} )
 
install(TARGETS LoRe RUNTIME DESTINATION bin)