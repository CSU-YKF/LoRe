cmake_policy(SET CMP0144 OLD)
cmake_policy(SET CMP0074 OLD)
cmake_policy(VERSION 3.10)

cmake_minimum_required(VERSION 2.6)
project(LoRe)
 
find_package(PCL 1.14.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable(LoRe src/BoxClipper3D.cpp
        src/pcd_reader.cpp
        src/passthrough.cpp
        src/passthrough.cpp
        src/voxel_grid.cpp
        src/statistical_removal.cpp
        src/statistical_removal.cpp
        src/icp.cpp
        src/icp.cpp
        src/interactive_icp.cpp
        src/interactive_icp.cpp
        src/lore.cpp
        src/lore.cpp
        src/cluster_extraction.cpp)
target_link_libraries (LoRe ${PCL_LIBRARIES} )
 
install(TARGETS LoRe RUNTIME DESTINATION bin)