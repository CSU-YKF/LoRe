cmake_minimum_required(VERSION 2.6)
project(BoxClipper3D)
 
find_package(PCL 1.14.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable(BoxClipper3D BoxClipper3D.cpp )
target_link_libraries (BoxClipper3D ${PCL_LIBRARIES} )
 
install(TARGETS BoxClipper3D RUNTIME DESTINATION bin)