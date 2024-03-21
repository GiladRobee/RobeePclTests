cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(${PROJECT_NAME} LANGUAGES CXX)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo src/viewer_main.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})
target_include_directories(
    pcl_visualizer_demo 
    PUBLIC 
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
)