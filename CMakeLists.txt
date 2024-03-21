cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(${PROJECT_NAME} LANGUAGES CXX)

find_package(PCL 1.2 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo src/viewer_main.cpp)
target_link_libraries(pcl_visualizer_demo PRIVATE ${PCL_LIBRARIES})
target_link_libraries(pcl_visualizer_demo PRIVATE ${OpenCV_LIBS})
target_link_libraries(pcl_visualizer_demo PRIVATE realsense2::realsense2)
target_link_libraries(pcl_visualizer_demo PRIVATE  Eigen3::Eigen)
target_include_directories(
    pcl_visualizer_demo 
    PUBLIC 
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
)