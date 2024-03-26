cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(${PROJECT_NAME} LANGUAGES CXX)
find_package(PCL 1.2 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo src/viewer_main.cpp)
add_executable (pcl_benchmark src/pcl_benchmark.cpp)
add_executable (pcl_algorithm src/pcl_algo_main.cpp)



target_link_libraries(pcl_visualizer_demo PRIVATE
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  realsense2::realsense2
  Eigen3::Eigen
  spdlog::spdlog_header_only
)

target_link_libraries(pcl_algorithm PRIVATE
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  realsense2::realsense2
  Eigen3::Eigen
  spdlog::spdlog_header_only
)

target_link_libraries(pcl_benchmark PRIVATE
  ${PCL_LIBRARIES}
  ${CMAKE_SOURCE_DIR}/../CppBenchmark/bin/libcppbenchmark.a
  ${CMAKE_SOURCE_DIR}/../CppBenchmark/temp/modules/libcpp-optparse.a
  ${CMAKE_SOURCE_DIR}/../CppBenchmark/temp/modules/libHdrHistogram.a
  ${CMAKE_SOURCE_DIR}/../CppBenchmark/temp/modules/libzlib.a
  realsense2::realsense2
  Eigen3::Eigen
  ${OpenCV_LIBS}
  spdlog::spdlog_header_only
)

target_include_directories(
  pcl_algorithm 
    PUBLIC 
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
)

target_include_directories(
    pcl_visualizer_demo 
    PUBLIC 
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
)

target_include_directories(
    pcl_benchmark 
    PUBLIC 
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
)
target_include_directories(
    pcl_benchmark
    PUBLIC
      ${CMAKE_SOURCE_DIR}/../CppBenchmark/include 
      ${CMAKE_SOURCE_DIR}/../CppBenchmark/source
      ${CMAKE_SOURCE_DIR}/../temp
      ${CMAKE_SOURCE_DIR}/../bin
      ${CMAKE_SOURCE_DIR}/../build 
)