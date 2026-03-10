cmake_minimum_required(VERSION 3.8)
project(concrete_block_registration_teaser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Open3D CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(concrete_block_perception REQUIRED)

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../concrete_block_perception/external/teaser-plusplus/CMakeLists.txt)
  message(FATAL_ERROR
    "TEASER++ submodule missing at ../external/teaser-plusplus. "
    "Run: git submodule update --init --recursive")
endif()

set(BUILD_DOC OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(BUILD_MATLAB_BINDINGS OFF CACHE BOOL "" FORCE)
set(BUILD_TEASER_FPFH OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../concrete_block_perception/external/teaser-plusplus ${CMAKE_CURRENT_BINARY_DIR}/teaser-plusplus EXCLUDE_FROM_ALL)

set(PERCEPTION_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../concrete_block_perception)
add_subdirectory(${PERCEPTION_ROOT}/external/pcd_block_estimation ${CMAKE_CURRENT_BINARY_DIR}/pcd_block_estimation EXCLUDE_FROM_ALL)

add_executable(block_registration_teaser_node
  src/block_registration_teaser_node.cpp
  src/teaser_registration_pipeline.cpp
  ${PERCEPTION_ROOT}/src/registration/registration_config.cpp
  ${PERCEPTION_ROOT}/src/registration/ros_debug_helpers.cpp
  ${PERCEPTION_ROOT}/src/utils/io_utils.cpp
)

target_include_directories(block_registration_teaser_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PERCEPTION_ROOT}/include>
  $<BUILD_INTERFACE:${PERCEPTION_ROOT}/external/pcd_block_estimation/include>
)

target_link_libraries(block_registration_teaser_node
  teaserpp::teaser_registration
  pcd_block_estimation
  Open3D::Open3D
  ${OpenCV_LIBS}
)

ament_target_dependencies(block_registration_teaser_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  vision_msgs
  visualization_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  concrete_block_perception
)

install(TARGETS
  block_registration_teaser_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
