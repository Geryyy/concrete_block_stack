cmake_minimum_required(VERSION 3.8)
project(concrete_block_world_model_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Block.msg"
  "msg/BlockArray.msg"
  "msg/PlanningScene.msg"
  "msg/PlanningSceneObject.msg"
  "srv/GetCoarseBlocks.srv"
  "srv/GetPlanningScene.srv"
  "srv/RunPoseEstimation.srv"
  "srv/SetBlockTaskStatus.srv"
  "srv/SetPerceptionMode.srv"
  "srv/UpsertBlock.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
