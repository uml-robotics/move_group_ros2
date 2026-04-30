cmake_minimum_required(VERSION 3.8)
project(move_group_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(gpd_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(std_srvs REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CartesianMoveToPose.srv"
  "srv/MoveToNamedPose.srv"
  "srv/MoveToPose.srv"
  DEPENDENCIES
    geometry_msgs
    std_msgs
    sensor_msgs
    pcl_msgs
)

include_directories(
  ${PCL_INCLUDE_DIRS}
)


# Create executable
add_executable(cartesian_move_to_pose_service src/cartesian_move_to_pose_service.cpp)
ament_target_dependencies(cartesian_move_to_pose_service
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(move_to_named_pose_service src/move_to_named_pose_service.cpp)
ament_target_dependencies(move_to_named_pose_service
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(move_to_pose_service src/move_to_pose_service.cpp)
ament_target_dependencies(move_to_pose_service
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(reach_to_grasp_service src/reach_to_grasp_service.cpp)
ament_target_dependencies(reach_to_grasp_service
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
)
target_link_libraries(reach_to_grasp_service ${PCL_LIBRARIES})

install(TARGETS 
  cartesian_move_to_pose_service
  move_to_named_pose_service
  move_to_pose_service
  reach_to_grasp_service
  DESTINATION lib/${PROJECT_NAME})

# Link generated service headers
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(cartesian_move_to_pose_service ${cpp_typesupport_target})
# target_link_libraries(move_to_named_pose_service ${cpp_typesupport_target})
# target_link_libraries(move_to_pose_service ${cpp_typesupport_target})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cartesian_move_to_pose_service ${cpp_typesupport_target})
target_link_libraries(move_to_named_pose_service ${cpp_typesupport_target})
target_link_libraries(move_to_pose_service ${cpp_typesupport_target})
target_link_libraries(reach_to_grasp_service ${cpp_typesupport_target})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
