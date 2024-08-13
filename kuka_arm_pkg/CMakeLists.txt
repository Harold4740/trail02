cmake_minimum_required(VERSION 3.8)
project(kuka_arm_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core  REQUIRED)
find_package(graph_msgs  REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)






set(Moveit_includes
rclcpp
moveit_ros_planning
moveit_ros_planning_interface
moveit_core
graph_msgs
sensor_msgs

)



add_executable(jtc_node src/joint_trajectory_controller.cpp)
ament_target_dependencies(jtc_node rclcpp trajectory_msgs)

add_executable(moveit_drive src/motion_drive_moveit.cpp)
ament_target_dependencies(moveit_drive ${Moveit_includes})








install(TARGETS
  jtc_node moveit_drive
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY src urdf meshes config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)


# Install Python modules
install(PROGRAMS src/gazebo_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python modules
install(PROGRAMS src/cam_coord.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install Python modules
install(PROGRAMS src/pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)





ament_package()
