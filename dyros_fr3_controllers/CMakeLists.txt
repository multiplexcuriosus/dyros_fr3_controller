cmake_minimum_required(VERSION 3.5)
project(dyros_fr3_controllers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake                REQUIRED)
find_package(controller_interface       REQUIRED)
find_package(franka_msgs                REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs              REQUIRED)
find_package(hardware_interface         REQUIRED)
find_package(pinocchio                  REQUIRED)
find_package(pluginlib                  REQUIRED)
find_package(rclcpp                     REQUIRED)
find_package(rclcpp_lifecycle           REQUIRED)
find_package(std_srvs                   REQUIRED)
find_package(std_msgs                   REQUIRED)
find_package(Eigen3                     REQUIRED)
find_package(dyros_fr3_interfaces       REQUIRED)

set(CONTROLLER_LIB ${PROJECT_NAME}_plugin)

add_library(${CONTROLLER_LIB} SHARED
  src/my_effort_controller.cpp
)

target_include_directories(${CONTROLLER_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${CONTROLLER_LIB}
  controller_interface
  franka_msgs
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pinocchio
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
  std_msgs
  dyros_fr3_interfaces
)

pluginlib_export_plugin_description_file(controller_interface dyros_fr3_controllers_plugin.xml)

install(TARGETS ${CONTROLLER_LIB}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY utils/
  DESTINATION lib/${PROJECT_NAME}/utils
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  ${PROJECT_NAME}/my_effort_controller/ros2_qt_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME My_effort_controllerQT
)

ament_export_include_directories(include)
ament_export_libraries(${CONTROLLER_LIB})

ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)

ament_package()