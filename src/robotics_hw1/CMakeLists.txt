cmake_minimum_required(VERSION 3.0.2)
project(robotics_hw1)

# set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
# set(CMAKE_C_COMPILER "/usr/bin/clang")

find_package(
  catkin REQUIRED
  COMPONENTS
  roscpp
  message_generation
  message_filters
    nav_msgs
    std_msgs
  dynamic_reconfigure
  tf2_ros
)

add_message_files(
  FILES
    MotorSpeed.msg
    CalibratedParamsOdom.msg
    CalibratedParamsGtPose.msg
    IntegratedOdometry.msg
)

add_service_files(
  FILES
  SetOdometry.srv
  )


generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
)

generate_dynamic_reconfigure_options (
  cfg/OdometryCalculator.cfg
  )


catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    message_runtime
    tf2_ros
)


include_directories(
include ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_calculator src/odometry_calculator.cpp)
target_link_libraries(odometry_calculator ${catkin_LIBRARIES})
add_dependencies(odometry_calculator ${catkin_EXPORTED_TARGETS})
add_dependencies(odometry_calculator ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(odometry_calculator ${PROJECT_NAME}_gencfg)

add_executable(parameters_calibrator src/parameters_calibrator.cpp)
target_link_libraries(parameters_calibrator ${catkin_LIBRARIES})
add_dependencies(parameters_calibrator ${catkin_EXPORTED_TARGETS})
add_dependencies(parameters_calibrator ${${PROJECT_NAME}_EXPORTED_TARGETS})
