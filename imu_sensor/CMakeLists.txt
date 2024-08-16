cmake_minimum_required(VERSION 3.8)
project(imu_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Trajectory.msg"
  "msg/RawIMUData.msg"
  "msg/IMUData.msg"
)

add_executable(imu_sensor_exe src/imu_sensor.cpp)
rosidl_target_interfaces(imu_sensor_exe ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(imu_sensor_exe rclcpp)

add_executable(fake_trajectory src/fake_trajectory_publisher.cpp)
rosidl_target_interfaces(fake_trajectory ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(fake_trajectory rclcpp)


install(TARGETS
  imu_sensor_exe
  fake_trajectory
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
