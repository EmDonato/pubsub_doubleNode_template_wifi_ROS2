cmake_minimum_required(VERSION 3.8)
project(pubsub_template_uros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
install(TARGETS talker DESTINATION lib/${PROJECT_NAME})

add_executable(listener src/listener.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
install(TARGETS listener DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

