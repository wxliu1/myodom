cmake_minimum_required(VERSION 3.16)
project(plot)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(myodom REQUIRED)
find_package(atp_info REQUIRED)
# find_package(Boost REQUIRED) # circular buffer
find_package(myresult REQUIRED)

add_executable(plot
  src/main.cpp
  src/myodom.cpp
)

ament_target_dependencies(plot
  rclcpp
  std_msgs 
  sensor_msgs
  message_filters
  myodom
  atp_info
  myresult
)

install(TARGETS
  plot
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
