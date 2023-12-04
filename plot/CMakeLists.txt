cmake_minimum_required(VERSION 3.16)
project(plot)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  # tf2_ros
  # tf2_eigen
  # cv_bridge
  # image_transport
  # nav_msgs
  # sensor_msgs
  # visualization_msgs
  # lwx add
  # sensor_pub
  # message_generation
  # message_runtime
  #camera_models
  std_msgs
  roslib
  # actionlib
  # actionlib_msgs
  # loop_action
  message_filters
  # the end.
  # pcl_ros
  myodom
  atp_info
  myresult

)

find_package(yaml-cpp REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES binocular_sensor
  CATKIN_DEPENDS roscpp std_msgs rosmsg
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIR}
  "./"
  "/usr/local/include"
)

add_executable(plot
  src/main.cpp
  src/myodom.cpp
)

target_link_libraries(plot 
  ${catkin_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############
## 添加可执行文件或者库文件，Mark executables and/or libraries for installation
install(TARGETS plot
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  
file(GLOB EXTERN_LIB ../../devel/lib/*.so)

MESSAGE(STATUS "EXTERN_LIB= " ${EXTERN_LIB})

install(FILES ${EXTERN_LIB}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


# ---ros2---
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(message_filters REQUIRED)
# find_package(myodom REQUIRED)
# find_package(atp_info REQUIRED)
# # find_package(Boost REQUIRED) # circular buffer
# find_package(myresult REQUIRED)

# add_executable(plot
#   src/main.cpp
#   src/myodom.cpp
# )

# ament_target_dependencies(plot
#   rclcpp
#   std_msgs 
#   sensor_msgs
#   message_filters
#   myodom
#   atp_info
#   myresult
# )

# install(TARGETS
#   plot
#   DESTINATION lib/${PROJECT_NAME}
# )


# ament_package()
