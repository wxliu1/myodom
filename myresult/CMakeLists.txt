cmake_minimum_required(VERSION 3.13)
project(myresult)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmsg
  std_msgs
  message_generation
  message_runtime
)

add_message_files(
  FILES
  MyResult.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dynamic_connect
 CATKIN_DEPENDS roscpp rosmsg message_runtime
#  DEPENDS system_lib
)