cmake_minimum_required(VERSION 2.8.3)
project(robocup_nocom_pila)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint

  sensor_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros_msgs
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  kobuki_msgs
  behaviortree_cpp_v3 
  sensor_msgs 
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(pid SHARED src/robocup_nocom_pila/PIDController.cpp)

add_library(nc_bag_taken SHARED src/robocup_nocom_pila/Bag_taken.cpp)
add_library(nc_detect_bag SHARED src/robocup_nocom_pila/Detect_bag.cpp)
add_library(nc_detect_person SHARED src/robocup_nocom_pila/Detect_person.cpp)
add_library(nc_follow_person_ SHARED src/robocup_nocom_pila/Follow_person.cpp)
add_library(nc_listen_person SHARED src/robocup_nocom_pila/Listen_person.cpp)
add_library(nc_search_person SHARED src/robocup_nocom_pila/Search_person.cpp)
add_library(nc_take_bag SHARED src/robocup_nocom_pila/Take_bag.cpp)
add_library(nc_go_home SHARED src/robocup_nocom_pila/Go_home.cpp)
list(APPEND plugin_libs
  nc_bag_taken
  nc_detect_bag
  nc_detect_person
  nc_follow_person_
  nc_listen_person
  nc_search_person
  nc_take_bag
  nc_go_home
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES} pid)
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(nodo_cml 
  src/nodo_cml.cpp
)
target_link_libraries(nodo_cml ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

roslint_cpp(
  src/nodo_cml.cpp

  include/robocup_nocom_pila/Bag_taken.h src/robocup_nocom_pila/Bag_taken.cpp
  include/robocup_nocom_pila/Detect_bag.h src/robocup_nocom_pila/Detect_bag.cpp
  include/robocup_nocom_pila/Detect_person.h src/robocup_nocom_pila/Detect_person.cpp
  include/robocup_nocom_pila/Follow_person.h src/robocup_nocom_pila/Follow_person.cpp
  include/robocup_nocom_pila/Listen_person.h src/robocup_nocom_pila/Listen_person.cpp
  include/robocup_nocom_pila/Search_person.h src/robocup_nocom_pila/Search_person.cpp
  include/robocup_nocom_pila/Take_bag.h src/robocup_nocom_pila/Take_bag.cpp
  include/robocup_nocom_pila/Go_home.h src/robocup_nocom_pila/Go_home.cpp
  
  include/robocup_nocom_pila/PIDController.hpp src/robocup_nocom_pila/PIDController.cpp
)
