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
  gb_dialog
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
  gb_dialog
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(pid SHARED src/robocup_nocom_pila/PIDController.cpp)

add_library(nc_go_operator_cml SHARED src/robocup_nocom_pila/Go_operator_cml.cpp)
add_library(nc_speak_person_cml SHARED src/robocup_nocom_pila/Speak_person_cml.cpp)
add_library(nc_detect_bag_cml SHARED src/robocup_nocom_pila/Detect_bag_cml.cpp)
add_library(nc_detect_person_cml SHARED src/robocup_nocom_pila/Detect_person_cml.cpp)
add_library(nc_follow_person_cml SHARED src/robocup_nocom_pila/Follow_person_cml.cpp)
add_library(nc_go_home_cml SHARED src/robocup_nocom_pila/Go_home_cml.cpp)

add_library(nc_describe_person_fmm SHARED src/robocup_nocom_pila/Describe_person_fmm.cpp)
add_library(nc_detect_person_fmm SHARED src/robocup_nocom_pila/Detect_person_fmm.cpp)
add_library(nc_go_home_fmm SHARED src/robocup_nocom_pila/Go_home_fmm.cpp)
add_library(nc_go_operator_fmm SHARED src/robocup_nocom_pila/Go_operator_fmm.cpp)
add_library(nc_go_person_fmm SHARED src/robocup_nocom_pila/Go_person_fmm.cpp)
add_library(nc_speak_person_fmm SHARED src/robocup_nocom_pila/Speak_person_fmm.cpp)

list(APPEND plugin_libs
  nc_go_operator_cml
  nc_speak_person_cml
  nc_detect_bag_cml
  nc_detect_person_cml
  nc_follow_person_cml
  nc_go_home_cml

  nc_describe_person_fmm
  nc_detect_person_fmm
  nc_go_home_fmm
  nc_go_operator_fmm
  nc_go_person_fmm
  nc_speak_person_fmm
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES} pid)
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(nodo_cml 
  src/nodo_cml.cpp
)
target_link_libraries(nodo_cml ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

add_executable(nodo_fmm 
  src/nodo_fmm.cpp
)
target_link_libraries(nodo_fmm ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

add_executable(chatbot 
  src/chatbot.cpp
)
target_link_libraries(chatbot ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})

roslint_cpp(
  src/nodo_cml.cpp
  src/nodo_fmm.cpp

  include/robocup_nocom_pila/Go_operator_cml.h src/robocup_nocom_pila/Go_operator_cml.cpp
  include/robocup_nocom_pila/Speak_person_cml.h src/robocup_nocom_pila/Speak_person_cml.cpp
  include/robocup_nocom_pila/Detect_bag_cml.h src/robocup_nocom_pila/Detect_bag_cml.cpp
  include/robocup_nocom_pila/Detect_person_cml.h src/robocup_nocom_pila/Detect_person_cml.cpp
  include/robocup_nocom_pila/Follow_person_cml.h src/robocup_nocom_pila/Follow_person_cml.cpp
  include/robocup_nocom_pila/Go_home_cml.h src/robocup_nocom_pila/Go_home_cml.cpp
  
  include/robocup_nocom_pila/Describe_person_fmm.h src/robocup_nocom_pila/Describe_person_fmm.cpp
  include/robocup_nocom_pila/Detect_person_fmm.h src/robocup_nocom_pila/Detect_person_fmm.cpp
  include/robocup_nocom_pila/Go_home_fmm.h src/robocup_nocom_pila/Go_home_fmm.cpp
  include/robocup_nocom_pila/Go_operator_fmm.h src/robocup_nocom_pila/Go_operator_fmm.cpp
  include/robocup_nocom_pila/Go_person_fmm.h src/robocup_nocom_pila/Go_person_fmm.cpp
  include/robocup_nocom_pila/Speak_person_fmm.h src/robocup_nocom_pila/Speak_person_fmm.cpp
  
  include/robocup_nocom_pila/PIDController.hpp src/robocup_nocom_pila/PIDController.cpp
  src/chatbot.cpp
  src/nav.cpp
)
