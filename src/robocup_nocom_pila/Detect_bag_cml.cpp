// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "robocup_nocom_pila/Detect_bag_cml.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace robocup_nocom_pila
{

Detect_bag_cml::Detect_bag_cml(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
    bagDarknet = nh.subscribe("/darknet_ros/bounding_boxes",1,&Detect_bag_cml::ImageCallback,this);
    orderDiag = nh.subscribe("/sound/order",1,&Detect_bag_cml::OrderCallback,this);

}

void
Detect_bag_cml::get_order()
{
  ROS_INFO("Detect_bag_cml get order");
  sleep(1);
  forwarder.speak("Which bag?");
  sleep(1);
  forwarder.listen();
  sleep(1);
}

void
Detect_bag_cml::ImageCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bagDarknet)
{
  for (const auto & box : bagDarknet->bounding_boxes) {
    
    px_max = box.xmax;
    px_min = box.xmin;
    std::cerr << box.Class << std::endl;
    std::cerr << "xmax: "<< px_max << "\t"<< "xmin: "<< px_min<< std::endl;
        
  }

}

void
Detect_bag_cml::OrderCallback(const std_msgs::String::ConstPtr& msg)
{
  order = msg->data;
  std::cerr << order << std::endl;
  setOutput<std::string>("r_order", order);
}

void
Detect_bag_cml::halt()
{
  ROS_INFO("Detect_bag_cml halt");
}

BT::NodeStatus
Detect_bag_cml::tick()
{
  ROS_INFO("Detect_bag_cml tick");

  std::cerr << order << std::endl;

  //forwarder.speak("Which bag baby?");

  if( order != "" )
  {
    num = 1;
  }

  if (repeticiones == 0)
  {
    switch(num)
    {
      case 0:
        get_order();
        if(px_max >= 500 || order == "left")
        {
          //order = "left";
          repeticiones = 1;
        }
        else if(px_min <= 100 || order == "right" )
        {
          //order == "right";
          repeticiones = 1;
        }
        
        return BT::NodeStatus::RUNNING;
      break;
      case 1:
        //forwarder.speak("The bag is on"+order+"side");
        num = 0; 
        order = "";
        return BT::NodeStatus::SUCCESS;
      break;
    }
  }
  else
  {
    repeticiones = 0;
    return BT::NodeStatus::RUNNING;
  }
  

}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Detect_bag_cml>("Detect_bag_cml");
}
