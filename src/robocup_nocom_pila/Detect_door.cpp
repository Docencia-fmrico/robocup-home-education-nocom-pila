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

#include "robocup_nocom_pila/Detect_door.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"

namespace robocup_nocom_pila
{

Detect_door::Detect_door(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name, config), counter_(0)
{
  sub_laser_ = nh.subscribe("/scan", 1, &Detect_door::LaserCallback, this);
}

void
Detect_door::halt()
{
  ROS_INFO("Detect_door halt");
}


void Detect_door :: LaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  n_door = getInput<int>("r_door").value();
  max_array_ = (msg->angle_max - msg->angle_min) / msg->angle_increment;

  max_sweep_left = max_array_ - max_array_/10;  // initialization of every angle of detection
  min_sweep_left = max_array_;
  min_sweep_right = 0;
  max_sweep_right = max_array_/10;

  for (int i = max_sweep_left; i < min_sweep_left; i++)  // reads every value in the range (LEFT)
  {
    laser_detected_ = msg->ranges[i] >= DISTANCE_;  // updates the value of laser_detected_ (true if is under 0.4m)
    if(!laser_detected_)
    {
      setOutput<int>("w_door", n_door++);
      door = true;
      break;
    }
  }
  
  for (int i = min_sweep_right; i < max_sweep_right; i++)  // reads every value in the range (RIGTH)
  {
    if(!laser_detected_)  // to prevent the case that the last value of the left range detect an object and the rigth range overwrite it
    {
      break;
    }
    laser_detected_ = msg->ranges[i] >= DISTANCE_;  // Updates the value of laser_detected_ (true if is under 0.4m)
    dist = msg->ranges[min_sweep_right];
    if(!laser_detected_)
    {
      door = true;
      setOutput<int>("w_door", n_door++);
      break;
    }
  }  

} 


BT::NodeStatus
Detect_door::tick()
{
  ROS_INFO("Detect_door tick");
  std::cerr << dist << std::endl;
  if(door)
    return BT::NodeStatus::RUNNING;
  else
    return BT::NodeStatus::SUCCESS;
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Detect_door>("Detect_door");
}
