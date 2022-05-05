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

#include "robocup_nocom_pila/Detect_person_cml.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace robocup_nocom_pila
{

Detect_person_cml::Detect_person_cml(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
  objects_bbx = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &Detect_person_cml::DetectPersonBBXCallback, this);
  objects_image = nh_.subscribe("/camera/depth/image_raw", 1, &Detect_person_cml::DetectPersonImageCallback, this);  
}
BT::NodeStatus
Detect_person_cml::tick()
{
  ROS_INFO("Detect_person_cml tick");
  if (prev_dist_ == -1.0 && dist_w < 5 && dist_w > 0.0)
  {
    prev_dist_ = dist_w;
  }
  if(is_person == true && dist_w != 0.0)
  {
    std::cerr << "Distancia Real: " << dist_w <<std::endl;
    diff_dist_ = prev_dist_- dist_w;
    if (diff_dist_ < 0.0)
      {
        diff_dist_ = -diff_dist_;
      }
    std::cerr << prev_dist_ << " - "<< dist_w <<  " = " << prev_dist_- dist_w << " = " << diff_dist_ <<std::endl;
    if (diff_dist_ < 0.5)
    {
      if (dist_w < 4.5 && dist_w > 0.0)
      {
      //std::cerr << "HAY PERSONA" << std::endl;
        setOutput<float>("w_dist", dist_w);
        setOutput<double>("w_centre", centre_w);
        cont = 0;
        is_person = false;
        prev_dist_ = dist_w;
        return BT::NodeStatus::SUCCESS;
      }
      else
      std::cerr << "posible nan         " << dist_w <<std::endl;
      if (cont >= 20)
      {
        std::cerr << "Acércate o cierra las piernas, porfa "<< std::endl;
      }
      cont++;
      return BT::NodeStatus::RUNNING;
    }
    else
    {
      if (cont >= 20)
      {
        std::cerr << "No encuentro persona, AYUDA "<< std::endl;
      }
      cont++;
      return BT::NodeStatus::RUNNING;
    }  
  }
  else 
  {
    std::cerr << "!! is_person == true && dist_w != 0.0      " << dist_w <<std::endl;
    return BT::NodeStatus::RUNNING;
  }
}


void
Detect_person_cml::halt()
{
    ROS_INFO("Detect_person_cml halt");
}

void Detect_person_cml::DetectPersonBBXCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  //ROS_INFO("DetectPersonBBXCallback");
  for (const auto & box : boxes->bounding_boxes)
  {
    if (box.Class == "person")
    {
      //std::cerr << "CALLBACK PX" << std::endl;
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;
      is_person = true;
    }
  } 
}

void Detect_person_cml::DetectPersonImageCallback(const sensor_msgs::ImageConstPtr& image)
{
  //ROS_INFO("DetectPersonImageCallback");
  sleep(0.1);
  try
  {
    img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  dist_w = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f;
  //dist_w = dist_w * 1000; //--------------------Para Simulador
  centre_w = (px - 300) / 300.0;


  if (&dist_w == NULL)
  {
    std::cerr << "dist_w == NULL" << std::endl;
    dist_w = 0.0;
  }
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Detect_person_cml>("Detect_person_cml");
}
