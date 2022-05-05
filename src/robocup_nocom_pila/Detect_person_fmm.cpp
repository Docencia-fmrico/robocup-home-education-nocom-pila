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

#include "robocup_nocom_pila/Detect_person_fmm.h"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace robocup_nocom_pila
{

Detect_person_fmm::Detect_person_fmm(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), counter_(0)
{
  objects_bbx = nh.subscribe("/darknet_ros/bounding_boxes", 1, &Detect_person_fmm::DetectPersonBBXCallback, this);
  objects_image = nh.subscribe("/camera/depth/image_raw", 1, &Detect_person_fmm::DetectPersonImageCallback, this);
}

void
Detect_person_fmm::halt()
{
    ROS_INFO("Detect_person_fmm halt");
}

BT::NodeStatus
Detect_person_fmm::tick()
{
  ROS_INFO("Detect_person_fmm tick");

  sleep(0.5);

  if (is_person == true && dist <= 1.75 && dist != 0)
  {
    // std::cerr << "HAY PERSONA" << std::endl;
    // std::cerr << dist << std::endl;
    setOutput<int>("w_person", person);
    setOutput<int>("w_state", 0);
    repeticiones = 0;
    person++;
    px = 0;
    py = 0;
    is_person = false;
    dist = 0;

    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    // std::cerr << "NO HAY PERSONA" << std::endl;
    // std::cerr << dist << std::endl;
    if (repeticiones >= 20)
    {
      setOutput<int>("w_person", person);
      setOutput<int>("w_state", 1);
      repeticiones = 0;
      person++;
      return BT::NodeStatus::SUCCESS;
    }
    // std::cerr << repeticiones << std::endl;

    repeticiones++;

    return BT::NodeStatus::RUNNING;
  }
}

void Detect_person_fmm::DetectPersonBBXCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  for (const auto & box : boxes->bounding_boxes)
  {
    if (box.Class == "person")
    {
      px = (box.xmax + box.xmin) / 2;
      py = (box.ymax + box.ymin) / 2;
      is_person = true;
      // std::cerr << "yes, " << px << " , " << py << std::endl;
    }
  }
}

void Detect_person_fmm::DetectPersonImageCallback(const sensor_msgs::ImageConstPtr& image)
{
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

  dist = img_ptr_depth->image.at<float>(cv::Point(px, py)) * 0.001f;
  // std::cerr << "dist, " << px << " , " << py << " , " << dist << std::endl;
}

}  // namespace robocup_nocom_pila

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_nocom_pila::Detect_person_fmm>("Detect_person_fmm");
}
