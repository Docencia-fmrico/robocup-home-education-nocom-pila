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

#include <gb_dialog/DialogInterface.h>
#include <string>
#include "std_msgs/String.h"


namespace ph = std::placeholders;

namespace gb_dialog
{
class ExampleDF: public DialogInterface
{
  public:
    ExampleDF(): nh_()
    {
      sound_pub = nh_.advertise<std_msgs::String>("/sound/word", 1);
      this->registerCallback(std::bind(&ExampleDF::noIntentCB, this, ph::_1));
      this->registerCallback(
        std::bind(&ExampleDF::welcomeIntentCB, this, ph::_1),
        "FMM");
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[ExampleDF] noIntentCB: intent [%s]", result.intent.c_str());
    }

    void welcomeIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      std_msgs::String msg;
      msg.data = "alone";
      ROS_INFO("[ExampleDF] welcomeIntentCB: intent [%s]", result.intent.c_str());
      sound_pub.publish(msg);
      speak(result.fulfillment_text);
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher sound_pub;
};

}  // namespace gb_dialog

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_df_node");
  gb_dialog::ExampleDF forwarder;
  forwarder.listen();
  ros::spin();
  return 0;
}
