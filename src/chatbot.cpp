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
#include "std_msgs/Char.h"


namespace ph = std::placeholders;

namespace gb_dialog
{
class ExampleDF: public DialogInterface
{
  public:
    ExampleDF(): nh_()
    {
      sound_pub_name = nh_.advertise<std_msgs::String>("/sound/name", 1);
      sound_pub_color = nh_.advertise<std_msgs::String>("/sound/color", 1);
      sound_pub_order = nh_.advertise<std_msgs::String>("/sound/order", 1);
      sound_pub_start = nh_.advertise<std_msgs::String>("/sound/start", 1);
      sound_pub_drink = nh_.advertise<std_msgs::String>("/sound/drink", 1);
      sound_pub_age = nh_.advertise<std_msgs::String>("/sound/age", 1);
      sound_pub_end = nh_.advertise<std_msgs::String>("/sound/end", 1);

      this->registerCallback(std::bind(&ExampleDF::noIntentCB, this, ph::_1));
      this->registerCallback(
        std::bind(&ExampleDF::nameIntentCB, this, ph::_1),
        "FMM_name");
      this->registerCallback(
        std::bind(&ExampleDF::colorIntentCB, this, ph::_1),
        "FMM_color");
      this->registerCallback(
        std::bind(&ExampleDF::orderIntentCB, this, ph::_1),
        "CML_order");
      this->registerCallback(
        std::bind(&ExampleDF::startIntentCB, this, ph::_1),
        "Start_");
      this->registerCallback(
        std::bind(&ExampleDF::drinkIntentCB, this, ph::_1),
        "Drinks");
      this->registerCallback(
        std::bind(&ExampleDF::ageIntentCB, this, ph::_1),
        "Age_rec");
      this->registerCallback(
        std::bind(&ExampleDF::endIntentCB, this, ph::_1),
        "End_");
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[ExampleDF] noIntentCB: intent [%s]", result.intent.c_str());
    }

    void nameIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      std_msgs::String msg_name;

      ROS_INFO("[ExampleDF] nameIntentCB: intent [%s]", result.intent.c_str());

      for (const auto & param : result.parameters)
      {
        // std::cerr << "Param: "<< param << std::endl;
        for (const auto &value : param.value)
        {
          // std::cerr << "\t" << value << std::endl;
          msg_name.data = value;
        }
      }
      if (msg_name.data != "")
      {
        sound_pub_name.publish(msg_name);
      }
      // speak(result.fulfillment_text);
    }

    void colorIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      std_msgs::String msg_color;

      ROS_INFO("[ExampleDF] colorIntentCB: intent [%s]", result.intent.c_str());

      for (const auto & param : result.parameters)
      {
        // std::cerr << "Param: "<< param << std::endl;
        for (const auto &value : param.value)
        {
          // std::cerr << "\t" << value << std::endl;
          msg_color.data = value;
        }
      }
      if (msg_color.data != "")
      {
        sound_pub_color.publish(msg_color);
      }
      // speak(result.fulfillment_text);
    }

    void orderIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {  // Metodo para pedir que maleta coger.
      std_msgs::String msg_order;

      ROS_INFO("[ExampleDF] orderIntentCB: intent [%s]", result.intent.c_str());

      for (const auto & param : result.parameters)
      {
        // std::cerr << "Param: "<< param << std::endl;
        for (const auto &value : param.value)
        {
          // std::cerr << "\t" << value << std::endl;
          msg_order.data = value;
        }
      }
      if (msg_order.data != "")
      {
        sound_pub_order.publish(msg_order);
      }
      //speak(result.fulfillment_text);
    }

    void endIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {  // Metodo para pedir que maleta coger.
      std_msgs::String msg_end;

      ROS_INFO("[ExampleDF] orderIntentCB: intent [%s]", result.intent.c_str());

      for (const auto & param : result.parameters)
      {
        // std::cerr << "Param: "<< param << std::endl;
        for (const auto &value : param.value)
        {
          // std::cerr << "\t" << value << std::endl;
          msg_end.data = value;
        }
      }
      if (msg_end.data != "")
      {
        sound_pub_end.publish(msg_end);
      }
      //speak(result.fulfillment_text);
    }

    void startIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {  // Metodo para comenzar a funcionar.
      std_msgs::String msg_start;

      ROS_INFO("[ExampleDF] startIntentCB: intent [%s]", result.intent.c_str());

      for (const auto & param : result.parameters)
      {
        // std::cerr << "Param: "<< param << std::endl;
        for (const auto &value : param.value)
        {
          // std::cerr << "\t" << value << std::endl;
          msg_start.data = value;
        }
      }
      if (msg_start.data != "")
      {
        sound_pub_start.publish(msg_start);
      }
      speak(result.fulfillment_text);
    }


    void drinkIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {// Metodo para pedir que maleta coger.
      std_msgs::String msg_drink;
      
      ROS_INFO("[ExampleDF] drinkIntentCB: intent [%s]", result.intent.c_str());

      for(const auto & param : result.parameters)
      {
        // std::cerr << "Param: "<< param << std::endl;
        for(const auto &value : param.value)
        {
          // std::cerr << "\t" << value << std::endl;
          msg_drink.data = value;
        }

      }
      if ( msg_drink.data != "")
      {
        sound_pub_drink.publish(msg_drink);
      }
      speak(result.fulfillment_text);      
    }

    void ageIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {// Metodo para pedir que maleta coger.
      std_msgs::String msg_years;
      
      ROS_INFO("[ExampleDF] drinkIntentCB: intent [%s]", result.intent.c_str());

      msg_years.data = result.parameters[0].value[0].c_str();
      // for(auto & param : result.parameters)
      // {
      //   // std::cerr << "Param: "<< param << std::endl;
      //   for(auto &value : param.value)
      //   {
      //     // std::cerr << "\t" << value << std::endl;
      //     msg_years.data = value;
      //   }

      // }
      if ( msg_years.data != "")
      {
        sound_pub_age.publish(msg_years);
      }
      speak(result.fulfillment_text);      
    }


  private:
    ros::NodeHandle nh_;
    ros::Publisher sound_pub_name;
    ros::Publisher sound_pub_color;
    ros::Publisher sound_pub_order;
    ros::Publisher sound_pub_start;
    ros::Publisher sound_pub_drink;
    ros::Publisher sound_pub_age;
    ros::Publisher sound_pub_end;
};

}  // namespace gb_dialog
/*
int main(int argc, char** argv)
{
  ros::init(argc, argv, "chatbot_node");
  gb_dialog::ExampleDF forwarder;
  forwarder.listen();
  ros::spin();
  return 0;
}
*/