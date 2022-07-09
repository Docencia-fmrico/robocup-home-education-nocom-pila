// Copyright 2022 Nocom-pila
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

#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "../src/chatbot.cpp"

#include <string>
#include "std_msgs/String.h"


class Listener
{
public:
  Listener()
  {
    end_sub = nh_.subscribe("/sound/end", 1, &Listener::endCallback, this);
    pub_end = nh_.advertise<std_msgs::String>("/sound/end_listener", 1);
  }

  void endCallback(const std_msgs::String::ConstPtr& msg)
  {
    end = msg->data;
  }

  void
  get_end()
  {
    sleep(0.1);
    forwarder.listen();
  }

  void
  listener()
  {
    if (end == "")
    {
      get_end();
    }
    else
    {
      msg_end.data = "stop";
      pub_end.publish(msg_end);
    }
  }

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_end;
    ros::Subscriber end_sub;
    std_msgs::String msg_end;
    std::string end;
    gb_dialog::ExampleDF forwarder;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener_node");
  ros::NodeHandle n;
  Listener listen;
  // ros::Publisher num_pub = n.advertise<std_msgs::Int64>("/message", 1);

  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    listen.listener();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

