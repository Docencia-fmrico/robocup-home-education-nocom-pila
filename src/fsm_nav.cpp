// Copyright 2019 Intelligent Robotics Lab
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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class MyNode
{
public:
	MyNode()
	: ac("move_base", true)
	{
		ROS_INFO("Waiting for action server to start.");
		ac.waitForServer();
		ROS_INFO("Action server started, sending goal.");
	}

	
	void doWork(int px, long int until)
	{
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = -1.4613;
		goal.target_pose.pose.position.y = 5.6371;
		goal.target_pose.pose.position.z = 0.01017;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;

		ROS_INFO("Sending action");
			ac.sendGoal(goal,
					boost::bind(&MyNode::doneCb, this, _1, _2),
					Client::SimpleActiveCallback(),
					boost::bind(&MyNode::feedbackCb, this, _1));

		ROS_INFO("Action sent");

	}

	void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
		ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
			const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		finish = true;
		ros::shutdown();
	}

	bool checkstatus()
	{
		return finish;
	}

private:
	bool finish = false; 
	Client ac;
};
