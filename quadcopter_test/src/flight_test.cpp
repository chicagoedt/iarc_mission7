#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "flight_test.h"
#include <geometry_msgs/PoseStamped.h>


flight::flight():
	nh("talker"),
	loop_rate(1)
{
	publ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	sub	= nh.subscribe("ground_truth_to_tf/pose", 1, &flight::callback, this);
}

flight::~flight()
{
}

void flight::callback(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
	feedbackMsg = *pos;
}

void flight::fly()
{      	
	msg.linear.z =1.0;
	publ.publish(msg);
	ros::spinOnce();
        loop_rate.sleep();	
	while(feedbackMsg.pose.position.z<=20)
  	{	
		publ.publish(msg);
		ros::spinOnce();
        	loop_rate.sleep();
 	}
}	

void flight::run()
{
	fly();
}
