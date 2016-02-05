#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "Tap_Decision.h"
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Bool.h>

Decision::Decision():
	nh(),
	loop_rate(1)
{
	checker = 1; // checker to check rate of change of roomba. rotates b/w the if-else in callback.
	initdec = 1; // initially declare feedbackMsgrp2 as 0. This should happen only once. hence global.
	timer = 1;
	publ = nh.advertise<std_msgs::Bool>("Tap", 1); // FIXIT (std_msgs::Int16)publishing the values to move the qc, currently publishes a velociy in a direction 	
	subrp = nh.subscribe("roomba/pose", 1, &Decision::callbackrp, this); // subscribe to roomba pose, revise after Harsh chooses ros topic
	ROS_INFO_STREAM("Initialized Topics!");
}

Decision::~Decision()
{

}

void Decision::callbackrp(const geometry_msgs::PoseStamped::ConstPtr& posrp)
{
	if(initdec == 1)
	{	
		feedbackMsgrp2.pose.position.x = 0;
		feedbackMsgrp2.pose.position.y = 0;
		feedbackMsgrp2.pose.position.z = 0;
		initdec = 0;
		
	}	
// Calculate RateOfChange in x and y component.
	if(checker==1) // constantly alters new and old position.once 1:new & 2:old. Next 1:old & 2:new.
	{
		feedbackMsgrp1 = *posrp;
		dxbydt = (feedbackMsgrp1.pose.position.x - feedbackMsgrp2.pose.position.x);
		dybydt = (feedbackMsgrp1.pose.position.y - feedbackMsgrp2.pose.position.y);
		dxbydt = dxbydt/0.2; // dividing to get rate of change. 0.2 b/c the freq is 5. Hence time diff b/w two pos. is 0.2s.
		dybydt = dybydt/0.2;
		checker = 0;
		
	}
	else
	{
		feedbackMsgrp2 = *posrp;
		dxbydt = feedbackMsgrp2.pose.position.x - feedbackMsgrp1.pose.position.x;
		dybydt = feedbackMsgrp2.pose.position.y - feedbackMsgrp1.pose.position.y;
		dxbydt = dxbydt/0.2; // dividing to get rate of change. 0.2 b/c the freq is 5. Hence time diff b/w two pos. is 0.2s.
		dybydt = dybydt/0.2;
		checker = 1;
		
	}
		
}

void Decision::Calculate()
{
	std_msgs::Bool canTap;
	canTap.data = false; // 0 = false, 1 = true.
	//int SecBwTap = 4; // Time between allowing taps
	ros::Duration seconds(4);
	if(timer == 1)
	{
		begin = ros::Time::now();
		timer = 0;
	}
	/*if(ros::Time::now()-begin<seconds)//ros::Duration(SecBwTap)) // FIXME!!!!!!!!!!!!!!!
	{
		canTap.data = false;
	}
	else
	{*/
		if(dybydt<0)
		{
			canTap.data = true;
		}
		else if((atan(dybydt/dxbydt)>=-0.785) && (atan(dybydt/dxbydt)<=0.524))
		{
			canTap.data = true;
		}
		else
		{
			canTap.data = false;
		}
		timer = 1;
	//}
	publ.publish(canTap);
}

void Decision::run()
{	
	while(ros::ok())
	{
		Calculate();
		ros::spinOnce();
    	loop_rate.sleep();
	}

}