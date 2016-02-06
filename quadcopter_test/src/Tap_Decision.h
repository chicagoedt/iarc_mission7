#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>


class Decision
{
public:
	void run();
	Decision();
	~Decision();

private:
	ros::NodeHandle nh;
	ros::Publisher publ;
	ros::Rate loop_rate;
	ros::Subscriber subrp;
	
	float dxbydt; // rate of change along x axis
	float dybydt; // rate of change along y axis
	
	int checker; // checker to check rate of change of roomba. rotates b/w the if-else in callback.
	int initdec; // initially declare feedbackMsgrp2 as 0. This should happen only once. hence global.

	int timer;
	ros::Time begin;

	geometry_msgs::PoseStamped feedbackMsgrp1;// To find rate of change of roomba. old position
	geometry_msgs::PoseStamped feedbackMsgrp2;// ^ new position
	void Calculate();
	void callbackrp(const geometry_msgs::PoseStamped::ConstPtr& posrp);

};