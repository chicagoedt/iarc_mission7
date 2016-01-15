#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


class vector
{

public:
	void run();

	vector();
	~vector();

private:
	
 	ros::NodeHandle nh; 
	geometry_msgs::Twist msg;
	ros::Publisher publ;
	ros::Publisher pubviz;
	ros::Rate loop_rate;
	ros::Subscriber subqcp;
	ros::Subscriber subrp;  // subqcp subscribes to groundtruth of quadcopter. subrp to twist of roomba
	int state;
	int timecheck;
	float dxbydt;
	float dybydt;
	float x2;
	float y2;
	int checker; // checker to check rate of change of roomba. rotates b/w the if-else in callback.
	int initdec; // initially declare feedbackMsgrp2 as 0. This should happen only once. hence global.
	ros::Time begin;
	//ros::Duration zero(0.0);
	
	geometry_msgs::PoseStamped feedbackMsgqcp; // to store position of qc
	geometry_msgs::PoseStamped feedbackMsgrp; // to store position of roomba
	geometry_msgs::PoseStamped feedbackMsgrp1;// To find rate of change of roomba. old position
	geometry_msgs::PoseStamped feedbackMsgrp2;// ^ new position.

 	void calculate();
 	void callbackqcp(const geometry_msgs::PoseStamped::ConstPtr& posqcp); // sub callback for qcp
	void callbackrp(const geometry_msgs::PoseStamped::ConstPtr& posrp); // sub callback for rp

};
