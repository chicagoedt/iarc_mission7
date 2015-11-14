#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

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
	ros::Rate loop_rate;
	ros::Subscriber subqcp,subrp;  // subqcp subscribes to groundtruth of quadcopter. subrp to twist of roomba

	geometry_msgs::PoseStamped feedbackMsgqcp; // to store position of qc
	geometry_msgs::PoseStamped feedbackMsgrp; // to store position of roomba

 	void calculate();
 	void callbackqcp(const geometry_msgs::PoseStamped::ConstPtr& posqcp); // sub callback for qcp
	void callbackrp(const geometry_msgs::PoseStamped::ConstPtr& posrp); // sub callback for rp

};
