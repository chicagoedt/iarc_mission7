#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

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
	ros::Subscriber subtap;
	int state;
	int timecheck;
	float dxbydt; // rate of change along x axis
	float dybydt; // rate of change along y axis
/*x2, y2 are global since they shouldn't change when descending.
z2 isn't global since z-component of roomba is always zero
x2, y2, z2 are the estimated points that the QC must descend to.
*/
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
	std_msgs::Bool Tap_it;

	int i;
	int timerun;
	float x1;
	float y1;
	float z1;
	float z2; // height it'd reach when going down.
	float vmag;
	float vmag2;
	float constvelsq; // constant velocity squared for the velocity you want it to go in
	float unitx1;
	float unity1;
	float unitz1;
	float unitx2;
	float unity2;
	float unitz2;
	float c; // constant for calculation
	float disp; // vector variables to point direction
	int npts; //no. of pts for rviz path
	float dpts;  // distance b/w pts
	float rx;
	float ry;
	float rz; //  rviz coordinates

 	void calculate();
	void Init();
 	void callbackqcp(const geometry_msgs::PoseStamped::ConstPtr& posqcp); // sub callback for qcp
	void callbackrp(const geometry_msgs::PoseStamped::ConstPtr& posrp); // sub callback for rp
	void callbacktap(const std_msgs::Bool::ConstPtr& Tap_Sub); // Tap Subscriber

};
