#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

class flight
{

public:
	void run();

	flight();
	~flight();

private:
	
 	ros::NodeHandle nh;
	geometry_msgs::Twist msg;
	ros::Publisher publ;
	ros::Rate loop_rate;
	ros::Subscriber sub;

	geometry_msgs::PoseStamped feedbackMsg;

 	void fly();
 	void callback(const geometry_msgs::PoseStamped::ConstPtr& pos);

};
