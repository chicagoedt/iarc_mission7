/* I don't think the angle calculation works.
 * The coordinates of the roomba are not accurate.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <time.h>
#include <math.h>

geometry_msgs::PoseStamped poseMsg;

void copterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	poseMsg = *msg;
	ROS_INFO_STREAM("I heard: " << poseMsg.pose.position.x);
}

double rand0to1(){																	//Return a random number 0 to 1
	return (double) rand()/RAND_MAX;
}

int main(int argc, char **argv)
{
	srand(1);

	int looprate = 10;
 	ros::init(argc, argv, "roomba");

 	ros::NodeHandle n;

 	ros::Publisher pubMov = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);		//This is used to publish movement messages

 	ros::Subscriber sub = n.subscribe("copter", 1, copterCallback);					//Subscribe to the copters messages

 	ros::Rate loop_rate(looprate);													//This determines the length of time between loop iterations
 	geometry_msgs::Twist mov;														//Twist object to publish messages

 	bool forward = true;															//Boolean to determine the direction and switch faster

 	int last_20;																	//The last number on the 20 second interval
 	int last_5;																		//The last number on the 5 second interval

 	double sim_time;																//The time of the current simulation in seconds

 	double speed = 0.33;
 	double current_speed = 0;

 	double total_ang = 0;															//The total angle turned so far
 	double ang_5 = (M_PI) / 9;														//20 degrees
 	double ang_20 = M_PI;															//180 degrees
 	double current_ang = 0;

 	while (ros::ok())
 	{
		sim_time = ros::Time::now().toSec();										//Set sim_time to the current time

		current_speed = speed / looprate;											//Move this much every iteration, so the movement is not jerky
		mov.linear.x = current_speed;

		if(((int) sim_time) % 5 == 0 && ((int) sim_time) != last_5){				//Run this once every five seconds
			current_ang = rand0to1() * (2 * ang_5); 								//Random number the length of the angle * 2
			current_ang -= (ang_5);													//Center the angle at 0
			total_ang += current_ang;												//Add to total to keep track of the current angle
			last_5 = (int) sim_time;												//This is necessary to not run the next iteration
		}

		if(((int) sim_time) % 20 == 0 && ((int) sim_time) != last_20){				//This is just to keep track of the total angle turned
			total_ang += ang_20;
			last_20 = (int) sim_time;
		}

		if (((int) sim_time) % 20 == 0){											//Run on the 20 second interval
			mov.angular.z = (ang_20 * 1.76) + (current_ang * 1.76);					//Turn pi radians plus whatever would be on the 5 second interval
		}
		else if (((int) sim_time) % 5 == 0){										//Run this every time it's on the 5 second interval
			mov.angular.z = current_ang * 1.76;										//I don't know why, but multiplying by 1.76 works here
		}
		else{
			mov.angular.z = 0;														//Don't turn if not on the interval
		}

		ROS_INFO_STREAM("Angle turned is " << total_ang);

		pubMov.publish(mov);

		ros::spinOnce();
		loop_rate.sleep();															//Wait until the next iteration
	}

	return 0;
}

/*-20<x<20 vs 0<x<20
 */