#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "vector_calc.h"
#include <geometry_msgs/PoseStamped.h>
#define RVEL 3
/* Consider vector (x,y,z) to be the dirn to go, publish twist as (x,y,z). This will make the qc go in the particular dirn. As for it to stop we check if the distance b/w it's current position and the original position is equal to the distance b/w the roomba and it's original position. or stop when it is a certain distance away. (using shortest distance formula).
*/
/* qcp = quad copter position
   rp = roomba position
*/
// What if I put the subscriber into the calculator function? 
vector::vector():
	nh(),
	loop_rate(5)
{
	publ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5); // publishing the values to move the qc, currently publishes a velociy in a direction 
	subqcp = nh.subscribe("ground_truth_to_tf/pose", 5, &vector::callbackqcp, this); // subscribe to qc pose
	subrp = nh.subscribe("current_roomba/pose", 5, &vector::callbackrp, this); // subscribe to roomba pose, revise after Harsh chooses ros topic
}

vector::~vector()
{
}

void vector::callbackqcp(const geometry_msgs::PoseStamped::ConstPtr& posqcp)
{
	feedbackMsgqcp = *posqcp; // copy values from pointer into feedback for qcp
} 

void vector::callbackrp(const geometry_msgs::PoseStamped::ConstPtr& posrp)
{
	feedbackMsgrp = *posrp; // copy values from pointer into feedback for rp
}

void vector::calculate()
{
		float x1,y1,z1,vmag,unitx1,unity1,unitz1,c, disp; // vector variables to point direction
		//x1 = 4 - feedbackMsgqcp.pose.position.x; // calculating x,y,z values
		//y1 = 4 - feedbackMsgqcp.pose.position.y;
		//z1 = (4 + 1) - feedbackMsgqcp.pose.position.z; // 1m above rp
		x1 = feedbackMsgrp.pose.position.x - feedbackMsgqcp.pose.position.x; // calculating x,y,z values
		y1 = feedbackMsgrp.pose.position.y - feedbackMsgqcp.pose.position.y;				
		z1 = (feedbackMsgrp.pose.position.z + 1) - feedbackMsgqcp.pose.position.z; // 1m above rp
		vmag = sqrt( x1*x1 + y1*y1 + z1*z1); // magnitude of qc velocity
		unitx1 = x1/vmag;
		unity1 = y1/vmag;
		unitz1 = z1/vmag;
		disp = sqrt( x1*x1 + y1*y1 + z1*z1 );
		if(disp>4)
		{
			c = sqrt( 4/((unitx1*unitx1)+(unity1*unity1)+(unitz1*unitz1)) );
			msg.linear.x = c*unitx1;
			msg.linear.y = c*unity1;
			msg.linear.z = c*unitz1;
		}
		else if(disp>1)
		{
			c = sqrt( 1/((unitx1*unitx1)+(unity1*unity1)+(unitz1*unitz1)) );
			msg.linear.x = c*unitx1;
			msg.linear.y = c*unity1;
			msg.linear.z = c*unitz1;
		}
		else if(disp>0.2)
		{
			c = sqrt( 0.25/((unitx1*unitx1)+(unity1*unity1)+(unitz1*unitz1)) );
			msg.linear.x = c*unitx1;
			msg.linear.y = c*unity1;
			msg.linear.z = c*unitz1;
		}
			
		else  // 0.8 best option for not shaking
		{
			msg.linear.x = 0;
			msg.linear.y = 0;
			msg.linear.z = 0;
		}
	
		publ.publish(msg);
		
	

	
	/*if(vmag<RVEL) // if vmag is less then roomba velocity !! Find roomba vel (say rvel = 3)
	{
		while(vmag<RVEL)
		{
			x1 = x1*1.5;
			y1 = y1*1.5;
			z1 = z1*1.5;
			vmag = sqrt( x1*x1 + y1*y1 + z1*z1);
		}
	}
	msg.linear.x = x1;
	msg.linear.y = y1;
	msg.linear.z = z1;
	publ.publish(msg);
	ros::spinOnce();
        loop_rate.sleep();
	*/
}
void vector::run()
{	
	while(ros::ok())
	{
		calculate();
		ros::spinOnce();
        	loop_rate.sleep();
	}

}

	/* To keep a constan velocity say 3. 
	vmag = sqrt( x1*x1 + y1*y1 + z1*z1);
	unitx1 = x1/vmag;	gives unit vectors
	unity1 = y1/vmag;
	unitz1 = z1/vmag;
	c = sqrt( 9/((unitx1*unitx1)+(unity1*unity1)+(unitz1*unitz1)) ); find the constant to * to unit vectors
	msg.linear.x = c*unitx1;
	msg.linear.y = c*unity1;
	msg.linear.z = c*unitz1;
	*/

// Another METHOD: Fly right above the roomba then go down on to the roomba. 
/* Make sure that the velocity of qc is greater than the velocity of roomba so that it reaches the roomba and doesnt just fly behind it*/
/* Assumptions made: z vector will never be greater than 2, (3m max height, qc must be about 1m above the roomba). x,y vector wouldn't be more than 3 or 4 since the camera will choose the closest roomba.*/
/* NEW: take distance b/w original qcp and rp. Then take distance b/w new qcp and rp. Take the ratio : (new/original). This ratio has to be taken everytime. Multiply the twist velocity values (which is equal to the vector) by the ratio (twis.linear.x = x1.ratio). This makes the velocity lower and lower everytime. Include a threshold velocity so that it doesnt go slower than a certain amount or stop. 
*/
/* twist linear velocities as x1,y1,z1. Theoretically, with these values the qc should reach he roomba's position in exactly one second. However practically the qc may noot be able to go in this speed, so dividing it by a constant would make it more feasible. 
Eg: dividing it by 2 would make it reach the roomba's position in 2 seconds.
There is a chance of error with air drag, etc.
*/
/* Say we take a rate of 2 Hz. Then, in half a second the quad copter would go half the distance and would not reach the roomba's position. ( considering we take twist values equal to vector). Therefore, the quadcopter would never reach the roomba's position but it will keep getting closer
*/
	

