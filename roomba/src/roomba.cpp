/* One thing I'm unsure of is the speed of the roomba.
 * I had it set to 0.33, but that didn't seem like 0.33 m/s.
 * I think it might be in cm/s, but I'm really not 100% sure.
 * I have it set to 3 right now.
 *
 * The other thing that I'm unsure of is the method of turning.
 * When I add to the Twist to turn, it didn't turn the amount that I added.
 * This is most likely because of it uses the wheels to turn, it doesn't just turn the base.
 * Through trial and error, I found that multiplying by 1.76 and turning for a whole second turns the correct amount.
 * The roombas in the competition videos do turn slower though, I don't have any details on it though.
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <time.h>
#include <math.h>

geometry_msgs::PoseStamped poseMsg;

void copterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	poseMsg = *msg;
}

double rand0to1(){																	//Return a random number 0 to 1
	return (double) rand()/RAND_MAX;
}

void simplifyAngle(double& total_ang){												//Simplifies a radian
	while(total_ang < 0 - (M_PI*2) || total_ang > (M_PI*2)){
		if (total_ang < 0 - (M_PI*2)){
			total_ang += (M_PI*2);
		}
		else if (total_ang > (M_PI*2)){
			total_ang -= (M_PI*2);
		}
	}
}

bool checkCopter(double copter_x, double copter_y, double copter_z, 
					double x, double y, double z){
	double radius = 0.05;															//The radius that the copter has to be within to tap the roomba
	double roomba_height = z + radius;												//The height of the roomba (meters)

	double bottom_x = x - radius;													//The bottom limit of the x zone
	double top_x = x + radius;														//The top limit of the x zone

	double bottom_y = y - radius;													//The bottom limit of the y zone
	double top_y = y + radius;														//The top limit of the y zone

	ROS_INFO_STREAM("Copter coordinates are (" << copter_x << "," << copter_y << "," << copter_z << ")");

	if (copter_x == 0 && copter_y == 0 && copter_z == 0) return false;				//If it's reading 0 for every position

	if (bottom_x < copter_x && copter_x < top_x){									//Check the x xone
		if (bottom_y < copter_y && copter_y < top_y){								//Check the y zone
			if(copter_y <= roomba_height){
				return true;
			}
		}
	}
	return false;
}

int main(int argc, char **argv)
{
	srand(1);																		//Seed for the random number generator

	int looprate = 10;
 	ros::init(argc, argv, "roomba");

 	ros::NodeHandle n;

 	ros::Publisher pubMov = n.advertise<geometry_msgs::Twist>("roomba/cmd_vel", 1);		//This is used to publish movement messages

 	ros::Publisher pubPos = n.advertise<geometry_msgs::PoseStamped>("roomba/pose", 1);	//Publishes the coordinates of the roomba

 	ros::Subscriber sub = n.subscribe("ground_truth_to_tf/pose", 1, copterCallback);					//Subscribe to the copters messages

 	ros::ServiceClient gms_c = 
 		n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");		//ModelState_Client

 	gazebo_msgs::GetModelState model;
 	model.request.model_name = "create";

 	ros::Rate loop_rate(looprate);													//This determines the length of time between loop iterations
 	geometry_msgs::Twist mov;														//Twist object to publish messages
 	geometry_msgs::PoseStamped pos;													//PoseStamped object to publish coordinates

 	int last_20 = 0;																//The last number on the 20 second interval
 	int last_5 = 0;																	//The last number on the 5 second interval
 	int last_1 = 0;																	//The last number on the 1 second interval

 	double sim_time;																//The time of the current simulation in seconds

 	double speed = 0.33;
 	speed = 3;
 	double current_speed = 0;														//Will be used to move a certain amount every iteration of the loop

 	double x = 0;																	//The x coordinate of the roomba
 	double y = 0;																	//The y coordinate of the roomba
 	double z = 0;																	//The z coordinate of the roomba

 	double copter_x;																//The x coordinate of the copter
 	double copter_y;																//The y coordinate of the copter
 	double copter_z;																//The z coordinate of the copter

 	double total_ang = 0;															//The total angle turned so far
 	double ang_5 = (M_PI) / 9;														//20 degrees
 	double ang_20 = 0 - M_PI;														//180 degrees clockwise
 	double ang_touch = 0 - M_PI/4;													//45 degrees clockwise
 	double current_ang = 0;

 	double last_touch = 0;															//The last second that the roomba was touched on

 	bool can_turn;																	//If the roomba can turn (prevents the rookmba from turning >45 on touch)

 	double rand_num = 0;

 	while (ros::ok())
 	{
 		gms_c.call(model);															//Update the coordinates in the model

 		x = model.response.pose.position.x;											//Set the x variable
		y = model.response.pose.position.y;											//Set the y variable
		z = model.response.pose.position.z;											//Set the z variable

		sim_time = ros::Time::now().toSec();										//Set sim_time to the current time

		current_speed = speed;														//Move this much every iteration, so the movement is not jerky
		mov.linear.x = current_speed;

		/*Move the roomba as it normally should on the 5 and 20 second intervals*/

		if(((int) sim_time) % 5 == 0 && ((int) sim_time) != last_5 && 				//Run this once every five seconds
				((int) sim_time) % 20 != 0){	
			rand_num = rand0to1();
			current_ang = rand_num * (2 * ang_5);	 								//Random number the length of the angle * 2
			current_ang -= (ang_5);													//Center the angle at 0
			//ROS_INFO_STREAM("Adding " << (rand_num * (2 * ang_5)) - ang_5);
			total_ang += current_ang;												//Add to total to keep track of the current angle
			last_5 = (int) sim_time;												//This is necessary to not run the next iteration
		}

		if(((int) sim_time) % 20 == 0 && ((int) sim_time) != last_20){				//This is just to keep track of the total angle turned
			total_ang += ang_20;													//Add ang_20 to the current angle the roommba is facing
			//ROS_INFO_STREAM("Adding " << ang_20);
			last_20 = (int) sim_time;												//Update the last_20 variable
		}

		//if (last_1 != (int) sim_time){											//Do this once every second
		//	calcCoords(x, z, total_ang, speed);										//Update the coordinates of the roomba
		//	last_1 = (int) sim_time;												//Update the last_1 variable
		//}

		if (((int) sim_time) % 20 == 0){											//Run on the 20 second interval
			mov.angular.z = (ang_20 * 1.76);										//Turn pi radians plus whatever would be on the 5 second interval
			mov.linear.x = 0;														//Don't move while turning. It will mess up the coordinates.
			current_speed = 0;
		}
		else if (((int) sim_time) % 5 == 0){										//Run this every time it's on the 5 second interval
			mov.angular.z = current_ang * 1.76;										//I don't know why, but multiplying by 1.76 works here
		}
		else{
			mov.angular.z = 0;														//Don't turn if not on the interval
		}

		/*Move the roomba when the copter taps it*/
		copter_x = poseMsg.pose.position.x;											//Set the copter_x variable based on the poseMsg
		copter_y = poseMsg.pose.position.y;											//Set the copter_y variable based on the poseMsg
		copter_z = poseMsg.pose.position.z;											//Set the copter_v variable based on the poseMSg

		if (checkCopter(copter_x, copter_y, copter_z, x, y, z) && can_turn == true){//If the roomba is touched and can turn
			ROS_INFO_STREAM("The copter has touched");
			last_touch = sim_time;													//Update the last_touch variable
			current_ang += ang_touch;												//Add the ang_touch variable to the total angle being turned
			can_turn = false;														//The roomba cannot turn after this until it is no longer being touched
		}
		if (!checkCopter(copter_x, copter_y, copter_z, x, y, z)) can_turn = true;	//If the roomba is not being touched, the roomba can turn next time it is

		if (last_touch + 1 > sim_time && last_touch != 0){
			mov.angular.z += (ang_touch * 1.76);
		}

		simplifyAngle(total_ang);

		//mov.linear.x = 0;

		ROS_INFO_STREAM("Roomba coordinates are (" << x << "," << y << ")");
		//ROS_INFO_STREAM("Angle turned is " << total_ang);

		pos.pose.position.x = x;
		pos.pose.position.y = y;

		pubMov.publish(mov);
		pubPos.publish(pos);

		ros::spinOnce();
		loop_rate.sleep();															//Wait until the next iteration
	}

	return 0;
}
