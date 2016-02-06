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
geometry_msgs::PoseStamped poseMsgRoomba;

void copterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	poseMsg = *msg;
}

void roombaCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	poseMsgRoomba = *msg;
}

//Returns a random number numder 0 to 1
double rand0to1(){
	return (double) rand()/RAND_MAX;
}

//Simplifies a radian
void simplifyAngle(double& total_ang){
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
	//The radius that the copter has to be in to tap the roomba
	double radius = 0.05;
	double roomba_height = z + (radius / 10);


	//The x coordinates that the copter has to be in
	double bottom_x = x - radius;
	double top_x = x + radius;

	//The y coordinates that the copter has to be in
	double bottom_y = y - radius;
	double top_y = y + radius;


	//If the copter coordinates are (0,0,0), something is probably wrong
	if (copter_x == 0 && copter_y == 0 && copter_z == 0) return false;
	
	//If the copter is within the radius, return true
	if (bottom_x <= copter_x && copter_x <= top_x){
		if (bottom_y <= copter_y && copter_y <= top_y){
			if(copter_z <= roomba_height){
				return true;
			}
		}
	}
	return false;
}

int main(int argc, char **argv)
{
	//Seed for the random number generator
	srand(1);

	int looprate = 5;
 	ros::init(argc, argv, "roomba");

 	ros::NodeHandle n;

	//Published movement messages
 	ros::Publisher pubMov = n.advertise<geometry_msgs::Twist>("roomba/cmd_vel", 1);

	//Published roomba coordinates
 	ros::Publisher pubPos = n.advertise<geometry_msgs::PoseStamped>("roomba/pose", 1);

	//Subscribe to copter messages
 	ros::Subscriber sub = n.subscribe("ground_truth_to_tf/pose", 1, copterCallback);

	//Subscribe to roomba messages
	ros::Subscriber roomba_sub = n.subscribe("roomba/pose1", 1, roombaCallback);

	//GetModelState Client. Used to gett roomba coordinates from Gazebo
 	ros::ServiceClient gms_c = 
 		n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

 	gazebo_msgs::GetModelState model;
 	model.request.model_name = "create";

 	ros::Rate loop_rate(looprate);
	//twist object to publish messages
 	geometry_msgs::Twist mov;
 	geometry_msgs::PoseStamped pos;	

 	pos.header.stamp = ros::Time::now();
 	pos.header.frame_id = "roomba_odom";


	//The last number on the 20 second interval
 	int last_20 = 0;
	//The last number on the 5 second interval
 	int last_5 = 0;
	//The last number on the 1 second interval
 	int last_1 = 0;

	//The time of the current simulation in seconds
 	double sim_time;

 	double speed = 0.33;

 	//speed = 3;
 	double current_speed = 0;

	//coordinates of the roomba
 	double x = 0;
 	double y = 0;
 	double z = 0;
	
	//coordinates of the copter
 	double copter_x;
 	double copter_y;
 	double copter_z;



	//the coordinates of the other roomba
	double roomba_x;
	double roomba_y;
	double roomba_z;

	//The total angle turned so far
 	double total_ang = 0;
	//The angle to be turned on the 5 second interval
 	double ang_5 = (M_PI) / 9;
	//Angle to be turned on the 20 second interval
 	double ang_20 = 0 - M_PI;
	//Angle to be turned on touch
 	double ang_touch = 0 - M_PI/4;
	//The total angle to be turned this iteration
 	double current_ang = 0;

	//The last second that the roomba was touched on
 	double last_touch = 0;

	//Determines if the roomba can turn (prevents turning >45 degrees on touch
 	bool can_turn;

	//used to check if the copter has touched the target
	bool check = 0;

 	double rand_num = 0;

 	while (ros::ok())
 	{
 		pos.header.stamp = ros::Time::now();

		//update the coordinates in the model
 		gms_c.call(model);
		//Update the coordinates of the roomba
 

		//Set all of the coordinate variables
 		x = model.response.pose.position.x;
		y = model.response.pose.position.y;
		z = model.response.pose.position.z;

		//Update the time of the simulation
		sim_time = ros::Time::now().toSec();

		//Move according the the speed
		mov.linear.x = speed;

		/*Move the roomba as it normally should on the 5 and 20 second intervals*/

		//Run this once every five seconds.
		//It started the turning of the roomba over the course of one second
		if(((int) sim_time) % 5 == 0 && ((int) sim_time) != last_5 && ((int) sim_time) % 20 != 0){
			//Create a random number to determine the angle * 2
			//The roomba can turn 20 degrees in any direction, so it's a total of 40 degrees
			rand_num = rand0to1();
			current_ang = rand_num * (2 * ang_5);

			//Set the center of the 40 degree angle to 0
			current_ang -= (ang_5);

			//ROS_INFO_STREAM("Adding " << (rand_num * (2 * ang_5)) - ang_5);
			//Add to total to keep track of the total angle
			total_ang += current_ang;

			//This is necessary to not run more than once on the five second interval
			last_5 = (int) sim_time;
		}

		//On the 20 second interval, update the total angle turned
		if(((int) sim_time) % 20 == 0 && ((int) sim_time) != last_20){
			total_ang += ang_20;
			//ROS_INFO_STREAM("Adding " << ang_20);

			//This is necessary to not run more than once on the twenty second interval
			last_20 = (int) sim_time;
		}

		//Run this during every loop on the 20 second interval
		if (((int) sim_time) % 20 == 0){
			//Turn this much every iteration of the loop
			mov.angular.z = (ang_20 * 1.76);

			//Don't move forward on the turn
			mov.linear.x = 0;
		}
		//Run this during every loop on the 5 second interval
		else if (((int) sim_time) % 5 == 0){
			mov.angular.z = current_ang * 1.76;
		}
		//Don't turn if you're not on any special interval
		else{
			mov.angular.z = 0;
		}

		/*Move the roomba when the copter taps it*/

		//Set the coordinates for the copter
		copter_x = poseMsg.pose.position.x;
		copter_y = poseMsg.pose.position.y;
		copter_z = poseMsg.pose.position.z;

		//Check if the roomba is touched and can turn
		check = checkCopter(copter_x, copter_y, copter_z, x, y, z);

		if (check && can_turn == true){
			ROS_INFO_STREAM("The copter has touched");
			
			//Keep track of the last touch of the roomba
			last_touch = sim_time;

			//Update the total angle
			current_ang += ang_touch;

			//The roomba cannot turn until it is no longer touched again
			can_turn = false;
		}
		//If the roomba is not touched, it can turn the next time it is
		if (!check) can_turn = true;

		//Run this every iteration for one second after a touch. It turns the roomba
		if (last_touch + 1 > sim_time && last_touch != 0){
			mov.angular.z += (ang_touch * 1.76);
		}

		simplifyAngle(total_ang);

		//get the coordinates from the other roomba
		roomba_x = poseMsgRoomba.pose.position.x;
		roomba_y = poseMsgRoomba.pose.position.y;
		roomba_z = poseMsgRoomba.pose.position.z;

		//mov.linear.x = 0;
		
		ROS_INFO_STREAM("Copter coordinates are (" << copter_x << "," << copter_y << "," << copter_z << ")");
		ROS_INFO_STREAM("Roomba coordinates are (" << x << "," << y  << "," << z << ")");
		//ROS_INFO_STREAM("Angle turned is " << total_ang);

		pos.pose.position.x = x;
		pos.pose.position.y = y;


		pos.pose.orientation.x = model.response.pose.orientation.x;	
		pos.pose.orientation.y = model.response.pose.orientation.y;
		pos.pose.orientation.z = model.response.pose.orientation.z;
		pos.pose.orientation.w = model.response.pose.orientation.w;

		if(mov.angular.z != 0)
		{
			mov.linear.x = 0;
		}

		//std::cout<<"Last Touch:"<<last_touch<<std::endl;

		pubMov.publish(mov);
		pubPos.publish(pos);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
