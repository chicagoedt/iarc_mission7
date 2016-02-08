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
#include <visualization_msgs/Marker.h>

//the looprate for the loop in main
const int looprate = 20;

//the radius that the copter has to be in to touch the roomba
const double Radius = 0.15;

//The maximum angle to be turned on the 5 second interval
const double ang_5 = (M_PI) / 9;
//Angle to be turned on the 20 second interval
const double ang_20 = 0 - M_PI;
//Angle to be turned on touch
const double ang_touch = 0 - M_PI/4;

//the actual angle to be turned on the 5 second interval
double current_ang = 0;
//the total angle turned so far
double total_ang = 0;

double count_5 = 0;

double count_20 = 0;

geometry_msgs::PoseStamped poseMsg;
geometry_msgs::PoseStamped poseMsgRoomba;


void callback_5(const ros::TimerEvent& event){
	current_ang = rand()/RAND_MAX * (2 * ang_5);
	current_ang -= (ang_5);
	total_ang += current_ang;

	count_5 = 0;
}

void callback_20(const ros::TimerEvent& event){
	count_5 = looprate;
	total_ang += ang_20;
	count_20 = 0;
}

//To get the coordinates of the copter
void copterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	poseMsg = *msg;
}

//To get the coordinates of the roomba
void roombaCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
	poseMsgRoomba = *msg;
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
					double x, double y, double z, char &colour){
	//The radius that the copter has to be in to tap the roomba
	
	// Initially red. For when the QC is far away.
	colour = 'r';

	// Region when marker should turn yellow. When the QC is reaching the tapping vicinity.
	double markerZone_height = z + Radius*2;
	double markerZone_bottom_x = x - Radius*2;
	double markerZone_top_x = x + Radius*2;
	double markerZone_bottom_y = y - Radius*2;
	double markerZone_top_y = y + Radius*2;

	//This uses one tenth of the radius. If it seems to be too much, raise the "10" below.
	//The radius does need to be present otherwise it may not detect the copter every time.
	double roomba_height = z + 0.0625;

	//The x coordinates that the copter has to be in
	double bottom_x = x - Radius;
	double top_x = x + Radius;

	//The y coordinates that the copter has to be in
	double bottom_y = y - Radius;
	double top_y = y + Radius;



	//If the copter coordinates are (0,0,0), something is probably wrong
	if (copter_x == 0 && copter_y == 0 && copter_z == 0) return false;

	// If the copter is close to the marker.
	if(markerZone_bottom_x <= copter_x && copter_x <= markerZone_top_x){
		if(markerZone_bottom_y <= copter_y && copter_y <= markerZone_top_y){
			if(copter_z <= markerZone_height){
				colour = 'y';
			}
		}
	}
	
	//If the copter is within the radius, return true
	if (bottom_x <= copter_x && copter_x <= top_x){
		if (bottom_y <= copter_y && copter_y <= top_y){
			if((copter_z - 0.182) <= roomba_height){ // 0.182 is the length between copter centre of mass and the tip of its legs (base)
				colour = 'g';
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

 	ros::init(argc, argv, "roomba");

 	ros::NodeHandle n;

	ros::Timer timer_5 = n.createTimer(ros::Duration(5.0), callback_5);
	ros::Timer timer_20 = n.createTimer(ros::Duration(20.0), callback_20);
	//Published movement messages
 	ros::Publisher pubMov = n.advertise<geometry_msgs::Twist>("roomba/cmd_vel", 1);
	//Published roomba coordinates
 	ros::Publisher pubPos = n.advertise<geometry_msgs::PoseStamped>("roomba/pose", 1);
 	//Publishes the Rviz visualization marker
 	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
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

	//coordinates of the roomba
 	double x = 0;
 	double y = 0;
 	double z = 0;
	
	//coordinates of the copter
 	double copter_x;
 	double copter_y;
 	double copter_z;

   	// Colour of the marker
 	char colour;
 	// Marker id
 	int i =0;

	//the coordinates of the other roomba
	double roomba_x;
	double roomba_y;
	double roomba_z;

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

 		// Rviz visualization object
		visualization_msgs::Marker marker;
		// Set the frame ID and timestamp.
		marker.header.frame_id = "roomba_odom";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "Roomba_Shape";
		marker.id = i;

		marker.type = visualization_msgs::Marker::CYLINDER;

		// Set the marker action. ADD: create or modify
		marker.action = visualization_msgs::Marker::ADD; 

		//Set all of the coordinate variables
 		x = model.response.pose.position.x;
		y = model.response.pose.position.y;
		z = model.response.pose.position.z;

		//Update the time of the simulation
		sim_time = ros::Time::now().toSec();

		std::cout << sim_time << std::endl;

		//Move according the the speed
		mov.linear.x = speed;

		/*Move the roomba as it normally should on the 5 and 20 second intervals*/
		mov.angular.z = 0;

		if (count_5 < looprate){
			mov.angular.z = current_ang ;//* 1.76;
			count_5++;
		}

		if (count_20 < looprate){
			mov.angular.z = ang_20 ;//* 1.76;
			count_20++;
		}

		/*Move the roomba when the copter taps it*/

		//Set the coordinates for the copter
		copter_x = poseMsg.pose.position.x;
		copter_y = poseMsg.pose.position.y;
		copter_z = poseMsg.pose.position.z;

		//Check if the roomba is touched and can turn
		check = checkCopter(copter_x, copter_y, copter_z, x, y, z, colour);

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

		//Simplify the angle, just for readibility
		simplifyAngle(total_ang);

		//get the coordinates from the other roomba
		roomba_x = poseMsgRoomba.pose.position.x;
		roomba_y = poseMsgRoomba.pose.position.y;
		roomba_z = poseMsgRoomba.pose.position.z;

		//mov.linear.x = 0;
		
		std::cout<<"Copter coordinates are (" << copter_x << "," << copter_y << "," << copter_z << ")" << std::endl;
		
		std::cout<<"Roomba coordinates are (" << x << "," << y  << "," << z << ")"<<std::endl;//<< std::endl;

		pos.pose.position.x = x;
		pos.pose.position.y = y;


		pos.pose.orientation.x = model.response.pose.orientation.x;	
		pos.pose.orientation.y = model.response.pose.orientation.y;
		pos.pose.orientation.z = model.response.pose.orientation.z;
		pos.pose.orientation.w = model.response.pose.orientation.w;

	    
		marker.pose.position.x = pos.pose.position.x;
        marker.pose.position.y = pos.pose.position.y;
        marker.pose.position.z =Radius/2;
        marker.pose.orientation.x = pos.pose.orientation.x;
        marker.pose.orientation.y = pos.pose.orientation.y;
        marker.pose.orientation.z = pos.pose.orientation.z;
        marker.pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = Radius*2;
        marker.scale.y = Radius*2;
        marker.scale.z = 0.0625;
        // Set the color -- be sure to set alpha to something non-zero!
        if(colour == 'y')
        {	
        	marker.color.r = 1.0f;
        	marker.color.g = 1.0f;
        	marker.color.b = 0.0f;
        	marker.color.a = 1.0;
        }
        else if(colour == 'g')
        {
        	marker.color.r = 0.0f;
        	marker.color.g = 1.0f;
        	marker.color.b = 0.0f;
        	marker.color.a = 1.0; // drop marker here with new ids.
        	++i;	
        }
        else
        {
        	marker.color.r = 1.0f;
        	marker.color.g = 0.0f;
        	marker.color.b = 0.0f;
        	marker.color.a = 1.0;
        }	

		if(mov.angular.z != 0)
		{
			mov.linear.x = 0;
		}

		//std::cout<<"Last Touch:"<<last_touch<<std::endl;

		pubMov.publish(mov);
		pubPos.publish(pos);
		marker_pub.publish(marker);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
