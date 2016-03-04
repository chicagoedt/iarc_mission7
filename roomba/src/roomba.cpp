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
const double ang_5 = (M_PI) / 9.0f;
//Angle to be turned on the 20 second interval
const double ang_20 = 0 - M_PI;
//Angle to be turned on touch
const double ang_touch = 0 - M_PI/4;

//the distance that the copter will have to be to block the roomba
const int distance = Radius;
//The width of the hitbox for the copter to block the roomba
const int blockRadius = 500;

//the actual angle to be turned on the 5 second interval
double current_ang = 0;
//the total angle turned so far
double total_ang = 0;

double count_5 = 0;

double count_20 = 0;

geometry_msgs::PoseStamped poseMsg;

ros::Timer timer_5;
ros::Timer timer_20;

// we set count_5 and count_20 to 0 so in the main loop, they execute and increment
// until we hit the loop rate. (i.e. these will slowly increase in main, and
// will become 0 after 5 and 20 seconds, respectively)
void callback_5(const ros::TimerEvent& event){
	std::srand(std::time(NULL));
	current_ang = (std::rand() % 41-20)/180.0f*M_PI;
	//current_ang = rand() / RAND_MAX *(2* ang_5);
	total_ang += current_ang;
	ROS_INFO_STREAM("Angle turned: " << total_ang << " " << current_ang);
	count_5 = 0;
}

void callback_20(const ros::TimerEvent& event){
	count_5 = looprate; // set it to 20, so we stop callback_5 when 20 sec pass
	total_ang += ang_20;
	ROS_INFO_STREAM("Angle turned: " << total_ang << " " << ang_20);
	count_20 = 0;
}

//To get the coordinates of the copter
void copterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	poseMsg = *msg;
}

//Simplifies a radian
double reduceAngle(double ang){
	while(ang > (M_PI/2)){
		ang -= (M_PI/2);
	}
	while (ang < (M_PI/2)){
		ang += (M_PI/2);
	}
	return ang;

}

bool checkCopter(double copter_x, double copter_y, double copter_z, 
					double x, double y, double z, char &colour){
	// Initially red. For when the QC is far away.
	colour = 'r';

	// Region when marker should turn yellow. When the QC is reaching the tapping vicinity.
	double markerZone_height = z + (Radius/2) + 0.0625;
	double markerZone_bottom_x = x - Radius*2;
	double markerZone_top_x = x + Radius*2;
	double markerZone_bottom_y = y - Radius*2;
	double markerZone_top_y = y + Radius*2;

	//This uses one tenth of the radius. If it seems to be too much, raise the "10" below.
	//The radius does need to be present otherwise it may not detect the copter every time.
	double roomba_height = z + (Radius/2) + 0.0625;

	//The x coordinates that the copter has to be in
	double bottom_x = x - Radius*2;
	double top_x = x + Radius*2;

	//The y coordinates that the copter has to be in
	double bottom_y = y - Radius*2;
	double top_y = y + Radius*2;

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
				timer_5.stop();
				timer_20.stop();
				timer_5.start();
				timer_20.start();
				count_5 = looprate*3; // 3 so it never goes lower than limit
				count_20 = looprate*3;
				colour = 'g';
				return true;
			}
		}
	}
	return false;
}

bool checkBlock(double x, double y, double velocity_x, double velocity_y){
	
}

int main(int argc, char **argv)
{
	//Seed for the random number generator
	std::srand(std::time(NULL));

 	ros::init(argc, argv, "roomba");

 	ros::NodeHandle n;

	timer_5 = n.createTimer(ros::Duration(5.0), callback_5);
	timer_20 = n.createTimer(ros::Duration(20.0), callback_20);
	//Published movement messages
 	ros::Publisher pubMov = n.advertise<geometry_msgs::Twist>("roomba/cmd_vel", 1);
	//Published roomba coordinates
 	ros::Publisher pubPos = n.advertise<geometry_msgs::PoseStamped>("roomba/pose", 1);
 	//Publishes the Rviz visualization marker
 	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
 	//Publishes the hitbox for the copter blocking the roomba
 	ros::Publisher block_pub = n.advertise<visualization_msgs::Marker>("visualization_block", 1);
	//Subscribe to copter messages
 	ros::Subscriber sub = n.subscribe("ground_truth_to_tf/pose", 1, copterCallback);
	//GetModelState Client. Used to gett roomba coordinates from Gazebo
 	ros::ServiceClient gms_c = 
 		n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

 	gazebo_msgs::GetModelState model;
 	model.request.model_name = "create";

 	ros::Rate loop_rate(looprate);
	//twist object to p}ublish messages
 	geometry_msgs::Twist mov;
 	geometry_msgs::PoseStamped pos;

 	pos.header.stamp = ros::Time::now();
 	pos.header.frame_id = "roomba_odom";

	//The time of the current simulation in seconds
 	double sim_time;

 	double speed = 0.33;

	//coordinates of the roomba
 	double x = 0;
 	double y = 0;
 	double z = 0;

 	double last_x;
 	double last_y;
 	double velocity_x;
 	double velocity_y;
	
	//coordinates of the copter
 	double copter_x;
 	double copter_y;
 	double copter_z;

 	double ang_reduced;

   	// Colour of the marker
 	char colour;
 	// Marker id
 	int i =0;

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

		//Rviz object for the copter block zone
		visualization_msgs::Marker block;
		block.header.frame_id = "roomba_odom";
		block.header.stamp = ros::Time::now();

		block.ns = "Block_zone";
		block.id = 1;
		block.type = visualization_msgs::Marker::CUBE;
		block.action = visualization_msgs::Marker::ADD;


		//Set all of the coordinate variables
 		x = model.response.pose.position.x;
		y = model.response.pose.position.y;
		z = model.response.pose.position.z;

		velocity_x = (x - last_x) / looprate;
		velocity_y = (y - last_y) / looprate;

		last_x = x;
		last_y = y;

		//Update the time of the simulation
		sim_time = ros::Time::now().toSec();

		//std::cout << sim_time << std::endl;

		//Move according the the speed
		mov.linear.x = speed;

		/*Move the roomba as it normally should on the 5 and 20 second intervals*/
		mov.angular.z = 0;
		
		// 2.456 is how long a 180 degree turn should take as per iarc docs
		// based on 2.456s/180 degr, we can find how long it takes to turn x degr
		// by doing: 2.456s/180 degr * degrees to turn = time to turn
		if (count_5 < (fabs(current_ang)/M_PI*2.456*looprate)){ // 2.456 is how long to turn as per iarc docs
			mov.angular.z = 1.279; // this speed is iarc specified
			std::cout << current_ang << std::endl;
			count_5++;
		}

		if (count_20 < 2.456*looprate){
			mov.linear.x = 0;
			mov.angular.z = 1.279;// ang_20
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
			total_ang += ang_touch; //changed
			ROS_INFO_STREAM("Angle turned: " << total_ang << " " << ang_touch);

			//The roomba cannot turn until it is no longer touched again
			can_turn = false;
		}
		//If the roomba is not touched, it can turn the next time it is
		if (!check) can_turn = true;

		//Run this every iteration for one second after a touch. It turns the roomba
		if (last_touch + 2.456/4.0 > sim_time && last_touch != 0){
			mov.angular.z = 1.279;//+=(ang_touch * 1.76);
		}

		//Simplify the angle, just for readibility
		//simplifyAngle(total_ang);

		//mov.linear.x = 0;
		
		//std::cout<<"Copter coordinates are (" << copter_x << "," << copter_y << "," << copter_z << ")" << std::endl;
		
		//std::cout<<"Roomba coordinates are (" << x << "," << y  << "," << z << ")"<<std::endl;//<< std::endl;

		pos.pose.position.x = x;
		pos.pose.position.y = y;


		pos.pose.orientation.x = model.response.pose.orientation.x;	
		pos.pose.orientation.y = model.response.pose.orientation.y;
		pos.pose.orientation.z = model.response.pose.orientation.z;
		pos.pose.orientation.w = model.response.pose.orientation.w;

	    
		marker.pose.position.x = pos.pose.position.x;
        marker.pose.position.y = pos.pose.position.y;
        marker.pose.position.z = Radius/2;
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

		block.pose.position.y = pos.pose.position.y + (velocity_y * blockRadius);
		block.pose.position.x = pos.pose.position.x + (velocity_x * blockRadius);
        block.pose.position.z = pos.pose.position.z;
        block.pose.orientation.w = 1.0;
        //this scale is temporary
        block.scale.x = Radius*2;
        block.scale.y = Radius*2;
        block.scale.z = marker.scale.z;
        //this color is also temporary
        block.color.r = 0.0f;
        block.color.g = 0.0f;
        block.color.b = 1.0f;
        block.color.a = 1.0;

        std::cout << "Block position is: " << block.pose.position.x << "," << block.pose.position.y << "," << block.pose.position.z << std::endl;
        std::cout << "Block scale is: " << block.scale.x << "," << block.scale.y << "," << block.scale.z << std::endl;

        std::cout << "Total angle is " << total_ang << std::endl;

		if(mov.angular.z != 0)
		{
			mov.linear.x = 0;
		}

		//std::cout<<"Last Touch:"<<last_touch<<std::endl;

		pubMov.publish(mov);
		pubPos.publish(pos);
		marker_pub.publish(marker);
		block_pub.publish(block);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
