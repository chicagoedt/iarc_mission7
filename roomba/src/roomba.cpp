#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <math.h>


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);         //This is used to publish messages
  ros::Rate loop_rate(10);                                                      //This determines the length of time between loop iterations
  geometry_msgs::Twist msg;                                                     //Twist object to publish messages

  bool forward = true;                                                          //Boolean to determine the direction and switch faster
  int last_20;                                                                  //The last number on the 20 second interval
  int last_5;                                                                   //The last number on the 5 second interval
  int sim_time;                                                                 //The time of the current simulation in seconds
  double dir;                                                                   //The direction to change on the 5 second intervals
  //simTime is an int because anything less than a second is never used

  /* This program changes the direction of the roomba every 20 seconds.
   * 
   */

  while (ros::ok())
  {
    if (forward) msg.linear.x = 0.33;                                           //The speed is 0.33m/s
    else msg.linear.x = -0.33;

    sim_time = ros::Time::now().toSec();                                        //Set sim_time to the current time

    ROS_INFO_STREAM("Time is " << sim_time);                                    //Print the time to the console

    if (sim_time % 20 == 0 && sim_time != last_20){                             //Switch the direction every 20 seconds
      forward = !forward;
      last_20 = sim_time;
    }

    if (sim_time % 5 == 0 && sim_time != last_5){                               //Change direction every 5 seconds
      dir = (rand() % 41) - 20;    	                                            //Random number 0-pi/9
      msg.angular.z = dir;                                                      //Change the direction based on that number
      last_5 = sim_time;
    }
    else msg.angular.z = 0;                                                     //Otherwise, don't change direction
 
    pub.publish(msg);                                                           //Publish the message
    ros::spinOnce();
    loop_rate.sleep();                                                          //Wait until the next iteration
  }

  return 0;
}