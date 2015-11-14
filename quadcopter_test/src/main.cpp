#include "flight_test.h"


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  flight fl;
  fl.run();
  
/*  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub= n.subscribe("ground_truth_to_tf/pose", 1, callback);
  ros::Rate loop_rate(1);


  geometry_msgs::Twist msg;
  /* ros::Time begin = ros::Time::now();
  
   while(ros::Time::now()-begin<5.0)
    {	 		
     msg.linear.z = 2.0; // +1.0 m/s Z (upward) velocity

     ROS_INFO_STREAM("Z velocity is: " << msg.linear.z );
     pub.publish(msg);
  ros::spinOnce();
	loop_rate.sleep();

    }
ros::Time begin = ros::Time::now();
   while(ros::Time::now()-begin<5.0)
   { msg.linear.z = 0.0; // +1.0 m/s Z (upward) velocity
     msg.linear.x = 2.0;
     ROS_INFO_STREAM("X velocity is: " << msg.linear.x );

     pub.publish(msg);
      ros::spinOnce();

    loop_rate.sleep();
    }
    

  */
/* for(int i=0; i<=12;++i)
 {msg.linear.z = 2.0;


     ROS_INFO_STREAM("Z velocity is: " << msg.linear.z );
     pub.publish(msg);
  ros::spinOnce();
	loop_rate.sleep();
}
 for(int i=0; i<=5;++i)
{msg.linear.z = 0;
 msg.linear.x = 1.0;
 pub.publish(msg);
 ros::spinOnce();
 loop_rate.sleep();
}
for(int i=0; i<=5; ++i)
{msg.linear.z=-2.0;
 msg.linear.x=0.0;
 pub.publish(msg);
 ros::spinOnce();
 loop_rate.sleep();
}
for(int i=0; i<=5; ++i)
{msg.linear.z = 0.0;
 msg.linear.x = -1.0;
 pub.publish(msg);
 ros::spinOnce();
 loop_rate.sleep();
}

 for(int i=0; i<=7;++i)
 { msg.linear.z = -2.0;
   msg.linear.x = 0.0;
    
     ROS_INFO_STREAM("Z velocity is: " << msg.linear.z );

     pub.publish(msg);
      ros::spinOnce();

    loop_rate.sleep();
    }
msg.linear.z = 0.0;
msg.angular.z = 0.0;
msg.linear.x = 0.0;
pub.publish(msg);
      ros::spinOnce();

    loop_rate.sleep();

 */   

  return 0;
}
