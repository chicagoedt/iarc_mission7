#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
//#include <math/gzmath.hh>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo/common/common.hh>
#include <stdio.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo/transport/transport.hh>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <gazebo_plugins/PubQueue.h>

const float roombaSpeed = 0.33;     // roombaSpeed at any given time
const float frequencyUpdate = 5;    // 5Hz

namespace gazebo
{
class RoombaModelPlugin : public ModelPlugin
{
private: physics::ModelPtr model;
private: common::Time pastTime;

private: event::ConnectionPtr updateConnection;
private: float timeInc;
private: ros::NodeHandle* rosnode_;
private: ros::Publisher pub_;
private: PubQueue<geometry_msgs::PoseStamped>::Ptr pub_Queue;
private: PubMultiQueue pmq;
private: geometry_msgs::PoseStamped pose;

public:  RoombaModelPlugin() : ModelPlugin()
  {
  }

public:  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    this->timeInc = 0;
    this->pastTime = 0;
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    this->rosnode_ = new ros::NodeHandle();
    this->pmq.startServiceThread();

    this->model = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&RoombaModelPlugin::OnUpdate, this, _1));
    this->model->SetLinkWorldPose(math::Pose(0,0,0,0,0,0), "base");
    ROS_INFO("Model Loaded, Will Start Updating");
  }
public:  void OnUpdate(const common::UpdateInfo & _info)
    {
        common::Time currentTime = _info.simTime;
	double timeDifference = currentTime.Double()-pastTime.Double();
	if (timeDifference < (1/frequencyUpdate)) return;
	
	// TODO: make the vectors multidimensional
	double currentX = this->model->GetChildLink("base")->GetWorldPose().pos.x;
        double nextX = currentX+timeDifference*roombaSpeed;	
        ROS_INFO("%f",currentX);
	this->pose.pose.position.x = nextX;
	this->pose.pose.position.y = 0;
	this->pose.pose.position.z = 0;

	this->pose.pose.orientation.x = 0;
	this->pose.pose.orientation.y = 0;
	this->pose.pose.orientation.z = 0;
	this->pose.pose.orientation.w = 0;

	this->pose.header.stamp.sec = (uint32_t)currentTime.Double();
	this->pub_Queue = this->pmq.addPub<geometry_msgs::PoseStamped>();
	this->pub_ = this->rosnode_->advertise<geometry_msgs::PoseStamped>("current_roomba/pose",1);
	this->pub_Queue->push(this->pose, this->pub_);
	this->model->SetLinkWorldPose(math::Pose(nextX,0,0,0,0,0), "base");
        this->pastTime = currentTime;

    }

};
GZ_REGISTER_MODEL_PLUGIN(RoombaModelPlugin)
}
