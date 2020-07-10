#ifndef GAZEBO_PLUGINS_BATTERYPLUGIN_HH_
#define GAZEBO_PLUGINS_BATTERYPLUGIN_HH_

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <ignition/transport/Node.hh>
#include <gazebo/transport/Node.hh>

#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

#include <math.h>

#define PI 3.14159265

namespace gazebo
{

  class GAZEBO_VISIBLE BatteryPlugin : public ModelPlugin
  {
    // Constructor
    public: BatteryPlugin();
    // Deconstructor
    public: virtual ~BatteryPlugin();
    // Load Function
    // Runs Once on Initialization
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
    // Update Function
    // Updates on Every World Update
    protected: void onPoseReceive(const std_msgs::StringConstPtr &msg);
    // Connection to World Update events.
    private: virtual std::string GetModelFromContact(msgs::Contact contact);
    // Animate Battery
    private: virtual void Animate();
    // Action to perform on on pickup
    private: virtual void onPickup(std::string robot_name);
    // Connection to Contact Update events.
    private: event::ConnectionPtr contactConnection;
    // Connection to World Update events.
    private: event::ConnectionPtr worldConnection;
    // Node Used for Communication.
    private: std::mutex mutex;
    // Pointer to the World
    private: physics::WorldPtr world;
    // Pointer to the Model
    private: physics::ModelPtr model;
    // Pointer to the Link
    private: physics::LinkPtr link;
    //Initial Pose of Model
    private: ignition::math::Pose3d pose;
    // Name of the Model
    private: std::string battery_name;


    //ROS Node Handler
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    //ROS publisher
    private: ros::Publisher battery_pub;
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    // Robot pose subscriber
    private: ros::Subscriber robotsubscriber;
  };
}
#endif
