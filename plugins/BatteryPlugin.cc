#include "BatteryPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin)


// Constructor
BatteryPlugin::BatteryPlugin() : ModelPlugin()
{
}


// Deconstructor
BatteryPlugin::~BatteryPlugin()
{
}


// Runs once on initialization
void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Model Pointer
  this->model = _model;
  this->battery_name = this->model->GetName();
  // World Pointer Model is in
  this->world = this->model->GetWorld();
  // Link Pointer to Link
  this->link = this->model->GetLink("battery");

  this->pose = this->model->RelativePose();

  // Connect to the world update event.
  this->worldConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&BatteryPlugin::Animate, this));

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, this->battery_name,
        ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle());

  // Battery publisher
  this->battery_pub = this->rosNode->advertise<std_msgs::String>("collision", 1000);

  std::string topic = "robot_pose";
  ros::SubscribeOptions so =
    ros::SubscribeOptions::create<std_msgs::String>(
        topic,
        10,
        boost::bind(&BatteryPlugin::onPoseReceive, this, _1),
        ros::VoidConstPtr(), NULL);

  this->robotsubscriber = this->rosNode->subscribe(so);
}


// Update Callback
void BatteryPlugin::onPoseReceive(const std_msgs::StringConstPtr &msg)
{
  std::string data = msg->data.c_str();
  std::string delimiter = ":";

  std::vector<std::string> params;

  size_t pos = 0;
  std::string token;
  while ((pos = data.find(delimiter)) != std::string::npos) {
      token = data.substr(0, pos);
      params.push_back(token);
      data.erase(0, pos + delimiter.length());
  }
  params.push_back(data);
  double robot_x = std::stof(params[1]);
  double robot_y = std::stof(params[2]);

  if ( (abs(robot_x - this->pose.Pos().X()) < 0.1) &&
  (abs(robot_y - this->pose.Pos().Y()) < 0.1) )
   {
    this->onPickup(params[0]);
  }
}


std::string BatteryPlugin::GetModelFromContact(msgs::Contact contact)
{
  std::string response;
  if(!this->battery_name.compare(contact.collision1().substr(0, contact.collision1().find("::"))))
  {
    response = contact.collision2().substr(0, contact.collision2().find("::"));
  }
  else if(!this->battery_name.compare(contact.collision2().substr(0, contact.collision2().find("::"))))
  {
    response = contact.collision1().substr(0, contact.collision1().find("::"));
  }
  return response;
}


void BatteryPlugin::onPickup(std::string robot_name)
{
  std_msgs::String msg;
  msg.data = robot_name + ":" + this->battery_name;
  std::cout << msg.data << '\n';
  battery_pub.publish(msg);

  this->robotsubscriber.shutdown();

  this->world->RemoveModel(this->battery_name);
}


void BatteryPlugin::Animate()
{
  double time = this->world->SimTime().Double();

  double pos_z = sin((time*96)*PI/180)/20 + 0.26;
  double yaw = time*96*PI/180;

  ignition::math::Pose3d new_pose = ignition::math::Pose3d(
    this->pose.Pos().X(),
    this->pose.Pos().Y(),
    pos_z,
    0,
    0.5,
    yaw);

  this->model->SetLinkWorldPose(new_pose, this->link);

  if (ros::ok())
  {
    ros::spinOnce();
  }
}
