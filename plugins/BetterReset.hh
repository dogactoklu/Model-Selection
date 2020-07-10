#ifndef GAZEBO_PLUGINS_BETTER_RESET_PLUGIN_HH_
#define GAZEBO_PLUGINS_BETTER_RESET_PLUGIN_HH_

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include <gazebo/sensors/sensors.hh>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "sdf/Element.hh"

#include <ignition/transport/Node.hh>
#include <gazebo/transport/Node.hh>

namespace gazebo
{

  class GAZEBO_VISIBLE BetterReset : public WorldPlugin
  {
    // Constructor
    public: BetterReset();
    // Deconstructor
    public: virtual ~BetterReset();

    // Load Function
    // Runs Once on Initialization
    public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    // World Reset Callback
    public: virtual void OnReset();
    // Get Model Names on the world
    private: virtual std::vector<std::string> GetModelNames();

    private: virtual sdf::ElementPtr GetModelSdf(std::string model_name);

    // Connection to World Reset Event.
    private: event::ConnectionPtr resetConnection;

    // Pointer to the World
    private: physics::WorldPtr world;
    // SDF Variables
    private: sdf::ElementPtr worldSDF;

    // Name of models on the initial world
    private: std::vector<std::string> initial_model_names;




  };
}
#endif
