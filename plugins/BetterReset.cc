#include "BetterReset.hh"

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(BetterReset)


// Constructor
BetterReset::BetterReset() : WorldPlugin()
{

}

// Deconstructor
BetterReset::~BetterReset()
{

}

// Runs once on initialization
void BetterReset::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->world = _parent;
  this->worldSDF = _parent->SDF()->Clone();

  this->initial_model_names = this->GetModelNames();

  /*
  For Debug Purposes
  for (std::vector<std::string>::iterator ni = this->initial_model_names.begin();
    ni != this->initial_model_names.end(); ++ni)
  {
      std::cout << (*ni) << '\n';
  }
  */

  //sdf::ElementPtr prev_link = this->worldSDF->GetFirstElement();

  // %this->link_rate is used because at every 50 link model is refreshed and written on text file
  /*for (int i = 0; i<3; i++) {

      prev_link = prev_link->GetNextElement("model");
      std::string prev_pose = prev_link->GetAttribute("name")->GetAsString();

      std::cout << prev_pose << '\n';
  }*/


  // Connect to the world update event.
  this->resetConnection = event::Events::ConnectWorldReset(
          std::bind(&BetterReset::OnReset, this));
}


// Runs on world reset
void BetterReset::OnReset()
{
  std::vector<std::string> model_names = this->GetModelNames();
  std::vector<std::string> removed_models;

  std::set_difference(
    this->initial_model_names.begin(), this->initial_model_names.end(),
    model_names.begin(), model_names.end(),
    std::back_inserter(removed_models)
  );

  for (std::vector<std::string>::iterator ni = removed_models.begin();
    ni != removed_models.end(); ++ni)
  {
      sdf::ElementPtr model = this->GetModelSdf((*ni));

      sdf::SDF modelSDF;
      modelSDF.Root(model);
      this->world->InsertModelSDF(modelSDF);
  }
}


std::vector<std::string> BetterReset::GetModelNames()
{
    physics::Model_V models = this->world->Models();
    std::vector<std::string> model_names;

    for (std::vector<physics::ModelPtr>::iterator mi = models.begin();
      mi != models.end(); ++mi)
    {
        model_names.push_back((*mi)->GetName());
    }
    std::sort(model_names.begin(), model_names.end());
    return model_names;
}


sdf::ElementPtr BetterReset::GetModelSdf(std::string model_name)
{
  sdf::ElementPtr model = this->worldSDF->GetFirstElement();

  if (!model->GetName().compare("model"))
  {
    if(!model->GetAttribute("name")->GetAsString().compare(model_name))
    {
      return model;
    }
  }

  for (int i = 0; i<this->initial_model_names.size(); i++) {
      model = model->GetNextElement("model");
      std::string model_data_name = model->GetAttribute("name")->GetAsString();

      if(!model->GetAttribute("name")->GetAsString().compare(model_name))
      {
        return model;
      }
  }
  return model;
}
