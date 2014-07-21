/*
 * Copyright (C) 2013-2014 IMS LICENSE
 * You may obtain a copy of the License at
 *
 *     http://www.bitbucket.org/italiammarssociety/eras/src/
 *     LICENSE
 *
 * @author Kunal Tyagi
 *
*/

#include "SwivelPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(SwivelPlugin)


/////////////////////////////////////////////////
SwivelPlugin::SwivelPlugin()
{
  this->maxForce = 5.0;
  this->jointAngle = 0.0;
}

/////////////////////////////////////////////////
int SwivelPlugin::RegisterJoint(const std::string &_name)
{
  // if NUMBER_OF_JOINTS != 1 in future, make _index one of the
  // parameters of the function call
  int _index = SWIVEL_PIVOT;

  // Bounds checking on index
  if (NUMBER_OF_JOINTS != 1)
  {
    gzerr << "Joint index " << NUMBER_OF_JOINTS <<  " out of bounds [0, 1] in model " << this->model->GetName()
          << "." << std::endl;
  }

  // Find the specified joint and add it to out list
  this->joints[_index] = this->model->GetJoint(_name);
  if (!this->joints[_index])
  {
    gzerr << "Unable to find the " << _name
          <<  " joint in model " << this->model->GetName() << "." << std::endl;
    return 1;
  }

  // Success!
  return 0;
}

/////////////////////////////////////////////////
void SwivelPlugin::Load(physics::ModelPtr _model,
                                sdf::ElementPtr   _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  int err = 0;

  err += RegisterJoint("swivel_pivot");

  if (err > 0)
    return;

  if (_sdf->HasElement("max_force"))
    this->maxForce = _sdf->GetElement("max_force")->Get<double>();
  else
    gzwarn << "No MaxForce value set in the model sdf, default value is 5.0.\n";

  if (_sdf->HasElement("max_force"))
    this->axis = _sdf->GetElement("axis")->Get<int>();
  else
    gzwarn << "No MaxForce value set in the model sdf, default value is 0 (X axis).\n";

  // Validity checks...
  while (this->jointAngle > M_PI)
  {
    gzwarn << "Angle out of bounds, trying to fit it right in\n"
           << "Maybe because the angle should be in Radians (-pi, pi]" << std::endl;
    this->jointAngle = (2 * M_PI) - this->jointAngle;
  }
  while (this->jointAngle < -M_PI)
  {
    gzwarn << "Angle out of bounds, trying to fit it right in\n"
           << "Maybe because the angle should be in Radians (-pi, pi]" << std::endl;
    this->jointAngle = (2 * M_PI) + this->jointAngle;
  }


  this->intSub = this->node->Subscribe(
    std::string("~/") + this->model->GetName() + std::string("/angle"),
    &SwivelPlugin::OnIntMsg, this);
}


/////////////////////////////////////////////////
void SwivelPlugin::OnIntMsg(ConstIntPtr &_msg)
{
  // gzdbg << "Target angle: " << _msg->data() <<  std::endl;

  for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    this->joints[i]->SetMaxForce(0, this->maxForce);

  double targetAngle = (_msg->data()) * (M_PI/180);

  // this->joints[SWIVEL_PIVOT]->SetPosition((unsigned int)axis, (double)targetAngle);
  this->joints[SWIVEL_PIVOT]->SetForce(axis,
          this->maxForce*(targetAngle - this->joints[SWIVEL_PIVOT]->GetAngle(axis).Radian()));
}
