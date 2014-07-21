/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "TrackDrivePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(TrackDrivePlugin)


/////////////////////////////////////////////////
TrackDrivePlugin::TrackDrivePlugin()
{
  this->maxForce = 5.0;
  this->wheelRadius = 0.0;
  this->wheelSeparation = 0.0;
  this->type = SOIL;
  this->maxTrackForce = 5.0;
  // this->resistance1 = this->resistance2 = this->resistance3 = 0;
  this->slipRatio = 1.0;
  this->weight = 20.0;
  updateSoilProperty();
}

/////////////////////////////////////////////////
TrackDrivePlugin::updateSoilType(SoilType _soil)
{
  this->type = _soil;
}

/////////////////////////////////////////////////
TrackDrivePlugin::updateSoilProperty()
{
  switch(this->type)
  {
    default:
    case SAND:
      n = 1.1;
      k_c = 0.95;
      k_phi = 1528.43;
      c = 1.04;
      phi = 28;
      K = 1;
      mu_r = 0.2;
      mu_l = 1.15;
      break;
    case SANDY LOAM:
      n = 0.7;
      k_c = 5.27;
      k_phi = 1515.04;
      c = 1.72;
      phi = 29;
      K = 2.5;
      mu_r = 0.2;
      mu_l = 1.3;
      break;
    case CLAYEY SOIL:
      n = 0.5;
      k_c = 13.19;
      k_phi = 692.15;
      c = 1.72;
      phi = 13;
      K = 0.6;
      mu_r = 0.3;
      mu_l = 0.6;
      break;
    case DRY CLAY:
      n = 0.13;
      k_c = 12.70;
      k_phi = 1555.95;
      c = 0;
      phi = 34;
      K = 0.6;
      mu_r = 0.1;
      mu_l = 0.8;
      break;
  }
}

/////////////////////////////////////////////////
TrackDrivePlugin::updateSlipRatio()
{
  // Calaculate it each time for the each track

  // Relative error in Slip Ratio calculates increases as it goes below 0.1
  // but absolute error is very small, for any condition
  slipRatio = -(soil.K / track.length);
  slipRatio = slipRatio/(ln(torque * wheelRadius/(track.length * track.breadth * soil.c + weight * tan(soil.phi))));
}

/////////////////////////////////////////////////
TrackDrivePlugin::updateTractionForce()
{
  maxTrackForce = (track.length * track.breadth * soil.c + weight * tan(phi));
  maxTrackForce = maxTrackForce*(1 - soil.K *(1 - exp(-slipRatio * track.length / soil.K)));
  if(maxTrackForce > (mu_r * weight)/2))
  {
    maxTrackForce = (mu_r * weight)/2);
  }
}

/////////////////////////////////////////////////
TrackDrivePlugin::updateTrack()
{
  // this->SetLinkWorldPose(math::Pose pose, linkname or linkptr);
  // linkptr->MoveFrame(Pose src, Pose dest (world frame));

  // pause the sim??
  this->model->GetWorld()->SetPaused(true);
  // A-> move the wheels back to 0 0 0 0 0 0 using the axle
  math::Pose pose;
  math::Vector3 vec;
  pose.pos.Set(0, 0, 0);
  vec = Vector3(0, 0, 0);
  pose.rot.SetFromEuler(vec);
  this->SetLinkWorldPose(math::Pose pose, linkname );
  // this sets everything back in its place
  this->model->GetWorld()->SetPaused(false);
  // unpause it??
}

/*/////////////////////////////////////////////////
TrackDrivePlugin::updateSinkage()
{
  sinkage = (weight/(track.length * track.breadth))/(soil.k_c/track.breadth + soil.k_phi);
  sinkage = pow(sinkage, 1/soil.n);
}*/

/////////////////////////////////////////////////
int TrackDrivePlugin::RegisterWheelJoint(int _index, const std::string &_name)
{
  // Bounds checking on index
  if (_index < 0 or _index >= NUMBER_OF_WHEELS)
  {
    gzerr << "Joint index " << _index <<  " out of bounds [0, "
          << NUMBER_OF_WHEELS << "] in model " << this->model->GetName()
          << "." << std::endl;
  }

  // Find the specified joint and add it to out list
  this->wheelJoints[_index] = this->model->GetJoint(_name);
  if (!this->wheelJoints[_index])
  {
    gzerr << "Unable to find the " << _name
          <<  " joint in model " << this->model->GetName() << "." << std::endl;
    return 1;
  }

  // Success!
  return 0;
}

/////////////////////////////////////////////////
void TrackDrivePlugin::Load(physics::ModelPtr _model,
                                sdf::ElementPtr   _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  int err = 0;

  err += RegisterWheelJoint(RIGHT_FRONT, "right_front");
  err += RegisterWheelJoint(RIGHT_REAR,  "right_rear");
  err += RegisterWheelJoint(LEFT_FRONT,  "left_front");
  err += RegisterWheelJoint(LEFT_REAR,   "left_rear");

  // Put something for number of links in the chain

  if (err > 0)
    return;

  if (_sdf->HasElement("max_force"))
    this->maxForce = _sdf->GetElement("max_force")->Get<double>();
  else
    gzwarn << "No MaxForce value set in the model sdf, default value is 5.0.\n";

  if (_sdf->HasElement("weight"))
    this->maxForce = _sdf->GetElement("weight")->Get<double>();
  else
    gzwarn << "No value for weight set in the model sdf, default value is 20.0.\n";

  if (_sdf->HasElement("soil_type"))
    this->type = _sdf->GetElement("soil_type")->Get<int>();
  else
    gzwarn << "No soil type set in the model sdf, default value is 0 (SOIL).\n";
  if(this->type > DRY_CLAY || this->type < 0)
  {
    this->type = SAND;
  }

  // This assumes that front and rear wheel spacing is identical
  this->wheelSeparation = this->wheelJoints[RIGHT_FRONT]->GetAnchor(0).Distance(
                          this->wheelJoints[LEFT_FRONT]->GetAnchor(0));

  // This assumes that the largest dimension of the wheel is the diameter
  // and that all wheels have the same diameter
  physics::EntityPtr wheelLink = boost::dynamic_pointer_cast<physics::Entity>(
                                        this->wheelJoints[RIGHT_FRONT]->GetChild() );
  if (wheelLink)
  {
    math::Box bb = wheelLink->GetBoundingBox();
    this->wheelRadius = bb.GetSize().GetMax() * 0.5;
  }

  // Validity checks...
  if (this->wheelSeparation <= 0)
  {
    gzerr << "Unable to find the wheel separation distance." << std::endl
          << "  This could mean that the right_front link and the left_front "
          << "link are overlapping." << std::endl;
    return;
  }
  if (this->wheelRadius <= 0)
  {
    gzerr << "Unable to find the wheel radius." << std::endl
          << "  This could mean that the sdf is missing a wheel link on "
          << "the right_front joint." << std::endl;
    return;
  }

  this->velSub = this->node->Subscribe(
    std::string("~/") + this->model->GetName() + std::string("/vel_cmd"),
    &TrackDrivePlugin::OnVelMsg, this);

  for (int i = 0; i < NUMBER_OF_WHEELS; i++)
    this->wheelJoints[i]->SetMaxForce(0, this->maxForce);
}

/////////////////////////////////////////////////
void TrackDrivePlugin::OnVelMsg(ConstPosePtr &_msg)
{
  // gzmsg << "cmd_vel: " << msg->position().x() << ", "
  //       << msgs::Convert(msg->orientation()).GetAsEuler().z << std::endl;

  // these are the omega for the wheels
  double vel_lin = _msg->position().x() / this->wheelRadius;
  double vel_rot = -1 * msgs::Convert(_msg->orientation()).GetAsEuler().z
                   * (this->wheelSeparation / this->wheelRadius);

  torque = (vel_lin - vel_rot);
  updateSlipRatio();
  updateTractionForce();
  for (int i = 0; i < NUMBER_OF_WHEELS; i++)
    this->wheelJoints[i]->SetMaxForce(0, this->maxTrackForce);
  this->wheelJoints[RIGHT_FRONT]->SetVelocity(0, (vel_lin - vel_rot)*(1 - slipRatio));
  this->wheelJoints[RIGHT_REAR ]->SetVelocity(0, (vel_lin - vel_rot)*(1 - slipRatio));

  torque = (vel_lin + vel_rot);
  updateSlipRatio();
  updateTractionForce();
  for (int i = 0; i < NUMBER_OF_WHEELS; i++)
    this->wheelJoints[i]->SetMaxForce(0, this->maxTrackForce);
  this->wheelJoints[LEFT_FRONT ]->SetVelocity(0, (vel_lin + vel_rot)*(1 - slipRatio));
  this->wheelJoints[LEFT_REAR  ]->SetVelocity(0, (vel_lin + vel_rot)*(1 - slipRatio));
}
