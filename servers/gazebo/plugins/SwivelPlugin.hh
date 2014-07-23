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
#ifndef _TREVOR_Swivel_PLUGIN_HPP_
#define _TREVOR_Swivel_PLUGIN_HPP_

#include <string>
#include <cmath>

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
//#include "gazebo/util/system.hh"

#define NUMBER_OF_JOINTS 1

namespace gazebo
{
  // \class SwivelPlugin SwivelPlugin.hpp
  class SwivelPlugin : public ModelPlugin
  {
    /// \brief Default Contstuctor
    public: SwivelPlugin();

    /// \brief Called when the plugin is loaded
    /// \param[in] _model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \def ID for each of the joint(s)
    // for future use, incase more than one joints are used
    public: enum {SWIVEL_PIVOT};

    /// \brief Associates a joint to each of the movable part
    /// \param[in] _name Name of the joint
    /// \return {0: Success, else: Error}
    private: int RegisterJoint(const std::string &_name);

    /// \brief Callback for gazebo topic
    /// \param[in] _msg Pose message from external Publisher
    // in this case, from PyGazebo publisher
    // angle is in degrees for sake of convinence
    private: void OnIntMsg(ConstIntPtr &_msg);

    /// \brief Node for subscriber
    private: transport::NodePtr node;

    /// \brief Gazebo topic subscriber
    private: transport::SubscriberPtr intSub;

    /// \brief Pointer to the model which this plugin is attached
    private: physics::ModelPtr model;

    /// \brief Pointer to each wheel joint
    private: physics::JointPtr joints[NUMBER_OF_JOINTS];

    /// \brief Max force limit for each joint (Default 5.0)
    private: double maxForce;

    /// \brief Axis for the rotation. 0 is X, 1 is Y and 2 is Z (Default 0)
    private: int axis;

    /// \brief Angle, in radians, to be achieved (Determined from published message, default 0.0)
    private: double jointAngle;
  };
}
#endif
