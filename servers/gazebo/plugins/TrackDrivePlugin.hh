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

/** Assumptions:
 * Constant soil type
 * 2 tracks only
 * For now, 2 wheels for each track
 * Constant soil properties
 * Negleting air drag forces (negligible due to low speed)
 * Neglecting vehicle belly drag forces (negligible due to minimal ineraction  of land and vehicle belly)
 * Constant contact area
 * Uniform slope
 * Tension in the track is taken care of by Gazebo
 * So are frictional forces on it
 * Total torque by motors is ZERO
 * Center of mass is in the center of the tracks (working on removing it)
 * No memory of terrain
 * Terrain is uncompressed
*/

#ifndef _GAZEBO_SkidSteerDrive_PLUGIN_HH_
#define _GAZEBO_SkidSteerDrive_PLUGIN_HH_

#include <string>
#include <cmath>
#include <iostream>

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/util/system.hh"

#define NUMBER_OF_WHEELS 4
#define NUMBER_OF_TRACKS 2

namespace gazebo
{
  // \class TrackDrivePlugin TrackDrivePlugin.hh
  /// \brief A gazebo model plugin that controls a four wheel skid-steer
  ///        robot via a gazebo topic. See the Pioneer3AT model in the
  ///        OSRF model database for an example use case.
  class GAZEBO_VISIBLE TrackDrivePlugin : public ModelPlugin
  {
    /// \brief Default Contstuctor
    public: TrackDrivePlugin();

    /// \brief Called when the plugin is loaded
    /// \param[in] _model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \def ID for each of the four wheels
    public: enum {RIGHT_FRONT, RIGHT_REAR, LEFT_FRONT, LEFT_REAR};

    /// \def ID for different soil types
    public: enum SoilType {SAND, SANDY_LOAM, CLAYEY_SOIL, DRY_CLAY};

    /// \brief Properties of a soil
    public: class soilProperty
    {

        /// \brief [Wong 1989]
        public: double n, k_c, k_phi, c, phi, K, mu_r, mu_l;
        /// \brief [Wong and Preston 1984]
        public: double z_0, z_u, k_0, k_u, A_u, b;
        /// \brief [Rutherd 1986]
        public: double gamma_s, k_c2, k_phi2;
        /// \brief Actually property of soil and tracks
        /// \detail k_r depends on the size of belly(distance and length of the tracks) (Default 3.0)
        ///         s is distance moved bu soil particles along with the track
        public: double k_r, s;

        public: soilProperty()
        {
            n = k_c = k_phi = c = phi = K = mu_r = mu_l = 0;
            z_0 = z_u = k_0 = k_u = A_u = b = 0;
            gamma_s = k_c2 = k_phi2 = 0;
            k_r = s = 0;
        }
    };

    /// \brief Properties of a track
    public: class trackProperty
    {
        /// \brief Dimension of the track
        public: double length, breadth;

        /// \bredth Weight applied on the tracks
        public: double weight;

        public: trackProperty()
        {
            length = breadth = 0;
            weight = 0;
        }
    };

    /// \brief Update the parameters for the soil
    public: void updateSoilProperty();

    /// \brief Calculate the slip ratio
    public: void updateSlipRatio();

    /// \brief Update the max possible traction force
    public: void updateTractionForce();

    /// \brief Finish the animation of the track
    public: void updateTrack();

    /// \brief Update the type of soil
    /// \param[in] _soil ID to identify the soil type
    public: void updateSoilType(SoilType _soil);

    /// \brief Associates a joint to each of the wheels
    /// \param[in] _index Internal wheel index (Zero based)
    /// \param[in] _name Name wheel joint
    /// \return {0: Success, else: Error}
    private: int RegisterWheelJoint(int _index, const std::string &_name);

    /// \brief Callback for gazebo topic
    /// \param[in] _msg Pose message from external publisher
    private: void OnVelMsg(ConstPosePtr &_msg);

    /// \brief Node for subscriber
    private: transport::NodePtr node;

    /// \brief Gazebo topic subscriber
    private: transport::SubscriberPtr velSub;

    /// \brief Pointer to the model which this plugin is attached
    private: physics::ModelPtr model;

    /// \brief Pointer to each wheel joint
    private: physics::JointPtr wheelJoints[NUMBER_OF_WHEELS];

    /// \brief Max force limit for each wheel joint (Default 5.0)
    private: double maxForce;

    /// \brief slip ratio for the tracks (Default 1.0)
    private: double slipRatio;

    /// \brief Max force possible to be applied by the terra firma to the tracks (Default 5.0)
    /// \formula [Ac + Wtan phi](1 - K * (1 - exp(- slipRatio * l / K)) / (slipRatio * l))
    private: double maxTrackForce;

    /// \brief Actual torque being applied by the wheels
    private: double torque;

    /// \brief Weight of the vehicle (or rather the sum of p(x) * g over the length of the track) (Default 20.0)
    private: double weight;

    /// \brief Properties of the soil, temporary until I figure someway to get
    ///        soil property from the world
    private: soilProperty soil;

    /// \brief Properties of the track
    private: trackProperty track;

    /// \brief Type of soil, same condition as soilProperty soil (Default SAND)
    private: SoilType type;

    /*
    Not in use currently
    /// \brief Resistance due to friction between soil and track
    /// \formula \frac{2 * b* K * z_{0}^{n+1} }{n + 1}
    /// \brief Resistance due to friction in sliding the wheel over new soil
    /// \formula (\frac{2 * b* sin(alpha + theta)}{sin alpha * cos theta})(2* z_{0} * c * k_{c} + gamma_{s} * z_{0}^{2} * k_{r})
    /// \brief Resistance due to friction between soil particles
    /// \formula 2 * k_{r} * z_{0} * s * c
    /// \brief However, all these resistances can be summed up in the following manner
    ///        Resistance on each wheel: mu_r * weight /2
    ///        Rest: wheelSeparation/2 * (omega_1 * (1- slipRatio1) + omega_2 * (1- slipRatio2)) / (omega_2 * (1- slipRatio2) - omega_1 * (1- slipRatio1))
    ///        where omega*wheelRadius is the speed of movement of track
    private: double resistance1;
    private: double resistance2;
    private: double resistance3;

    /// \brief Amount by which the ground sinks
    private: double sinkage;
    */

    /// \brief Distance between wheels on the same axis (Determined from SDF)
    private: double wheelSeparation;

    /// \brief Radius of the wheels (Determined from SDF)
    private: double wheelRadius;
  };
}
#endif
