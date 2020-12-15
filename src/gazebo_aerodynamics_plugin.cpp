/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Aerodynmaics Plugin
 *
 * Model aerodynamic forces 
 *
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 */

#include <gazebo_aerodynamics_plugin.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(AerodynamicsPlugin)

AerodynamicsPlugin::AerodynamicsPlugin() : ModelPlugin()
{ }

AerodynamicsPlugin::~AerodynamicsPlugin()
{
    if (updateConnection_)
      updateConnection_->~Connection();
}

void AerodynamicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model.
  model_ = _model;
  world_ = model_->GetWorld();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_aerodynamics_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_aerodynamics_plugin] Please specify a linkName.\n";
  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_aerodynamics_plugin] Couldn't find specified link \"" << link_name_ << "\".");

  if (_sdf->HasElement("surface_area")) {
    surface_area_ = _sdf->GetElement("surface_area")->Get<double>();
  } else {
    gzerr << "[gazebo_aerodynamics_plugin] Please specify the surface area.\n";
  }
  std::string actuator_sub_topic_ = "/gazebo/command/motor_speed";
  if (_sdf->HasElement("commandSubTopic")) {
    actuator_sub_topic_ = _sdf->GetElement("commandSubTopic")->Get<std::string>();
  }

  int n_out_max = 16;
  cl_delta_vector_ = Eigen::VectorXd::Zero(n_out_max);
  cd_delta_vector_ = Eigen::VectorXd::Zero(n_out_max);
  cm_delta_vector_ = Eigen::VectorXd::Zero(n_out_max);

  if (_sdf->HasElement("cl0")) {
    CL_0_ = _sdf->Get<double>("cl0");
  }
  if (_sdf->HasElement("cd0")) {
    CD_0_ = _sdf->Get<double>("cd0");
  }
  if (_sdf->HasElement("cm0")) {
    CM_0_ = _sdf->Get<double>("cm0");
  }

  if (_sdf->HasElement("cl_alpha")) {
    CL_alpha_ = _sdf->Get<double>("cl_alpha");
  }
  if (_sdf->HasElement("cd_alpha")) {
    CD_alpha_ = _sdf->Get<double>("cd_alpha");
  }
  if (_sdf->HasElement("cm_alpha")) {
    CM_alpha_ = _sdf->Get<double>("cm_alpha");
  }

  if (_sdf->HasElement("control_channels")) {
    sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
    sdf::ElementPtr channel = control_channels->GetElement("channel");

    while (channel) {
      if (channel->HasElement("input_index")) {
        int index = channel->Get<int>("input_index");
        if (index < n_out_max) {
          if (channel->HasElement("cl_delta")) {
            cl_delta_vector_[index] = channel->Get<double>("cl_delta");
          }
          if (channel->HasElement("cd_delta")) {
            cd_delta_vector_[index] = channel->Get<double>("cd_delta");
          }
          if (channel->HasElement("cm_delta")) {
            cm_delta_vector_[index] = channel->Get<double>("cm_delta");
          }
        }
      }
      channel = channel->GetNextElement("channel");
    }
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&AerodynamicsPlugin::OnUpdate, this, _1));

  wind_sub_ = node_handle_->Subscribe("~/world_wind", &AerodynamicsPlugin::WindVelocityCallback, this);
  actuator_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + actuator_sub_topic_, &AerodynamicsPlugin::ActuatorCallback, this);
}

void AerodynamicsPlugin::OnUpdate(const common::UpdateInfo&){
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time current_time = world_->SimTime();
#else
  common::Time current_time = world_->GetSimTime();
#endif

#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Pose3d T_W_I = link_->WorldPose();
#else
  ignition::math::Pose3d T_W_I = ignitionFromGazeboMath(link_->GetWorldPose());
#endif
  ignition::math::Quaterniond C_W_I = T_W_I.Rot();

  //Calculate airspeed vector in body frame
#if GAZEBO_MAJOR_VERSION >= 9
  ignition::math::Vector3d vel_a = link_->RelativeLinearVel() - C_W_I.RotateVector(wind_vel_);
#else
  ignition::math::Vector3d vel_a = ignitionFromGazeboMath(link_->GetRelativeLinearVel()) - C_W_I.RotateVector(wind_vel_);
#endif

  last_time_ = current_time;

  double rho = 1.225f;

  //Calculate air relative attitudes
  double alpha = atan2(vel_a.Z(), vel_a.X());
  double beta = atan2(vel_a.Y(), vel_a.X());

  const double dynamic_pressure = 0.005f * rho * vel_a.X() * vel_a.X();
  const double q_bar_S = dynamic_pressure * surface_area_;
  
  const double drag_coefficient=0.01;
  const double sideslip_coefficient=0.01;

  const double lift = q_bar_S * getLiftCoefficient(alpha);
  const double drag = q_bar_S * getDragCoefficient(alpha);
  const double moment = q_bar_S * getMomentCoefficient(alpha);
  const double sideslip = q_bar_S * getSideSlipCoefficient(beta);

  ignition::math::Vector3d force(-drag, sideslip, lift);
  ignition::math::Vector3d cp; //Center of pressure
  //TODO: Hook this up to a real flight
  // link_->AddForceAtRelativePosition(force, cp);
  // link_->AddTorque(moment);
}

double AerodynamicsPlugin::getLiftCoefficient(double alpha) {
  return CL_0_ + CL_alpha_ * alpha + cl_delta_vector_.dot(input_reference_);
}

double AerodynamicsPlugin::getDragCoefficient(double alpha) {
  return CD_0_ + CD_alpha_ * alpha + cd_delta_vector_.dot(input_reference_);
}

double AerodynamicsPlugin::getMomentCoefficient(double beta) {
  return CM_0_ + CM_alpha_ * beta + cm_delta_vector_.dot(input_reference_);
}

double AerodynamicsPlugin::getSideSlipCoefficient(double beta) {
  return CN_0_ + CN_beta_ * beta + cn_delta_vector_.dot(input_reference_);
}

void AerodynamicsPlugin::WindVelocityCallback(WindPtr& msg) {
  wind_vel_ = ignition::math::Vector3d(msg->velocity().x(),
            msg->velocity().y(),
            msg->velocity().z());
}

void AerodynamicsPlugin::ActuatorCallback(CommandMotorSpeedPtr &rot_velocities) {
  //TODO: Subscribe to actuators and filter out what is used and not used
}

} // namespace gazebo
