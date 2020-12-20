/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#include <stdexcept>

#include <sensor_msgs/JointState.h>

#include <abb_robot_msgs/SystemState.h>
#include <abb_rapid_sm_addin_msgs/RuntimeState.h>

#include <abb_robot_cpp_utilities/initialization.h>
#include <abb_robot_cpp_utilities/mapping.h>
#include <abb_robot_cpp_utilities/parameters.h>
#include <abb_robot_cpp_utilities/verification.h>

#include "abb_rws_state_publisher/rws_state_publisher.h"

namespace
{
/**
 * \brief Name for ROS logging in the 'init' context.
 */
constexpr char ROS_LOG_INIT[]{"init"};

/**
 * \brief Name for ROS logging in the 'publisher' context.
 */
constexpr char ROS_LOG_PUBLISHER[]{"publisher"};

/**
 * \brief Time [s] for throttled ROS logging.
 */
constexpr double THROTTLE_TIME{10.0};
}

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Struct definitions: RWSServiceProvider::ParameterHandler
 */

RWSStatePublisher::ParameterHandler::ParameterHandler(ros::NodeHandle& nh)
:
robot_port{0},
polling_rate{0.0}
{
  ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Get ROS parameters (namespace '" << nh.getNamespace() << "')");

  // IP address.
  utilities::getParameter(nh, "robot_ip", robot_ip, std::string{"127.0.0.1"});
  utilities::verifyIPAddress(robot_ip);

  // Port number.
  utilities::getParameter(nh, "robot_port", robot_port, 80);
  utilities::verifyPortNumber(robot_port);

  // Robot controller nickname/identifier.
  utilities::getParameter(nh, "robot_nickname", robot_nickname, std::string{});

  // Indicator for allowing no connection timeout during initialization.
  utilities::getParameter(nh, "no_connection_timeout", no_connection_timeout, false);

  // Polling rate.
  utilities::getParameter(nh, "polling_rate", polling_rate, 5.0);
  utilities::verifyRate(polling_rate);
}

/***********************************************************************************************************************
 * Class definitions: RWSStatePublisher
 */

/***********************************************************
 * Primary methods
 */

RWSStatePublisher::RWSStatePublisher(ros::NodeHandle& nh_params, ros::NodeHandle& nh_msgs)
:
parameters_{nh_params},
rws_manager_{parameters_.robot_ip,
             static_cast<unsigned short>(parameters_.robot_port),
             rws::SystemConstants::General::DEFAULT_USERNAME,
             rws::SystemConstants::General::DEFAULT_PASSWORD},
polling_rate_{parameters_.polling_rate}
{
  ROS_INFO_NAMED(ROS_LOG_INIT, "Initializing...");

  //--------------------------------------------------------
  // Connect to the robot controller
  //--------------------------------------------------------
  robot_controller_description_ = utilities::establishRWSConnection(rws_manager_,
                                                                    parameters_.robot_nickname,
                                                                    parameters_.no_connection_timeout);

  ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Robot controller description:\n" << summaryText(robot_controller_description_));

  utilities::verifyRobotWareVersion(robot_controller_description_.header().robot_ware_version());
  initializeMotionData(motion_data_, robot_controller_description_);

  //--------------------------------------------------------
  // Advertise publishers
  //--------------------------------------------------------
  joint_state_publisher_ = nh_msgs.advertise<sensor_msgs::JointState>("joint_states", 1);
  system_state_publisher_ = nh_msgs.advertise<abb_robot_msgs::SystemState>("system_states", 1);

  if(utilities::verifyStateMachineAddInPresence(robot_controller_description_.system_indicators()))
  {
    sm_runtime_state_publisher_ = nh_msgs.advertise<abb_rapid_sm_addin_msgs::RuntimeState>("sm_addin/runtime_states",1);
  }

  //--------------------------------------------------------
  // Timer
  //--------------------------------------------------------
  polling_timer_ = nh_msgs.createTimer(polling_rate_, &RWSStatePublisher::pollingTimerCallback, this);

  ROS_INFO_NAMED(ROS_LOG_INIT, "Initialization succeeded, and the node is ready for use");
}

/***********************************************************
 * Auxiliary methods
 */

void RWSStatePublisher::pollingTimerCallback(const ros::TimerEvent& event)
{
  (void) event;

  try
  {
    rws_manager_.collectAndUpdateRuntimeData(system_state_data_, motion_data_);
  }
  catch(const std::runtime_error& exception)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_TIME, ROS_LOG_INIT,
                                   "Periodic polling of runtime data via RWS failed with '" << exception.what() <<
                                   "' (will try again later)");
  }

  //--------------------------------------------------------
  // Parse joint states
  //--------------------------------------------------------
  sensor_msgs::JointState joint_state_message{};
  for(auto& group : motion_data_.groups)
  {
    for(auto& unit : group.units)
    {
      for(auto& joint : unit.joints)
      {
        joint_state_message.name.push_back(joint.name);
        joint_state_message.position.push_back(joint.state.position);
      }
    }
  }

  //--------------------------------------------------------
  // Parse general system states
  //--------------------------------------------------------
  abb_robot_msgs::SystemState system_state_message{};
  system_state_message.motors_on = system_state_data_.motors_on.isTrue();
  system_state_message.auto_mode = system_state_data_.auto_mode.isTrue();
  system_state_message.rapid_running = system_state_data_.rapid_running.isTrue();

  for(const auto& task : system_state_data_.rapid_tasks)
  {
    abb_robot_msgs::RAPIDTaskState state{};

    state.name = task.name;
    state.activated = task.is_active;
    state.execution_state = utilities::map(task.execution_state);
    state.motion_task = task.is_motion_task;

    system_state_message.rapid_tasks.push_back(state);
  }

  for(const auto& unit : system_state_data_.mechanical_units)
  {
    abb_robot_msgs::MechanicalUnitState state{};
    state.name = unit.first;
    state.activated = unit.second.active;
    system_state_message.mechanical_units.push_back(state);
  }

  //--------------------------------------------------------
  // Parse StateMachine Add-In states
  //--------------------------------------------------------
  abb_rapid_sm_addin_msgs::RuntimeState sm_runtime_state_message{};
  const auto& system_indicators{robot_controller_description_.system_indicators()};
  if(utilities::verifyStateMachineAddInPresence(system_indicators))
  {
    for(const auto& sm : system_state_data_.state_machines)
    {
      abb_rapid_sm_addin_msgs::StateMachineState state{};
      state.rapid_task = sm.rapid_task;
      state.sm_state = utilities::mapStateMachineState(sm.sm_state);
      state.egm_action = utilities::mapStateMachineEGMAction(sm.egm_action);
      sm_runtime_state_message.state_machines.push_back(state);
    }
  }

  //--------------------------------------------------------
  // Publish the messages
  //--------------------------------------------------------
  auto time_now{ros::Time::now()};

  joint_state_message.header.stamp = time_now;
  joint_state_publisher_.publish(joint_state_message);

  system_state_message.header.stamp = time_now;
  system_state_publisher_.publish(system_state_message);

  sm_runtime_state_message.header.stamp = time_now;
  sm_runtime_state_publisher_.publish(sm_runtime_state_message);
}

}
}
