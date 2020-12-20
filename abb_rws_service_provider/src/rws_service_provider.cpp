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

#include <abb_robot_cpp_utilities/initialization.h>
#include <abb_robot_cpp_utilities/parameters.h>
#include <abb_robot_cpp_utilities/verification.h>

#include <abb_robot_msgs/ServiceResponses.h>

#include "abb_rws_service_provider/rws_service_provider.h"

namespace
{
/**
 * \brief Name for ROS logging in the 'init' context.
 */
constexpr char ROS_LOG_INIT[]{"init"};
}

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Struct definitions: RWSServiceProvider::ParameterHandler
 */

/***********************************************************
 * Primary methods
 */

RWSServiceProvider::ParameterHandler::ParameterHandler(ros::NodeHandle& nh)
:
robot_port{0}
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
}

/***********************************************************************************************************************
 * Class definitions: RWSServiceProvider
 */

/***********************************************************
 * Primary methods
 */

RWSServiceProvider::RWSServiceProvider(ros::NodeHandle& nh_params, ros::NodeHandle& nh_srvs)
:
parameters_{nh_params},
rws_manager_{parameters_.robot_ip,
             static_cast<unsigned short>(parameters_.robot_port),
             rws::SystemConstants::General::DEFAULT_USERNAME,
             rws::SystemConstants::General::DEFAULT_PASSWORD}
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

  //--------------------------------------------------------
  // Register subscribers
  //--------------------------------------------------------
  system_states_subscriber_ = nh_srvs.subscribe("system_states", 1, &RWSServiceProvider::systemStatesCallback, this);

  ros::NodeHandle nh_sm_addin{nh_srvs, "sm_addin"};
  if(utilities::verifyStateMachineAddInPresence(robot_controller_description_.system_indicators()))
  {
    sm_runtime_states_subscriber_ = nh_sm_addin.subscribe("runtime_states", 1,
                                                          &RWSServiceProvider::smAddInRuntimeStatesCallback, this);
  }

  //--------------------------------------------------------
  // Advertise auxiliary services
  //--------------------------------------------------------
  services_.push_back(nh_srvs.advertiseService("get_robot_controller_description",
                                               &RWSServiceProvider::getRCDescription, this));

  //--------------------------------------------------------
  // Advertise core services
  //--------------------------------------------------------
  ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Adding basic services");
  services_.push_back(nh_srvs.advertiseService("get_file_contents", &RWSServiceProvider::getFileContents, this));
  services_.push_back(nh_srvs.advertiseService("get_io_signal",     &RWSServiceProvider::getIOSignal,     this));
  services_.push_back(nh_srvs.advertiseService("get_rapid_bool",    &RWSServiceProvider::getRAPIDBool,    this));
  services_.push_back(nh_srvs.advertiseService("get_rapid_dnum",    &RWSServiceProvider::getRAPIDDnum,    this));
  services_.push_back(nh_srvs.advertiseService("get_rapid_num",     &RWSServiceProvider::getRAPIDNum,     this));
  services_.push_back(nh_srvs.advertiseService("get_rapid_string",  &RWSServiceProvider::getRAPIDString,  this));
  services_.push_back(nh_srvs.advertiseService("get_rapid_symbol",  &RWSServiceProvider::getRAPIDSymbol,  this));
  services_.push_back(nh_srvs.advertiseService("get_speed_ratio",   &RWSServiceProvider::getSpeedRatio,   this));
  services_.push_back(nh_srvs.advertiseService("pp_to_main",        &RWSServiceProvider::ppToMain,        this));
  services_.push_back(nh_srvs.advertiseService("set_file_contents", &RWSServiceProvider::setFileContents, this));
  services_.push_back(nh_srvs.advertiseService("set_io_signal",     &RWSServiceProvider::setIOSignal,     this));
  services_.push_back(nh_srvs.advertiseService("set_motors_off",    &RWSServiceProvider::setMotorsOff,    this));
  services_.push_back(nh_srvs.advertiseService("set_motors_on",     &RWSServiceProvider::setMotorsOn,     this));
  services_.push_back(nh_srvs.advertiseService("set_rapid_bool",    &RWSServiceProvider::setRAPIDBool,    this));
  services_.push_back(nh_srvs.advertiseService("set_rapid_dnum",    &RWSServiceProvider::setRAPIDDnum,    this));
  services_.push_back(nh_srvs.advertiseService("set_rapid_num",     &RWSServiceProvider::setRAPIDNum,     this));
  services_.push_back(nh_srvs.advertiseService("set_rapid_string",  &RWSServiceProvider::setRAPIDString,  this));
  services_.push_back(nh_srvs.advertiseService("set_rapid_symbol",  &RWSServiceProvider::setRAPIDSymbol,  this));
  services_.push_back(nh_srvs.advertiseService("set_speed_ratio",   &RWSServiceProvider::setSpeedRatio,   this));
  services_.push_back(nh_srvs.advertiseService("start_rapid",       &RWSServiceProvider::startRAPID,      this));
  services_.push_back(nh_srvs.advertiseService("stop_rapid",        &RWSServiceProvider::stopRAPID,       this));

  //--------------------------------------------------------
  // Advertise StateMachine Add-In services
  //--------------------------------------------------------
  const auto& system_indicators{robot_controller_description_.system_indicators()};

  if(system_indicators.addins().has_state_machine_1_0())
  {
    ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "StateMachine Add-In 1.0 detected, adding additional services");

    services_.push_back(nh_sm_addin.advertiseService("run_rapid_routine", &RWSServiceProvider::runRAPIDRoutine, this));
    services_.push_back(nh_sm_addin.advertiseService("set_rapid_routine", &RWSServiceProvider::setRAPIDRoutine, this));

    if(system_indicators.options().egm())
    {
      services_.push_back(nh_sm_addin.advertiseService("get_egm_settings", &RWSServiceProvider::getEGMSettings, this));
      services_.push_back(nh_sm_addin.advertiseService("set_egm_settings", &RWSServiceProvider::setEGMSettings, this));
      services_.push_back(nh_sm_addin.advertiseService("start_egm_joint",  &RWSServiceProvider::startEGMJoint,  this));
      services_.push_back(nh_sm_addin.advertiseService("start_egm_pose",   &RWSServiceProvider::startEGMPose,   this));
      services_.push_back(nh_sm_addin.advertiseService("stop_egm",         &RWSServiceProvider::stopEGM,        this));
    }

    if(system_indicators.addins().smart_gripper())
    {
      services_.push_back(nh_sm_addin.advertiseService("run_sg_routine", &RWSServiceProvider::runSGRoutine, this));
      services_.push_back(nh_sm_addin.advertiseService("set_sg_command", &RWSServiceProvider::setSGCommand, this));
    }
  }
  else if(system_indicators.addins().has_state_machine_1_1())
  {
    ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "StateMachine Add-In 1.1 detected, adding additional services");

    services_.push_back(nh_sm_addin.advertiseService("run_rapid_routine", &RWSServiceProvider::runRAPIDRoutine, this));
    services_.push_back(nh_sm_addin.advertiseService("set_rapid_routine", &RWSServiceProvider::setRAPIDRoutine, this));

    if(system_indicators.options().egm())
    {
      services_.push_back(nh_sm_addin.advertiseService("get_egm_settings", &RWSServiceProvider::getEGMSettings, this));
      services_.push_back(nh_sm_addin.advertiseService("set_egm_settings", &RWSServiceProvider::setEGMSettings, this));
      services_.push_back(nh_sm_addin.advertiseService("start_egm_joint",  &RWSServiceProvider::startEGMJoint,  this));
      services_.push_back(nh_sm_addin.advertiseService("start_egm_pose",   &RWSServiceProvider::startEGMPose,   this));
      services_.push_back(nh_sm_addin.advertiseService("start_egm_stream", &RWSServiceProvider::startEGMStream, this));
      services_.push_back(nh_sm_addin.advertiseService("stop_egm",         &RWSServiceProvider::stopEGM,        this));
      services_.push_back(nh_sm_addin.advertiseService("stop_egm_stream",  &RWSServiceProvider::stopEGMStream,  this));
    }

    if(system_indicators.addins().smart_gripper())
    {
      services_.push_back(nh_sm_addin.advertiseService("run_sg_routine", &RWSServiceProvider::runSGRoutine, this));
      services_.push_back(nh_sm_addin.advertiseService("set_sg_command", &RWSServiceProvider::setSGCommand, this));
    }
  }

  ROS_INFO_NAMED(ROS_LOG_INIT, "Initialization succeeded, and the node is ready for use");
}

/***********************************************************
 * Auxiliary methods (miscellaneous)
 */

void RWSServiceProvider::systemStatesCallback(const abb_robot_msgs::SystemState::ConstPtr& message)
{
  system_state_ = *message;
}

void RWSServiceProvider::smAddInRuntimeStatesCallback(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& message)
{
  sm_addin_runtime_state_ = *message;
}

bool RWSServiceProvider::getRCDescription(GetRCDescription::Request&, GetRCDescription::Response& response)
{
  response.description = robot_controller_description_.DebugString();

  response.message = abb_robot_msgs::ServiceResponses::SUCCESS;
  response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;

  return true;
}

/***********************************************************
 * Auxiliary methods (verifications)
 */

bool RWSServiceProvider::verifyAutoMode(uint16_t& result_code, std::string& message)
{
  if(!system_state_.auto_mode)
  {
    message = abb_robot_msgs::ServiceResponses::NOT_IN_AUTO_MODE;
    result_code = abb_robot_msgs::ServiceResponses::RC_NOT_IN_AUTO_MODE;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyArgumentFilename(const std::string& filename, uint16_t& result_code, std::string& message)
{
  if(filename.empty())
  {
    message = abb_robot_msgs::ServiceResponses::EMPTY_FILENAME;
    result_code = abb_robot_msgs::ServiceResponses::RC_EMPTY_FILENAME;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyArgumentRAPIDSymbolPath(const abb_robot_msgs::RAPIDSymbolPath& path,
                                                       uint16_t& result_code, std::string& message)
{
  if(path.task.empty())
  {
    message = abb_robot_msgs::ServiceResponses::EMPTY_RAPID_TASK_NAME;
    result_code = abb_robot_msgs::ServiceResponses::RC_EMPTY_RAPID_TASK_NAME;
    return false;
  }

  if(path.module.empty())
  {
    message = abb_robot_msgs::ServiceResponses::EMPTY_RAPID_MODULE_NAME;
    result_code = abb_robot_msgs::ServiceResponses::RC_EMPTY_RAPID_MODULE_NAME;
    return false;
  }

  if(path.symbol.empty())
  {
    message = abb_robot_msgs::ServiceResponses::EMPTY_RAPID_SYMBOL_NAME;
    result_code = abb_robot_msgs::ServiceResponses::RC_EMPTY_RAPID_SYMBOL_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyArgumentRAPIDTask(const std::string& task, uint16_t& result_code, std::string& message)
{
  if(task.empty())
  {
    message = abb_robot_msgs::ServiceResponses::EMPTY_RAPID_TASK_NAME;
    result_code = abb_robot_msgs::ServiceResponses::RC_EMPTY_RAPID_TASK_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyArgumentSignal(const std::string& signal, uint16_t& result_code, std::string& message)
{
  if(signal.empty())
  {
    message = abb_robot_msgs::ServiceResponses::EMPTY_SIGNAL_NAME;
    result_code = abb_robot_msgs::ServiceResponses::RC_EMPTY_SIGNAL_NAME;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyMotorsOff(uint16_t& result_code, std::string& message)
{
  if(system_state_.motors_on)
  {
    message = abb_robot_msgs::ServiceResponses::MOTORS_ARE_ON;
    result_code = abb_robot_msgs::ServiceResponses::RC_MOTORS_ARE_ON;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyMotorsOn(uint16_t& result_code, std::string& message)
{
  if(!system_state_.motors_on)
  {
    message = abb_robot_msgs::ServiceResponses::MOTORS_ARE_OFF;
    result_code = abb_robot_msgs::ServiceResponses::RC_MOTORS_ARE_OFF;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyRAPIDRunning(uint16_t& result_code, std::string& message)
{
  if(!system_state_.rapid_running)
  {
    message = abb_robot_msgs::ServiceResponses::RAPID_NOT_RUNNING;
    result_code = abb_robot_msgs::ServiceResponses::RC_RAPID_NOT_RUNNING;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyRAPIDStopped(uint16_t& result_code, std::string& message)
{
  if(system_state_.rapid_running)
  {
    message = abb_robot_msgs::ServiceResponses::RAPID_NOT_STOPPED;
    result_code = abb_robot_msgs::ServiceResponses::RC_RAPID_NOT_STOPPED;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifySMAddInRuntimeStates(uint16_t& result_code, std::string& message)
{
  if(sm_addin_runtime_state_.state_machines.empty())
  {
    message = abb_robot_msgs::ServiceResponses::SM_RUNTIME_STATES_MISSING;
    result_code = abb_robot_msgs::ServiceResponses::RC_SM_RUNTIME_STATES_MISSING;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifySMAddInTaskExist(const std::string& task, uint16_t& result_code, std::string& message)
{
  auto it{std::find_if(sm_addin_runtime_state_.state_machines.begin(),
                       sm_addin_runtime_state_.state_machines.end(),
                       [&](const auto& sm){return sm.rapid_task == task;})};

  if(it == sm_addin_runtime_state_.state_machines.end())
  {
    message = abb_robot_msgs::ServiceResponses::SM_UNKNOWN_RAPID_TASK;
    result_code = abb_robot_msgs::ServiceResponses::RC_SM_UNKNOWN_RAPID_TASK;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifySMAddInTaskInitialized(const std::string& task, uint16_t& result_code, std::string& message)
{
  if(!verifySMAddInTaskExist(task, result_code, message)) return false;

  auto it{std::find_if(sm_addin_runtime_state_.state_machines.begin(),
                       sm_addin_runtime_state_.state_machines.end(),
                       [&](const auto& sm){return sm.rapid_task == task;})};

  if(it->sm_state == abb_rapid_sm_addin_msgs::StateMachineState::SM_STATE_UNKNOWN ||
     it->sm_state == abb_rapid_sm_addin_msgs::StateMachineState::SM_STATE_INITIALIZE)
  {
    message = abb_robot_msgs::ServiceResponses::SM_UNINITIALIZED;
    result_code = abb_robot_msgs::ServiceResponses::RC_SM_UNINITIALIZED;
    return false;
  }

  return true;
}

bool RWSServiceProvider::verifyRWSManagerReady(uint16_t& result_code, std::string& message)
{
  if(!rws_manager_.isInterfaceReady())
  {
    message = abb_robot_msgs::ServiceResponses::SERVER_IS_BUSY;
    result_code = abb_robot_msgs::ServiceResponses::RC_SERVER_IS_BUSY;
    return false;
  }

  return true;
}

}
}
