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

#include <abb_robot_cpp_utilities/mapping.h>

#include <abb_robot_msgs/ServiceResponses.h>

#include "abb_rws_service_provider/rws_service_provider.h"

namespace
{
/**
 * \brief Name for ROS logging in the 'services' context.
 */
constexpr char ROS_LOG_SERVICES[]{"services"};
}

namespace abb
{
namespace robot
{

using RAPIDSymbols = rws::RWSStateMachineInterface::ResourceIdentifiers::RAPID::Symbols;

/***********************************************************************************************************************
 * Class definitions: RWSServiceProvider
 */

/***********************************************************
 * Auxiliary methods (StateMachine Add-In RWS services)
 */

bool RWSServiceProvider::getEGMSettings(GetEGMSettings::Request& request, GetEGMSettings::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDTask(request.task, response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifySMAddInTaskExist(request.task, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    rws::RWSStateMachineInterface::EGMSettings settings{};

    // Get the EGM RAPID settings.
    if(interface.services().egm().getSettings(request.task, &settings))
    {
      response.settings = utilities::map(settings);
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::setEGMSettings(SetEGMSettings::Request& request, SetEGMSettings::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDTask(request.task, response.result_code, response.message)) return true;
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifySMAddInTaskExist(request.task, response.result_code, response.message)) return true;
  if(!verifySMAddInTaskInitialized(request.task, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    rws::RWSStateMachineInterface::EGMSettings settings = utilities::map(request.settings);

    // Set the EGM RAPID settings.
    if(interface.services().egm().setSettings(request.task, settings))
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::setRAPIDRoutine(SetRAPIDRoutine::Request& request, SetRAPIDRoutine::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDTask(request.task, response.result_code, response.message)) return true;
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifySMAddInTaskExist(request.task, response.result_code, response.message)) return true;
  if(!verifySMAddInTaskInitialized(request.task, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the SmartGripper RAPID variables.
    if(interface.services().rapid().setRoutineName(request.task, request.routine))
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::setSGCommand(SetSGCommand::Request& request, SetSGCommand::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDTask(request.task, response.result_code, response.message)) return true;
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifySMAddInTaskExist(request.task, response.result_code, response.message)) return true;
  if(!verifySMAddInTaskInitialized(request.task, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Map argument
  //--------------------------
  unsigned int request_command{0};
  try
  {
    request_command = utilities::mapStateMachineSGCommand(request.command);
  }
  catch(const std::runtime_error& exception)
  {
    response.message = exception.what();
    response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
    return true;
  }

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    rws::RAPIDNum sg_command_input{static_cast<float>(request_command)};
    rws::RAPIDNum sg_target_position_input{request.target_position};

    // Set the SmartGripper RAPID variables.
    if(interface.setRAPIDSymbolData(request.task, RAPIDSymbols::SG_COMMAND_INPUT, sg_command_input) &&
       interface.setRAPIDSymbolData(request.task, RAPIDSymbols::SG_TARGET_POSTION_INPUT, sg_target_position_input))
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::runRAPIDRoutine(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRAPIDRunning(response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Signal start of EGM joint motions.
    if(interface.services().rapid().signalRunRAPIDRoutine())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::runSGRoutine(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRAPIDRunning(response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Signal start of EGM joint motions.
    if(interface.services().sg().signalRunSGRoutine())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::startEGMJoint(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRAPIDRunning(response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Signal start of EGM joint motions.
    if(interface.services().egm().signalEGMStartJoint())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::startEGMPose(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRAPIDRunning(response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Signal start of EGM pose motions.
    if(interface.services().egm().signalEGMStartPose())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::startEGMStream(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRAPIDRunning(response.result_code, response.message)) return true;
  if(!verifySMAddInRuntimeStates(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Signal start of EGM position streaming.
    if(interface.services().egm().signalEGMStartStream())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::stopEGM(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRAPIDRunning(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run priority service
  //--------------------------
  rws_manager_.runPriorityService([&](rws::RWSStateMachineInterface& interface)
  {
    // Signal stop of EGM motions.
    if(interface.services().egm().signalEGMStop())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::stopEGMStream(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRAPIDRunning(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Signal stop of EGM position streaming.
    if(interface.services().egm().signalEGMStopStream())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

}
}
