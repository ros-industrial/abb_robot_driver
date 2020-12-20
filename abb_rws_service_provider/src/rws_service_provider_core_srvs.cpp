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

/***********************************************************************************************************************
 * Class definitions: RWSServiceProvider
 */

/***********************************************************
 * Auxiliary methods (Core RWS services)
 */

bool RWSServiceProvider::getFileContents(GetFileContents::Request& request, GetFileContents::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentFilename(request.filename, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Retrieve file contents from the robot controller's home directory.
    if(interface.getFile(rws::RWSClient::FileResource(request.filename), &response.contents))
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

bool RWSServiceProvider::getIOSignal(GetIOSignal::Request& request, GetIOSignal::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentSignal(request.signal, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Get the IO-signal.
    response.value = interface.getIOSignal(request.signal);

    if(!response.value.empty())
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

bool RWSServiceProvider::getRAPIDBool(GetRAPIDBool::Request& request, GetRAPIDBool::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Get the RAPID symbol.
    rws::RAPIDBool rapid_bool{};
    if(interface.getRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, &rapid_bool))
    {
      response.value = rapid_bool.value;
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

bool RWSServiceProvider::getRAPIDDnum(GetRAPIDDnum::Request& request, GetRAPIDDnum::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Get the RAPID symbol.
    rws::RAPIDDnum rapid_dnum{};
    if(interface.getRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, &rapid_dnum))
    {
      response.value = rapid_dnum.value;
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

bool RWSServiceProvider::getRAPIDNum(GetRAPIDNum::Request& request, GetRAPIDNum::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Get the RAPID symbol.
    rws::RAPIDNum rapid_num{};
    if(interface.getRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, &rapid_num))
    {
      response.value = rapid_num.value;
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

bool RWSServiceProvider::getRAPIDString(GetRAPIDString::Request& request, GetRAPIDString::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Get the RAPID symbol.
    rws::RAPIDString rapid_string{};
    if(interface.getRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, &rapid_string))
    {
      response.value = rapid_string.value;
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

bool RWSServiceProvider::getRAPIDSymbol(GetRAPIDSymbol::Request& request, GetRAPIDSymbol::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Get the RAPID symbol.
    response.value = interface.getRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol);

    if(!response.value.empty())
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

bool RWSServiceProvider::getSpeedRatio(GetSpeedRatio::Request&, GetSpeedRatio::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Get the speed ratio.
    try
    {
      response.speed_ratio = interface.getSpeedRatio();
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    catch(const std::runtime_error& exception)
    {
      response.message = exception.what();
      response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
    }
  });

  return true;
}

bool RWSServiceProvider::ppToMain(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRAPIDStopped(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Reset the RAPID program pointer.
    if(interface.resetRAPIDProgramPointer())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::setFileContents(SetFileContents::Request& request, SetFileContents::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentFilename(request.filename, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Upload file contents to the robot controller's home directory.
    if(interface.uploadFile(rws::RWSClient::FileResource(request.filename), request.contents))
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

bool RWSServiceProvider::setIOSignal(SetIOSignal::Request& request, SetIOSignal::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentSignal(request.signal, response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the IO-signal.
    if(interface.setIOSignal(request.signal, request.value))
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

bool RWSServiceProvider::setMotorsOff(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyMotorsOn(response.result_code, response.message)) return true;

  //--------------------------
  // Run priority service
  //--------------------------
  rws_manager_.runPriorityService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the motors off.
    if(interface.setMotorsOff())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::setMotorsOn(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyMotorsOff(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the motors on.
    if(interface.setMotorsOn())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::setRAPIDBool(SetRAPIDBool::Request& request, SetRAPIDBool::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the RAPID symbol.
    rws::RAPIDBool rapid_bool{static_cast<bool>(request.value)};
    if(interface.setRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, rapid_bool))
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

bool RWSServiceProvider::setRAPIDDnum(SetRAPIDDnum::Request& request, SetRAPIDDnum::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the RAPID symbol.
    rws::RAPIDDnum rapid_dnum{request.value};
    if(interface.setRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, rapid_dnum))
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

bool RWSServiceProvider::setRAPIDNum(SetRAPIDNum::Request& request, SetRAPIDNum::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the RAPID symbol.
    rws::RAPIDNum rapid_num{request.value};
    if(interface.setRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, rapid_num))
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

bool RWSServiceProvider::setRAPIDString(SetRAPIDString::Request& request, SetRAPIDString::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the RAPID symbol.
    rws::RAPIDString rapid_string{request.value};
    if(interface.setRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, rapid_string))
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

bool RWSServiceProvider::setRAPIDSymbol(SetRAPIDSymbol::Request& request, SetRAPIDSymbol::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyArgumentRAPIDSymbolPath(request.path, response.result_code, response.message)) return true;
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Set the RAPID symbol.
    if(interface.setRAPIDSymbolData(request.path.task, request.path.module, request.path.symbol, request.value))
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

bool RWSServiceProvider::setSpeedRatio(SetSpeedRatio::Request& request, SetSpeedRatio::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    try
    {
      // Set the speed ratio.
      if(interface.setSpeedRatio(request.speed_ratio))
      {
        response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
      }
      else
      {
        response.message = abb_robot_msgs::ServiceResponses::FAILED;
        response.result_code = abb_robot_msgs::ServiceResponses::RC_FAILED;
        ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
      }
    }
    catch(const std::exception& exception)
    {
      response.message = exception.what();
    }
  });

  return true;
}

bool RWSServiceProvider::startRAPID(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyAutoMode(response.result_code, response.message)) return true;
  if(!verifyMotorsOn(response.result_code, response.message)) return true;
  if(!verifyRWSManagerReady(response.result_code, response.message)) return true;

  //--------------------------
  // Run service
  //--------------------------
  rws_manager_.runService([&](rws::RWSStateMachineInterface& interface)
  {
    // Start RAPID execution.
    if(interface.startRAPIDExecution())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

bool RWSServiceProvider::stopRAPID(TriggerWithResultCode::Request&, TriggerWithResultCode::Response& response)
{
  //--------------------------
  // Verification
  //--------------------------
  if(!verifyRAPIDRunning(response.result_code, response.message)) return true;

  //--------------------------
  // Run priority service
  //--------------------------
  rws_manager_.runPriorityService([&](rws::RWSStateMachineInterface& interface)
  {
    // Stop RAPID execution.
    if(interface.stopRAPIDExecution())
    {
      response.result_code = abb_robot_msgs::ServiceResponses::RC_SUCCESS;
    }
    else
    {
      response.message = abb_robot_msgs::ServiceResponses::FAILED;
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_SERVICES, interface.getLogTextLatestEvent());
    }
  });

  return true;
}

}
}
