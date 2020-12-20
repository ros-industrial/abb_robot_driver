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
#include <abb_robot_cpp_utilities/parameters.h>

#include "abb_egm_hardware_interface/egm_controller_stopper.h"

namespace
{
/**
 * \brief Name for ROS logging in the 'init' context.
 */
constexpr char ROS_LOG_INIT[]{"init"};

/**
 * \brief Name for ROS logging in the 'runtime' context.
 */
constexpr char ROS_LOG_RUNTIME[]{"runtime"};

/**
 * \brief Timeout [s] for waiting on ROS services.
 */
constexpr double ROS_SERVICE_TIMEOUT{30.0};
}

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Struct definitions: EGMControllerStopper::ParameterHandler
 */

EGMControllerStopper::ParameterHandler::ParameterHandler(ros::NodeHandle& nh)
{
  ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Get ROS parameters (namespace '" << nh.getNamespace() << "')");

  // Indicator for allowing no connection timeout during initialization.
  utilities::getParameter(nh, "no_service_timeout", no_service_timeout, false);

  // List of controllers that are allowed to keep running if EGM becomes inactive.
  utilities::getParameter(nh, "ros_control/controllers/ok_to_keep_running", controllers_ok_to_keep_running, {""});
}

/***********************************************************************************************************************
 * Class definitions: EGMControllerStopper
 */

/***********************************************************
 * Primary methods
 */

EGMControllerStopper::EGMControllerStopper(ros::NodeHandle& nh_params,
                                           ros::NodeHandle& nh_msgs,
                                           ros::NodeHandle& nh_srvs)
:
parameters_{nh_params},
egm_states_subscriber_{nh_msgs.subscribe("egm_states", 1, &EGMControllerStopper::egmStatesCallback, this)},
list_controllers_client_{nh_srvs.serviceClient<controller_manager_msgs::ListControllers>("list_controllers")},
switch_controllers_client_{nh_srvs.serviceClient<controller_manager_msgs::SwitchController>("switch_controller")},
egm_previously_active_{false}
{
  ROS_INFO_NAMED(ROS_LOG_INIT, "Initializing...");

  // Prepare service request for switching ros_control controllers.
  switch_controllers_.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

  //--------------------------------------------------------
  // Wait for the ros_control controller manager services
  //--------------------------------------------------------
  if(parameters_.no_service_timeout)
  {
    list_controllers_client_.waitForExistence();
    switch_controllers_client_.waitForExistence();
  }
  else
  {
    if(!list_controllers_client_.waitForExistence(ros::Duration(ROS_SERVICE_TIMEOUT)))
    {
      throw std::runtime_error{"Timed out while waiting for service '" + list_controllers_client_.getService() + "'"};
    }

    if(!switch_controllers_client_.waitForExistence(ros::Duration(ROS_SERVICE_TIMEOUT)))
    {
      throw std::runtime_error{"Timed out while waiting for service '" + switch_controllers_client_.getService() + "'"};
    }
  }

  ROS_INFO_NAMED(ROS_LOG_INIT, "Initialization succeeded, and the node is ready for use");
}

/***********************************************************
 * Auxiliary methods
 */

void EGMControllerStopper::egmStatesCallback(const abb_egm_msgs::EGMState::ConstPtr& message)
{
  bool egm_currently_active{false};

  // Check if any EGM channel is active.
  for(const auto& egm_channel : message->egm_channels)
  {
    if(egm_channel.active) egm_currently_active = true;
  }

  // Stop controllers that are not allowed to keep running if EGM has become inactive.
  if(egm_previously_active_ && !egm_currently_active)
  {
    checkAndStopControllers();
  }

  egm_previously_active_ = egm_currently_active;
}

void EGMControllerStopper::checkAndStopControllers()
{
  // Reset switch service request.
  switch_controllers_.request.start_controllers.clear();
  switch_controllers_.request.stop_controllers.clear();

  // Get the currenlty loaded ros_control controllers.
  if(!list_controllers_client_.call(list_controllers_))
  {
    ROS_ERROR_NAMED(ROS_LOG_RUNTIME, "Failed to call service (to get list of currently loaded ros_control controllers)");
    return;
  }

  // Check for running controllers, which are not allowed to keep running.
  for(const auto& controller : list_controllers_.response.controller)
  {
    if(controller.state == "running")
    {
      auto it = std::find(parameters_.controllers_ok_to_keep_running.begin(),
                          parameters_.controllers_ok_to_keep_running.end(),
                          controller.name);

      if(it == parameters_.controllers_ok_to_keep_running.end())
      {
        switch_controllers_.request.stop_controllers.push_back(controller.name);
      }
    }
  }

  // Request controllers to be stopped.
  if(!switch_controllers_.request.stop_controllers.empty())
  {
    ROS_WARN_STREAM_NAMED(ROS_LOG_RUNTIME, "EGM became inactive, stopping controllers not allowed to keep running: " <<
                                            utilities::mapVectorToString(switch_controllers_.request.stop_controllers));

    if(!switch_controllers_client_.call(switch_controllers_))
    {
      ROS_ERROR_NAMED(ROS_LOG_RUNTIME, "Failed to call service (to stop ros_control controllers)");
      return;
    }

    if(!switch_controllers_.response.ok)
    {
      ROS_ERROR_NAMED(ROS_LOG_RUNTIME, "Failed to stop ros_control controllers");
      return;
    }
  }
}

}
}
