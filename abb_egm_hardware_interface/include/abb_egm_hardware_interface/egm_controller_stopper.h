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

#ifndef ABB_EGM_HARDWARE_INTERFACE_EGM_CONTROLLER_STOPPER_H
#define ABB_EGM_HARDWARE_INTERFACE_EGM_CONTROLLER_STOPPER_H

#include <string>
#include <vector>

#include <ros/ros.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>

#include <abb_egm_msgs/EGMState.h>

namespace abb
{
namespace robot
{

/**
 * \brief Utility class for observing EGM channel states and, if needed, stopping ros_control controllers.
 *
 * Note: A user provided list specifies which controllers that are allowed to keep running if EGM becomes inactive.
 */
class EGMControllerStopper
{
public:
  /**
   * \brief Creates a stopper of ros_control controllers.
   *
   * \param nh_params node handle (in the namespace where ROS parameters should be gathered from).
   * \param nh_msgs node handle (in the namespace where ROS messages should be subscribed to).
   * \param nh_srvs node handle (in the namespace where ROS services should be called in).
   *
   * \throw std::runtime_error if the creation failed (e.g. any initialization step fails).
   */
  EGMControllerStopper(ros::NodeHandle& nh_params, ros::NodeHandle& nh_msgs, ros::NodeHandle& nh_srvs);

private:
  /**
   * \brief Parameter handler for gathering required ROS parameters.
   */
  struct ParameterHandler
  {
    /**
     * \brief Creates a parameter handler.
     *
     * \param nh node handle in the namespace where ROS parameters should be gathered from.
     */
    ParameterHandler(ros::NodeHandle& nh);

    /**
     * \brief Indicator for whether the node is allowed to wait indefinitely for ROS services.
     *
     * Note: This only applies during the node's initialization phase.
     */
    bool no_service_timeout;

    /**
     * \brief List of user specifed ros_control controllers which are allowed to keep running if EGM becomes inactive.
     */
    std::vector<std::string> controllers_ok_to_keep_running;
  };

  /**
   * \brief Callback for processing EGM channel states.
   *
   * \param message to process.
   */
  void egmStatesCallback(const abb_egm_msgs::EGMState::ConstPtr& message);

  /**
   * \brief Checks and stops ros_control controllers that are not allowed to keep running.
   */
  void checkAndStopControllers();

  /**
   * \brief Handler for gathering required ROS parameters.
   */
  ParameterHandler parameters_;

  /**
   * \brief Subscriber for EGM channel states.
   */
  ros::Subscriber egm_states_subscriber_;

  /**
   * \brief Service client for requesting a list of currently loaded ros_control controllers.
   */
  ros::ServiceClient list_controllers_client_;

  /**
   * \brief Service client for requesting a switch (i.a. stopping) of ros_control controllers.
   */
  ros::ServiceClient switch_controllers_client_;

  /**
   * \brief The previously received EGM activity state.
   */
  bool egm_previously_active_;

  /**
   * \brief Service request/response for list of currently loaded ros_control controllers.
   */
  controller_manager_msgs::ListControllers list_controllers_;

  /**
   * \brief Service request/response for switching (i.a. stopping) ros_control controllers.
   */
  controller_manager_msgs::SwitchController switch_controllers_;
};

}
}

#endif
