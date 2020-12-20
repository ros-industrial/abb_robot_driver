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

#ifndef ABB_RWS_STATE_PUBLISHER_RWS_STATE_PUBLISHER_H
#define ABB_RWS_STATE_PUBLISHER_RWS_STATE_PUBLISHER_H

#include <ros/ros.h>

#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_egm_rws_managers/system_data_parser.h>

namespace abb
{
namespace robot
{

/**
 * \brief State publisher that collects the current system states from an ABB robot controller via RWS.
 */
class RWSStatePublisher
{
public:
  /**
   * \brief Creates a state publisher.
   *
   * \param nh_params node handle (in the namespace where ROS parameters should be gathered from).
   * \param nh_msgs node handle (in the namespace where ROS messages should be published in).
   *
   * \throw std::runtime_error if the creation failed (e.g. any initialization step fails).
   */
  RWSStatePublisher(ros::NodeHandle& nh_params, ros::NodeHandle& nh_msgs);

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
     *
     * \throw std::runtime_error if the creation failed (e.g. missing any mandatory ROS parameter).
     */
    ParameterHandler(ros::NodeHandle& nh);

    /**
     * \brief IP address for the robot controller's RWS server.
     */
    std::string robot_ip;

    /**
     * \brief Port number for the robot controller's RWS server.
     */
    int robot_port;

    /**
     * \brief Arbitrary user nickname/identifier for the robot controller.
     */
    std::string robot_nickname;

    /**
     * \brief Indicator for whether the node is allowed to wait indefinitely for the robot controller.
     *
     * Note: This only applies during the node's initialization phase.
     */
    bool no_connection_timeout;

    /**
     * \brief Polling rate [Hz] for collecting system states and publishing of ROS messages.
     */
    double polling_rate;
  };

  /**
   * \brief Timer callback for polling and publishing of system states.
   */
  void pollingTimerCallback(const ros::TimerEvent&);

  /**
   * \brief Handler for gathering required ROS parameters.
   */
  ParameterHandler parameters_;

  /**
   * \brief Manager for handling RWS communication with the robot controller.
   */
  RWSManager rws_manager_;

  /**
   * \brief Description of the connected robot controller.
   */
  RobotControllerDescription robot_controller_description_;

  /**
   * \brief Data about the robot controller's system state.
   */
  SystemStateData system_state_data_;

  /**
   * \brief Motion data for each mechanical unit defined in the robot controller.
   */
  MotionData motion_data_;

  /**
   * \brief Polling rate for collecting and publishing system states.
   */
  ros::Rate polling_rate_;

  /**
   * \brief Timer for polling and publishing of system states.
   */
  ros::Timer polling_timer_;

  /**
   * \brief Publisher for joint states.
   */
  ros::Publisher joint_state_publisher_;

  /**
   * \brief Publisher for general system states.
   */
  ros::Publisher system_state_publisher_;

  /**
   * \brief Publisher for RobotWare StateMachine Add-In runtime states.
   *
   * Note: Only used if the Add-In is present in the system.
   */
  ros::Publisher sm_runtime_state_publisher_;
};

}
}

#endif
