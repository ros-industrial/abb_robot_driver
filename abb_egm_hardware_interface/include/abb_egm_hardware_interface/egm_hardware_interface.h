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

#ifndef ABB_EGM_HARDWARE_INTERFACE_EGM_HARDWARE_INTERFACE_H
#define ABB_EGM_HARDWARE_INTERFACE_EGM_HARDWARE_INTERFACE_H

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits_interface.h>

#include <abb_egm_rws_managers/egm_manager.h>

#include "egm_state_interface.h"

namespace abb
{
namespace robot
{

/**
 * \brief Hardware interface towards ABB robot controllers supporting the Externally Guided Motion (EGM) interface.
 */
class EGMHardwareInterface : public hardware_interface::RobotHW
{
public:
  /**
   * \brief Creates a hardware interface for interacting with an ABB robot controller via EGM.
   */
  EGMHardwareInterface();

  /**
   * \brief Initializes the hardware interface.
   *
   * \param root_nh node handle (in the namespace where ROS interfaces should operate in during runtime).
   * \param robot_hw_nh node handle (in the namespace where ROS parameters should be read from during initialization).
   *
   * \return bool true if the initialization succeeded.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);

  /**
   * \brief Prepares (in non-realtime) to start and stop (ros_control) controllers.
   *
   * \param start_list specifying controllers to start.
   * \param stop_list specifying controllers to stop.
   *
   * \return bool true if is ok to start and stop the controllers.
   */
  bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                     const std::list<hardware_interface::ControllerInfo>& stop_list);

  /**
   * \brief Waits for a message from the hardware.
   *
   * \return ros::Duration it took when waiting for the message.
   */
  ros::Duration waitForMessage();

  /**
   * \brief Reads a message from the hardware.
   *
   * \param time specifying the current time.
   * \param period since the last call.
   *
   * \throw std::runtime_error if the hardware interface is uninitialized.
   */
  void read(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Writes a command to the hardware.
   *
   * \param time specifying the current time.
   * \param period since the last call.
   *
   * \throw std::runtime_error if the hardware interface is uninitialized.
   */
  void write(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Checks if a reset of (ros_control) controllers is needed.
   *
   * This method assumes that the reset will be performed and the task is passed to the calling code.
   *
   * \return bool true if a reset is needed.
   */
  bool resetNeeded();

private:
  /**
   * \brief Parameters for a single EGM channel.
   */
  struct EGMChannelParameters
  {
    /**
     * \brief Name of the channel.
     */
    std::string name;

    /**
     * \brief Port number used by the channel.
     */
    unsigned short port_number;

    /**
     * \brief Mechanical unit group that is expected to use the channel.
     */
    std::string mech_unit_group;
  };

  /**
   * \brief Initializes the robot contoller description by calling a ROS service to get the description in text format.
   *
   * \param nh for the node handle to use when calling the ROS service.
   *
   * \return bool true if the initialization succeeded.
   */
  bool initializeRobotControllerDescription(ros::NodeHandle& nh);

  /**
   * \brief Initializes the EGM channel parameters by collecting them from the ROS parameter server.
   *
   * \param nh for the node handle to use when reading the ROS parameters.
   *
   * \return bool true if the initialization succeeded.
   */
  bool initializeEGMChannelParameters(ros::NodeHandle& nh);

  /**
   * \brief Initializes the EGM manager.
   *
   * \return bool true if the initialization succeeded.
   */
  bool initializeEGMManager();

  /**
   * \brief Initializes components for 'ros_control' interaction.
   *
   * \param nh specifying a node handle to use.
   */
  void initializeROSControlLayer(ros::NodeHandle& nh);

  /**
   * \brief Manager for handling EGM communication with the robot controller.
   */
  std::unique_ptr<EGMManager> p_egm_manager_;

  /**
   * \brief Interface for handling the state data concerning the EGM channels.
   */
  EGMStateInterface egm_state_interface_;

  /**
   * \brief Interface for handling all joint state resources.
   */
  hardware_interface::JointStateInterface joint_state_interface_;

  /**
   * \brief Interface for handling all (controllable) joint position resources.
   */
  hardware_interface::PositionJointInterface joint_position_interface_;

  /**
   * \brief Interface for handling all (controllable) joint velocity resources.
   */
  hardware_interface::VelocityJointInterface joint_velocity_interface_;

  /**
   * \brief Interface for handling all (controllable) joint position and velocity resources.
   */
  hardware_interface::PosVelJointInterface joint_position_velocity_interface_;

  /**
   * \brief Interface for applying soft limits to (controllale) joint position resources.
   */
  joint_limits_interface::PositionJointSoftLimitsInterface joint_position_soft_limits_interface_;

  /**
   * \brief Interface for applying soft limits to (controllale) joint velocity resources.
   */
  joint_limits_interface::VelocityJointSoftLimitsInterface joint_velocity_soft_limits_interface_;

  /**
   * \brief Description of the connected robot controller.
   */
  RobotControllerDescription robot_controller_description_;

  /**
   * \brief EGM channel parameters collected from the ROS parameter server.
   */
  std::vector<EGMChannelParameters> egm_channel_parameters_;

  /**
   * \brief Indicator for if EGM is active.
   */
  std::atomic_bool egm_active_;

  /**
   * \brief Indicator for if an EGM client is in running state (i.e. listening for motion references).
   */
  std::atomic_bool egm_client_running_;

  /**
   * \brief Indicator for if a reset of (ros_control) controllers is needed.
   */
  std::atomic_bool reset_of_ros_control_controllers_needed_;

  /**
   * \brief Motion data for each mechanical unit defined in the connected robot controller.
   *
   * For example, joint states and commands.
   */
  MotionData motion_data_;

  /**
   * \brief Indicator for if a log message should be printed when feedback has been received.
   */
  bool print_feedback_received_;

  /**
   * \brief Indicator for if any ros_control controller has been started.
   */
  bool any_controller_started_;

  /**
   * \brief List of user specified ros_control controllers that are always ok to start.
   *
   * For example, when the EGM session is inactive, or the EGM client is not in running state.
   */
  std::vector<std::string> controllers_always_ok_to_start_;
};

}
}

#endif
