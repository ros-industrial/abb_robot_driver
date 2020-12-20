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

#include <algorithm>
#include <sstream>

#include <google/protobuf/text_format.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <abb_robot_cpp_utilities/parameters.h>
#include <abb_robot_cpp_utilities/verification.h>

#include "abb_egm_hardware_interface/egm_hardware_interface.h"

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
 * \brief Time [s] for throttled ROS logging.
 */
constexpr double THROTTLE_TIME{10.0};

/**
 * \brief Timeout [ms] to wait for new messages.
 */
constexpr unsigned int MESSAGE_TIMEOUT_MS{50};
}

namespace abb
{
namespace robot
{

/***********************************************************************************************************************
 * Class definitions: EGMHardwareInterface
 */

/***********************************************************
 * Primary methods
 */

EGMHardwareInterface::EGMHardwareInterface()
:
egm_active_{false},
reset_of_ros_control_controllers_needed_{false},
print_feedback_received_{false},
any_controller_started_{false}
{}

bool EGMHardwareInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  (void) root_nh;

  ROS_INFO_NAMED(ROS_LOG_INIT, "Initializing...");

  //--------------------------------------------------------
  // Initialize robot controller description and motion data
  //--------------------------------------------------------
  if(!initializeRobotControllerDescription(robot_hw_nh))
  {
    ROS_FATAL_NAMED(ROS_LOG_INIT, "Failed to initialize robot controller description");
    return false;
  }

  try
  {
    initializeMotionData(motion_data_, robot_controller_description_);
  }
  catch(...)
  {
    ROS_FATAL_NAMED(ROS_LOG_INIT, "Failed to initialize motion data from robot controller description");
    return false;
  }

  //--------------------------------------------------------
  // Initialize EGM channel parameters
  //--------------------------------------------------------
  if(!initializeEGMChannelParameters(robot_hw_nh))
  {
    ROS_FATAL_NAMED(ROS_LOG_INIT, "Failed to initialize EGM channel parameters");
    return false;
  }

  //--------------------------------------------------------
  // Initialize EGM manager for handling EGM communication
  //--------------------------------------------------------
  if(!initializeEGMManager())
  {
    ROS_FATAL_NAMED(ROS_LOG_INIT, "Failed to initialize EGM manager");
    return false;
  }

  //--------------------------------------------------------
  // Initialize 'ros_control' abstraction layer
  //--------------------------------------------------------
  initializeROSControlLayer(robot_hw_nh);

  ROS_INFO_NAMED(ROS_LOG_INIT, "Initialization succeeded, and the node is ready for use");

  return true;
}

ros::Duration EGMHardwareInterface::waitForMessage()
{
  ros::Time wait_start{ros::Time::now()};
  if(p_egm_manager_)
  {
    p_egm_manager_->waitForMessage(MESSAGE_TIMEOUT_MS);
  }
  return ros::Time::now() - wait_start;
}

void EGMHardwareInterface::read(const ros::Time& time, const ros::Duration& period)
{
  (void) time;
  (void) period;

  if(!p_egm_manager_) throw std::runtime_error{"The EGM manager has not been initialized"};

  //--------------------------------------------------------
  // Read feedback from the EGM channels
  //--------------------------------------------------------
  bool read_ok{p_egm_manager_->read(motion_data_)};

  // Print log messages.
  if(any_controller_started_ && !read_ok)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_TIME, ROS_LOG_RUNTIME,
                                   "Timed out while waiting for EGM feedback "
                                   "(did the EGM session end on the robot controller?)");
    print_feedback_received_ = true;
  }
  else if(any_controller_started_ && print_feedback_received_)
  {
    ROS_INFO_THROTTLE_NAMED(THROTTLE_TIME, ROS_LOG_RUNTIME, "Controller has resumed EGM feedback");
    print_feedback_received_ = false;
  }

  //--------------------------------------------------------
  // Check if commands need to be reset, and keep track
  // of activation status of the EGM channels
  //--------------------------------------------------------
  reset_of_ros_control_controllers_needed_ = false;

  bool any_egm_channel_active{false};
  bool any_egm_channel_activated_or_deactivated{false};
  bool any_egm_client_state_changed{false};
  bool any_egm_client_state_is_running{false};
  bool any_rapid_execution_status_changed{false};
  for(auto& group : motion_data_.groups)
  {
    if(group.egm_channel_data.is_active)
    {
      any_egm_channel_active = true;
    }

    if(group.egm_channel_data.was_activated_or_deactivated)
    {
      any_egm_channel_activated_or_deactivated = true;
    }

    if(group.egm_channel_data.egm_client_state_changed)
    {
      any_egm_client_state_changed = true;
    }

    if(group.egm_channel_data.input.status().egm_state() == egm::wrapper::Status::EGM_RUNNING)
    {
      any_egm_client_state_is_running = true;
    }

    if(group.egm_channel_data.rapid_execution_status_changed)
    {
      any_rapid_execution_status_changed = true;
    }

    if(group.egm_channel_data.input.status().utilization_rate() > 100.0)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(THROTTLE_TIME, ROS_LOG_RUNTIME,
                                     "Utilization rate for '" <<
                                     group.egm_channel_data.name <<"' (using port number '" <<
                                     group.egm_channel_data.port_number << "') is " <<
                                     group.egm_channel_data.input.status().utilization_rate() << "% " <<
                                     "(desired commands are too aggressive and should be reduced)");
    }

    for(auto& unit : group.units)
    {
      for(auto& joint : unit.joints)
      {
        joint.command.position = joint.state.position;
        joint.command.velocity = 0.0;
      }
    }
  }

  if(any_egm_channel_activated_or_deactivated ||
     any_egm_client_state_changed ||
     any_rapid_execution_status_changed)
  {
    reset_of_ros_control_controllers_needed_ = true;
    joint_position_soft_limits_interface_.reset();
  }

  if(reset_of_ros_control_controllers_needed_)
  {
    ROS_INFO_THROTTLE_NAMED(THROTTLE_TIME, ROS_LOG_RUNTIME,
                            "EGM session state changed: ros_control controllers will be reset");
  }

  egm_active_ = any_egm_channel_active;
  egm_client_running_ = any_egm_client_state_is_running;
}

void EGMHardwareInterface::write(const ros::Time& time, const ros::Duration& period)
{
  (void) time;

  if(!p_egm_manager_) throw std::runtime_error{"The EGM manager has not been initialized"};

  //--------------------------------------------------------
  // Enforce joint limits on the desired commands
  //--------------------------------------------------------
  joint_position_soft_limits_interface_.enforceLimits(period);
  joint_velocity_soft_limits_interface_.enforceLimits(period);

  //--------------------------------------------------------
  // Write commands to the EGM channels
  //--------------------------------------------------------
  p_egm_manager_->write(motion_data_);
}

/***********************************************************
 * Auxiliary methods (public)
 */

bool EGMHardwareInterface::prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                         const std::list<hardware_interface::ControllerInfo>& stop_list)
{
  (void) stop_list;

  // Retrive the latest known EGM statuses.
  bool egm_active{egm_active_};
  bool egm_client_running{egm_client_running_};

  // Verify that the start list doesn't specify any controllers that aren't allowed to start.
  //
  // I.e. while EGM is inactive, or EGM clients aren't in running state,
  // refuse to start controllers that aren't in the user specified list
  // of controllers that are always allowed to start.
  bool refuse_start_list{false};
  std::size_t counter{0};
  std::stringstream ss{};
  ss << "[";
  for(const auto& controller_info : start_list)
  {
    ss << controller_info.name << (++counter < start_list.size() ? ", " : "]");

    const auto it = std::find(controllers_always_ok_to_start_.begin(),
                              controllers_always_ok_to_start_.end(),
                              controller_info.name);

    if(it == controllers_always_ok_to_start_.end() && (!egm_active || !egm_client_running))
    {
      refuse_start_list = true;
    }
  }

  // Check if the start list was refused.
  if(refuse_start_list)
  {
    if(egm_active)
    {
      ROS_WARN_STREAM_NAMED(ROS_LOG_RUNTIME,
                            "EGM is active, but not in running mode (i.e. listening for command references)! "
                            "Refusing to start controller(s): " << ss.str());
    }
    else
    {
      ROS_WARN_STREAM_NAMED(ROS_LOG_RUNTIME, "EGM is inactive! Refusing to start controller(s): " << ss.str());
    }

    return false;
  }

  // Keep track of when any controller has been started.
  if(!start_list.empty())
  {
    any_controller_started_ = true;
  }

  return true;
}

bool EGMHardwareInterface::resetNeeded()
{
  return reset_of_ros_control_controllers_needed_;
}

/***********************************************************
 * Auxiliary methods (private)
 */

bool EGMHardwareInterface::initializeRobotControllerDescription(ros::NodeHandle& nh)
{
  std::string robot_controller_description{};
  utilities::getParameter(nh, "robot_controller_description", robot_controller_description);

  //--------------------------------------------------------
  // Parse the robot controller description and verify
  // RobotWare version and required options
  //--------------------------------------------------------
  if(!google::protobuf::TextFormat::ParseFromString(robot_controller_description, &robot_controller_description_))
  {
    ROS_FATAL_NAMED(ROS_LOG_INIT, "Failed to parse RWS robot controller description");
    return false;
  }

  ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Robot controller description:\n" << summaryText(robot_controller_description_));

  try
  {
    utilities::verifyRobotWareVersion(robot_controller_description_.header().robot_ware_version());
  }
  catch(...)
  {
    return false;
  }

  if(!robot_controller_description_.system_indicators().options().egm())
  {
    ROS_FATAL_NAMED(ROS_LOG_INIT, "Required RobotWare option EGM seems to be missing on the robot controller");
    return false;
  }

  return true;
}

bool EGMHardwareInterface::initializeEGMChannelParameters(ros::NodeHandle& nh)
{
  ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Get ROS parameters (namespace: '" << nh.getNamespace() << "/egm')");

  std::vector<int> port_numbers{};
  auto channel_number{1};
  auto ok{true};

  //--------------------------------------------------------
  // Search for (user specified) EGM channel parameters
  //--------------------------------------------------------
  while(ros::ok() && ok)
  {
    auto channel{"channel_" + std::to_string(channel_number++)};

    ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Check for '" << channel << "' parameter");

    ok = nh.hasParam("egm/" + channel);

    if(ok)
    {
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Found '" << channel << "' parameter");

      EGMChannelParameters parameters{};
      parameters.name = channel;

      // Mechanical unit group.
      utilities::getParameter(nh, "egm/" + channel + "/mech_unit_group", parameters.mech_unit_group, std::string{});

      // Port number.
      try
      {
        int port_number{-1};
        utilities::getParameter(nh, "egm/" + channel + "/port_number", port_number);
        utilities::verifyPortNumber(port_number);

        if(std::find(port_numbers.begin(), port_numbers.end(), port_number) == port_numbers.end())
        {
          ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Port number parameter '" << port_number << "' found");

          // Keep track of used port numbers.
          port_numbers.push_back(port_number);

          // Store the EGM channel's parameters.
          parameters.port_number = static_cast<unsigned short>(port_number);
          egm_channel_parameters_.push_back(parameters);
        }
        else
        {
          ROS_FATAL_STREAM_NAMED(ROS_LOG_INIT,
                                "EGM channel " << channel << " is configured to use a port (" << port_number << ") "
                                "which was already used by another channel. This is not supported. Please check "
                                "parameter 'egm/" << channel << "/port_number' to verify no other channels are using "
                                "this port.");
          return false;
        }
      }
      catch(const std::runtime_error& exception)
      {
        ROS_FATAL_STREAM_NAMED(ROS_LOG_INIT,
                              "Runtime error while parsing channel " << channel << ": '" << exception.what() << "'");
        return false;
      }
    }
  }

  //--------------------------------------------------------
  // Verify that parameters for at least one EGM channel
  // has been found
  //--------------------------------------------------------
  if(egm_channel_parameters_.empty())
  {
    ROS_FATAL_NAMED(ROS_LOG_INIT, "No valid EGM channel parameters found");
    return false;
  }

  return true;
}

bool EGMHardwareInterface::initializeEGMManager()
{
  ROS_DEBUG_NAMED(ROS_LOG_INIT, "Initializing EGM manager");

  std::vector<EGMManager::ChannelConfiguration> egm_channel_configurations{};

  const auto& options{robot_controller_description_.system_indicators().options()};

  //--------------------------------------------------------
  // Support lambdas
  //--------------------------------------------------------
  auto verifyParameters = [&](const EGMChannelParameters& parameters)
  {
    auto& groups{motion_data_.groups};
    const auto& mug_name{parameters.mech_unit_group};

    if(mug_name.empty() && options.multimove())
    {
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Skipping (no mechanical unit group specified)");
      return false;
    }

    auto it{groups.begin()};

    if(options.multimove())
    {
      // Find the motion group that corresponds to the specified mechanical unit group.
      it = std::find_if(groups.begin(), groups.end(), [&](const auto& x) { return x.name == mug_name; });
    }

    if(it != groups.end())
    {
      int count{0};

      // Count the number of mechanical units in the group that are supported by EGM.
      for(const auto& unit : it->units)
      {
        if(unit.supported_by_egm)
        {
          ++count;
        }
      }

      if(count == 0)
      {
        ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT,
                               "Skipping (mechanical unit group '" << mug_name << "' doesn't appear to "
                               "contain any EGM supported units)");
        return false;
      }
      else if(count > 1)
      {
        ROS_WARN_STREAM_NAMED(ROS_LOG_INIT,
                              "Mechanical unit group '" << mug_name << "' seems to contain more than one EGM "
                              "supported unit. Only one unit can be moved per EGM channel");
      }

      // Store the EGM parameter information as part of the group.
      it->egm_channel_data.name = parameters.name;
      it->egm_channel_data.port_number = parameters.port_number;

      return true;
    }
    else
    {
      ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Skipping (mechanical unit group '" << mug_name << "' not found)");
      return false;
    }
  };

  //--------------------------------------------------------
  // Verify ROS parameters
  //--------------------------------------------------------
  ROS_DEBUG_NAMED(ROS_LOG_INIT, "Verifying ROS parameters against the RWS robot controller description");

  if(egm_channel_parameters_.size() > 1 && !options.multimove())
  {
    ROS_WARN_STREAM_NAMED(ROS_LOG_INIT, "ROS parameters contain more than one EGM channel, but the MultiMove option "
                                        "seems to be missing in the robot controller system");
  }

  for(const auto& parameters : egm_channel_parameters_)
  {
    if(verifyParameters(parameters))
    {
      auto mug{findMechanicalUnitGroup(parameters.mech_unit_group, robot_controller_description_)};
      egm_channel_configurations.emplace_back(parameters.port_number, mug);
    }
  }

  if(egm_channel_configurations.empty())
  {
    ROS_FATAL_NAMED(ROS_LOG_INIT, "Failed to verify any ROS parameters against the RWS robot controller description");
    return false;
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(ROS_LOG_INIT, "Verified '" << egm_channel_configurations.size() << "' EGM channel "
                                         "configuration(s) against the RWS robot controller description");
  }

  //--------------------------------------------------------
  // Instantiate the EGM manager
  //--------------------------------------------------------
  p_egm_manager_ = std::make_unique<EGMManager>(egm_channel_configurations);

  return p_egm_manager_ != nullptr;
}

void EGMHardwareInterface::initializeROSControlLayer(ros::NodeHandle& nh)
{
  // Get the user specified list of ros_control controllers that are always ok to start.
  utilities::getParameter(nh, "ros_control/controllers/always_ok_to_start", controllers_always_ok_to_start_, {""});

  //--------------------------------------------------------
  // Motion interfaces
  //--------------------------------------------------------
  for(auto& group : motion_data_.groups)
  {
    // Skip mechanical unit groups without
    // valid EGM channel configurations.
    if(group.egm_channel_data.name.empty()) continue;

    //------------------------------------------------------
    // EGM state interface
    //------------------------------------------------------
    EGMStateHandle egm_state_handle{group.egm_channel_data.name, &group.egm_channel_data};
    egm_state_interface_.registerHandle(egm_state_handle);

    for(auto& unit : group.units)
    {
      //----------------------------------------------------
      // Joint interfaces
      //----------------------------------------------------
      for(auto& joint : unit.joints)
      {
        //--------------------------------------------------
        // State resource
        //--------------------------------------------------
        hardware_interface::JointStateHandle joint_state_handle{joint.name,
                                                                &joint.state.position,
                                                                &joint.state.velocity,
                                                                &joint.state.effort};
        joint_state_interface_.registerHandle(joint_state_handle);

        //--------------------------------------------------
        // Controllable resources
        //--------------------------------------------------
        hardware_interface::JointHandle joint_pos_handle{joint_state_handle, &joint.command.position};
        joint_position_interface_.registerHandle(joint_pos_handle);

        hardware_interface::JointHandle joint_vel_handle{joint_state_handle, &joint.command.velocity};
        joint_velocity_interface_.registerHandle(joint_vel_handle);

        hardware_interface::PosVelJointHandle joint_pos_vel_handle{joint_state_handle, &joint.command.position,
                                                                                       &joint.command.velocity};
        joint_position_velocity_interface_.registerHandle(joint_pos_vel_handle);

        //--------------------------------------------------
        // Limits
        //--------------------------------------------------
        joint_limits_interface::JointLimits limits{};
        joint_limits_interface::SoftJointLimits soft_limits{};

        // Retrieve user-specified limits.
        joint_limits_interface::getJointLimits(joint.name, nh, limits);

        // Make sure that there are joint position limits, and that they
        // are within the limits collected from the real system.
        if(limits.has_position_limits)
        {
          if(limits.min_position < joint.lower_limit)
          {
            ROS_WARN_STREAM_NAMED(ROS_LOG_INIT,
                                  "Overriding lower joint position limit [" << (joint.rotational ? "rad" : "m") <<
                                  "] for '" << joint.name << "': " << limits.min_position <<
                                  " (limit set from ROS) less than " << joint.lower_limit <<
                                  " (limit collected from robot controller)");
            limits.min_position = joint.lower_limit;
          }

          if(limits.max_position > joint.upper_limit)
          {
            ROS_WARN_STREAM_NAMED(ROS_LOG_INIT,
                                  "Overriding upper joint position limit [" << (joint.rotational ? "rad" : "m") <<
                                  "] for '" << joint.name << "': " << limits.max_position <<
                                  " (limit set from ROS) greater than " << joint.upper_limit <<
                                  " (limit collected from robot controller)");
            limits.max_position = joint.upper_limit;
          }
        }
        else
        {
          limits.has_position_limits = true;
          limits.min_position = joint.lower_limit;
          limits.max_position = joint.upper_limit;
        }

        // Set conservative velocity limits (if they are missing).
        if(!limits.has_velocity_limits)
        {
          limits.has_velocity_limits = true;
          limits.max_velocity = (joint.rotational ? 10.0*Constants::DEG_TO_RAD : 0.1);
          ROS_WARN_STREAM_NAMED(ROS_LOG_INIT,
                                "Configured default velocity limit for '" << joint.name <<
                                "': " << limits.max_velocity << (joint.rotational ? " rad/s" : " m/s"));
        }

        // Ignore any user specified acceleration limits.
        if(limits.has_acceleration_limits)
        {
          // Notes:
          // - It has been observed that specifying joint acceleration limits, on the ROS side, can lead
          //   to oscillatory motions when trying to enforce the limits.
          // - The underlying EGM communication library only provide velocity estimates, based on
          //   differentiating previous and latest position feedback, which may lead to noisy estimations
          //   and this is a likely cause for the issue when enforcing the limits.
          //   See: https://github.com/ros-industrial/abb_libegm/blob/master/src/egm_common_auxiliary.cpp#L236-L257
          limits.has_acceleration_limits = false;
          ROS_WARN_STREAM_NAMED(ROS_LOG_INIT,
                                "Ignoring acceleration limit for '" << joint.name <<
                                "': " << limits.max_acceleration << (joint.rotational ? " rad/s^2" : " m/s^2") <<
                                " (enforcing the limit on the ROS side may lead to oscillatory motions, due to only" <<
                                " estimated velocity feedback available)");
        }

        // Retrieve user-specified soft limits (and set default values if they are missing).
        if(!joint_limits_interface::getSoftJointLimits(joint.name, nh, soft_limits))
        {
          soft_limits.min_position = 0.99*joint.lower_limit;
          soft_limits.max_position = 0.99*joint.upper_limit;
          soft_limits.k_position = 1.0;
          ROS_WARN_STREAM_NAMED(ROS_LOG_INIT,
                                "Configured default soft limits for '" << joint.name <<
                                "': position [" << soft_limits.min_position << "; "
                                << soft_limits.max_position << "], k: "
                                << soft_limits.k_position);
        }

        joint_limits_interface::PositionJointSoftLimitsHandle pos_limits_handle(joint_pos_handle, limits, soft_limits);
        joint_limits_interface::VelocityJointSoftLimitsHandle vel_limits_handle(joint_vel_handle, limits, soft_limits);

        joint_position_soft_limits_interface_.registerHandle(pos_limits_handle);
        joint_velocity_soft_limits_interface_.registerHandle(vel_limits_handle);
      }
    }
  }

  //--------------------------------------------------------
  // Register the interfaces
  //--------------------------------------------------------
  registerInterface(&egm_state_interface_);
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_position_interface_);
  registerInterface(&joint_velocity_interface_);
  registerInterface(&joint_position_velocity_interface_);
}

}
}
