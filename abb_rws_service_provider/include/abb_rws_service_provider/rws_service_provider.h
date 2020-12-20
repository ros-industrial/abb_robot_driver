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

#ifndef ABB_RWS_SERVICE_PROVIDER_RWS_SERVICE_PROVIDER_H
#define ABB_RWS_SERVICE_PROVIDER_RWS_SERVICE_PROVIDER_H

#include <ros/ros.h>

#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_egm_rws_managers/system_data_parser.h>

#include <abb_robot_msgs/GetFileContents.h>
#include <abb_robot_msgs/GetIOSignal.h>
#include <abb_robot_msgs/GetRAPIDBool.h>
#include <abb_robot_msgs/GetRAPIDDnum.h>
#include <abb_robot_msgs/GetRAPIDNum.h>
#include <abb_robot_msgs/GetRAPIDString.h>
#include <abb_robot_msgs/GetRAPIDSymbol.h>
#include <abb_robot_msgs/GetRobotControllerDescription.h>
#include <abb_robot_msgs/GetSpeedRatio.h>
#include <abb_robot_msgs/SetFileContents.h>
#include <abb_robot_msgs/SetIOSignal.h>
#include <abb_robot_msgs/SetRAPIDBool.h>
#include <abb_robot_msgs/SetRAPIDDnum.h>
#include <abb_robot_msgs/SetRAPIDNum.h>
#include <abb_robot_msgs/SetRAPIDString.h>
#include <abb_robot_msgs/SetRAPIDSymbol.h>
#include <abb_robot_msgs/SetSpeedRatio.h>
#include <abb_robot_msgs/SystemState.h>
#include <abb_robot_msgs/TriggerWithResultCode.h>

#include <abb_rapid_sm_addin_msgs/GetEGMSettings.h>
#include <abb_rapid_sm_addin_msgs/RuntimeState.h>
#include <abb_rapid_sm_addin_msgs/SetEGMSettings.h>
#include <abb_rapid_sm_addin_msgs/SetRAPIDRoutine.h>
#include <abb_rapid_sm_addin_msgs/SetSGCommand.h>

namespace abb
{
namespace robot
{

/**
 * \brief Service provider exposing ROS services that interact with an ABB robot controller via RWS.
 */
class RWSServiceProvider
{
public:
  /**
   * \brief Creates a service provider.
   *
   * \param nh_params node handle in the namespace where ROS parameters should be gathered from.
   * \param nh_srvs node handle in the namespace where ROS services should be advertised in.
   *
   * \throw std::runtime_error if the creation failed (e.g. any initialization step fails).
   */
  RWSServiceProvider(ros::NodeHandle& nh_params, ros::NodeHandle& nh_srvs);

private:
  using GetFileContents       = abb_robot_msgs::GetFileContents;
  using GetIOSignal           = abb_robot_msgs::GetIOSignal;
  using GetRAPIDBool          = abb_robot_msgs::GetRAPIDBool;
  using GetRAPIDDnum          = abb_robot_msgs::GetRAPIDDnum;
  using GetRAPIDNum           = abb_robot_msgs::GetRAPIDNum;
  using GetRAPIDString        = abb_robot_msgs::GetRAPIDString;
  using GetRAPIDSymbol        = abb_robot_msgs::GetRAPIDSymbol;
  using GetRCDescription      = abb_robot_msgs::GetRobotControllerDescription;
  using GetSpeedRatio         = abb_robot_msgs::GetSpeedRatio;
  using SetFileContents       = abb_robot_msgs::SetFileContents;
  using SetIOSignal           = abb_robot_msgs::SetIOSignal;
  using SetRAPIDBool          = abb_robot_msgs::SetRAPIDBool;
  using SetRAPIDDnum          = abb_robot_msgs::SetRAPIDDnum;
  using SetRAPIDNum           = abb_robot_msgs::SetRAPIDNum;
  using SetRAPIDString        = abb_robot_msgs::SetRAPIDString;
  using SetRAPIDSymbol        = abb_robot_msgs::SetRAPIDSymbol;
  using SetSpeedRatio         = abb_robot_msgs::SetSpeedRatio;
  using TriggerWithResultCode = abb_robot_msgs::TriggerWithResultCode;
  using GetEGMSettings        = abb_rapid_sm_addin_msgs::GetEGMSettings;
  using SetEGMSettings        = abb_rapid_sm_addin_msgs::SetEGMSettings;
  using SetRAPIDRoutine       = abb_rapid_sm_addin_msgs::SetRAPIDRoutine;
  using SetSGCommand          = abb_rapid_sm_addin_msgs::SetSGCommand;

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
  };

  /**
   * \brief Callback for robot controller system state messages.
   *
   * \param message to process.
   */
  void systemStatesCallback(const abb_robot_msgs::SystemState::ConstPtr& message);

  /**
   * \brief Callback for RobotWare StateMachine Add-In runtime state messages.
   *
   * Note: Only used if the Add-In is present in the robot controller's system.
   *
   * \param message to process.
   */
  void smAddInRuntimeStatesCallback(const abb_rapid_sm_addin_msgs::RuntimeState::ConstPtr& message);

  /**
   * \brief Gets the contents of a file.
   *
   * The file must be located in the robot controller's home directory.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getFileContents(GetFileContents::Request& request, GetFileContents::Response& response);

  /**
   * \brief Gets an IO-signal.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getIOSignal(GetIOSignal::Request& request, GetIOSignal::Response& response);

  /**
   * \brief Gets a RAPID 'bool' symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDBool(GetRAPIDBool::Request& request, GetRAPIDBool::Response& response);

  /**
   * \brief Gets a RAPID 'dnum' symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDDnum(GetRAPIDDnum::Request& request, GetRAPIDDnum::Response& response);

  /**
   * \brief Gets a RAPID 'num' symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDNum(GetRAPIDNum::Request& request, GetRAPIDNum::Response& response);

  /**
   * \brief Gets a RAPID 'string' symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDString(GetRAPIDString::Request& request, GetRAPIDString::Response& response);

  /**
   * \brief Gets a RAPID symbol (in raw text format).
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRAPIDSymbol(GetRAPIDSymbol::Request& request, GetRAPIDSymbol::Response& response);

  /**
   * \brief Gets a description of the connected robot controller.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getRCDescription(GetRCDescription::Request& request, GetRCDescription::Response& response);

  /**
   * \brief Gets the controller speed ratio (in the range [0, 100]) for RAPID motions.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getSpeedRatio(GetSpeedRatio::Request& request, GetSpeedRatio::Response& response);

  /**
   * \brief Sets all RAPID program pointers to respective main procedure.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool ppToMain(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Signals that custom RAPID routine(s) should be run for all RAPID programs.
   *
   * Notes:
   * - Requires the StateMachine Add-In.
   * - The desired RAPID routine(s) needs to be specified beforehand (one per RAPID task).
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool runRAPIDRoutine(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Signals that SmartGripper command(s) should be run for all RAPID programs.
   *
   * Notes:
   * - Requires the StateMachine Add-In.
   * - The desired SmartGripper command(s) needs to be specified beforehand (one per RAPID task).
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool runSGRoutine(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Sets the contents of a file.
   *
   * The file will be uploaded to the robot controller's home directory.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setFileContents(SetFileContents::Request& request, SetFileContents::Response& response);

  /**
   * \brief Sets an IO-signal.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setIOSignal(SetIOSignal::Request& request, SetIOSignal::Response& response);

  /**
   * \brief Sets the motors off.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setMotorsOff(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Sets the motors on.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setMotorsOn(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Sets a RAPID 'bool' symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDBool(SetRAPIDBool::Request& request, SetRAPIDBool::Response& response);

  /**
   * \brief Sets a RAPID 'dnum' symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDDnum(SetRAPIDDnum::Request& request, SetRAPIDDnum::Response& response);

  /**
   * \brief Sets a RAPID 'num' symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDNum(SetRAPIDNum::Request& request, SetRAPIDNum::Response& response);

  /**
   * \brief Sets a RAPID 'string' symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDString(SetRAPIDString::Request& request, SetRAPIDString::Response& response);

  /**
   * \brief Sets a RAPID symbol.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDSymbol(SetRAPIDSymbol::Request& request, SetRAPIDSymbol::Response& response);

  /**
   * \brief Sets the controller speed ratio (in the range [0, 100]) for RAPID motions.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setSpeedRatio(SetSpeedRatio::Request& request, SetSpeedRatio::Response& response);

  /**
   * \brief Starts all RAPID programs.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool startRAPID(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Stop all RAPID programs.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool stopRAPID(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Gets EGM settings used by a specific RAPID task.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool getEGMSettings(GetEGMSettings::Request& request, GetEGMSettings::Response& response);

  /**
   * \brief Sets EGM settings used by a specific RAPID task.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setEGMSettings(SetEGMSettings::Request& request, SetEGMSettings::Response& response);

  /**
   * \brief Sets desired custom RAPID routine for a specific RAPID task.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setRAPIDRoutine(SetRAPIDRoutine::Request& request, SetRAPIDRoutine::Response& response);

  /**
   * \brief Sets desired SmartGripper command for a specific RAPID task.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool setSGCommand(SetSGCommand::Request& request, SetSGCommand::Response& response);

  /**
   * \brief Starts EGM joint motions for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool startEGMJoint(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Starts EGM pose motions for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool startEGMPose(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Starts EGM position streaming for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool startEGMStream(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Stops EGM motions for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool stopEGM(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Stops EGM position streaming for all RAPID programs.
   *
   * Note: Requires the StateMachine Add-In.
   *
   * \param request to process.
   * \param response for containing the result.
   *
   * \return bool true if the request was processed.
   */
  bool stopEGMStream(TriggerWithResultCode::Request& request, TriggerWithResultCode::Response& response);

  /**
   * \brief Verify that auto mode is active.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if auto mode is active.
   */
  bool verifyAutoMode(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a filename is not empty.
   *
   * \param filename to verify.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the filename is not empty.
   */
  bool verifyArgumentFilename(const std::string& filename, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a RAPID symbol path does not contain any empty subcomponents.
   *
   * \param path to verify.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RAPID symbol path does not contain any empty subcomponents..
   */
  bool verifyArgumentRAPIDSymbolPath(const abb_robot_msgs::RAPIDSymbolPath& path, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a RAPID task name is not empty.
   *
   * \param task to verify.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RAPID task name is not empty.
   */
  bool verifyArgumentRAPIDTask(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that an IO-signal name is not empty.
   *
   * \param signal to verify.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the IO-signal name is not empty.
   */
  bool verifyArgumentSignal(const std::string& signal, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that motors are off.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the motors are off.
   */
  bool verifyMotorsOff(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that motors are on.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the motors are on.
   */
  bool verifyMotorsOn(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that runtime states has been received for StateMachine Add-In instances.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if runtime states has been received.
   */
  bool verifySMAddInRuntimeStates(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a StateMachine Add-In instance is used by a RAPID task.
   *
   * \param task to check.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RAPID task is used by a StateMachine Add-In instance.
   */
  bool verifySMAddInTaskExist(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that a StateMachine Add-In instance is used by a RAPID task and that it has been initialized.
   *
   * \param task to check.
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RAPID task is used by a StateMachine Add-In instance and has been initialized.
   */
  bool verifySMAddInTaskInitialized(const std::string& task, uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that RAPID is running.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if RAPID is running.
   */
  bool verifyRAPIDRunning(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that RAPID is stopped.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if RAPID is stopped.
   */
  bool verifyRAPIDStopped(uint16_t& result_code, std::string& message);

  /**
   * \brief Verify that the RWS communication manager is ready.
   *
   * \param[out] result_code numerical error code.
   * \param[out] message container for possible error message.
   *
   * \return bool true if the RWS communication manager is ready.
   */
  bool verifyRWSManagerReady(uint16_t& result_code, std::string& message);

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
   * \brief Subscriber for robot controller system states.
   */
  ros::Subscriber system_states_subscriber_;

  /**
   * \brief Subscriber for RobotWare StateMachine Add-In runtime states.
   */
  ros::Subscriber sm_runtime_states_subscriber_;

  /**
   * \brief List of provided ROS services.
   */
  std::vector<ros::ServiceServer> services_;

  /**
   * \brief The latest known robot controller system state.
   */
  abb_robot_msgs::SystemState system_state_;

  /**
   * \brief The latest known RobotWare StateMachine Add-In runtime state.
   */
  abb_rapid_sm_addin_msgs::RuntimeState sm_addin_runtime_state_;
};

}
}

#endif
