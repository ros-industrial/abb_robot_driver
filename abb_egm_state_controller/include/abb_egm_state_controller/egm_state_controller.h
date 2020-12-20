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

#ifndef ABB_EGM_STATE_CONTROLLER_EGM_STATE_CONTROLLER_H
#define ABB_EGM_STATE_CONTROLLER_EGM_STATE_CONTROLLER_H

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <controller_interface/controller.h>
#include <realtime_tools/realtime_publisher.h>

#include <abb_egm_hardware_interface/egm_state_interface.h>

#include <abb_egm_msgs/EGMState.h>

namespace abb
{
namespace robot
{

/**
 * \brief Controller for publishing the state of Externally Guided Motion (EGM) communication channels.
 */
class EGMStateController : public controller_interface::Controller<EGMStateInterface>
{
public:
  /**
   * \brief Creates a controller.
   */
  EGMStateController() = default;

  /**
   * \brief Initializes the controller.
   *
   * \param p_hw specifying the controller's hardware interface.
   * \param root_nh for a ROS node handle in the root namespace.
   * \param controller_nh for a ROS node handle in the controller's namespace.
   *
   * \return bool true if the initialization succeeded.
   */
  bool init(EGMStateInterface* p_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  /**
   * \brief Starts the controller.
   *
   * \param time specifying the start time.
   */
  void starting(const ros::Time& time);

  /**
   * \brief Updates the controller.
   *
   * \param time specifying the current time.
   * \param period specifying the time period since the last update.
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /**
   * \brief Stops the controller.
   *
   * \param time specifying the stop time.
   */
  void stopping(const ros::Time& time);

private:
  using EGMStatePublisher = realtime_tools::RealtimePublisher<abb_egm_msgs::EGMState>;

  /**
   * \brief Default publish rate [Hz].
   */
  const double DEFAULT_PUBLISH_RATE{250.0};

  /**
   * \brief The resource handles used by the controller.
   */
  std::vector<EGMStateHandle> egm_state_handles_;

  /**
   * \brief The period that messages should be published at.
   */
  ros::Duration publish_period_;

  /**
   * \brief The last time messages were published.
   */
  ros::Time last_publish_time_;

  /**
   * \brief EGM state publisher.
   */
  std::shared_ptr<EGMStatePublisher> p_egm_state_publisher_;
};

}
}

#endif
