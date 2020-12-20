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

#include <cstdlib>
#include <stdexcept>

#include <ros/ros.h>

#include <abb_egm_hardware_interface/egm_controller_stopper.h>

namespace
{
/**
 * \brief Name for ROS logging in the 'main' context.
 */
constexpr char ROS_LOG_MAIN[]{"main"};
}

int main(int argc, char** argv)
{
  //--------------------------------------------------------
  // Preparations
  //--------------------------------------------------------
  ros::init(argc, argv, "egm_controller_stopper");

  // Create a node handle, in the namespace where
  // the node should read parameters from.
  ros::NodeHandle nh_params{"~"};

  // Create a node handle, in the namespace where
  // the node should subscribe to messages in.
  ros::NodeHandle nh_msgs{"egm"};

  // Create a node handle, in the namespace where
  // the node should call services in.
  ros::NodeHandle nh_srvs{"egm/controller_manager"};

  //--------------------------------------------------------
  // Start node execution
  //--------------------------------------------------------
  try
  {
    abb::robot::EGMControllerStopper egm_controller_stopper{nh_params, nh_msgs, nh_srvs};
    ros::spin();
  }
  catch(const std::runtime_error& exception)
  {
    ROS_FATAL_STREAM_NAMED(ROS_LOG_MAIN, "Runtime error: '" << exception.what() << "'");
    return EXIT_FAILURE;
  }
  catch(const std::exception& exception)
  {
    ROS_FATAL_STREAM_NAMED(ROS_LOG_MAIN, "Exception '" << exception.what() << "'");
    return EXIT_FAILURE;
  }
  catch(...)
  {
    ROS_FATAL_NAMED(ROS_LOG_MAIN, "Unknown exception");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
