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

#include <limits>
#include <stdexcept>

#include <ros/ros.h>

#include "abb_robot_cpp_utilities/verification.h"

namespace
{
/**
 * \brief Name for ROS logging in the 'verify' context.
 */
constexpr char ROS_LOG_VERIFY[]{"verify"};
}

namespace abb
{
namespace robot
{
namespace utilities
{

/***********************************************************************************************************************
 * Utility function definitions
 */

void verifyIPAddress(const std::string& ip_address)
{
  Poco::Net::IPAddress poco_ip_address{};
  if(!Poco::Net::IPAddress::tryParse(ip_address, poco_ip_address))
  {
    auto error_message{"Invalid IP address '" + ip_address + "' provided"};
    ROS_FATAL_STREAM_NAMED(ROS_LOG_VERIFY, error_message);
    throw std::runtime_error{error_message};
  }
}

void verifyPortNumber(const int port_number)
{
  if(port_number < 0 || port_number > std::numeric_limits<unsigned short>::max())
  {
    auto error_message{"Invalid port number '" + std::to_string(port_number) + "' provided"};
    ROS_FATAL_STREAM_NAMED(ROS_LOG_VERIFY, error_message);
    throw std::runtime_error{error_message};
  }
}

void verifyRate(const double rate)
{
  if(rate <= 0.0)
  {
    auto error_message{"Invalid rate [Hz] '" + std::to_string(rate) + "' provided"};
    ROS_FATAL_STREAM_NAMED(ROS_LOG_VERIFY, error_message);
    throw std::runtime_error{error_message};
  }
}

void verifyRobotWareVersion(const RobotWareVersion& rw_version)
{
  if(rw_version.major_number() == 6 &&
     rw_version.minor_number() < 7 &&
     rw_version.patch_number() < 1)
  {
    auto error_message{"Unsupported RobotWare version (" + rw_version.name() + ", need at least 6.07.01)"};
    ROS_FATAL_STREAM_NAMED(ROS_LOG_VERIFY, error_message);
    throw std::runtime_error{error_message};
  }
}

bool verifyStateMachineAddInPresence(const SystemIndicators& system_indicators)
{
  return system_indicators.addins().state_machine_1_0() ||
         system_indicators.addins().state_machine_1_1();
}

}
}
}
