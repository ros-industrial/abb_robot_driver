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

#ifndef ABB_ROBOT_CPP_UTILITIES_VERIFICATION_H
#define ABB_ROBOT_CPP_UTILITIES_VERIFICATION_H

#include <string>

#include <abb_egm_rws_managers/utilities.h>

namespace abb
{
namespace robot
{
namespace utilities
{

/**
 * \brief Verifies that an IP address string represents a valid IP address.
 *
 * \param ip_address to verify.
 *
 * \throw std::runtime_error if the IP address is invalid.
 */
void verifyIPAddress(const std::string& ip_address);

/**
 * \brief Verifies that a port number is valid, i.e. in the range [0, 65535].
 *
 * \param port_number to verify.
 *
 * \throw std::runtime_error if the port number is invalid.
 */
void verifyPortNumber(const int port_number);

/**
 * \brief Verifies that a rate [Hz] is valid, i.e. positive.
 *
 * \param rate to verify.
 *
 * \throw std::runtime_error if the rate is invalid.
 */
void verifyRate(const double rate);

/**
 * \brief Verifies that the RobotWare version is supported.
 *
 * Note: For now, only RobotWare versions in the range [6.07.01, 7.0) are supported (i.e. excluding 7.0).
 *
 * \param rw_version to verify.
 *
 * \throw std::runtime_error if the RobotWare version is not supported.
 */
void verifyRobotWareVersion(const RobotWareVersion& rw_version);

/**
 * \brief Verifies that the RobotWare StateMachine Add-In is present in a system.
 *
 * \param system_indicators to verify.
 *
 * \return bool true if the StateMachine Add-In is present.
 */
bool verifyStateMachineAddInPresence(const SystemIndicators& system_indicators);

}
}
}

#endif
