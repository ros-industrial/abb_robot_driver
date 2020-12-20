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

#ifndef ABB_ROBOT_CPP_UTILITIES_PARAMETERS_H
#define ABB_ROBOT_CPP_UTILITIES_PARAMETERS_H

#include <string>

#include <ros/ros.h>

namespace abb
{
namespace robot
{
namespace utilities
{

/**
 * \brief Gets an optional parameter from the ROS parameter server.
 *
 * \param nh ROS node handle in the namespace where the parameter should be retrieved from.
 * \param key of the parameter.
 * \param value for storing the retrieved parameter.
 * \param default_value to use if the parameter is not found in the parameter server.
 */
template <typename type>
void getParameter(ros::NodeHandle& nh, const std::string& key, type& value, const type& default_value);

/**
 * \brief Gets a mandatory parameter from the ROS parameter server.
 *
 * \param nh ROS node handle in the namespace where the parameter should be retrieved from.
 * \param key of the parameter.
 * \param value for storing the retrieved parameter.
 *
 * \throw std::runtime_error if the parameter is not found in the parameter server.
 */
template <typename type>
void getParameter(ros::NodeHandle& nh, const std::string& key, type& value);

/**
 * \brief Gets an optional list parameter from the ROS parameter server.
 *
 * \param nh ROS node handle in the namespace where the parameter should be retrieved from.
 * \param key of the parameter.
 * \param value for storing the retrieved parameter.
 * \param default_value to use if the parameter is not found in the parameter server.
 */
template <typename type>
void getParameter(ros::NodeHandle& nh,
                  const std::string& key,
                  std::vector<type>& value,
                  const std::vector<type>& default_value);

/**
 * \brief Gets a mandatory list parameter from the ROS parameter server.
 *
 * \param nh ROS node handle in the namespace where the parameter should be retrieved from.
 * \param key of the parameter.
 * \param value for storing the retrieved parameter.
 *
 * \throw std::runtime_error if the parameter is not found in the parameter server.
 */
template <typename type>
void getParameter(ros::NodeHandle& nh, const std::string& key, std::vector<type>& value);

}
}
}

#endif
