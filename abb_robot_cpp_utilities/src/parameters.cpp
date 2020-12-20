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

#include <stdexcept>

#include "abb_robot_cpp_utilities/mapping.h"
#include "abb_robot_cpp_utilities/parameters.h"

namespace
{
/**
 * \brief Name for ROS logging in the 'param' context.
 */
constexpr char ROS_LOG_PARAM[]{"param"};
}

namespace abb
{
namespace robot
{
namespace utilities
{

/***********************************************************************************************************************
 * Utility template function definitions
 */

template <typename type>
void getParameter(ros::NodeHandle& nh, const std::string& key, type& value, const type& default_value)
{
  ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Check for (optional) parameter '" << key << "'");

  if(nh.param<type>(key, value, default_value))
  {
    ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Found parameter '" << key << "'='" << value << "'");
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Parameter '" << key << "' not found, using default value '" << value << "'");
  }
}

template <typename type>
void getParameter(ros::NodeHandle& nh, const std::string& key, type& value)
{
  ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Check for (mandatory) parameter '" << key << "'");

  if(!nh.getParam(key, value))
  {
    auto error_message{"Parameter '" + key + "' not found"};
    ROS_FATAL_STREAM_NAMED(ROS_LOG_PARAM, error_message);
    throw std::runtime_error{error_message};
  }

  ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Found parameter '" << key << "'='" << value << "'");
}

template <typename type>
void getParameter(ros::NodeHandle& nh,
                  const std::string& key,
                  std::vector<type>& value,
                  const std::vector<type>& default_value)
{
  ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Check for (optional) parameter '" << key << "'");

  if(nh.param<std::vector<type>>(key, value, default_value))
  {
    ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Found parameter '" << key << "'='" << mapVectorToString(value) << "'");
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Parameter '" << key << "' not found, using default value '" <<
                                           mapVectorToString(value) << "'");
  }
}

template <typename type>
void getParameter(ros::NodeHandle& nh, const std::string& key, std::vector<type>& value)
{
  ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Check for (mandatory) parameter '" << key << "'");

  if(!nh.getParam(key, value))
  {
    auto error_message{"Parameter '" + key + "' not found"};
    ROS_FATAL_STREAM_NAMED(ROS_LOG_PARAM, error_message);
    throw std::runtime_error{error_message};
  }

  ROS_DEBUG_STREAM_NAMED(ROS_LOG_PARAM, "Found parameter '" << key << "'='" << mapVectorToString(value) << "'");
}

/***********************************************************************************************************************
 * Utility template function instantiations
 */

template void getParameter<std::string>(ros::NodeHandle&, const std::string&, std::string&, const std::string&);
template void getParameter<bool>(ros::NodeHandle&, const std::string&, bool&, const bool&);
template void getParameter<int>(ros::NodeHandle&, const std::string&, int&, const int&);
template void getParameter<double>(ros::NodeHandle&, const std::string&, double&, const double&);

template void getParameter<std::string>(ros::NodeHandle&, const std::string&, std::string&);
template void getParameter<bool>(ros::NodeHandle&, const std::string&, bool&);
template void getParameter<int>(ros::NodeHandle&, const std::string&, int&);
template void getParameter<double>(ros::NodeHandle&, const std::string&, double&);

template void getParameter<std::string>(ros::NodeHandle&,
                                        const std::string&,
                                        std::vector<std::string>&,
                                        const std::vector<std::string>&);
template void getParameter<bool>(ros::NodeHandle&, const std::string&, std::vector<bool>&, const std::vector<bool>&);
template void getParameter<int>(ros::NodeHandle&, const std::string&, std::vector<int>&, const std::vector<int>&);
template void getParameter<double>(ros::NodeHandle&,
                                   const std::string&,
                                   std::vector<double>&,
                                   const std::vector<double>&);

template void getParameter<std::string>(ros::NodeHandle&, const std::string&, std::vector<std::string>&);
template void getParameter<bool>(ros::NodeHandle&, const std::string&, std::vector<bool>&);
template void getParameter<int>(ros::NodeHandle&, const std::string&, std::vector<int>&);
template void getParameter<double>(ros::NodeHandle&, const std::string&, std::vector<double>&);

}
}
}
