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

#ifndef ABB_EGM_HARDWARE_INTERFACE_EGM_STATE_INTERFACE_H
#define ABB_EGM_HARDWARE_INTERFACE_EGM_STATE_INTERFACE_H

#include <cassert>
#include <string>

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <abb_egm_rws_managers/data_containers.h>

namespace abb
{
namespace robot
{

/**
 * \brief Resource handle used to read the state of a single EGM channel.
 */
class EGMStateHandle
{
public:
  /**
   * \brief Creates a default handle.
   */
  EGMStateHandle() = default;

  /**
   * \brief Creates a handle.
   *
   * \param name specifying the EGM channel's name.
   * \param p_egm_channel_data specifying the EGM channel's data container.
   *
   * \throw hardware_interface::HardwareInterfaceException if the creation failed.
   */
  EGMStateHandle(const std::string& name, const EGMChannelData* p_egm_channel_data)
  :
  name_{name},
  p_egm_channel_data_{p_egm_channel_data}
  {
    if (!p_egm_channel_data_)
    {
      throw hardware_interface::HardwareInterfaceException{"Cannot create handle '" + name_ + "' (null data pointer)"};
    }
  }

  /**
   * \brief Retrieves the handle's name.
   *
   * \return std::string& reference to the name.
   */
  const std::string& getName() const { return name_; }

  /**
   * \brief Retrieves the handle's EGM channel data container.
   *
   * \return EGMChannelData* pointer to the container.
   */
  const EGMChannelData* getEGMChannelDataPtr() const { return p_egm_channel_data_; }

private:
  /**
   * \brief The handle's name.
   */
  std::string name_;

  /**
   * \brief The handle's EGM channel data container.
   */
  const EGMChannelData* p_egm_channel_data_{nullptr};
};

/**
 * \brief Hardware interface used to read the state of an array of EGM channels.
 */
class EGMStateInterface : public hardware_interface::HardwareResourceManager<EGMStateHandle> {};

}
}

#endif
