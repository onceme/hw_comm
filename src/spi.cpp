/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Aaron Wang Shi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Aaron Wang Shi nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/spi/spidev.h>
#include <ros/console.h>
#include "hw_comm/spi.h"

namespace hw_comm {
namespace spi {

HwCommSPI::HwCommSPI(const char* dev_name)
{
}

HwCommSPI::~HwCommSPI()
{
}

int32_t HwCommSPI::setMode(const SPIMode& mode)
{
    return 0;
}

int32_t HwCommSPI::getMode(SPIMode& mode)
{
    return 0;
}

int32_t HwCommSPI::setLSB(const SPISB& mode)
{
    return 0;
}

int32_t HwCommSPI::getLSB(SPISB& mode)
{
    return 0;
}

int32_t HwCommSPI::setBitsPerWord(const uint8_t bpw)
{
    return 0;
}

int32_t HwCommSPI::getBitsPerWord(uint8_t& bpw)
{
    return 0;
}

int32_t HwCommSPI::setSpeed(const uint32_t speed)
{
    return 0;
}

int32_t HwCommSPI::getSpeed(uint32_t& speed)
{
    return 0;
}

int32_t HwCommSPI::transmit(const uint8_t* buffer, const uint32_t length)
{
    return 0;
}

int32_t HwCommSPI::receive(uint8_t* buffer, const uint32_t length)
{
    return 0;
}

} // namespace spi
} // namespace hw_comm
