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

#ifndef HW_COMM_GPIO_H
#define HW_COMM_GPIO_H

namespace hw_comm {
namespace gpio {

typedef enum GPIODirectionTag {
    GPIO_DIRE_IN = 0,
    GPIO_DIRE_OUT
} GPIODirection;

typedef enum GPIOEdgeTag {
    GPIO_EDGE_RAISING = 0,
    GPIO_EDGE_FALLING,
    GPIO_EDGE_BOTH
} GPIOEdge;

typedef enum GPIOValueTag {
    GPIO_VALUE_LOW = 0,
    GPIO_VALUE_HIGH
} GPIOValue;

class HwCommGPIOBase
{
public:
    virtual ~HwCommGPIOBase() {}

    virtual int32_t setDirection(const GPIODirection& direction) = 0;
    virtual int32_t setEdge(const GPIOEdge& edge) = 0;
    virtual int32_t setValue(const GPIOValue& value) = 0;
    virtual int32_t getValue(GPIOValue& value) = 0;
};

class HwCommGPIO : public HwCommGPIOBase
{
public:
    explicit HwCommGPIO(const uint32_t& gpio);
    virtual ~HwCommGPIO();

    virtual int32_t setDirection(const GPIODirection& direction);
    virtual int32_t setEdge(const GPIOEdge& edge);
    virtual int32_t setValue(const GPIOValue& value);
    virtual int32_t getValue(GPIOValue& value);

private:
    uint32_t gpio_;
};

} // namespace gpio
} // namespace hw_comm
#endif // HW_COMM_GPIO_H
