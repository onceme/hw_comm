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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include "ros/ros.h"
#include "i2c.h"

namespace hw_comm {
namespace i2c {

namespace {

const char* const NAME = "HwCommI2C";

} // namespace

HwCommI2C::HwCommI2C(const char* dev_name)
  : fd_(-1)
{
  fd_ = open(dev_name, O_RDWR);
  if (fd_ < 0) {
    ROS_ERROR_NAMED(NAME, "open i2c device error: %s\n", strerror(errno));
  }
}

HwCommI2C::~HwCommI2C()
{
  if (close(fd_) < 0) {
    ROS_ERROR_NAMED(NAME, "close i2c device error: %s\n", strerror(errno));
  }
}

int32_t HwCommI2C::writeByte(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t value)
{
  if (ioctl(fd_, I2C_SLAVE, dev_addr) < 0) {
    ROS_ERROR_NAMED(NAME, "ioctl i2c device error: %s\n", strerror(errno));
    return -1;
  }
  return i2c_smbus_write_byte_data(fd_, reg_addr, value);  
}

uint8_t HwCommI2C::readByte(const uint8_t dev_addr, const uint8_t reg_addr)
{
  if (ioctl(fd_, I2C_SLAVE, dev_addr) < 0) {
    ROS_ERROR_NAMED(NAME, "ioctl i2c device error: %s\n", strerror(errno));
    return -1;
  }
  return i2c_smbus_read_byte_data(fd_, dev_addr);
}

} // namespace i2c
} // namespace hw_comm
