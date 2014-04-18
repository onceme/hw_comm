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

#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <ros/console.h>
#include "hw_comm/i2c.h"

namespace hw_comm {
namespace i2c {

namespace {

const char* const NAME = "HwCommI2C";

int32_t i2cDevIoctl(const int32_t& fd, const int32_t& req,
                    uint8_t& curr_dev_addr, const uint8_t& new_dev_addr)
{
    if (curr_dev_addr == new_dev_addr) {
        return 0;
    }
    else if (ioctl(fd, req, new_dev_addr) < 0) {
        ROS_ERROR_NAMED(NAME, "ioctl i2c device error: %s\n", strerror(errno));
        return -1;
    }
    curr_dev_addr = new_dev_addr;
    return 0;
}

} // namespace

HwCommI2C::HwCommI2C(const char* bus_name)
    : fd_(-1),
      dev_addr_(0x00)
{
    fd_ = open(bus_name, O_RDWR);
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

int32_t HwCommI2C::writeByte(const uint8_t dev_addr, const uint8_t value)
{
    return (((0 == i2cDevIoctl(fd_, I2C_SLAVE, dev_addr_, dev_addr))
             && (0 == i2c_smbus_write_byte(fd_, value))) ? 0 : -1);
}

uint8_t HwCommI2C::readByte(const uint8_t dev_addr)
{
    return ((0 == i2cDevIoctl(fd_, I2C_SLAVE, dev_addr_, dev_addr)) ? i2c_smbus_read_byte(fd_) : 0);
}

int32_t HwCommI2C::writeByteData(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t value)
{
    return (((0 == i2cDevIoctl(fd_, I2C_SLAVE, dev_addr_, dev_addr))
             && (0 == i2c_smbus_write_byte_data(fd_, reg_addr, value))) ? 0 : -1);
}

uint8_t HwCommI2C::readByteData(const uint8_t dev_addr, const uint8_t reg_addr)
{
    return ((0 == i2cDevIoctl(fd_, I2C_SLAVE, dev_addr_, dev_addr)) ? i2c_smbus_read_byte_data(fd_, reg_addr) : 0);
}

int32_t HwCommI2C::writeWordData(const uint8_t dev_addr, const uint8_t reg_addr, const uint16_t value)
{
    uint8_t values[2] = {0};
    return writeBlockData(dev_addr, reg_addr, 2, values);
}

uint16_t HwCommI2C::readWordData(const uint8_t dev_addr, const uint8_t reg_addr)
{
    uint8_t values[2] = {0};
    return (0 > readBlockData(dev_addr, reg_addr, 2, values)) ? 0 : ((values[0] << 8) | values[1]);
}

int32_t HwCommI2C::writeBlockData(const uint8_t dev_addr, const uint8_t reg_addr,
                                  const uint8_t length, const uint8_t* values)
{
    return (((0 == i2cDevIoctl(fd_, I2C_SLAVE, dev_addr_, dev_addr))
             && (0 == i2c_smbus_write_i2c_block_data(fd_, reg_addr, length, values))) ? 0 : -1);
}

int32_t HwCommI2C::readBlockData(const uint8_t dev_addr, const uint8_t reg_addr,
                                 const uint8_t length, uint8_t* values)
{
    return ((0 == i2cDevIoctl(fd_, I2C_SLAVE, dev_addr_, dev_addr)) ? i2c_smbus_read_i2c_block_data(fd_, reg_addr, length, values) : 0);
}

} // namespace i2c
} // namespace hw_comm
