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

#ifndef HW_COMM_I2C_H
#define HW_COMM_I2C_H

namespace hw_comm {
namespace i2c {

class HwCommI2CBase
{
public:
    virtual ~HwCommI2CBase() {}

    virtual int32_t writeByte(const uint8_t dev_addr, const uint8_t value) = 0;
    virtual uint8_t readByte(const uint8_t dev_addr) = 0;

    virtual int32_t writeByteData(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t value) = 0;
    virtual uint8_t readByteData(const uint8_t dev_addr, const uint8_t reg_addr) = 0;

    virtual int32_t writeWordData(const uint8_t dev_addr, const uint8_t reg_addr, const uint16_t value) = 0;
    virtual uint16_t readWordData(const uint8_t dev_addr, const uint8_t reg_addr) = 0;

    virtual int32_t writeBlockData(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t length, const uint8_t* values) = 0;
    virtual int32_t readBlockData(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t length, uint8_t* values) = 0;
};

class HwCommI2C : public HwCommI2CBase
{
public:
    explicit HwCommI2C(const char* dev_name);
    virtual ~HwCommI2C();

    virtual int32_t writeByte(const uint8_t dev_addr, const uint8_t value);
    virtual uint8_t readByte(const uint8_t dev_addr);

    virtual int32_t writeByteData(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t value);
    virtual uint8_t readByteData(const uint8_t dev_addr, const uint8_t reg_addr);

    virtual int32_t writeWordData(const uint8_t dev_addr, const uint8_t reg_addr, const uint16_t value);
    virtual uint16_t readWordData(const uint8_t dev_addr, const uint8_t reg_addr);

    virtual int32_t writeBlockData(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t length, const uint8_t* values);
    virtual int32_t readBlockData(const uint8_t dev_addr, const uint8_t reg_addr, const uint8_t length, uint8_t* values);

private:
    int32_t fd_;
    uint8_t dev_addr_;
};

} // namespace i2c
} // namespace hw_comm
#endif // HW_COMM_I2C_H
