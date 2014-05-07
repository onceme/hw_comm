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

#ifndef HW_COMM_SPI_H
#define HW_COMM_SPI_H

namespace hw_comm {
namespace spi {

typedef enum {
    SPI_MODE_DEFAULT = SPI_MODE_0,
    SPI_MODE_CPHA = SPI_MODE_1,
    SPI_MODE_CPOL = SPI_MODE_2,
    SPI_MODE_CPOL_CPHA = SPI_MODE_3
} SPIMode;

typedef enum {
    SPI_SB_MSB = 0,
    SPI_SB_LSB
} SPISB;

class HwCommSPIBase
{
public:
    virtual ~HwCommSPIBase() {}

    virtual int32_t setMode(const SPIMode& mode) = 0;
    virtual int32_t getMode(SPIMode& mode) = 0;

    virtual int32_t setLSB(const SPISB& sb) = 0;
    virtual int32_t getLSB(SPISB& sb) = 0;

    virtual int32_t setBitsPerWord(const uint8_t bpw) = 0;
    virtual int32_t getBitsPerWord(uint8_t& bpw) = 0;

    virtual int32_t setSpeed(const uint32_t speed) = 0;
    virtual int32_t getSpeed(uint32_t& speed) = 0;

    virtual int32_t transmit(const uint8_t* tx, const uint32_t tx_len) = 0;
    virtual int32_t receive(uint8_t* rx, const uint32_t rx_len) = 0;
    virtual int32_t message(const uint8_t* tx, const uint32_t tx_len, uint8_t* rx, const uint32_t rx_len) = 0;
};

class HwCommSPI : public HwCommSPIBase
{
public:
    explicit HwCommSPI(const char* bus_name);
    virtual ~HwCommSPI();

    virtual int32_t setMode(const SPIMode& mode);
    virtual int32_t getMode(SPIMode& mode);

    virtual int32_t setLSB(const SPISB& sb);
    virtual int32_t getLSB(SPISB& sb);

    virtual int32_t setBitsPerWord(const uint8_t bpw);
    virtual int32_t getBitsPerWord(uint8_t& bpw);

    virtual int32_t setSpeed(const uint32_t speed);
    virtual int32_t getSpeed(uint32_t& speed);

    virtual int32_t transmit(const uint8_t* tx, const uint32_t tx_len);
    virtual int32_t receive(uint8_t* rx, const uint32_t rx_len);
    virtual int32_t message(const uint8_t* tx, const uint32_t tx_len, uint8_t* rx, const uint32_t rx_len);

private:
    int32_t fd_;
};

} // namespace spi
} // namespace hw_comm
#endif // HW_COMM_SPI_H
