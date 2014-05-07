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
#include <fcntl.h>
#include <sys/ioctl.h>
#include <ros/console.h>
#include "hw_comm/spi.h"

namespace hw_comm {
namespace spi {

namespace {

const char* const NAME = "HwCommSPI";

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

template <typename T>
int32_t spiDevIoctl(const int32_t& fd, const int32_t& req, T& arg)
{
    if (ioctl(fd, req, &arg) < 0) {
        ROS_ERROR_NAMED(NAME, "ioctl spi device error: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

} // namespace

HwCommSPI::HwCommSPI(const char* bus_name)
    : fd_(-1)
{
    fd_ = open(bus_name, O_RDWR);
    if (fd_ < 0) {
        ROS_ERROR_NAMED(NAME, "open spi bus %s error: %s\n", bus_name, strerror(errno));
    }
}

HwCommSPI::~HwCommSPI()
{
    if (close(fd_) < 0) {
        ROS_ERROR_NAMED(NAME, "close spi bus error: %s\n", strerror(errno));
    }
}

int32_t HwCommSPI::setMode(const SPIMode& mode)
{
    return ((0 == spiDevIoctl(fd_, SPI_IOC_WR_MODE, mode)) ? 0 : -1);
}

int32_t HwCommSPI::getMode(SPIMode& mode)
{
    return ((0 == spiDevIoctl(fd_, SPI_IOC_RD_MODE, mode)) ? 0 : -1);
}

int32_t HwCommSPI::setLSB(const SPISB& sb)
{
    return ((0 == spiDevIoctl(fd_, SPI_IOC_WR_LSB_FIRST, sb)) ? 0 : -1);
}

int32_t HwCommSPI::getLSB(SPISB& sb)
{
    return ((0 == spiDevIoctl(fd_, SPI_IOC_RD_LSB_FIRST, sb)) ? 0 : -1);
}

int32_t HwCommSPI::setBitsPerWord(const uint8_t bpw)
{
    return ((0 == spiDevIoctl(fd_, SPI_IOC_WR_BITS_PER_WORD, bpw)) ? 0 : -1);
}

int32_t HwCommSPI::getBitsPerWord(uint8_t& bpw)
{
    return ((0 == spiDevIoctl(fd_, SPI_IOC_RD_BITS_PER_WORD, bpw)) ? 0 : -1);
}

int32_t HwCommSPI::setSpeed(const uint32_t speed)
{
    return ((0 == spiDevIoctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, speed)) ? 0 : -1);
}

int32_t HwCommSPI::getSpeed(uint32_t& speed)
{
    return ((0 == spiDevIoctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, speed)) ? 0 : -1);
}

int32_t HwCommSPI::transmit(const uint8_t* tx, const uint32_t tx_len)
{
    struct spi_ioc_transfer xfer;
    memset(&xfer, 0, sizeof(xfer));

    // BWINA : external structure requires a __u64 to store the buffer pointer
    xfer.tx_buf = reinterpret_cast<__u64>(tx);
    xfer.len = tx_len;

    int32_t ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer);

    if (ret < 0) {
        ROS_ERROR_NAMED(NAME, "ioctl spi transmit error: %s\n", strerror(errno));
    }

    return ret;
}

int32_t HwCommSPI::receive(uint8_t* rx, const uint32_t rx_len)
{
    int32_t read_len = read(fd_, rx, MIN(rx_len, SSIZE_MAX));

    if ((read_len < 0) || (MIN(rx_len, SSIZE_MAX) != static_cast<uint32_t>(read_len))) {
        ROS_ERROR_NAMED(NAME, "ioctl spi receive error: %s\n", strerror(errno));
    }

    return read_len;
}

int32_t HwCommSPI::message(const uint8_t* tx, const uint32_t tx_len, uint8_t* rx, const uint32_t rx_len)
{
#if 1 // half duplex
    struct spi_ioc_transfer xfer[2];
    memset(&xfer, 0, sizeof(xfer));

    // BWINA : external structure requires a __u64 to store the buffer pointer
    xfer[0].tx_buf = reinterpret_cast<__u64>(tx);
    xfer[0].len = tx_len;

    xfer[1].rx_buf = reinterpret_cast<__u64>(rx);
    xfer[1].len = rx_len;

    int32_t ret = ioctl(fd_, SPI_IOC_MESSAGE(2), &xfer);
#else // full duplex
    struct spi_ioc_transfer xfer;
    memset(&xfer, 0, sizeof(xfer));

    // BWINA : external structure requires a __u64 to store the buffer pointer
    xfer.tx_buf = reinterpret_cast<__u64>(tx);
    xfer.rx_buf = reinterpret_cast<__u64>(rx);
    xfer.len = MAX(tx_len, rx_len);

    int32_t ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &xfer);
#endif
    if (ret < 0) {
        ROS_ERROR_NAMED(NAME, "ioctl spi message error: %s\n", strerror(errno));
    }

    return ret;
}

} // namespace spi
} // namespace hw_comm
