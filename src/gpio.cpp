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

#include <ros/console.h>
#include <fcntl.h>
#include "hw_comm/gpio.h"

namespace hw_comm {
namespace gpio {

namespace {

const char* const NAME = "HwCommGPIO";

const char* const SYSFS_GPIO_DIR = "/sys/class/gpio";
const uint32_t MAX_BUF_LEN = 64;

int32_t doOpen(const uint32_t gpio, const char* const op, int32_t flag)
{
    int32_t fd(-1);
    char buf[MAX_BUF_LEN] = {0};

    snprintf(buf, sizeof(buf), "%s/gpio%d%s", SYSFS_GPIO_DIR, gpio, op);
    fd = open(buf, flag);
    return fd;
}

int32_t doWrite(int32_t fd, const char* buf, uint32_t len)
{
    uint32_t wrote_len(0);
    uint32_t total_len(0);

    while (total_len < len) {
        wrote_len = write(fd, buf + total_len, len - total_len);
        if (wrote_len < 0) {
            return wrote_len;
        }
        total_len += wrote_len;
    }
    return total_len;
}

int32_t manipulateGPIOFd(const uint32_t gpio, const char* const op)
{
    int32_t fd(-1);
    uint32_t len(0);
    char buf[MAX_BUF_LEN] = {0};

    fd = open(op, O_WRONLY);
    if (fd < 0) {
        ROS_ERROR_NAMED(NAME, "manipulate %s error: %s\n", op, strerror(errno));
        return -1;
    }

    len = snprintf(buf, sizeof(buf), "%d", gpio);
    if (doWrite(fd, buf, len) < 0) {
        ROS_ERROR_NAMED(NAME, "manipulate %s error: %s\n", op, strerror(errno));
    }
    close(fd);

    return 0;
}

int32_t exportGPIO(const uint32_t gpio)
{
    char buf[MAX_BUF_LEN] = {0};
    snprintf(buf, sizeof(buf), "%s%s", SYSFS_GPIO_DIR, "/export");
    return manipulateGPIOFd(gpio, buf);
}

int32_t unexportGPIO(const uint32_t gpio)
{
    char buf[MAX_BUF_LEN] = {0};
    snprintf(buf, sizeof(buf), "%s%s", SYSFS_GPIO_DIR, "/unexport");
    return manipulateGPIOFd(gpio, buf);
}

} // namespace


class HwCommGPIO::IrqMonitor
{
public:
    IrqMonitor()
    {}

    ~IrqMonitor()
    {}

private:

};

HwCommGPIO::HwCommGPIO(const uint32_t& gpio)
    : gpio_(gpio),
      irq_monitor_(0)
{
    exportGPIO(gpio_);
}

HwCommGPIO::~HwCommGPIO()
{
    unexportGPIO(gpio_);
}

int32_t HwCommGPIO::setDirection(const GPIODirection& direction)
{
    int32_t fd = doOpen(gpio_, "/direction", O_WRONLY);
    if (fd < 0) {
        ROS_ERROR_NAMED(NAME, "gpio %d set direction error: %s\n", gpio_, strerror(errno));
        return -1;
    }

    int32_t ret = (GPIO_DIRE_IN == direction)  ? doWrite(fd, "in", 3)  :
                  (GPIO_DIRE_OUT == direction) ? doWrite(fd, "out", 4) : -1;
    if (ret < 0) {
        ROS_ERROR_NAMED(NAME, "gpio %d set direction error: %s\n", gpio_, strerror(errno));
    }

    close(fd);
    return ((ret < 0) ? -1 : 0);
}

int32_t HwCommGPIO::setEdge(const GPIOEdge& edge)
{
    int32_t fd = doOpen(gpio_, "/edge", O_WRONLY);
    if (fd < 0) {
        ROS_ERROR_NAMED(NAME, "gpio %d set edge error: %s\n", gpio_, strerror(errno));
        return -1;
    }

    int32_t ret = (GPIO_EDGE_RISING  == edge) ? doWrite(fd, "rising", 8)  :
                  (GPIO_EDGE_FALLING == edge) ? doWrite(fd, "falling", 8) :
                  (GPIO_EDGE_BOTH    == edge) ? doWrite(fd, "both", 5)    :
                  (GPIO_EDGE_NONE    == edge) ? doWrite(fd, "none", 5)    : -1;
    if (ret < 0) {
        ROS_ERROR_NAMED(NAME, "gpio %d set edge error: %s\n", gpio_, strerror(errno));
    }

    close(fd);
    return ((ret < 0) ? -1 : 0);
}

int32_t HwCommGPIO::setValue(const GPIOValue& value)
{
    int32_t fd = doOpen(gpio_, "/value", O_WRONLY);
    if (fd < 0) {
        ROS_ERROR_NAMED(NAME, "gpio %d set value error: %s\n", gpio_, strerror(errno));
        return -1;
    }

    int32_t ret = (GPIO_VALUE_LOW  == value) ? doWrite(fd, "0", 2) :
                  (GPIO_VALUE_HIGH == value) ? doWrite(fd, "1", 2) : -1;
    if (ret < 0) {
        ROS_ERROR_NAMED(NAME, "gpio %d set value error: %s\n", gpio_, strerror(errno));
    }

    close(fd);
    return ((ret < 0) ? -1 : 0);
}

int32_t HwCommGPIO::getValue(GPIOValue& value)
{
    int32_t fd = doOpen(gpio_, "/value", O_RDONLY);
    if (fd < 0) {
        ROS_ERROR_NAMED(NAME, "gpio %d get value error: %s\n", gpio_, strerror(errno));
        return -1;
    }

    char val('0');
    int32_t ret = read(fd, &val, 1);
    if (ret < 0) {
        ROS_ERROR_NAMED(NAME, "gpio %d get value error: %s\n", gpio_, strerror(errno));
    }
    else {
        value = ('0' == val) ? GPIO_VALUE_LOW : GPIO_VALUE_HIGH;
    }

    close(fd);
    return ((ret < 0) ? -1 : 0);
}

} // namespace gpio
} // namespace hw_comm
