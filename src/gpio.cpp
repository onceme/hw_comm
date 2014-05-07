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

#include <sys/syscall.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <mqueue.h>
#include <poll.h>
#include <boost/thread/thread.hpp>
#include <ros/console.h>
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
    int32_t wrote_len(0);
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

int32_t existGPIO(const uint32_t gpio)
{
    char gpio_name[MAX_BUF_LEN] = {0};
    snprintf(gpio_name, sizeof(gpio_name), "%s/gpio%d", SYSFS_GPIO_DIR, gpio);
    struct stat sb;
    return ((0 == stat(gpio_name, &sb)) && (S_ISDIR(sb.st_mode))) ? 0 : -1;
}

int32_t unexportGPIO(const uint32_t gpio)
{
    if (0 == existGPIO(gpio)) {
        char buf[MAX_BUF_LEN] = {0};
        snprintf(buf, sizeof(buf), "%s%s", SYSFS_GPIO_DIR, "/unexport");
        return manipulateGPIOFd(gpio, buf);
    }
    return 0;
}

int32_t exportGPIO(const uint32_t gpio)
{
    if (0 != existGPIO(gpio)) {
        char buf[MAX_BUF_LEN] = {0};
        snprintf(buf, sizeof(buf), "%s%s", SYSFS_GPIO_DIR, "/export");
        return manipulateGPIOFd(gpio, buf);
    }
    return 0;
}

} // namespace

class HwCommGPIO::IrqMonitor
{
public:
    explicit IrqMonitor(const uint32_t& gpio)
        : gpio_(gpio),
          irq_handler_list_(),
          monitor_entity_(),
          mutex_(),
          irq_chn_name_(),
          mqd_()
    {
        memset(irq_chn_name_, 0, MAX_BUF_LEN);
        snprintf(irq_chn_name_, MAX_BUF_LEN, "/mq_%d_%p_%d_%d", gpio, this, getpid, syscall(SYS_gettid));
        mqd_ = mq_open(irq_chn_name_, O_RDONLY | O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO, 0);
        if (mqd_ < 0) {
            ROS_ERROR_NAMED(NAME, "irq monitor mq open error: %s\n", strerror(errno));
        }
    }
    ~IrqMonitor()
    {
        if ((0 != mq_unlink(irq_chn_name_)) || (errno != ENOENT)) {
            ROS_ERROR_NAMED(NAME, "irq monitor mq unlink error: %s\n", strerror(errno));
        }

        if ((-1 != mqd_) && (0 != mq_close(mqd_))) {
            ROS_ERROR_NAMED(NAME, "irq monitor mq close error: %s\n", strerror(errno));
        }
    }

    virtual void addIrqHandler(HwCommGPIOIrqHandler& handler)
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        for (IrqHandlers::const_iterator cit = irq_handler_list_.begin();
             cit != irq_handler_list_.end();
             ++cit) {
            if (*cit == &handler) {
                return;
            }
        }
        irq_handler_list_.push_back(&handler);
        if (!monitor_entity_.get()) {
            monitor_entity_.reset(new MonitorEntity(gpio_, irq_chn_name_));
        }
    }

    virtual void delIrqHandler(HwCommGPIOIrqHandler& handler)
    {
        boost::lock_guard<boost::mutex> lock(mutex_);
        for (IrqHandlers::iterator it = irq_handler_list_.begin();
             it != irq_handler_list_.end();
             ++it) {
            if (*it == &handler) {
                if (1 == irq_handler_list_.size()) {
                    monitor_entity_.reset();
                }
                irq_handler_list_.erase(it);
                return;
            }
        }
    }

    virtual void waitIrq()
    {
        char msg_ptr[MAX_BUF_LEN] = {0};
        // TODO set proper msg_len
        uint32_t msg_len(MAX_BUF_LEN*1024);
        // TODO make it can be stopped
        while (1) {
            if (mq_receive(mqd_, msg_ptr, msg_len, 0) < 0) {
                ROS_ERROR_NAMED(NAME, "irq monitor mq receive error: %s\n", strerror(errno));
                continue;
            }
            {
                boost::lock_guard<boost::mutex> lock(mutex_);
                for (IrqHandlers::const_iterator cit = irq_handler_list_.begin();
                     cit != irq_handler_list_.end();
                     ++cit) {
                    (*cit)->handleIrq();
                }
            }
        }
    }

private:
    IrqMonitor(const IrqMonitor& other);
    IrqMonitor& operator=(const IrqMonitor& other);

    const uint32_t& gpio_;
    typedef std::vector<HwCommGPIOIrqHandler*> IrqHandlers;
    IrqHandlers irq_handler_list_;

    class MonitorEntity
    {
    public:
        MonitorEntity(const uint32_t& gpio, const char* irq_chn_name)
            : gpio_(gpio),
              thread_(),
              loop_(true),
              entity_mqd_()
        {
            entity_mqd_ = mq_open(irq_chn_name, O_WRONLY | O_CREAT, S_IRWXU | S_IRWXG | S_IRWXO, 0);
            if (entity_mqd_ < 0) {
                ROS_ERROR_NAMED(NAME, "irq monitor entity mq open error: %s\n", strerror(errno));
            }
            thread_ = boost::thread(&MonitorEntity::threadFunc, this);
        }
        ~MonitorEntity()
        {
            // TODO 'loop_' is not thread safe
            loop_ = false;
            thread_.join();

            if ((-1 != entity_mqd_) && (0 != mq_close(entity_mqd_))) {
                ROS_ERROR_NAMED(NAME, "irq monitor entity mq close error: %s\n", strerror(errno));
            }
        }

        void threadFunc()
        {
            // TODO impl epoll
            int32_t fd = doOpen(gpio_, "/value", O_RDONLY | O_NONBLOCK);
            if (fd < 0) {
                ROS_ERROR_NAMED(NAME, "monitor entity open gpio %d value error: %s\n", gpio_, strerror(errno));
                return;
            }

            struct pollfd fdset[1];
            memset(fdset, 0, sizeof(fdset));

            char val('0');
            while (loop_) {
                fdset[0].fd = fd;
                fdset[0].events = POLLPRI;

                if (poll(fdset, 1, -1) < 0) {
                    ROS_ERROR_NAMED(NAME, "monitor entity poll error: %s\n", gpio_, strerror(errno));
                    continue;
                }

                if (fdset[0].revents & POLLPRI) {
                    lseek(fdset[0].fd, 0, SEEK_SET);
                    if (read(fdset[0].fd, &val, 1) < 0) {
                        ROS_ERROR_NAMED(NAME, "monitor entity read gpio %d value error: %s\n", gpio_, strerror(errno));
                        continue;
                    }

                    if (mq_send(entity_mqd_, &val, 1, 0) < 0) {
                        ROS_ERROR_NAMED(NAME, "monitor entity mq send error: %s\n", strerror(errno));
                    }
                }
            }

            close(fd);
        }
    private:
        const uint32_t& gpio_;
        boost::thread thread_;
        bool loop_;
        mqd_t entity_mqd_;
    };
    std::auto_ptr<MonitorEntity> monitor_entity_;
    boost::mutex mutex_;
    char irq_chn_name_[MAX_BUF_LEN];
    mqd_t mqd_;
};

HwCommGPIO::HwCommGPIO(const uint32_t& gpio)
    : gpio_(gpio),
      irq_monitor_(new IrqMonitor(gpio))
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

void HwCommGPIO::addIrqHandler(HwCommGPIOIrqHandler& handler)
{
    irq_monitor_->addIrqHandler(handler);
}

void HwCommGPIO::delIrqHandler(HwCommGPIOIrqHandler& handler)
{
    irq_monitor_->delIrqHandler(handler);
}

void HwCommGPIO::waitIrq()
{
    irq_monitor_->waitIrq();
}

} // namespace gpio
} // namespace hw_comm
