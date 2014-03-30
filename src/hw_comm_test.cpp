#include <memory>
#include <cstdio>
#include <ros/types.h>
#include "hw_comm/gpio.h"
#include "hw_comm/i2c.h"

struct Handler : public hw_comm::gpio::HwCommGPIOIrqHandler
{
    virtual void handleIrq()
    {
        fprintf(stderr, "hello\n");
    }
};

int main(int argc, char** argv)
{
    hw_comm::gpio::HwCommGPIO gpio(1);
    Handler t;
    gpio.addIrqHandler(t);
    gpio.waitIrq();

    hw_comm::i2c::HwCommI2C i2c("name");
    return 0;
}
