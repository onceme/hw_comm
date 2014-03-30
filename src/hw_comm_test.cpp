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
    fprintf(stderr, "hello1\n");
    hw_comm::gpio::HwCommGPIO gpio(17);
    fprintf(stderr, "hello2\n");
    gpio.setDirection(hw_comm::gpio::GPIO_DIRE_IN);
    fprintf(stderr, "hello3\n");
    gpio.setEdge(hw_comm::gpio::GPIO_EDGE_BOTH);
    fprintf(stderr, "hello4\n");
    Handler t;
    gpio.addIrqHandler(t);
    fprintf(stderr, "hello5\n");
    gpio.waitIrq();
    fprintf(stderr, "hello6\n");

    //    hw_comm::i2c::HwCommI2C i2c("name");
    fprintf(stderr, "hello7\n");
    return 0;
}
