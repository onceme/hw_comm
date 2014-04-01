#include <memory>
#include <cstdio>
#include <ros/types.h>
#include "hw_comm/gpio.h"
#include "hw_comm/i2c.h"

#define MPU6050_ADDR  0x68
#define HMC5883L_ADDR 0x1E
#define MS6511_ADDR   0x77

struct Handler : public hw_comm::gpio::HwCommGPIOIrqHandler
{
    virtual void handleIrq()
    {
        fprintf(stderr, "hello\n");
    }
};

int main(int argc, char** argv)
{
    hw_comm::i2c::HwCommI2C i2c("/dev/i2c-1");
    fprintf(stderr, "hello0\n");

    i2c.writeByte(MPU6050_ADDR, 0x6B, 0x80);

    fprintf(stderr, "hello1\n");
    hw_comm::gpio::HwCommGPIO gpio(4);
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

    return 0;
}
