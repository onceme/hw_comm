#include <memory>
#include <cstdio>
#include <unistd.h>
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
    usleep(5 * 1000);
    i2c.writeByte(MPU6050_ADDR, 0x6B, 0x03);
    i2c.writeByte(MPU6050_ADDR, 0x19, 0x07);
    i2c.writeByte(MPU6050_ADDR, 0x1A, 0x06);
    i2c.writeByte(MPU6050_ADDR, 0x1B, 0x18);
    i2c.writeByte(MPU6050_ADDR, 0x37, 0x02);
    usleep(500 * 1000);

    i2c.writeByte(HMC5883L_ADDR, 0x00, 0x010 + 0x01);
    i2c.writeByte(HMC5883L_ADDR, 0x01, 0x02 << 0x05);
    i2c.writeByte(HMC5883L_ADDR, 0x02, 0x01);
    usleep(1000 * 1000);
    i2c.writeByte(HMC5883L_ADDR, 0x02, 0x01);
    i2c.writeByte(HMC5883L_ADDR, 0x00, 0x010 + 0x02);
    i2c.writeByte(HMC5883L_ADDR, 0x02, 0x01);
    usleep(1000 * 1000);
    i2c.writeByte(HMC5883L_ADDR, 0x00, 0x70);
    i2c.writeByte(HMC5883L_ADDR, 0x01, 0x20);
    i2c.writeByte(HMC5883L_ADDR, 0x02, 0x00);

    i2c.writeByte(MPU6050_ADDR, 0x1C, 0x10);
    i2c.writeByte(MPU6050_ADDR, 0x6A, 0b00100000);
    i2c.writeByte(MPU6050_ADDR, 0x37, 0b00000000);
    i2c.writeByte(MPU6050_ADDR, 0x38, 0b00000001);
    i2c.writeByte(MPU6050_ADDR, 0x24, 0x0D);
    i2c.writeByte(MPU6050_ADDR, 0x25, 0x80 | HMC5883L_ADDR);
    i2c.writeByte(MPU6050_ADDR, 0x26, 0x03);
    i2c.writeByte(MPU6050_ADDR, 0x27, 0x86);

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
