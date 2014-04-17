#include <memory>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <ros/types.h>
#include "hw_comm/gpio.h"
#include "hw_comm/i2c.h"

#define MPU6050_ADDR  0x68
#define HMC5883L_ADDR 0x1E
#define MS5611_ADDR   0x77

#define BUF_SIZE 3

struct Handler : public hw_comm::gpio::HwCommGPIOIrqHandler
{
    Handler(hw_comm::i2c::HwCommI2C& i2c_dev)
        : i2c(i2c_dev)
    {}

    hw_comm::i2c::HwCommI2C& i2c;

    virtual void handleIrq()
    {
        fprintf(stderr, "=====================================\n");

        uint8_t a_x_h, a_x_l;
        uint8_t a_y_h, a_y_l;
        uint8_t a_z_h, a_z_l;
        a_x_h = i2c.readByteData(MPU6050_ADDR, 0x3B);
        a_x_l = i2c.readByteData(MPU6050_ADDR, 0x3C);
        a_y_h = i2c.readByteData(MPU6050_ADDR, 0x3D);
        a_y_l = i2c.readByteData(MPU6050_ADDR, 0x3E);
        a_z_h = i2c.readByteData(MPU6050_ADDR, 0x3E);
        a_z_l = i2c.readByteData(MPU6050_ADDR, 0x3F);
#if 1
        uint16_t ax = a_x_h;
        ax = (ax << 8) | a_x_l;
        uint16_t ay = a_y_h;
        ay = (ay << 8) | a_y_l;
        uint16_t az = a_z_h;
        az = (az << 8) | a_z_l;
        fprintf(stderr, "%d, %d, %d\n", ax, ay, az);
#else
        fprintf(stderr, "%d, %d, %d, %d, %d, %d\n", a_x_h, a_x_l, a_y_h, a_y_l, a_z_h, a_z_l);
#endif

        uint8_t g_x_h, g_x_l;
        uint8_t g_y_h, g_y_l;
        uint8_t g_z_h, g_z_l;
        g_x_h = i2c.readByteData(MPU6050_ADDR, 0x43);
        g_x_l = i2c.readByteData(MPU6050_ADDR, 0x44);
        g_y_h = i2c.readByteData(MPU6050_ADDR, 0x45);
        g_y_l = i2c.readByteData(MPU6050_ADDR, 0x46);
        g_z_h = i2c.readByteData(MPU6050_ADDR, 0x47);
        g_z_l = i2c.readByteData(MPU6050_ADDR, 0x48);
#if 1
        uint16_t gx = g_x_h;
        gx = (gx << 8) | g_x_l;
        uint16_t gy = g_y_h;
        gy = (gy << 8) | g_y_l;
        uint16_t gz = g_z_h;
        gz = (gz << 8) | g_z_l;
        fprintf(stderr, "%d, %d, %d\n", gx, gy, gz);
#else
        fprintf(stderr, "%d, %d, %d, %d, %d, %d\n", g_x_h, g_x_l, g_y_h, g_y_l, g_z_h, g_z_l);
#endif

        uint8_t m_x_h, m_x_l;
        uint8_t m_y_h, m_y_l;
        uint8_t m_z_h, m_z_l;
        m_x_h = i2c.readByteData(MS5611_ADDR, 0x49);
        m_x_l = i2c.readByteData(MS5611_ADDR, 0x4A);
        m_y_h = i2c.readByteData(MS5611_ADDR, 0x4E);
        m_y_l = i2c.readByteData(MS5611_ADDR, 0x4F);
        m_z_h = i2c.readByteData(MS5611_ADDR, 0x4C);
        m_z_l = i2c.readByteData(MS5611_ADDR, 0x4D);
        fprintf(stderr, "%d, %d, %d, %d, %d, %d\n", m_x_h, m_x_l, m_y_h, m_y_l, m_z_h, m_z_l);

        uint8_t values[BUF_SIZE] = {0};

        uint32_t D1(0);
        uint32_t D2(0);
        i2c.writeByte(MS5611_ADDR, 0x48);
        i2c.readBlockData(MS5611_ADDR, 0x00, values);
        D1 = (values[0] << 16) | (values[1] << 8) | values[2];

        memset(values, 0, BUF_SIZE);
        i2c.writeByte(MS5611_ADDR, 0x58);
        i2c.readBlockData(MS5611_ADDR, 0x00, values);
        D2 = (values[0] << 16) | (values[1] << 8) | values[2];

        uint16_t C1(0);
        uint16_t C2(0);
        uint16_t C3(0);
        uint16_t C4(0);
        uint16_t C5(0);
        uint16_t C6(0);

        memset(values, 0, BUF_SIZE);
        i2c.readBlockData(MS5611_ADDR, 0xA2, values);
        C1 = (values[0] << 8) | values[1];

        memset(values, 0, BUF_SIZE);
        i2c.readBlockData(MS5611_ADDR, 0xA4, values);
        C2 = (values[0] << 8) | values[1];

        memset(values, 0, BUF_SIZE);
        i2c.readBlockData(MS5611_ADDR, 0xA6, values);
        C3 = (values[0] << 8) | values[1];

        memset(values, 0, BUF_SIZE);
        i2c.readBlockData(MS5611_ADDR, 0xA8, values);
        C4 = (values[0] << 8) | values[1];

        memset(values, 0, BUF_SIZE);
        i2c.readBlockData(MS5611_ADDR, 0xAA, values);
        C5 = (values[0] << 8) | values[1];

        memset(values, 0, BUF_SIZE);
        i2c.readBlockData(MS5611_ADDR, 0xAC, values);
        C6 = (values[0] << 8) | values[1];

        int32_t dT = D2 - C5 * 256;
        int32_t TEMP = 2000 + dT * C6 / 8388608;
        int64_t OFF = C2 * 65536 + (C4 * dT) / 128;
        int64_t SENS = C1 * 32768 + (C3 * dT) / 256;
        int32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

        fprintf(stderr, "Temperature: %f, Temperature F: %.2f, Pressure: %f, Pressure Adjusted: %.2f\n",
                TEMP / 100.0,
                TEMP / 100.0 * 9.0 / 5 + 32,
                P / 100.0,
                P / 100.0 + 55.5);
    }
};

int main(int argc, char** argv)
{
    hw_comm::i2c::HwCommI2C i2c("/dev/i2c-1");
    fprintf(stderr, "hello0\n");

    i2c.writeByteData(MPU6050_ADDR, 0x6B, 0x80);
    usleep(5 * 1000);
    i2c.writeByteData(MPU6050_ADDR, 0x6B, 0x03);
    i2c.writeByteData(MPU6050_ADDR, 0x19, 0x07);
    i2c.writeByteData(MPU6050_ADDR, 0x1A, 0x06);
    i2c.writeByteData(MPU6050_ADDR, 0x1B, 0x18);
    i2c.writeByteData(MPU6050_ADDR, 0x37, 0x02);
    usleep(500 * 1000);

    i2c.writeByteData(HMC5883L_ADDR, 0x00, 0x010 + 0x01);
    i2c.writeByteData(HMC5883L_ADDR, 0x01, 0x02 << 0x05);
    i2c.writeByteData(HMC5883L_ADDR, 0x02, 0x01);
    usleep(1000 * 1000);
    i2c.writeByteData(HMC5883L_ADDR, 0x02, 0x01);
    i2c.writeByteData(HMC5883L_ADDR, 0x00, 0x010 + 0x02);
    i2c.writeByteData(HMC5883L_ADDR, 0x02, 0x01);
    usleep(1000 * 1000);
    i2c.writeByteData(HMC5883L_ADDR, 0x00, 0x70);
    i2c.writeByteData(HMC5883L_ADDR, 0x01, 0x20);
    i2c.writeByteData(HMC5883L_ADDR, 0x02, 0x00);

    i2c.writeByteData(MPU6050_ADDR, 0x1C, 0x10);
    i2c.writeByteData(MPU6050_ADDR, 0x6A, 0b00100000);
    i2c.writeByteData(MPU6050_ADDR, 0x37, 0b00000000);
    i2c.writeByteData(MPU6050_ADDR, 0x38, 0b00000001);
    i2c.writeByteData(MPU6050_ADDR, 0x24, 0x0D);
    i2c.writeByteData(MPU6050_ADDR, 0x25, 0x80 | HMC5883L_ADDR);
    i2c.writeByteData(MPU6050_ADDR, 0x26, 0x03);
    i2c.writeByteData(MPU6050_ADDR, 0x27, 0x86);

    i2c.writeByte(MS5611_ADDR, 0x1E);

    fprintf(stderr, "hello1\n");
    hw_comm::gpio::HwCommGPIO gpio(4);
    fprintf(stderr, "hello2\n");
    gpio.setDirection(hw_comm::gpio::GPIO_DIRE_IN);
    fprintf(stderr, "hello3\n");
    gpio.setEdge(hw_comm::gpio::GPIO_EDGE_BOTH);
    fprintf(stderr, "hello4\n");
    Handler t(i2c);
    gpio.addIrqHandler(t);
    fprintf(stderr, "hello5\n");
    gpio.waitIrq();
    fprintf(stderr, "hello6\n");

    return 0;
}
