#include "i3g4250d.h"

#define I3G4250D_ADDR (0x68 << 1)
#define I3G4250D_WHO_AM_I 0x0F
#define I3G4250D_CTRL_REG1 0x20


HAL_StatusTypeDef I3G4250D_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // Check the WHO_AM_I register
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, I3G4250D_ADDR, I3G4250D_WHO_AM_I, 1, &data, 1, 100);
    if (status != HAL_OK || data != 0xD3) {
        return HAL_ERROR;
    }

    // Enable the gyroscope, set the output data rate to 95 Hz, and bandwidth to 12.5
    data = 0x0F;


}

I3G4250D::I3G4250D(I2C &i2c) : _i2c(i2c) {}

bool I3G4250D::init() {
    char data[2];

    // Check the WHO_AM_I register
    data[0] = 0x0F;
    _i2c.write(I3G4250D_ADDR, data, 1);
    _i2c.read(I3G4250D_ADDR, data, 1);

    if (data[0] != 0xD3) {
        return false;
    }

    // Enable the gyroscope, set the output data rate to 95 Hz, and bandwidth to 12.5
    data[0] = 0x20;
    data[1] = 0x0F;
    return _i2c.write(I3G4250D_ADDR, data, 2) == 0;
}

bool I3G4250D::read_gyro(int16_t &x, int16_t &y, int16_t &z) {
    char buffer[6];
    buffer[0] = 0x28 | 0x80;

    if (_i2c.write(I3G4250D_ADDR, buffer, 1) != 0) {
        return false;
    }

    if (_i2c.read(I3G4250D_ADDR, buffer, 6) != 0) {
        return false;
    }

    // Combine the high and low bytes to get the 16-bit values
    x = (int16_t)(buffer[1] << 8 | buffer[0]);
    y = (int16_t)(buffer[3] << 8 | buffer[2]);
    z = (int16_t)(buffer[5] << 8 | buffer[4]);

    return true;
}


