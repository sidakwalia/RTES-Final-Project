#ifndef I3G4250D_H
#define I3G4250D_H

#include "mbed.h"

class I3G4250D {
public:
    I3G4250D(I2C &i2c);
    bool init();
    bool read_gyro(int16_t &x, int16_t &y, int16_t &z);

private:
    I2C &_i2c;
    static const uint8_t I3G4250D_ADDR = 0x68 << 1;
};

#endif // I3G4250D_H
