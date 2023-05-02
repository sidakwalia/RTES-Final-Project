#include "mbed.h"
#include "I3G4250D.h"

I2C i2c(PB_9, PB_8); // SDA, SCL
DigitalOut led_green(PG_13);

I3G4250D gyro(i2c);

int main() {
    if (!gyro.init()) {
        // Handle the error (e.g., turn on a red LED)
    }

    int16_t x, y, z;
    uint8_t sequence_step = 0;
    int16_t threshold = 2000;
while (true) {
    if (gyro.read_gyro(x, y, z)) {
        // Check the hand movement sequence
        switch (sequence_step) {
            case 0:
                if (x > threshold) {
                    sequence_step++;
                }
                break;
            case 1:
                if (y > threshold) {
                    sequence_step++;
                }
                break;
            case 2:
                if (z > threshold) {
                    sequence_step = 0;
                    // Successful unlock: Toggle the green LED
                    led_green = !led_green;
                }
                break;
            default:
                sequence_step = 0;
                break;
        }
    }

    ThisThread::sleep_for(100ms);
}
}
// This code initializes the I3G4250D gyroscope, reads its data in a loop, and checks if a specific hand movement sequence is detected (X > threshold, Y > threshold, Z > threshold). If the sequence is detected, the green LED on the STM32F429I-Discovery board toggles, indicating a successful "unlock."

// You can modify the code to implement different hand movement sequences and corresponding actions based on your project requirements.

