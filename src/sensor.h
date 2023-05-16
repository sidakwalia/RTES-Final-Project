/**
 * @file sensor.h
 * 
 * @author  Group 12 : Jack Shkifati, Ishan U Taldekar, James Huang, and Inder Preet Walia
 * 
 * @brief  This code is a part of the sensor setup and handling for a gyroscope sensor. 
 * It configures the SPI interface used for communication with the sensor and sets up the 
 * registers of the sensor. It also includes two callback functions, one for the completion of 
 * SPI transfers and one for data readiness in the sensor.
 * 
 * @version 0.1
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "config.h"
#include "mbed.h"



// MOSI, MISO, SCK, CS pins are defined for the SPI interface
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); 

InterruptIn int2(PA_2, PullDown);

// EventFlags object used to handle synchronization events between threads
EventFlags flags;

// Buffers used for data transfer via SPI
uint8_t write_buf[32];
uint8_t read_buf[32];



/**
 * spi callback function:
 * This function is called when the SPI transfer is complete.
 * It sets the SPI_FLAG in the EventFlags object.
 */
void spi_cb(int event){

    flags.set(SPI_FLAG);
};

/**
 * Data callback function:
 * This function is called when the INT2 pin goes high (data is ready in the gyro sensor).
 * It sets the DATA_READY_FLAG in the EventFlags object.
 */
void data_cb(){

    flags.set(DATA_READY_FLAG);
};

/**
 * Function: setupGyro
 * This function is used to set up the gyroscope sensor.
 * It configures the SPI interface and the sensor registers.
 * It also sets up an interrupt to be called when data is ready.
 */
void setupGyro(){

    // Setup the spi for 8 bit data, high steady state clock,
    // second edge capture, with a 1MHz clock rate
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure the registers
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    // Configure the output data register
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    // configure the interrupt to call our function
    // when the pin becomes high
    int2.rise(&data_cb);
    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
    flags.wait_all(SPI_FLAG);

    // The gyroscope sensor keeps its configuration between power cycles.
    // This means that the gyroscope will already have it's data-ready interrupt
    // configured when we turn the board on the second time. This can lead to
    // the pin level rising before we have configured our interrupt handler.
    // To account for this, we manually check the signal and set the flag
    // for the first sample.
    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1))
    {
        flags.set(DATA_READY_FLAG);
    }
}


#endif // SENSOR_H
