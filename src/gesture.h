/**
 * @file gesture.h
 * 
 * @author  Group 12 : Jack Shkifati, Ishan U Taldekar, James Huang, and Inder Preet Walia
 * 
 * @brief The "gesture.h" header file provides functionality for recording, comparing, and determining the 
 * similarity between gestures. These gestures are captured by a gyroscope sensor and processed using 
 * a complementary filter and the Dynamic Time Warping (DTW) algorithm.
 * 
 * @version 0.1
 * 
 */
#ifndef GESTURE_H
#define GESTURE_H

#include "config.h"
#include "dtw.h"

// Global system flags variable
extern volatile SystemFlags system_flags;

// Data for each axis of the gyroscope
AxisData record_gyro_x_data, record_gyro_y_data, record_gyro_z_data;
AxisData unlock_gyro_x_data, unlock_gyro_y_data, unlock_gyro_z_data;

DtwDistance dtw_distance;

// Length of the recorded sequence
int seq_length = 0;

// Length of the sequence to compare
int compare_length = 0;

// Function prototype declarations
float complementaryFilter(float gyro_value, float accel_angle, float alpha);
void recordGesture();
void compareGesture();


/**
 * @brief Function to apply the complementary filter equation
 * 
 * @param gyro_value The gyroscope data
 * @param accel_angle The accelerometer angle
 * @param alpha The filter coefficient
 * 
 * @return float The filtered angle
 */
void complementaryFilter(float gyro_value, float accel_angle)
{
    accel_angle = ALPHA * gyro_value + (1 - ALPHA) * accel_angle;
}


/**
 * @brief Function to record a gesture
 * This function reads the data from the gyroscope, applies the complementary filter to the raw data,
 * and stores the filtered data in a sequence.
 */
void readGesture(Mode current_mode) {

    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    write_buf[0] = OUT_X_L | 0x80 | 0x40;

    int current_seq_length = 0;

    AxisData *gyro_x_data;
    AxisData *gyro_y_data;
    AxisData *gyro_z_data;

    if (current_mode == Mode::RECORD) {

        gyro_x_data = &record_gyro_x_data;
        gyro_y_data = &record_gyro_y_data;
        gyro_z_data = &record_gyro_z_data;

    } else {

        gyro_x_data = &unlock_gyro_x_data;
        gyro_y_data = &unlock_gyro_y_data;
        gyro_z_data = &unlock_gyro_z_data;

    }

    gyro_x_data->filtered_angle = 0;
    gyro_y_data->filtered_angle = 0;
    gyro_z_data->filtered_angle = 0;
    
    while (system_flags.record && current_seq_length < MAX_SEQUENCE_LENGTH)
    {

        flags.wait_all(DATA_READY_FLAG);
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb, SPI_EVENT_COMPLETE);
        flags.wait_all(SPI_FLAG);

        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = (((float)raw_gx) * SCALING_FACTOR);
        gy = (((float)raw_gy) * SCALING_FACTOR);
        gz = (((float)raw_gz) * SCALING_FACTOR);

        complementaryFilter(gx, gyro_x_data->filtered_angle);
        complementaryFilter(gy, gyro_y_data->filtered_angle);
        complementaryFilter(gz, gyro_z_data->filtered_angle);

        gyro_x_data->sequence[seq_length] = gyro_x_data->filtered_angle;
        gyro_y_data->sequence[seq_length] = gyro_y_data->filtered_angle;
        gyro_z_data->sequence[seq_length] = gyro_z_data->filtered_angle;

        current_seq_length++;

    }

    if (current_mode == Mode::UNLOCK) {

        dtw_distance.x = abs(DTW(record_gyro_x_data.sequence, unlock_gyro_x_data.sequence, seq_length));
        dtw_distance.y = abs(DTW(record_gyro_y_data.sequence, unlock_gyro_y_data.sequence, seq_length));
        dtw_distance.z = abs(DTW(record_gyro_x_data.sequence, unlock_gyro_x_data.sequence, seq_length));

        gesture_match = dtw_distance.x < DTW_THRESHOLD && dtw_distance.y < DTW_THRESHOLD && dtw_distance.z < DTW_THRESHOLD;

    } else {

        seq_length = current_seq_length;

    }

}


#endif // GESTURE_H