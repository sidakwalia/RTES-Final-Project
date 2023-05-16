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
AxisData gyroXdata, gyroYdata, gyroZdata;

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
 * @param gyro_value The gyroscope data
 * @param accel_angle The accelerometer angle
 * @param alpha The filter coefficient
 * @return float The filtered angle
 */
float complementaryFilter(float gyro_value, float accel_angle, float alpha)
{
    return alpha * gyro_value + (1 - alpha) * accel_angle;
}


/**
 * @brief Function to record a gesture
 * This function reads the data from the gyroscope, applies the complementary filter to the raw data,
 * and stores the filtered data in a sequence.
 */
void recordGesture() {

    mutex.lock();

    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    write_buf[0] = OUT_X_L | 0x80 | 0x40;

    while (system_flags.record && seq_length < MAX_SEQUENCE_LENGTH)
    {

        flags.wait_all(DATA_READY_FLAG);
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb, SPI_EVENT_COMPLETE);
        flags.wait_all(SPI_FLAG);

        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = (((float)raw_gx) * 0.0001 * 180) / 3.14159;
        gy = (((float)raw_gy) * 0.0001 * 180) / 3.14159;
        gz = (((float)raw_gz) * 0.0001 * 180) / 3.14159;

        gyroXdata.filtered_angle = complementaryFilter(gx, gyroXdata.filtered_angle, ALPHA);
        gyroYdata.filtered_angle= complementaryFilter(gy,  gyroYdata.filtered_angle, ALPHA);
        gyroZdata.filtered_angle = complementaryFilter(gz,  gyroZdata.filtered_angle , ALPHA);

        gyroXdata.sequence[seq_length] =  gyroXdata.filtered_angle;
        gyroYdata.sequence[seq_length] =  gyroYdata.filtered_angle;
        gyroZdata.sequence[seq_length] = gyroZdata.filtered_angle;

        seq_length++;

    }
}

/**
 * @brief Function to compare a gesture
 * This function reads the data from the gyroscope, applies the complementary filter to the raw data,
 * and stores the filtered data in a sequence to compare with a previously recorded gesture.
 * Then it uses the Dynamic Time Warping (DTW) algorithm to calculate the similarity between the recorded and the compared sequences.
 */
void compareGesture(){

    mutex.lock();

    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;

    write_buf[0] = OUT_X_L | 0x80 | 0x40;

    compare_length = 0;

    while (system_flags.compare && compare_length < MAX_SEQUENCE_LENGTH)
    {
        flags.wait_all(DATA_READY_FLAG);
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb, SPI_EVENT_COMPLETE);
        flags.wait_all(SPI_FLAG);

        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

        gx = (((float)raw_gx) * 0.0001 * 180) / 3.14159;
        gy = (((float)raw_gy) * 0.0001 * 180) / 3.14159;
        gz = (((float)raw_gz) * 0.0001 * 180) / 3.14159;

        gyroXdata.filtered_angle = complementaryFilter(gx, gyroXdata.filtered_angle, ALPHA);
        gyroYdata.filtered_angle= complementaryFilter(gy,  gyroYdata.filtered_angle, ALPHA);
        gyroZdata.filtered_angle = complementaryFilter(gz,  gyroZdata.filtered_angle , ALPHA);

       gyroXdata.compare_sequence[compare_length] = gyroXdata.filtered_angle;
       gyroYdata.compare_sequence[compare_length] =  gyroYdata.filtered_angle;
       gyroZdata.compare_sequence[compare_length] = gyroZdata.filtered_angle;

        compare_length++;
    }

    gyroXdata.dtw_distance= abs(DTW(gyroXdata.sequence, gyroXdata.sequence, seq_length));
    gyroYdata.dtw_distance= abs(DTW(gyroYdata.sequence, gyroYdata.compare_sequence, seq_length));
    gyroZdata.dtw_distance = abs(DTW(gyroZdata.sequence, gyroZdata.compare_sequence, seq_length));
    
    mutex.unlock();

}

/**
 * @brief Function to determine whether two sequences are similar based on the DTW distances
 * @return bool True if the sequences are similar, false otherwise
 */
bool isSimilar(){
   
    // printf("dtw_distance_x %f\n", dtw_distance_x);
    // printf("dtw_distance_y %f\n", dtw_distance_y);
    // printf("dtw_distance_z %f\n", dtw_distance_z);

    if (gyroXdata.dtw_distance < DTW_THRESHOLD && gyroYdata.dtw_distance < DTW_THRESHOLD && gyroZdata.dtw_distance < DTW_THRESHOLD)
    {
        return true; // The sequences are similar
    }
    else
    {
        return false; // The sequences are not similar
    }
}




#endif // GESTURE_H