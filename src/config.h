/**
 * @file config.h
 * 
 * @author  Group 12 : Jack Shkifati, Ishan U Taldekar, James Huang, and Inder Preet Walia
 * 
 * @brief  provides configurations, constants, and data structures necessary for the operation of a 
 * gyroscope-based gesture recognition system.
 * 
 * @version 0.1
 * 
 */
#ifndef CONFIG_H
#define CONFIG_H

//------------------------------------ Configuration Registers ------------------------------------

/**
  * Data sheet table 22
  * register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)
  * ODR : The rate at which sensor provides data
  * Cutoff: Filter out high frequency
  * configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
*/
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

#include <stdbool.h>

/*
  * READY Inturrput configuration
  * register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), 
    I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
  * configuration: Int1 disabled, Boot status disabled, active high interrupts, 
    push-pull, enable Int2 data ready, disable fifo interrupts
*/
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

/**
  
  * register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),
    self-test(2), SPI mode(1)

  * Controls how fast you want to measure degrees per second : 
    (00: 245 dps, 01: 500 dps, 10: 2000dps, 11: 2000 dps)

  * Configuration: reserved, little endian, 500 dps, reserved, disabled, 4-wire mode
*/
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0


// By setting the MSB to 1 we can auto increment through the output data from 0x28 to 0x2D (X: Z)
#define OUT_X_L 0x28


// ------------------------------------End of Configration Registers -------------------------------

// SPI flag when complete
#define SPI_FLAG 1     

// Data ready flag when next data is values of data are ready to be read.
#define DATA_READY_FLAG 2

/** 
 * SCALING_FACTOR is used to convert raw gyro sensor data into rad/s. 
 * It is calculated by multiplying the sensitivity of the gyro sensor (in this case, 17.5 when set 
   to +/- 2000 degrees per second) by a conversion factor to turn degrees into radians (pi/180), 
    and then dividing by 1000 to turn milli-units into units.
*/ 
#define SCALING_FACTOR (17.5f / 1000.0f)

// the maximum length of the gesture sequence that can be recorded and analyzed. 
#define MAX_SEQUENCE_LENGTH 100

 /**
  * the threshold used in the dynamic time warping algorithm (DTW). 
  * If the DTW distance between the recorded gesture and a pre-recorded gesture is less than this 
    threshold, it is considered a match. This value can be tuned to adjust the sensitivity of the 
    gesture recognition.
*/
#define DTW_THRESHOLD 2

#define ALPHA 0.75f  // Filter coefficient



/**
 * Struct: SystemFlags
 * -------------------
 * This struct is used to hold various boolean flags related to the system state.
 * Each member is a bit field that uses only 1 bit of storage.
 *
 * isRecording: Indicates whether the system is currently in a recording state.
 *
 * isReading: Indicates whether the system is currently in a reading state.
 *
 * stop: Indicates whether the system needs to stop its current operation.
 *
 * record: Indicates whether the system is ready to start recording.
 *
 * compare: Indicates whether the system needs to compare the recorded gesture with the stored one.
 *
 * complete: Indicates whether the gesture comparison is complete.
 */
typedef struct {
    bool isRecording;
    bool isReading;
    bool stop;
    bool record;
    bool compare;
    bool complete;
} SystemFlags;

/**
 * Struct: AxisData
 * ----------------
 * This struct is used to hold various data related to an axis of the gyroscope.
 *
 * sequence: An array to hold the recorded gesture sequence for the particular axis.
 *
 * compare_sequence: An array to hold the gesture sequence to be compared with.
 *
 * dtw_distance: A floating point variable to hold the Dynamic Time Warping distance value.
 *
 * filtered_angle: A floating point variable to hold the filtered angle data of the particular axis.
 */
typedef struct {
    float sequence[MAX_SEQUENCE_LENGTH];
    float filtered_angle;
} AxisData;

enum Mode {

  RECORD = 1,
  UNLOCK

};

typedef struct {

  float x;
  float y;
  float z;

} DtwDistance;

bool gesture_match = true;

#endif // CONFIG_H
