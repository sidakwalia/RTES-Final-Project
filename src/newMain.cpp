/**
 * @file main.cpp
 * @author  Group 12 : Jack Shkifati, Ishan U Taldekar, James Huang, and Inder Preet Walia
 * @brief 
 * 
 * Overview:
 * This program is designed to read data from a gyroscope, interpret it as hand gestures, and act upon 
 * these gestures. It features two modes: the RECORD mode, where it learns a new gesture, and the 
 * UNLOCK mode, where it attempts to recognize a gesture. These modes can be switched between by 
 * pressing a button.
 * 
 * Functions:
 * 1.readGyro(Mode current_mode): This function reads raw data from a gyroscope sensor and converts 
 * it into angles. In RECORD mode, the function collects data and stores it as a new gesture. In 
 * UNLOCK mode, the function tries to match the input gesture with the stored gesture. If the input 
 * gesture matches the stored gesture, it turns on LED1. If not, it turns on LED2.
 * 
 * 2. processMeasurement(current_mode, gesture_match): This function is essential for the operation of 
 * the system. Presumably, it processes the new measurements from the gyroscope, comparing them with the 
 * stored gesture in UNLOCK mode or storing them as a new gesture in RECORD mode.
 * 
 * 3. blinkLEDs(): This function provides visual feedback to the user by blinking LEDs while the system 
 * is reading and processing gyroscope data. It is meant to indicate that the system is actively working 
 * on a task.
 * 
 * 4. blinkMode(Mode current_mode): This function provides visual feedback about the current operating 
 * mode of the system. In RECORD mode, the LED stays on for 3 seconds. In UNLOCK mode, the LED blinks 
 * three times. This function also starts the blinkLEDs() function in a new thread to give continuous 
 * feedback during the reading process.
 * 
 * 5. main(): This function is the entry point of the program. It sets up SPI communication and configures 
 * the gyroscope sensor. It also implements a loop that constantly checks the state of the button to switch
 * modes or start reading from the gyroscope. The function also handles button debouncing logic.
 * 
 * Usage:
 * 
 * 1. Start the program. The system will be in an idle state.
 * 
 * 2. Press and hold the button for 2 seconds. The LED will light up indicating that the system has 
 * entered RECORD mode.
 * 
 * 3. Release the button to leave RECORD mode. The recorded gesture will be stored.
 * 
 * 4. Press and release the button quickly (less than 2 seconds). The LED will blink indicating that the 
 * system is now in UNLOCK mode.
 * 
 * 5. Perform the gesture that you want the system to recognize. If the gesture matches the stored gesture, 
 * LED1 will light up. If not, LED2 will light up.
 * 
 * @version 0.1
 */

#include "mbed.h"
#include <iostream>


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


// ------------------------------------  Constants and Flags ----------------------------------------


// SPI flag when complete
#define SPI_FLAG 1  

// Data ready flag when next data is values of data are ready to be read.
#define DATA_READY_FLAG 2  


// number of seconds that gyroscope will sense gesture: 100 -> 5 seconds, 200 -> 10 seconds, ...
#define SENSING_TIMEFRAME 100


// Create an instance of the SystemFlags struct, declared as volatile because it can be modified by an ISR.
EventFlags flags;

/** The 'show_reading_indicator' boolean variable is used as a flag to indicate whether the system 
 * is actively reading and processing the gyroscope data. When it is true, it triggers the 
 * blinking of LEDs to indicate data processing.
*/
bool show_reading_indicator = false;

/**
 * The 'reading_led_indicator_thread' is a separate thread that is used to run the 'blinkLEDs()' 
 * function. This allows the LEDs to blink independently of the main program flow, providing continuous 
 * visual feedback to the user without interrupting the reading and processing of the gyroscope data.
 */
Thread reading_led_indicator_thread;


// timeframe after which stability of measurement in any axis is gauged + compared about these values
const int32_t STABILITY_TIMEFRAME = 5;  

// upper bound on the change in degree a value under which is considered to be holding stable
const float STABILITY_THRESHOLD = 1;  

// the difference in degrees around the actual value of measurement that still register as a match
const int32_t COMPARE_THRESHOLD = 20;  

/**
 * The 'DELTA_THRESHOLD' is a constant value used in the gesture comparison process. It represents the 
 * maximum permissible difference in the angles detected by the gyroscope for a gesture to be considered 
 * a match with a predefined gesture. The value is in degrees. If the total difference in angles across 
 * all axes exceeds this threshold, the gesture is not considered a match.
 */
const float DELTA_THRESHOLD = 30;  

bool button_pressed = false;  // button state
int button_hold_time = 0;  // button time



// ----------------------------------  Data Structues ----------------------------------------------------

// MOSI, MISO, SCK, CS pins are defined for the SPI interface
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs

InterruptIn int2(PA_2, PullDown);

DigitalIn button(USER_BUTTON); // Button input

/* Status LED outputs: */
DigitalOut led1(LED1); 
DigitalOut led2(LED2);



// Buffers used for data transfer via SPI
uint8_t write_buf[32];
uint8_t read_buf[32];


enum Mode {

  RECORD = 1,
  UNLOCK

};

// const int16_t POSITIVE_DELTA_WINDOW_SIZE = 100;

// The 'Data' structure holds the information related to the gyroscope readings.
struct Data {

 // 'delta_x', 'delta_y', and 'delta_z' represent the current changes in the x, y, and z axis respectively.
  // These values are updated whenever the direction of the angular velocity changes or when the angle remains stable.
  float delta_x = 0.0f;  
  float delta_y = 0.0f;  
  float delta_z = 0.0f;  

  // 'positive_delta_x', 'positive_delta_y', and 'positive_delta_z' are arrays that store the direction of the change
  // in each axis for each frame. If the change is in the positive direction, the corresponding element is set to 'true'
  bool positive_delta_x[SENSING_TIMEFRAME];  
  bool positive_delta_y[SENSING_TIMEFRAME];  
  bool positive_delta_z[SENSING_TIMEFRAME];  
  
// 'angles_x', 'angles_y', and 'angles_z' are arrays that store the angles (in degrees) for each axis for each frame.
  float angles_x[SENSING_TIMEFRAME];
  float angles_y[SENSING_TIMEFRAME];
  float angles_z[SENSING_TIMEFRAME];

  // 'angles_index' is used to index the 'angles_x', 'angles_y', and 'angles_z' arrays.
  // 'positive_delta_index' is used to index the 'positive_delta_x', 'positive_delta_y', and 'positive_delta_z' arrays.
  // 'filtered_positive_delta_index' is used to index the 'filtered_positive_delta_x', 'filtered_positive_delta_y',
  // and 'filtered_positive_delta_z' arrays.
  int32_t angles_index = 0;
  int32_t positive_delta_index = -1;
  int32_t filtered_positive_delta_index = 1;

} data;


struct NewMeasurement {

  float x = 0.0f;  // angle measurement in x direction
  float y = 0.0f;  // angle measurement in y direction
  float z = 0.0f;  // angle measurement in z direction

} new_measurement;

enum Axis {  // used for labelling the axes

  X = 1,
  Y,
  Z

};

struct Gesture {  // container for recording gesture as axis of major change and amount of change

  Axis axis[SENSING_TIMEFRAME];  // axis of major change
  float angle_change[SENSING_TIMEFRAME];  // amount of change

  int32_t next_index = 0;  // index where next measurement goes
  int32_t compare_index = 0;  // measurement to be comapred next

};

Gesture recorded_gesture;  // gesture stored



//---------------------------------------- Gyroscope Configrations  --------------------------------------

/**
 * spi callback function:
 * This function is called when the SPI transfer is complete.
 * It sets the SPI_FLAG in the EventFlags object.
 */
void spi_cb(int event) {

  flags.set(SPI_FLAG);

};


/**
 * Data callback function:
 * This function is called when the INT2 pin goes high (data is ready in the gyro sensor).
 * It sets the DATA_READY_FLAG in the EventFlags object.
 */
void data_cb() {
  
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




// -----------------------------------------  Read and Measure  ----------------------------------------------------------

/**
 * This function processes a single gyroscope measurement and updates the recorded gesture or compares it with a previously recorded gesture.
 * @param current_mode The current mode of operation. It can be either RECORD or UNLOCK.
 * @param gesture_match Reference to a boolean that indicates whether the current gesture matches the previously recorded gesture (if in UNLOCK mode).
 */
void processMeasurement(Mode current_mode, bool &gesture_match) {

  // These booleans are used to decide if we should consider the change in each axis
  bool compare_axis_deltas_x = false;
  bool compare_axis_deltas_y = false;
  bool compare_axis_deltas_z = false;

   // If the absolute change in the x-axis is greater than the threshold, and either the current and previous positive deltas are different,
  // or the change in angle over the stability timeframe is less than the threshold, then we should consider the change in the x-axis
  if (abs(data.delta_x) > DELTA_THRESHOLD && ((data.positive_delta_index > 0 && data.positive_delta_x[data.positive_delta_index] != data.positive_delta_x[data.positive_delta_index - 1]) || 
      (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_x[data.angles_index - STABILITY_TIMEFRAME] - data.angles_x[data.angles_index]) <= STABILITY_THRESHOLD))) {

        compare_axis_deltas_x = true;
      
  }

  // Same logic applies for the y-axis
  if (abs(data.delta_y) > DELTA_THRESHOLD && ((data.positive_delta_index > 0 && data.positive_delta_y[data.positive_delta_index] != data.positive_delta_y[data.positive_delta_index - 1]) || 
     (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_y[data.angles_index - STABILITY_TIMEFRAME] - data.angles_y[data.angles_index]) <= STABILITY_THRESHOLD))) {

      compare_axis_deltas_y = true;

  }

  // And for the z-axis
  if (abs(data.delta_z) > DELTA_THRESHOLD && ((data.positive_delta_index > 0 && data.positive_delta_z[data.positive_delta_index] != data.positive_delta_z[data.positive_delta_index - 1]) || 
     (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_z[data.angles_index - STABILITY_TIMEFRAME] - data.angles_z[data.angles_index]) <= STABILITY_THRESHOLD))) {

      compare_axis_deltas_z = true;

  }

  // If we are considering the change in the x-axis, and the change is greater than the threshold,
  // and the change in the x-axis is greater than in the y-axis and z-axis, then the x-axis dominates
  if (compare_axis_deltas_x && abs(data.delta_x) > DELTA_THRESHOLD && abs(data.delta_x) >= abs(data.delta_y) && abs(data.delta_x) >= abs(data.delta_z)) {

       // If we are in recording mode, then we record the gesture
      // Otherwise, we compare the gesture with a previously recorded one
      if (current_mode == Mode::RECORD) {

        recorded_gesture.axis[recorded_gesture.next_index] = Axis::X;
        recorded_gesture.angle_change[recorded_gesture.next_index] = data.delta_x;
        ++recorded_gesture.next_index;

      } else {

        // If the recorded gesture does not match, then we update the gesture_match flag
        if (!(recorded_gesture.axis[recorded_gesture.compare_index] == Axis::X && abs(recorded_gesture.angle_change[recorded_gesture.compare_index] - data.delta_x) <= COMPARE_THRESHOLD)) {

          gesture_match = false;

        }

        ++recorded_gesture.compare_index;

      }

      // Reset deltas
      data.delta_x = 0;
      data.delta_y = 0;
      data.delta_z = 0;

    }

  // Similar logic is applied


  // if (compare_axis_deltas_z && abs(change_in_z) > DELTA_THRESHOLD) {
  if (compare_axis_deltas_z && abs(data.delta_z) > DELTA_THRESHOLD) {

    if (abs(data.delta_z) >= abs(data.delta_x) && abs(data.delta_z) >= abs(data.delta_y)) {

      if (current_mode == Mode::RECORD) {

        recorded_gesture.axis[recorded_gesture.next_index] = Axis::Z;
        recorded_gesture.angle_change[recorded_gesture.next_index] = data.delta_z;
        ++recorded_gesture.next_index;

      } else {

        if (!(recorded_gesture.axis[recorded_gesture.compare_index] == Axis::Z && abs(recorded_gesture.angle_change[recorded_gesture.compare_index] - data.delta_z) <= COMPARE_THRESHOLD)) {

          gesture_match = false;

        }

        ++recorded_gesture.compare_index;

      }
      // resset
      data.delta_x = 0;
      data.delta_y = 0;
      data.delta_z = 0;

    }

  }

}


// This function reads gyroscopic data and handles gesture recording or comparison.
void readGyro(Mode current_mode) {

    // Button must be released to start the recording.
    while (button.read()) {
      thread_sleep_for(1);
    }

    // Declare raw gyroscopic data.
    int16_t raw_gx, raw_gy, raw_gz;

    // Initially assuming the gesture will match.
    bool gesture_match = true;

    // Resetting data.
    data.delta_x = 0;
    data.delta_y = 0;
    data.delta_z = 0;

    data.filtered_positive_delta_index = 1;
    data.positive_delta_index = -1;
    data.angles_index = -1;
    
    // Resetting recorded gesture indices.
    recorded_gesture.compare_index = 0;
    recorded_gesture.next_index = 0;
         
    // Loop for the sensing timeframe.
    for (int16_t i = 0; i < SENSING_TIMEFRAME; ++i)
    {
      // Wait until new gyroscopic sample is ready.
      flags.wait_all(DATA_READY_FLAG);

      // Setting write buffer for SPI communication with the gyroscope.
      write_buf[0] = OUT_X_L | 0x80 | 0x40;

      // Start sequential sample reading  7= (read 6 registers and write to 1)
      spi.transfer(write_buf, 7, read_buf, 7, spi_cb, SPI_EVENT_COMPLETE);
      flags.wait_all(SPI_FLAG);

      // Convert the raw data from the sensor to a usable format.
      raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
      raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
      raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

      // Convert raw data to a float.
      new_measurement.x = ((float)raw_gx) * 0.001;
      new_measurement.y = ((float)raw_gy) * 0.001;
      new_measurement.z = ((float)raw_gz) * 0.001;

      // Update delta indices.
      ++data.positive_delta_index;

      // Store whether the measurement is positive.
      data.positive_delta_x[data.positive_delta_index] = new_measurement.x > 0;
      data.positive_delta_y[data.positive_delta_index] = new_measurement.y > 0;
      data.positive_delta_z[data.positive_delta_index] = new_measurement.z > 0;

      // Update angles index.
      ++data.angles_index;

      // If the measurement is large enough, update the corresponding delta and angle.
      // Otherwise, keep the angle the same.
      // Do this for all three axes.
      // ...
      
      // Process the measurement, which may involve recording or comparing a gesture.
      processMeasurement(current_mode, gesture_match);

      // Print the angles.
      printf("Angles ||\tx: %4.5f\t|\ty: %4.5f\t|\tz: %4.5f\t||\n", data.angles_x[data.angles_index], data.angles_y[data.angles_index], data.angles_z[data.angles_index]);


    }

    // Print the recorded gesture.
    for (int i = 0; i < SENSING_TIMEFRAME; ++i) {
      if (i < recorded_gesture.next_index)  std::cout << recorded_gesture.axis[i] << ", " << recorded_gesture.angle_change[i] << std::endl;
    }

    // Initially, both LEDs are turned off.
    led1 = 0;
    led2 = 0;

    // If gesture matches, turn on LED 1. Else, turn on LED 2.
    if (gesture_match) led1 = 1;
    else led2 = 1;

    // Keep the LEDs on for 5 seconds.
    thread_sleep_for(5000);

    // Turn off the LEDs.
    led1 = 0;
    led2 = 0;

    // Print "DONE" to indicate the end of reading and processing gyro data.
    std::cout << "DONE" << std::endl;

    // Stop the timer and print the elapsed time (commented out).
    // t1.stop();
    // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t1.elapsed_time()).count() << std::endl;

    // Indicate that we are done reading.
    show_reading_indicator = false;
} 


// This function blinks LEDs while the reading indicator is active
void blinkLEDs() {

  // Check if the reading indicator is active
  while (show_reading_indicator) {

    // Turn on both LEDs
    led1 = 1;
    led2 = 1;

    // Keep them on for 500 milliseconds
    thread_sleep_for(500);
  
    // Turn off both LEDs
    led1 = 0;
    led2 = 0;

    // Keep them off for 500 milliseconds
    thread_sleep_for(500);

  }

  // At the end, turn on both LEDs
  led1 = 1;
  led2 = 1;
}


// This function blinks LED2 based on the current mode
void blinkMode(Mode current_mode) {

  // If the current mode is UNLOCK, blink LED2 three times
  if (current_mode == Mode::UNLOCK) {

    for (int8_t i = 0; i < 3; i++) {

      // Turn on LED2
      led2 = 1;

      // Keep it on for 500 milliseconds
      thread_sleep_for(500);

      // Turn off LED2
      led2 = 0;

      // Keep it off for 500 milliseconds
      thread_sleep_for(500);

    }

  // If the current mode is RECORD, turn on LED2 for three seconds
  } else if (current_mode == Mode::RECORD) {
    
    // Turn on LED2
    led2 = 1;

    // Keep it on for 3 seconds
    thread_sleep_for(3000);

    // Turn off LED2
    led2 = 0;

  }

  // Activate the reading indicator
  show_reading_indicator = true;

  // Start a new thread that blinks the LEDs while the reading indicator is active
  Thread reading_led_indicator_thread;

  reading_led_indicator_thread.start(blinkLEDs);

}

// Entry point of the application
int main() {

   setupGyro();  // Set up the gyroscope.

  // Main execution loop
  while (1) {
    
    bool current_state = button.read();
    
    // If the button is pressed, the program enters one of two modes:
    // Record mode: Press and hold the button for 2s, green light will turn on to indicate record mode. 
    // In record mode, press and release to turn off green light and leave record mode.
    // Unlock mode: Press and release within 2s, red light will blink, indicating unlock mode.
    if (current_state == true && button_pressed == false) {

      // Button was just pressed
      button_pressed = true;
      button_hold_time = 0;
    
    }

    else if (current_state == true && button_pressed == true) {
      
      // Button is being held down
      button_hold_time++;

      if (button_hold_time >= 2*1000) {  // If button is held for 2 seconds or more
        
        // Enter record mode
        blinkMode(Mode::RECORD);
        readGyro(Mode::RECORD);

        // Turn off LEDs
        led1 = 0;
        led2 = 0;
      
      }

    } else if (current_state == false && button_pressed == true) {
      
      // Button was just released
      if (button_hold_time < 2*1000) {  // If button was released before 2 seconds
        
        // Enter unlock mode
        blinkMode(Mode::UNLOCK);
        readGyro(Mode::UNLOCK);

      }

      // Reset button state and hold time
      button_pressed = false;
      button_hold_time = 0;

    }

    // Wait for 1 millisecond before next iteration
    thread_sleep_for(1); 
  
  }

}
