#include "mbed.h"
#include "AccurateWaiter.h"
#include <iostream>

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs

InterruptIn int2(PA_2, PullDown);

DigitalIn button(USER_BUTTON); // Button input

/* Status LED outputs: */
DigitalOut led1(LED1); 
DigitalOut led2(LED2);

bool button_pressed = false;  // button state
int button_hold_time = 0;  // button time

#define CTRL_REG1 0x20  // register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)

/*
  Data sheet table 22
  ODR : The rate at which sensor provides data
  Cutoff: Filter out high frequency
  configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
*/
#define CTRL_REG1_CONFIG 0b11'10'1'1'1'1

#define CTRL_REG4 0x23  // register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)

/*
  Controls how fast you want to measure degrees per second : (00: 245 dps, 01: 500 dps, 10: 2000dps, 11: 2000 dps)
  Configuration: reserved,little endian,500 dps,reserved,disabled,4-wire mode
*/
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

/*
  READY Inturrput configuration
  register fields(bits): I1_Int1 (1), I1_Boot(1), H_Lactive(1), PP_OD(1), I2_DRDY(1), I2_WTM(1), I2_ORun(1), I2_Empty(1)
*/
#define CTRL_REG3 0x22

#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000  // configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts

#define OUT_X_L 0x28  // By setting the MSB to 1 we can auto increment through the output data from 0x28 to 0x2D (X: Z)

#define SPI_FLAG 1  // SPI flag when complete

#define DATA_READY_FLAG 2  // Data ready flag when next data is values of data are ready to be read.

uint8_t write_buf[32];
uint8_t read_buf[32];

EventFlags flags;

enum Mode {

  RECORD = 1,
  UNLOCK

};

bool show_reading_indicator = false;
Thread reading_led_indicator_thread;

#define SENSING_TIMEFRAME 100  // number of seconds that gyroscope will sense gesture: 100 -> 5 seconds, 200 -> 10 seconds, ...
const int32_t STABILITY_TIMEFRAME = 5;  // timeframe after which stability of measurement in any axis is gauged + compared about these values
const float STABILITY_THRESHOLD = 1;  // upper bound on the change in degree a value under which is considered to be holding stable
const int32_t COMPARE_THRESHOLD = 20;  // the difference in degrees around the actual value of measurement that still register as a match
const float DELTA_THRESHOLD = 30;  // amount of change in angle required for an major change along an axis is registered

// const int16_t POSITIVE_DELTA_WINDOW_SIZE = 100;

struct Data {

  float delta_x = 0.0f;  // current change in x axis since last time the direction flipped or angle held stable
  float delta_y = 0.0f;  // current change in y axis since last time the direction flipped or angle held stable
  float delta_z = 0.0f;  // current change in z axis since last time the direction flipped or angle held stable

  bool positive_delta_x[SENSING_TIMEFRAME];  // direction of change in x axis currently in the positive direction?
  bool positive_delta_y[SENSING_TIMEFRAME];  // direction of change in y axis currently in the positive direction?
  bool positive_delta_z[SENSING_TIMEFRAME];  // direction of change in z axis currently in the positive direction?
  
  bool filtered_positive_delta_x[SENSING_TIMEFRAME + 1] = {true};
  bool filtered_positive_delta_y[SENSING_TIMEFRAME + 1] = {true};
  bool filtered_positive_delta_z[SENSING_TIMEFRAME + 1] = {true};

  float angles_x[SENSING_TIMEFRAME];
  float angles_y[SENSING_TIMEFRAME];
  float angles_z[SENSING_TIMEFRAME];

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


/**
 * spi callback function:
 * The spi transfer function requires that the callback provided to it takes an int parameter
*/
void spi_cb(int event) {

  flags.set(SPI_FLAG);

};


/**
 * Data callback function
*/
void data_cb() {
  
  flags.set(DATA_READY_FLAG);

};
void processMeasurement(Mode current_mode, bool &gesture_match) {

  bool compare_axis_deltas_x = false;
  bool compare_axis_deltas_y = false;
  bool compare_axis_deltas_z = false;

  // If the absolute change in the x-axis is greater than the threshold, and either the current and previous positive deltas are different, or the change in angle over the stability timeframe is less than the threshold
  // then we should consider the change in the x-axis
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

  // Setup the SPI for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8, 3);
  spi.frequency(1'000'000);

  // Configure CTRL_REG1 register on the sensor
  write_buf[0] = CTRL_REG1;
  write_buf[1] = CTRL_REG1_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  // Configure CTRL_REG4 register on the sensor
  write_buf[0] = CTRL_REG4;
  write_buf[1] = CTRL_REG4_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  // Configure interrupt to trigger our callback function when the pin becomes high
  int2.rise(&data_cb);
  write_buf[0] = CTRL_REG3;
  write_buf[1] = CTRL_REG3_CONFIG;
  spi.transfer(write_buf, 2, read_buf, 2, spi_cb, SPI_EVENT_COMPLETE);
  flags.wait_all(SPI_FLAG);

  // As the gyroscope sensor keeps its configuration between power cycles,
  // manually check the signal and set the flag for the first sample if needed.
  if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {

    flags.set(DATA_READY_FLAG);
  
  }

  // Main execution loop
  while (1) {
    
    bool current_state = button.read();
    
    // Press and hold the button for 2s, green light will turn on to indicate record mode.
    // In record mode, press and release to turn off green light and leave record mode.
    // Press and release within 2s, red light will blink, indicating unlock mode.
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
