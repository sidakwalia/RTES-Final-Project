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

#define SENSING_TIMEFRAME 4000  // number of seconds that gyroscope will sense gesture
const int32_t STABILITY_TIMEFRAME = 200;  // timeframe after which stability of measurement in any axis is gauged + compared about these values
const float STABILITY_THRESHOLD = 20;  // upper bound on the change in degree a value under which is considered to be holding stable
const int32_t COMPARE_THRESHOLD = 20;  // the difference in degrees around the actual value of measurement that still register as a match
const int16_t POSITIVE_DELTA_WINDOW_SIZE = 100;

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

};

NewMeasurement new_measurement;

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

const float DELTA_THRESHOLD = 30;  // amount of change in angle required for an major change along an axis is registered

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

  int32_t positive_window_base = max(static_cast<int32_t>(0), static_cast<int32_t>(data.positive_delta_index - POSITIVE_DELTA_WINDOW_SIZE + 1));
  int32_t current_window_size = data.positive_delta_index - positive_window_base;

  int32_t count_x = 0;
  int32_t count_y = 0;
  int32_t count_z = 0;

  for (int32_t i = positive_window_base; i <= data.positive_delta_index; ++i) {

    if (data.positive_delta_x[i]) ++count_x;
    if (data.positive_delta_y[i]) ++count_y;
    if (data.positive_delta_z[i]) ++count_z;

  }

  bool filtered_positive_delta_x = true;
  bool filtered_positive_delta_y = true;
  bool filtered_positive_delta_z = true;

  if (count_x < (current_window_size / 2)) filtered_positive_delta_x = false; 
  if (count_y < (current_window_size / 2)) filtered_positive_delta_y = false; 
  if (count_z < (current_window_size / 2)) filtered_positive_delta_z = false; 

  bool compare_axis_deltas_x = false;
  bool compare_axis_deltas_y = false;
  bool compare_axis_deltas_z = false;

  float change_in_x = 0.0;
  float change_in_y = 0.0;
  float change_in_z = 0.0;

  if (abs(data.delta_x) > DELTA_THRESHOLD && (data.filtered_positive_delta_x[data.filtered_positive_delta_index - 1] != filtered_positive_delta_x || 
      (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_x[data.angles_index - STABILITY_TIMEFRAME] - data.angles_x[data.angles_index]) <= STABILITY_THRESHOLD))) {

        change_in_x = data.delta_x;
        compare_axis_deltas_x = true;
      
  }

  if (abs(data.delta_y) > DELTA_THRESHOLD && (data.filtered_positive_delta_y[data.filtered_positive_delta_index - 1] != filtered_positive_delta_y || 
     (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_y[data.angles_index - STABILITY_TIMEFRAME] - data.angles_y[data.angles_index]) <= STABILITY_THRESHOLD))) {

      change_in_y = data.delta_y;
      compare_axis_deltas_y = true;

  }

  if (abs(data.delta_z) > DELTA_THRESHOLD && (data.filtered_positive_delta_z[data.filtered_positive_delta_index - 1] != filtered_positive_delta_z || 
     (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_z[data.angles_index - STABILITY_TIMEFRAME] - data.angles_z[data.angles_index]) <= STABILITY_THRESHOLD))) {

      change_in_z = data.delta_z;
      compare_axis_deltas_z = true;

  }

  if (compare_axis_deltas_x && abs(change_in_x) > DELTA_THRESHOLD) {

    if (abs(change_in_x) >= abs(max(change_in_y, data.delta_y)) && abs(change_in_x) >= abs(max(change_in_z, data.delta_z))) {

      if (current_mode == Mode::RECORD) {

        recorded_gesture.axis[recorded_gesture.next_index] = Axis::X;
        recorded_gesture.angle_change[recorded_gesture.next_index] = change_in_x;
        ++recorded_gesture.next_index;

      } else {

        if (!(recorded_gesture.axis[recorded_gesture.compare_index] == Axis::X && abs(recorded_gesture.angle_change[recorded_gesture.compare_index] - change_in_x) <= COMPARE_THRESHOLD)) {

          gesture_match = false;
          ++recorded_gesture.compare_index;

        }

      }

      data.delta_x = new_measurement.x;

    }

  }

  if (compare_axis_deltas_y && abs(change_in_y) > DELTA_THRESHOLD) {

    if (abs(change_in_y) >= max(change_in_x, data.delta_x) && abs(change_in_y) >= max(change_in_z, data.delta_z)) {

      if (current_mode == Mode::RECORD) {

        recorded_gesture.axis[recorded_gesture.next_index] = Axis::Y;
        recorded_gesture.angle_change[recorded_gesture.next_index] = change_in_y;
        ++recorded_gesture.next_index;

      } else {

        if (!(recorded_gesture.axis[recorded_gesture.compare_index] == Axis::Y && abs(recorded_gesture.angle_change[recorded_gesture.compare_index] - change_in_y) <= COMPARE_THRESHOLD)) {

          gesture_match = false;
          ++recorded_gesture.compare_index;

        }

      }

      data.delta_y = new_measurement.y;

    }

  }

  if (compare_axis_deltas_z && abs(change_in_z) > DELTA_THRESHOLD) {

    if (abs(change_in_z) >= abs(max(change_in_y, data.delta_y)) && abs(change_in_z >= max(change_in_x, data.delta_x))) {

      if (current_mode == Mode::RECORD) {

        recorded_gesture.axis[recorded_gesture.next_index] = Axis::Z;
        recorded_gesture.angle_change[recorded_gesture.next_index] = change_in_z;
        ++recorded_gesture.next_index;

      } else {

        if (!(recorded_gesture.axis[recorded_gesture.compare_index] == Axis::Z && abs(recorded_gesture.angle_change[recorded_gesture.compare_index] - change_in_z) <= COMPARE_THRESHOLD)) {

          ++recorded_gesture.compare_index;
          gesture_match = false;

        }

      }

      data.delta_z = new_measurement.z;

    }

  }

  data.filtered_positive_delta_x[data.filtered_positive_delta_index] = filtered_positive_delta_x;
  data.filtered_positive_delta_y[data.filtered_positive_delta_index] = filtered_positive_delta_y;
  data.filtered_positive_delta_z[data.filtered_positive_delta_index] = filtered_positive_delta_z;

  ++data.filtered_positive_delta_index;


}

void readGyro(Mode current_mode) {

    while (button.read()) {  //Start the recording when the pressed button realeased

      thread_sleep_for(1);

    }

    int16_t raw_gx, raw_gy, raw_gz;

    bool gesture_match = true;

    data.delta_x = 0;
    data.delta_y = 0;
    data.delta_z = 0;
    data.filtered_positive_delta_index = 1;
    data.positive_delta_index = -1;
    data.angles_index = -1;
    
    recorded_gesture.compare_index = 0;
    recorded_gesture.next_index = 0;

    for (int16_t i = 0; i < SENSING_TIMEFRAME; ++i)
    {

      // Timer t;

      // t.start();

      // wait until new sample is ready
      flags.wait_all(DATA_READY_FLAG);

      //               data  | read | increment
      write_buf[0] = OUT_X_L | 0x80 | 0x40;

      // start sequential sample reading  7= (read 6 registers and write to 1)
      spi.transfer(write_buf, 7, read_buf, 7, spi_cb, SPI_EVENT_COMPLETE);
      flags.wait_all(SPI_FLAG);

      // read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
      // Put the high and low bytes in the correct order lowB,Highb -> HighB,LowB
      raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t)read_buf[1]);
      raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t)read_buf[3]);
      raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t)read_buf[5]);

      //  printf("RAW|\tgx: %d \t gy: %d \t gz: %d\n",raw_gx,raw_gy,raw_gz);

      new_measurement.x = ((float)raw_gx) * 0.001;
      new_measurement.y = ((float)raw_gy) * 0.001;
      new_measurement.z = ((float)raw_gz) * 0.001;

      ++data.positive_delta_index;

      data.positive_delta_x[data.positive_delta_index] = new_measurement.x > 0;
      data.positive_delta_y[data.positive_delta_index] = new_measurement.y > 0;
      data.positive_delta_z[data.positive_delta_index] = new_measurement.z > 0;

      ++data.angles_index;

      if (abs(new_measurement.x) > 0.15) {
        
          data.delta_x += new_measurement.x;
          data.angles_x[data.angles_index] = data.angles_index > 0 ? data.angles_x[data.angles_index - 1] + new_measurement.x: new_measurement.x;
      
      } else {

        data.angles_x[data.angles_index] = data.angles_index > 0 ? data.angles_x[data.angles_index - 1]: 0;

      }

      if (abs(new_measurement.y) > 0.15) {
        
        data.delta_y += new_measurement.y;
        data.angles_y[data.angles_index] = data.angles_index > 0 ? data.angles_y[data.angles_index - 1] + new_measurement.x: new_measurement.y;
      
      } else {

        data.angles_y[data.angles_index] = data.angles_index > 0 ? data.angles_y[data.angles_index - 1]: 0;

      }
      
      if (abs(new_measurement.z) > 0.15) {
        
        data.delta_z += new_measurement.z;
        data.angles_z[data.angles_index] = data.angles_index > 0 ? data.angles_z[data.angles_index - 1] + new_measurement.x : new_measurement.z;

      } else {

        data.angles_z[data.angles_index] = data.angles_index > 0 ? data.angles_z[data.angles_index - 1]: 0;

      }

      // processMeasurement(current_mode, gesture_match);
      
      // printf("Angles |\tx: %4.5f \t y: %4.5f \t z: %4.5f\n", angle_x , angle_y, angle_z);

      // t.stop();

      // waiter.wait_for(std::chrono::microseconds(100000 - std::chrono::duration_cast<std::chrono::microseconds>(t.elapsed_time()).count()));

      // std::cout << std::chrono::duration_cast<std::chrono::microseconds>(t.elapsed_time()).count() << std::endl;
      
    }

    // for (int i = 0; i < SENSING_TIMEFRAME; ++i) {

    //   if (i < recorded_gesture.next_index) std::cout << recorded_gesture.axis[i] << ", " << recorded_gesture.angle_change[i] << ", " << recorded_gesture.angle_change[i] << std::endl;

    // }

    for (int i = 0; i < SENSING_TIMEFRAME; ++i) {

      std::cout << data.angles_x[i] << ", " << data.angles_y[i] << "," << data.angles_z[i] << std::endl;

    }

    led1 = 0;
    led2 = 0;

    if (gesture_match) led1 = 1;
    else led2 = 1;

    thread_sleep_for(5000);

    led1 = 0;
    led2 = 0;

    std::cout << "DONE" << std::endl;

    // t1.stop();
    // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(t1.elapsed_time()).count() << std::endl;

    show_reading_indicator = false;

}

void blinkLEDs() {

  while (show_reading_indicator) {

    led1 = 1;
    led2 = 1;

    thread_sleep_for(500);
  
    led1 = 0;
    led2 = 0;

    thread_sleep_for(500);

  }

  led1 = 1;
  led2 = 1;

}

void blinkMode(Mode current_mode) {

  if (current_mode == Mode::UNLOCK) {

    for (int8_t i = 0; i < 3; i++) {

      led2 = 1;
      thread_sleep_for(500);
      led2 = 0;
      thread_sleep_for(500);

    }

  } else if (current_mode == Mode::RECORD) {
    
    led2 = 1;
    thread_sleep_for(3000);
    led2 = 0;

  }

  show_reading_indicator = true;

  Thread reading_led_indicator_thread;

  reading_led_indicator_thread.start(blinkLEDs);

}


int main() {

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
  if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {

    flags.set(DATA_READY_FLAG);
  
  }

  while (1) {
    
    bool current_state = button.read();
    
    //Press and hold the button for 2s, green light will on it will get into record mode.
    //In record mode, press and release, green light off and leave the record mode.
    //Press and release within 2s, red light will blink, unlock mode on (function needed)
    if (current_state == true && button_pressed == false) {

      // Button was just pressed
      button_pressed = true;
      button_hold_time = 0;
    
    }

    else if (current_state == true && button_pressed == true) {
      
      // Button is being held down
      button_hold_time++;

      if (button_hold_time >= 2*1000) {
        
        blinkMode(Mode::RECORD);
        readGyro(Mode::RECORD);

        led1 = 0;
        led2 = 0;
      
      }

    } else if (current_state == false && button_pressed == true) {
      
      // Button was just released
      if (button_hold_time < 2*1000) {
        
        blinkMode(Mode::UNLOCK);//unlock mode
        readGyro(Mode::UNLOCK);

      }

      button_pressed = false;
      button_hold_time = 0;

    }

    thread_sleep_for(1); 
  
  }

}
