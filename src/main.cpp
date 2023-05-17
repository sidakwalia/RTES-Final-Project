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
#include "AccurateWaiter.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/stm32f429i_discovery_ts.h"

#include <iostream>

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs

InterruptIn int2(PA_2, PullDown);

DigitalIn button(USER_BUTTON); // Button input

/* Status LED outputs: */
DigitalOut led1(LED1); 
DigitalOut led2(LED2);

bool button_pressed = false;  // button state
int button_hold_time = 0;  // button time

int32_t square_y_pos[5] = {220, 173, 126, 75, 30};

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

enum Color {

    BLUE = 1,
    GREEN, 
    RED

};

Color background_color = Color::BLUE;

bool reading_state = false;

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

Gesture recorded_gestures[5];  // gesture stored
int8_t selected_user_profile = 0;

LCD_DISCO_F429ZI lcd;
TS_StateTypeDef ts_state;

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


  if (current_mode == Mode::UNLOCK && recorded_gestures->next_index == 0) {

        gesture_match = false;
        return;

  }

  if (!gesture_match) return;

  if (abs(data.delta_x) > DELTA_THRESHOLD && ((data.positive_delta_index > 0 && data.positive_delta_x[data.positive_delta_index] != data.positive_delta_x[data.positive_delta_index - 1]) || 
      (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_x[data.angles_index - STABILITY_TIMEFRAME] - data.angles_x[data.angles_index]) <= STABILITY_THRESHOLD))) {

        compare_axis_deltas_x = true;
      
  }

  if (abs(data.delta_y) > DELTA_THRESHOLD && ((data.positive_delta_index > 0 && data.positive_delta_y[data.positive_delta_index] != data.positive_delta_y[data.positive_delta_index - 1]) || 
     (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_y[data.angles_index - STABILITY_TIMEFRAME] - data.angles_y[data.angles_index]) <= STABILITY_THRESHOLD))) {

      compare_axis_deltas_y = true;

  }

  if (abs(data.delta_z) > DELTA_THRESHOLD && ((data.positive_delta_index > 0 && data.positive_delta_z[data.positive_delta_index] != data.positive_delta_z[data.positive_delta_index - 1]) || 
     (data.angles_index >= STABILITY_TIMEFRAME && abs(data.angles_z[data.angles_index - STABILITY_TIMEFRAME] - data.angles_z[data.angles_index]) <= STABILITY_THRESHOLD))) {

      compare_axis_deltas_z = true;

  }

  if (compare_axis_deltas_x && abs(data.delta_x) > DELTA_THRESHOLD) {

    if (abs(data.delta_x) >= abs(data.delta_y) && abs(data.delta_x) >= abs(data.delta_z)) {

      if (current_mode == Mode::RECORD) {

        recorded_gestures[selected_user_profile].axis[recorded_gestures[selected_user_profile].next_index] = Axis::X;
        recorded_gestures[selected_user_profile].angle_change[recorded_gestures[selected_user_profile].next_index] = data.delta_x;
        ++recorded_gestures[selected_user_profile].next_index;

      } else {

        if (!(recorded_gestures[selected_user_profile].axis[recorded_gestures[selected_user_profile].compare_index] == Axis::X && abs(recorded_gestures[selected_user_profile].angle_change[recorded_gestures[selected_user_profile].compare_index] - data.delta_x) <= COMPARE_THRESHOLD)) {

          gesture_match = false;

        }

        ++recorded_gestures[selected_user_profile].compare_index;

      }

      data.delta_x = 0;
      data.delta_y = 0;
      data.delta_z = 0;

    }

  }

  if (compare_axis_deltas_y && abs(data.delta_y) > DELTA_THRESHOLD) {

    if (abs(data.delta_y) >= abs(data.delta_x) && abs(data.delta_y) >= abs(data.delta_z)) {

      if (current_mode == Mode::RECORD) {

        recorded_gestures[selected_user_profile].axis[recorded_gestures[selected_user_profile].next_index] = Axis::Y;
        recorded_gestures[selected_user_profile].angle_change[recorded_gestures[selected_user_profile].next_index] = data.delta_y;
        ++recorded_gestures[selected_user_profile].next_index;

      } else {

        if (!(recorded_gestures[selected_user_profile].axis[recorded_gestures[selected_user_profile].compare_index] == Axis::Y && abs(recorded_gestures[selected_user_profile].angle_change[recorded_gestures[selected_user_profile].compare_index] - data.delta_y) <= COMPARE_THRESHOLD)) {

          gesture_match = false;

        }

        ++recorded_gestures[selected_user_profile].compare_index;

      }

      data.delta_x = 0;
      data.delta_y = 0;
      data.delta_z = 0;

    }

  }

  if (compare_axis_deltas_z && abs(data.delta_z) > DELTA_THRESHOLD) {

    if (abs(data.delta_z) >= abs(data.delta_x) && abs(data.delta_z) >= abs(data.delta_y)) {

      if (current_mode == Mode::RECORD) {

        recorded_gestures[selected_user_profile].axis[recorded_gestures[selected_user_profile].next_index] = Axis::Z;
        recorded_gestures[selected_user_profile].angle_change[recorded_gestures[selected_user_profile].next_index] = data.delta_z;
        ++recorded_gestures[selected_user_profile].next_index;

      } else {

        if (!(recorded_gestures[selected_user_profile].axis[recorded_gestures[selected_user_profile].compare_index] == Axis::Z && abs(recorded_gestures[selected_user_profile].angle_change[recorded_gestures[selected_user_profile].compare_index] - data.delta_z) <= COMPARE_THRESHOLD)) {

          gesture_match = false;

        }

        ++recorded_gestures[selected_user_profile].compare_index;

      }

      data.delta_x = 0;
      data.delta_y = 0;
      data.delta_z = 0;

    }

  }

}

void readGyro(Mode current_mode) {

    reading_state = true;

    led1 = 1;
    led2 = 1;

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
    
    recorded_gestures[selected_user_profile].compare_index = 0;
    recorded_gestures[selected_user_profile].next_index = 0;
         
    for (int16_t i = 0; i < SENSING_TIMEFRAME; ++i)
    {

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
        data.angles_y[data.angles_index] = data.angles_index > 0 ? data.angles_y[data.angles_index - 1] + new_measurement.y: new_measurement.y;
      
      } else {

        data.angles_y[data.angles_index] = data.angles_index > 0 ? data.angles_y[data.angles_index - 1]: 0;

      }
      
      if (abs(new_measurement.z) > 0.15) {
        
        data.delta_z += new_measurement.z;
        data.angles_z[data.angles_index] = data.angles_index > 0 ? data.angles_z[data.angles_index - 1] + new_measurement.z : new_measurement.z;

      } else {

        data.angles_z[data.angles_index] = data.angles_index > 0 ? data.angles_z[data.angles_index - 1]: 0;

      }

      processMeasurement(current_mode, gesture_match);
      
      // thread_sleep_for(50);

      std::cout << data.angles_x[data.angles_index] << ", " << data.angles_y[data.angles_index] << ", " << data.angles_z[data.angles_index] << std::endl;
      
    }

    for (int8_t i = 0; i < SENSING_TIMEFRAME; ++i) {

        std::cout << recorded_gestures[selected_user_profile].axis[i] << ", " << recorded_gestures[selected_user_profile].angle_change[i] << std::endl;

    }

    led1 = 0;
    led2 = 0;

    thread_sleep_for(500);

    if (gesture_match && current_mode == Mode::UNLOCK) {

        
        for (int8_t i = 0; i < 5; ++i) {

            led1 = 1;
            background_color = Color::GREEN;

            thread_sleep_for(500);

            led1 = 0;
            background_color = Color::BLUE;

            thread_sleep_for(500); 

        }

    } else if (current_mode == Mode::UNLOCK) {
        
        for (int8_t i = 0; i < 5; ++i) {

            led2 = 1;
            background_color = Color::RED;

            thread_sleep_for(500);

            led2 = 0;
            background_color = Color::BLUE;

            thread_sleep_for(500); 

        }

    }

    led1 = 0;
    led2 = 0;

    reading_state = false;

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

}

void uiEventsLoop() {

    while (true) {
        
        if (background_color == Color::BLUE) {
            
            lcd.Clear(LCD_COLOR_DARKBLUE);
            lcd.SetBackColor(LCD_COLOR_DARKBLUE);

        } else if (background_color == Color::GREEN) {
            
            lcd.Clear(LCD_COLOR_GREEN);
            lcd.SetBackColor(LCD_COLOR_GREEN);
        
        } else {
            
            lcd.Clear(LCD_COLOR_RED);
            lcd.SetBackColor(LCD_COLOR_RED);

        }

        lcd.SetTextColor(LCD_COLOR_WHITE);

        char user_profiles[2] = {'1', NULL};
        
        for (int8_t i = 5; i >= 1; --i) {

            lcd.DisplayStringAt(0, LINE(i * 3), (uint8_t *)user_profiles, CENTER_MODE);
            ++user_profiles[0];

        }

        lcd.DrawRect(90, square_y_pos[selected_user_profile], 50, 50);
        thread_sleep_for(50);

    }

}

int32_t getUserProfileNumber(int32_t y) {

    return (y / (lcd.GetYSize() / 5));

}

void lcdInputThread() {

    BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());

    TS_StateTypeDef ts_state;

    while (1) {

        BSP_TS_GetState(&ts_state);

        if (ts_state.TouchDetected && !reading_state) {

            selected_user_profile = getUserProfileNumber(ts_state.Y);
            thread_sleep_for(500);

        }

    }

}

void mainBackgroundThread() {

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

            if (button_hold_time >= 400) {
                    
                blinkMode(Mode::RECORD);
                readGyro(Mode::RECORD);

                led1 = 0;
                led2 = 0;
            
            }

        } else if (current_state == false && button_pressed == true) {
        
            // Button was just released
            if (button_hold_time < 400) {
                
                blinkMode(Mode::UNLOCK);//unlock mode
                readGyro(Mode::UNLOCK);

            }

            button_pressed = false;
            button_hold_time = 0;

        }

        thread_sleep_for(1); 
  
    } 

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

  Thread ui_thread;
  ui_thread.start(uiEventsLoop);

  Thread lcd_input_thread;
  lcd_input_thread.start(lcdInputThread);

  Thread main_thread;
  main_thread.start(mainBackgroundThread);

  main_thread.join();
  lcd_input_thread.join();
  ui_thread.join();

}
