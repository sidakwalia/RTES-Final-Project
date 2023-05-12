#include "mbed.h"

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
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

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

Thread reading_led_indicator_thread;
bool show_reading_indicator = false;

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

void readGyro() {

    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;
    bool current = 0;

    float Xtotal= 0;
    float Ytotal= 0;
    float Ztotal= 0;

    float angle_x = 0.0f, angle_y = 0.0f, angle_z = 0.0f;

    //Start the recording when the pressed button realeased
    if(button.read()==0)
    {
      
      for (int i = 0; i < 5000; ++i)
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

        // gx=((float)raw_gx)*SCALING_FACTOR;
        // gy=((float)raw_gy)*SCALING_FACTOR;
        // gz=((float)raw_gz)*SCALING_FACTOR;

        gx = ((float)raw_gx) * 0.001;
        gy = ((float)raw_gy) * 0.001;
        gz = ((float)raw_gz) * 0.001;

        if (abs(gx) > 0.25) angle_x += gx; 
        if (abs(gy) > 0.25) angle_y += gy;
        if (abs(gz) > 0.25) angle_z += gz;
    
        printf("Angles |\tx: %4.5f \t y: %4.5f \t z: %4.5f\n", angle_x , angle_y, angle_z);
        
        thread_sleep_for(1);   
        
      }

    }

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

  if (current_mode == UNLOCK) {

    for (int8_t i = 0; i < 3; i++) {

      led2 = 1;
      thread_sleep_for(500);
      led2 = 0;
      thread_sleep_for(500);

    }

  } else if (current_mode == RECORD) {
    
    led2 = 1;
    thread_sleep_for(3000);
    led2 = 0;

  }

  show_reading_indicator = true;
  reading_led_indicator_thread.start(blinkLEDs);

}




int main()
{
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

  while (1)
  {
    bool current_state = button.read();
    //Press and hold the button for 2s, green light will on it will get into record mode.
    //In record mode, press and release, green light off and leave the record mode.
    //Press and release within 2s, red light will blink, unlock mode on (function needed)
    if (current_state == true && button_pressed == false) 
    {
      // Button was just pressed
      button_pressed = true;
      button_hold_time = 0;
    } 
    else if (current_state == true && button_pressed == true) 
    {
      // Button is being held down
      button_hold_time++;

      if (button_hold_time > 2*1000) 
      {
        blinkMode(Mode::RECORD);
        readGyro();
        led1 = 0;
        led2 = 0;
      }

    } 
    else if (current_state == false && button_pressed == true) 
    {
      // Button was just released
      if (button_hold_time < 2*1000) 
      {
        blinkMode(Mode::UNLOCK);//unlock mode
        readGyro();

      }
      button_pressed = false;
      button_hold_time = 0;
    }
    thread_sleep_for(1); 
  }
}
