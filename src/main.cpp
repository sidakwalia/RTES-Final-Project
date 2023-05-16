#include "mbed.h"
#include <math.h>
#include <float.h>
#include "rtos/Mutex.h"
#include "rtos/EventFlags.h"

#define CTRL_REG1 0x20 // register fields(bits): data_rate(2),Bandwidth(2),Power_down(1),Zen(1),Yen(1),Xen(1)

/*
  Data sheet table 22
  ODR : The rate at which sensor provides data
  Cutoff: Filter out high frequency
  configuration: 200Hz ODR,50Hz cutoff, Power on, Z on, Y on, X on
*/
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

#define CTRL_REG4 0x23 // register fields(bits): reserved(1), endian-ness(1),Full scale sel(2), reserved(1),self-test(2), SPI mode(1)

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

#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000 // configuration: Int1 disabled, Boot status disabled, active high interrupts, push-pull, enable Int2 data ready, disable fifo interrupts

#define OUT_X_L 0x28 // By setting the MSB to 1 we can auto increment through the output data from 0x28 to 0x2D (X: Z)

#define SPI_FLAG 1 // SPI flag when complete

#define DATA_READY_FLAG 2 // Data ready flag when next data is values of data are ready to be read.

#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)
#define MAX_SEQUENCE_LENGTH 100
#define DTW_THRESHOLD 2

rtos::Mutex mutex;
Mutex recMutex;
Mutex readMutex;
Mutex stopMutex;
Mutex completeMutex;
rtos::EventFlags buttonPressFlag;

float dtw_matrix[MAX_SEQUENCE_LENGTH][MAX_SEQUENCE_LENGTH];
float sequence_x[MAX_SEQUENCE_LENGTH];
float sequence_y[MAX_SEQUENCE_LENGTH];
float sequence_z[MAX_SEQUENCE_LENGTH];
int seq_length = 0.5;

float alpha = 0.75f; // Filter coefficient

SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs

InterruptIn int2(PA_2, PullDown);

InterruptIn button(USER_BUTTON); // Button input

/* Status LED outputs: */
DigitalOut greenLed(LED1);
DigitalOut redLed(LED2);

uint8_t write_buf[32];
uint8_t read_buf[32];

int16_t raw_gx, raw_gy, raw_gz;
float gx, gy, gz;
bool current = 0;

EventFlags flags;

volatile bool isRecording = true;
volatile bool isReading = false;
volatile bool stop = false;
volatile bool record = false;
volatile bool compare = false;
volatile bool complete = false;

float filtered_angle_x = 0.0f;
float filtered_angle_y = 0.0f;
float filtered_angle_z = 0.0f;

/**
 * spi callback function:
 * The spi transfer function requires that the callback provided to it takes an int parameter
 */
void spi_cb(int event)
{

    flags.set(SPI_FLAG);
};

/**
 * Data callback function
 */
void data_cb()
{

    flags.set(DATA_READY_FLAG);
};

void setupGyro()
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
}

float complementaryFilter(float gyro_value, float accel_angle, float alpha)
{
    // Apply complementary filter equation
    float filtered_angle = alpha * gyro_value + (1 - alpha) * accel_angle;
    return filtered_angle;
}

float min(float a, float b, float c)
{
    float m = a;
    if (b < m)
        m = b;
    if (c < m)
        m = c;
    return m;
}

float euclidean_distance(float a, float b)
{
    return sqrtf(powf(a - b, 2));
}

float dtw(float *s, float *t, int len)
{
    mutex.lock();
    for (int i = 0; i < len; i++)
    {
        for (int j = 0; j < len; j++)
        {
            dtw_matrix[i][j] = FLT_MAX;
        }
    }

    dtw_matrix[0][0] = 0;

    for (int i = 1; i < len; i++)
    {
        for (int j = 1; j < len; j++)
        {
            float cost = euclidean_distance(s[i], t[j]);
            dtw_matrix[i][j] = cost + min(dtw_matrix[i - 1][j],
                                          dtw_matrix[i][j - 1],
                                          dtw_matrix[i - 1][j - 1]);
        }
    }
     mutex.unlock();
    return dtw_matrix[len - 1][len - 1];
}

void recordGesture()
{

     mutex.lock();
    seq_length = 0;
    write_buf[0] = OUT_X_L | 0x80 | 0x40;

    while (record && seq_length < MAX_SEQUENCE_LENGTH)
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

        // gx = ((float)raw_gx) * 0.0001;
        // gy = ((float)raw_gy) * 0.0001;
        // gz = ((float)raw_gz) * 0.0001;

        // thread_sleep_for(1);

        //  printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n",  gx ,  gy, gz);

        filtered_angle_x = complementaryFilter(gx, filtered_angle_x, alpha);
        filtered_angle_y = complementaryFilter(gy, filtered_angle_y, alpha);
        filtered_angle_z = complementaryFilter(gz, filtered_angle_z, alpha);

        sequence_x[seq_length] = filtered_angle_x;
        sequence_y[seq_length] = filtered_angle_y;
        sequence_z[seq_length] = filtered_angle_z;

        seq_length++;

        //  printf("Actual|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n", filtered_angle_x, filtered_angle_y, filtered_angle_z);
    }
    mutex.unlock();
}

float compare_sequence_x[MAX_SEQUENCE_LENGTH];
float compare_sequence_y[MAX_SEQUENCE_LENGTH];
float compare_sequence_z[MAX_SEQUENCE_LENGTH];

float dtw_distance_x;
float dtw_distance_y;
float dtw_distance_z;

void compareGesture()
{
     mutex.lock();
    write_buf[0] = OUT_X_L | 0x80 | 0x40;
    int compare_length = 0;

    while (compare && compare_length < MAX_SEQUENCE_LENGTH)
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

        // gx = ((float)raw_gx) * 0.0001;
        // gy = ((float)raw_gy) * 0.0001;
        // gz = ((float)raw_gz) * 0.0001;


        // thread_sleep_for(1);

        filtered_angle_x = complementaryFilter(gx, filtered_angle_x, alpha);
        filtered_angle_y = complementaryFilter(gy, filtered_angle_y, alpha);
        filtered_angle_z = complementaryFilter(gz, filtered_angle_z, alpha);

        compare_sequence_x[compare_length] = filtered_angle_x;
        compare_sequence_y[compare_length] = filtered_angle_y;
        compare_sequence_z[compare_length] = filtered_angle_z;

        compare_length++;
    }

    dtw_distance_x = abs(dtw(sequence_x, compare_sequence_x, seq_length));
    dtw_distance_y = abs(dtw(sequence_y, compare_sequence_y, seq_length));
    dtw_distance_z = abs(dtw(sequence_z, compare_sequence_z, seq_length));
    mutex.unlock();

    // printf("d|\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f\n", dtw_distance_x , dtw_distance_y, dtw_distance_z);
    // Now you can use dtw_distance_x, dtw_distance_y, and dtw_distance_z for your comparison
}
bool isSimilar()
{
    printf("dtw_distance_x %f\n", dtw_distance_x);
    printf("dtw_distance_y %f\n", dtw_distance_y);
    printf("dtw_distance_z %f\n", dtw_distance_z);

    if (dtw_distance_x < DTW_THRESHOLD && dtw_distance_y < DTW_THRESHOLD && dtw_distance_z < DTW_THRESHOLD)
    {
        return true; // The sequences are similar
    }
    else
    {
        return false; // The sequences are not similar
    }
}
void startRecording()
{    
     recMutex.lock();
    isRecording = false;
    isReading = false;
    stop = false;
    record = true;
    complete = false;

    greenLed = 1; // Turn on green LED
    redLed = 0;
     recMutex.unlock();
}

void stopRecording()
{
    stopMutex.lock();
    isRecording = false;
    isReading = true;
    stop = false;
    record = false;

    greenLed = 0; // Turn off green LED
    redLed = 0;
     stopMutex.unlock();
}

void startReading()
{
   readMutex.lock();
    isReading = false;
    isRecording = false;
    stop = true;
    compare = true;

    greenLed = 0; // Turn off green LED
    redLed = 1;   // Turn on red LED
    readMutex.unlock();
}

void stopReading()
{
     completeMutex.lock();
    isReading = false;
    isRecording = true;
    stop = false;
    compare = false;
    complete = true;

    greenLed = 0; // Turn off green LED
    redLed = 0;   // Turn off red LED
     completeMutex.unlock();
}

void buttonPressedISR()
{
    // Set a flag to signal that the button has been pressed
    buttonPressFlag.set(1);
}

void buttonWorkerThread()
{
    // button.fall(NULL);  // Disable further interrupts until button is released

    while (true)
    {
        
        // Wait for the button press flag to be set
        buttonPressFlag.wait_any(1);
         

        mutex.lock();

         // Reset the button press flag
         buttonPressFlag.clear(1);

        if (button.read())
        {

            if (isRecording && !isReading && !stop)
                startRecording(); // Start recording hand gesture

            else if (!isRecording && !isReading && !stop)
                stopRecording(); // done recording hand gesture

            else if (!isRecording && isReading && !stop)
                startReading(); //  read hand gesture

            else if (!isRecording && !isReading && stop)
                stopReading(); //  done reading hand gesture
        }
        mutex.unlock();
    }
}

int main()
{

    setupGyro();
     // Set up the ISR
    button.rise(&buttonPressedISR);
    thread_sleep_for(0.1);

     // Start the worker thread
    Thread workerThread;
    workerThread.start(buttonWorkerThread);


    while (1)
    {

        if (record)
        {
            recordGesture();
        }
        else if (compare)
        {
            compareGesture();
        }
        else if (complete)
        {

            bool isGesture = isSimilar();
            if (isGesture)
            {
                printf("Gesture detected\n");
            }
            else
            {
                printf("Gesture Not detected\n");
            }
            complete = false;
        }
    }
}
