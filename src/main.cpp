/**
 * @file main.cpp
 * 
 * @author Group 12 : Jack Shkifati, Ishan U Taldekar, James Huang, and Inder Preet Walia
 * 
 * @brief 
 * Header files:
 * This program makes use of multiple header files to manage the functionality of a 
 * gesture recognition system. The config.h file contains system configurations and macro definitions. 
 * sensor.h manages the setup and data handling of the gyroscope sensor. gesture.h and gesture.cpp 
 * handle the recording and comparison of gestures. dtw.h contains the implementation of the 
 * Dynamic Time Warping (DTW) algorithm, which is used for gesture recognition.
 * 
 * Algorithm DTW: 
 * The Dynamic Time Warping (DTW) algorithm is used to measure the similarity between two temporal 
 * sequences, which may vary in speed. In the context of this project, it's used to compare a recorded 
 * gesture with a stored one. The algorithm essentially warps the time dimension of the two sequences to 
 * optimize the match between them. The sequences are "warped" non-linearly in the time dimension to 
 * determine a measure of their similarity independent of certain non-linear variations in the time 
 * dimension.
 * 
 * DTW has been implemented in this project presumably in the recordGesture(), compareGesture(), and 
 * isSimilar() functions, which are not shown in the given code but are called from the main loop. 
 * During the recording phase (recordGesture()), the gyroscope data is likely captured and stored. 
 * In the comparison phase (compareGesture()), the recorded gesture is probably compared with a previously 
 * stored gesture using the DTW algorithm. Lastly, the isSimilar() function might return a boolean 
 * indicating whether the recorded gesture is similar to the stored one, likely based on a certain 
 * similarity or threshold score from the DTW comparison.
 * 
 * Advantages:
 * The use of DTW in this project provides several advantages. The primary one is its ability to compare 
 * sequences of different lengths and speeds, which is crucial for gesture recognition as the same gesture 
 * can be performed at different speeds. Additionally, DTW is robust to noise and variations in gesture 
 * performance, making it a good choice for real-world applications such as this. The algorithm's 
 * flexibility and robustness make it a suitable choice for this gesture recognition system.
 * 
 * Flow Control:
 * The main flow of the program revolves around the manipulation of system_flags to determine the current 
 * state of the system (recording, reading, etc.) and respond to button presses accordingly. The use of 
 * mutexes ensures that the system_flags object is not accessed simultaneously by different threads, 
 * thereby preventing race conditions. The program also includes feedback to the user via LEDs to indicate 
 * when it is recording or reading a gesture. The program is event-driven, with a main loop that checks 
 * the current state and performs the appropriate action, and a separate worker thread that handles button 
 * press events.
 * 
 * Summary:
 * The code involves the use of several header files, each responsible for a specific part of the system:
 * 
 * 1. config.h: Contains macro definitions and configuration parameters for the system, such as register 
 * addresses and configurations, scaling factors, and system struct definitions.
 * 
 * 2. sensor.h: Likely contains the code required for setting up and reading data from the gyroscope.
 * 
 * 3. gesture.h and gesture.cpp: These files probably contain the code related to recording and comparing 
 * gestures. This is where the data from the sensor would be captured and processed into a format suitable 
 * for comparison.
 * 
 * 4. dtw.h: This file would contain the implementation of the Dynamic Time Warping (DTW) algorithm used 
 * for comparing gestures.
 * 
 * The main() function sets up the system, including the gyroscope and button interrupt, and then enters a 
 * loop where it continuously checks the system flags to determine whether it should be recording, comparing, 
 * or checking the results of a gesture comparison. The buttonWorkerThread() function handles button press 
 * events and updates the system flags accordingly.
 * 
 * The other functions (startRecording(), stopRecording(), startReading(), stopReading()) are used to 
 * update the system flags and LEDs based on the system's current state. These updates are performed 
 * within mutex locks to prevent other threads from accessing the flags at the same time, thereby 
 * preventing data inconsistencies and race conditions.
 * 
 * In conclusion, this code represents a well-structured and well-organized approach to a multi-threaded, 
 * event-driven system for gesture recognition using a gyroscope and the Dynamic Time Warping algorithm.
 * 
 * @version 0.1
 * 
 */

#include "config.h"  // Include all the macro definitions and configurations.
#include "sensor.h"  // Include code for gyroscope setup and data handling.
#include "gesture.h" // Include code for gesture recording and comparison.
#include "dtw.h"     // Include code for Dynamic Time Warping.
#include "rtos/EventFlags.h"  // Include the mbed OS library for event flag handling.

// Mutex objects for managing access to the system_flags object from different threads.
Mutex recMutex;
Mutex readMutex;
Mutex stopMutex;
Mutex completeMutex;
Mutex updateMutex;

// EventFlags object to handle button press events.
rtos::EventFlags buttonPressFlag;

/* Status LED outputs: */
DigitalOut greenLed(LED1); // Define a digital output for the green LED.
DigitalOut redLed(LED2);   // Define a digital output for the red LED.

// Button input
InterruptIn button(USER_BUTTON);   // Define an interrupt input for the button.

// Create an instance of the SystemFlags struct, declared as volatile because it can be modified by an ISR.
volatile SystemFlags system_flags; 



// Function to start recording the gesture.
void startRecording() {    
     
     // Lock the mutex to prevent other threads from accessing system_flags while we're using it.
     recMutex.lock();

    // Set the flags to indicate that the system is in a recording state.
    system_flags.isRecording = false;
    system_flags.isReading=false;
    system_flags.stop=false;
    system_flags.complete=false;
    system_flags.record= true;
    system_flags.compare=false;

    greenLed = 1; // Turn on green LED to indicate recording.
    redLed = 0;  // Unlock the mutex so that other threads can access system_flags.
    
     recMutex.unlock();
}

// Function to stop recording the gesture.
void stopRecording()
{

  // Lock the mutex to prevent other threads from accessing system_flags while we're using it.
    stopMutex.lock(); 

  // Set the flags to indicate that the system is in a stop recording state.
    system_flags.isRecording = false;
    system_flags.isReading= true;
    system_flags.stop= false;
    system_flags.complete= false;
    system_flags.record= false;
    system_flags.compare=false;

    greenLed = 0; // Turn off green LED.
    redLed = 0;   // Turn off red LED.
      
    // Unlock the mutex so that other threads can access system_flags.
     stopMutex.unlock();
}

// Function starts when you want to start recording the compare gesture.
void startReading()
{
   readMutex.lock();

 // Set the flags to indicate that the system is in a reading state.
    system_flags.isRecording = false;
    system_flags.isReading=false;
    system_flags.stop=true;
    system_flags.complete=false;
    system_flags.record=false;
    system_flags.compare=true;

    greenLed = 0; // Turn off green LED
    redLed = 1;   // Turn on red LED to indicate reading.
    readMutex.unlock();
}

// Function to stop reading the compare gesture.
void stopReading()
{
     completeMutex.lock();

    system_flags.isRecording = true;
    system_flags.isReading=false;
    system_flags.stop=false;
    system_flags.complete=true;
    system_flags.compare=false;

    greenLed = 0; // Turn off green LED
    redLed = 0;   // Turn off red LED
     completeMutex.unlock();
}

// Function to handle the button pressed interrupt.
void buttonPressedISR()
{
    // Set a flag to signal that the button has been pressed
    buttonPressFlag.set(1);
}

// Thread function to handle button presses.
void buttonWorkerThread()
{

    while (true)
    {
        
        // Wait for the button press flag to be set
        buttonPressFlag.wait_any(1);
         

        updateMutex.lock();

         // Reset the button press flag
         buttonPressFlag.clear(1);

        if (button.read())
        {
             // Check the current system state and take the appropriate action.
            if (system_flags.isRecording && !system_flags.isReading && !system_flags.stop)
                startRecording(); // Start recording hand gesture

            else if (!system_flags.isRecording && !system_flags.isReading && !system_flags.stop)
                stopRecording(); // done recording hand gesture

            else if (!system_flags.isRecording && system_flags.isReading && !system_flags.stop)
                startReading(); //  read hand gesture

            else if (!system_flags.isRecording && !system_flags.isReading && system_flags.stop)
                stopReading(); //  done reading hand gesture
        }
        updateMutex.unlock();
    }
}


int main()
{
    // system_flags.isRecording = true; // Set the initial system state to be recording.

    system_flags.isRecording = true;
    system_flags.isReading = false;
    system_flags.stop = false;
    system_flags.complete = false;
    system_flags.record = false;
    system_flags.compare =false;

    setupGyro(); // Set up the gyroscope.

    // Set up the ISR for the button press event.
    button.rise(&buttonPressedISR);


     // Start the worker thread to handle button presses.
    Thread workerThread;
    workerThread.start(buttonWorkerThread);


    while (1) {

        // Check the current system state and take the appropriate action.
         
         if (system_flags.record) {
            recordGesture();        // Record a hand gesture.
         } 
         
         else if (system_flags.compare) {
             compareGesture();  // Compare the recorded gesture with the stored gesture.
         }
         else if (system_flags.complete){
              
         // Check if the recorded gesture is similar to the stored gesture.
            bool isGesture = isSimilar(); 
            
            if (isGesture) printf("Gesture detected\n");
        
            else printf("Gesture Not detected\n");
        
            system_flags.complete = 0;   // Reset the complete flag.
        }
    }
}
