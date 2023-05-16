/**
 * @file dtw.h
 * 
 * @author  Group 12 : Jack Shkifati, Ishan U Taldekar, James Huang, and Inder Preet Walia
 * 
 * @brief This code defines functions for performing Dynamic Time Warping (DTW) on two sequences. 
 * DTW is a technique used for measuring the similarity between two temporal sequences. The script 
 * uses a Mutex for thread safety when performing calculations on the shared dtw_matrix.
 * 
 * @version 0.1
 */
#ifndef DTW_H
#define DTW_H

#include <math.h>
#include <float.h>
#include "rtos/Mutex.h"
#include "config.h"

// Mutex object used to synchronize access to shared resources in multi-threaded context
rtos::Mutex mutex;


// 2D array to store DTW (Dynamic Time Warping) distance matrix
float dtw_matrix[MAX_SEQUENCE_LENGTH][MAX_SEQUENCE_LENGTH];

// Function prototype declarations
float getMinimumOfThree(float num1, float num2, float num3);
float calculateEuclideanDistance(float point1, float point2);
float performDTW(float *sequence1, float *sequence2, int sequenceLength);


// Function to return the minimum of three floating-point numbers
float getMinimumOfThree(float num1, float num2, float num3){
    float minimumValue = num1;

    if (num2 < minimumValue)
        minimumValue = num2;
    if (num3 < minimumValue)
        minimumValue = num3;

    return minimumValue;
}

// Function to calculate the Euclidean distance between two points in one-dimensional space
float calculateEuclideanDistance(float point1, float point2)
{
    return sqrtf(powf(point1 - point2, 2));
}

/**
 * @brief Function to perform the Dynamic Time Warping (DTW) algorithm on two input sequences
 * DTW is an algorithm used for measuring similarity between two temporal sequences.
 * @param sequence1 The first sequence to be compared
 * @param sequence2 The second sequence to be compared
 * @param sequenceLength The length of the sequences
 * @return float The Dynamic Time Warping distance between the two sequences
 */
float DTW(float *sequence1, float *sequence2, int sequenceLength)
{
    // Lock the mutex to ensure safe access to the shared resources
    mutex.lock();

   // Initialize all elements of the DTW matrix to the maximum possible float value
    for (int i = 0; i < sequenceLength; i++)
    {
        for (int j = 0; j < sequenceLength; j++)
        {
            dtw_matrix[i][j] = FLT_MAX;
        }
    }

    dtw_matrix[0][0] = 0; // Set the first element of the DTW matrix to zero
    
    // Calculate the DTW distance matrix
    for (int i = 1; i < sequenceLength; i++)
    {
        for (int j = 1; j < sequenceLength; j++)
        {
            float cost = calculateEuclideanDistance(sequence1[i], sequence2[j]);
            dtw_matrix[i][j] = cost + getMinimumOfThree(dtw_matrix[i - 1][j],
                                                         dtw_matrix[i][j - 1],
                                                         dtw_matrix[i - 1][j - 1]);
        }
    }

    mutex.unlock(); // Unlock the mutex to allow other threads to access the shared resources

   // Return the bottom-right value of the DTW matrix which is the total DTW distance
    return dtw_matrix[sequenceLength - 1][sequenceLength - 1];
}


#endif // DTW_H