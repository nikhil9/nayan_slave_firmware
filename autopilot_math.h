/*
 * autopilot_math.h
 *
 *  Created on: 02-Oct-2015
 *      Author: atulya
 */

#ifndef AUTOPILOT_MATH_H_
#define AUTOPILOT_MATH_H_

typedef struct{
	float x;
	float y;
	float z;
}vector_3f;

/**
 * @brief implements a 2 dimensional vector of type float
 */
typedef struct
{
	float x;
	float y;
}Vector2f;

/**
 * @brief implements a 3 dimensional vector of type float
 */
typedef struct
{
	float x;
	float y;
	float z;
}Vector3f;

/**
 * @brief contains the basic elements that define a queue
 */
typedef struct
{
	uint8_t first;			/**< stores the index of first element of queue*/
	uint8_t last;			/**< stores the index of last element of queue*/
	uint8_t size;			/**< stores the size of the buffer queue*/
	uint8_t counter;		/**< a counter for number of variables stored in the queue*/
	uint8_t is_full;		/**< a flag for storing whether the buffer is full*/
	uint8_t is_empty;		/**< a flag for storing whether the buffer is empty*/
}Queue_property;

typedef struct
{
	float cutoff_freq;
	float sampling_freq;
	float alpha;

	float output;				//system state on which lpf is applied
}LowPassFilter;


/**
 * @brief pops the first element of the queue and returns it
 * @param arr the array storing all the elements of the queue
 * @param q_property stores the current state of the queue
 */
void resetQueue(float arr[], Queue_property *q_property);

/**
 * @brief pops the first element of the queue and returns it
 * @param arr the array storing all the elements of the queue
 * @param q_property stores the current state of the queue
 */
float popQueue(float arr[], Queue_property *q_property);

/**
 * @brief pushes an element into the queue
 * @param data element to be pushed into the array
 * @param arr the array storing all the elements of the queue
 * @param q_property stores the current state of the queue
 */
void pushToQueue(float data, float arr[], Queue_property *q_property);

////////////----Low Pass Filter-------/////////////
// TODO
void initializeLPF(LowPassFilter *lpf);

// TODO
void resetLPF(LowPassFilter *lpf, float val);

// TODO
void applyLPF(LowPassFilter *lpf, float input, float dt);
////------------------------------------------////

#endif /* AUTOPILOT_MATH_H_ */
