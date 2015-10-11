/*
 * autopilot_math.h
 *
 *  Created on: 02-Oct-2015
 *      Author: atulya
 */

#include "math.h"

#ifndef AUTOPILOT_MATH_H_
#define AUTOPILOT_MATH_H_

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

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

#define AC_PI_2D_FILT_HZ_DEFAULT  20.0f   // default input filter frequency
#define AC_PID_FILT_HZ_DEFAULT  20.0f   // default input filter frequency

typedef struct
{
	float kP;
}Controller_P;

typedef struct
{
	float kP;
	float kI;
	float Imax;
	float filt_hz;
	uint8_t reset_filter;

	float dt;
	Vector2f input;
	Vector2f integrator;
	float filt_alpha;
	Vector2f result;
}Controller_PI_2D;

typedef struct
{
	float kP;
	float kI;
	float kD;
	float Imax;
	float filt_hz;
	uint8_t reset_filter;

	float dt;
	float input;
	float integrator;
	float derivative;
	float filt_alpha;
	float result;
}Controller_PID;

// 2D vector length
static inline float pythagorous2(float a, float b) {
	return sqrt(a*a+b*b);
}
// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
static inline float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

// constrain a value
static inline float constrain_float(float amt, float low, float high)
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

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
// intialize the state and cutoff frequency of the lpf
void initializeLPF(LowPassFilter *lpf);

// reset the state of the LPF to the current value
void resetLPF(LowPassFilter *lpf, float val);

// apply the low pass complimentary filter on the output
float applyLPF(LowPassFilter *lpf, float input, float dt);
////------------------------------------------////

////////////////CONTROLLER LIBRARIES/////////////////////

/////////////------------PI-----------////////////

void initializePI(Controller_PI_2D *pi);
void setPIInput(Controller_PI_2D *pi, Vector2f input, float dt);
void resetPI_I(Controller_PI_2D *pi);
Vector2f getPI_P(Controller_PI_2D *pi);
Vector2f getPI_I(Controller_PI_2D *pi);
Vector2f getPI_I_shrink(Controller_PI_2D *pi);

////////////////////////PID///////////////////////

void intializePID(Controller_PID *pid);
void setPIDInput_FilterAll(Controller_PID *pid, float input);
void setPIDInput_FilterD(Controller_PID *pid, float input);
void resetPID_I(Controller_PID *pid);
float getPID_P(Controller_PID *pid);
float getPID_I(Controller_PID *pid);
float getPID_D(Controller_PID *pid);

//////////////////////////////////////////////////

#endif /* AUTOPILOT_MATH_H_ */
