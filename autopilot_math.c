/*
 * autopilot_math.c
 *
 *  Created on: 02-Oct-2015
 *      Author: atulya
 */
#include "main.h"

void initializeVector3fToZero(Vector3f *vec)
{
	vec->x = 0;
	vec->y = 0;
	vec->z = 0;
}

void initializeVector2fToZero(Vector2f *vec)
{
	vec->x = 0;
	vec->y = 0;
}

void resetQueue(float arr[], Queue_property *q_property)
{
	arr[0] = 0;
	q_property->first = 0;
	q_property->last = 0;
	q_property->counter = 0;
	q_property->is_empty = 1;
	q_property->is_full = 0;
}

float popQueue(float arr[], Queue_property *q_property)
{
	if(q_property->counter == 0)
		return -1;

	float elem = arr[q_property->first];

	q_property->first = (q_property->first + 1)%q_property->size;

	q_property->counter--;
	q_property->is_full = 0;

	if(q_property->counter == 0)
		q_property->is_empty = 1;
	else
		q_property->is_empty = 0;

	return elem;
}

void pushToQueue(float data, float arr[], Queue_property *q_property)
{
	arr[q_property->last] = data;
	q_property->last = (q_property->last + 1)%q_property->size;

	q_property->counter++;
	q_property->is_empty = 0;

	if(q_property->counter >= q_property->size)
	{
		if(q_property->counter > q_property->size)
			q_property->first = (q_property->first + 1)%q_property->size;
		q_property->counter = q_property->size;
		q_property->is_full = 1;
	}
	else
		q_property->is_full = 0;
}

void initializeLPF(LowPassFilter *lpf)
{
	lpf->output = 0;
	//TODO initialize frequency for the LPF
}

float applyLPF(LowPassFilter *lpf, float input, float dt)
{
	if (lpf->cutoff_freq <= 0.0f || dt <= 0.0f)
	{
		lpf->output = input;
		return lpf->output;
	}

	float rc = 1.0f/(2*M_PI_F*lpf->cutoff_freq);
	float alpha = constrain_float(dt/(dt+rc), 0.0f, 1.0f);
	lpf->output += (input - lpf->output) * alpha;

	return lpf->output;
}

void resetLPF(LowPassFilter *lpf, float val)
{
	lpf->output = val;
}

// initalizes P, I, filt_hz, Imax
void initializePI(Controller_PI_2D *pi, float kP, float kI, float imax, float filt_hz)
{
	pi->kP = kP;
	pi->kI = kI;
	pi->Imax = imax;
	pi->filt_hz = filt_hz;

	pi->integrator.x = 0;
	pi->integrator.y = 0;
	pi->dt = 0;
	pi->filt_alpha = 0;
	pi->input.x = 0;
	pi->input.y = 0;
	pi->reset_filter = 1;

	//TODO initialize the kP and kI values of the controller
}

// set_input - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void setPIInput(Controller_PI_2D *pi, Vector2f input, float dt)
{
    // don't process inf or NaN
    if (!isfinite(input.x) && !isfinite(input.y)) {
        return;
    }

    // reset input filter to value received
    if (pi->reset_filter) {
        pi->reset_filter = 0;
        pi->input = input;
    }

    // update filter and calculate derivative
    float rc = 1/(2*M_PI_F*pi->filt_hz);
    pi->filt_alpha = dt / (dt + rc);

    Vector2f input_filt_change;
    input_filt_change.x = (input.x - pi->input.x) * pi->filt_alpha;
    input_filt_change.y = (input.y - pi->input.y) * pi->filt_alpha;
    pi->input.x = pi->input.x + input_filt_change.x;
    pi->input.y = pi->input.y + input_filt_change.y;

    pi->dt = dt;
}

void resetPI_I(Controller_PI_2D *pi)
{
	pi->integrator.x = 0;
	pi->integrator.y = 0;
}

Vector2f getPI_P(Controller_PI_2D *pi)
{
	Vector2f P;
	P.x = pi->kP*pi->input.x;
	P.y = pi->kP*pi->input.y;
	return P;
}

Vector2f getPI_I(Controller_PI_2D *pi)
{
	Vector2f I;
	I.x = 0; I.y = 0;
	if(!(pi->kI == 0) && !(pi->dt == 0))
	{
		pi->integrator.x += ((float)pi->input.x * pi->kI) * pi->dt;
		pi->integrator.y += ((float)pi->input.y * pi->kI) * pi->dt;
		float integrator_length = pythagorous2(pi->integrator.x, pi->integrator.y);
		if(integrator_length > pi->Imax && integrator_length>0)
		{
			pi->integrator.x *= (pi->Imax / integrator_length);
			pi->integrator.y *= (pi->Imax / integrator_length);
		}
		I = pi->integrator;
	}
	return I;
}
// get_i_shrink - get_i but do not allow integrator to grow in length (it may shrink)
Vector2f getPI_I_shrink(Controller_PI_2D *pi)
{
	Vector2f I;
	I.x = 0; I.y = 0;
	if(!(pi->kI == 0) && !(pi->dt == 0))
	{
		float integrator_length = pythagorous2(pi->integrator.x, pi->integrator.y);
        float integrator_length_orig = min(integrator_length, pi->Imax);

        pi->integrator.x += ((float)pi->input.x * pi->kI) * pi->dt;
		pi->integrator.y += ((float)pi->input.y * pi->kI) * pi->dt;

        float integrator_length_new = pythagorous2(pi->integrator.x, pi->integrator.y);
        if ((integrator_length_new > integrator_length_orig) && (integrator_length_new > 0))
        {
        	pi->integrator.x *= (integrator_length_orig / integrator_length_new);
			pi->integrator.y *= (integrator_length_orig / integrator_length_new);
        }
        I = pi->integrator;
    }
    return I;
}

void initializePID(Controller_PID *pid, float kP, float kI, float kD, float imax, float filt_hz)
{
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->Imax = imax;
	pid->filt_hz = filt_hz;

	pid->derivative = 0;
	pid->dt = 0;
	pid->filt_alpha = 0;
	pid->input = 0;
	pid->integrator = 0;
	pid->reset_filter = 1;
	pid->result = 0;

	//TODO initialize kp, ki, kd and imax; dt at some other place
}

void resetPID_I(Controller_PID *pid)
{
	pid->integrator = 0;
}

void setPIDInput_FilterAll(Controller_PID *pid, float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (pid->reset_filter) {
        pid->reset_filter = 0;
        pid->input = input;
        pid->derivative = 0.0f;
    }

    // update filter and calculate derivative

    float rc = 1/(2*M_PI_F*pid->filt_hz);
	pid->filt_alpha = pid->dt / (pid->dt + rc);

	float input_filt_change = pid->filt_alpha * (input - pid->input);
    pid->input = pid->input + input_filt_change;

    if (pid->dt > 0.0f)
        pid->derivative = input_filt_change / pid->dt;

}

void setPIDInput_FilterD(Controller_PID *pid, float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (pid->reset_filter) {
            pid->reset_filter = 0;
            pid->input = input;
            pid->derivative = 0.0f;
        }

    float rc = 1/(2*M_PI_F*pid->filt_hz);
	pid->filt_alpha = pid->dt / (pid->dt + rc);

    // update filter and calculate derivative
    if (pid->dt > 0.0f) {
        float derivative = (input - pid->input) / pid->dt;
        pid->derivative = pid->derivative + pid->filt_alpha * (derivative-pid->derivative);
    }

    pid->input = input;
}

float getPID_P(Controller_PID *pid)
{
	return pid->kP*pid->input;
}

float getPID_I(Controller_PID *pid)
{
	float I = 0;
	if(!(pid->kI == 0) && !(pid->dt == 0))
	{
		pid->integrator += ((float)pid->input * pid->kI) * pid->dt;
		if (pid->integrator < -pid->Imax)
			pid->integrator = -pid->Imax;
		else
		{
			if (pid->integrator > pid->Imax)
				pid->integrator = pid->Imax;
		}
		I = pid->integrator;
	}
	return I;
}

float getPID_D(Controller_PID *pid)
{
	return pid->kD * pid->derivative;
}
