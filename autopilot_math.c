/*
 * autopilot_math.c
 *
 *  Created on: 02-Oct-2015
 *      Author: atulya
 */
#include "main.h"
#include "autopilot_math.h"


void resetQueue(float arr[], Queue_property *q_property)
{
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
