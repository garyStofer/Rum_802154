/*
 * measure.h
 *
 *  Created on: Feb 4, 2015
 *      Author: gary
 */

#ifndef SENSORS_MEASURE_H_
#define SENSORS_MEASURE_H_

#include "../sensors.h" // for sftSensorReading type

extern void
Do_SensorMeasurment( sftSensorReading *reading);

extern void
Do_SensorInit( void );

#endif /* SENSORS_MEASURE_H_ */
