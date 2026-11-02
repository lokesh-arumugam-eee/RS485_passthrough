#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include "sensirion_gas_index_algorithm.h"



// OPT4048	Precision Light/Color Sensor	0x45
// BME680	Temp, Humidity, Pressure & Gas	0x76 or 0x77
// BMM350	3-Axis Magnetometer (Compass)	0x10
// SHT45	High-Accuracy Humidity & Temp	0x44
// SGP4x	VOC/NOx Air Quality Sensor	0x59
// VL53L1CX	Time-of-Flight (ToF) Distance	0x29
//BNO086	9-Axis Orientation (IMU)	0x4B or 0x4A - In hardware it is not working
typedef enum
{
 PRECISION_LIGHT,
 TEMP_HUMIDITY,
 THREE_AXIS_MAGNETOMETER,
 HIGH_ACC_HUMIDITY_TEMP,
 AIR_QUALITY_SENSOR,
 TIME_OF_FLIGHT_DISTANCE,
 NINE_AXIS_ORIENTATION_SENSOR
}sensor_t;

void Read_sensor(uint8_t ucchoose_sensor);
void i2c_master_init();
void Read_sensor_task(void *arg);

extern SemaphoreHandle_t i2c_mutex;
#endif