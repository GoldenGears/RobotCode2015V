//*******************
// Golden Gears Robotics
// 2/1/15
// ADXRS453Z H
//*******************

// Tyler Robbins - 2-7-15 - Added a class for handling the digital gyro: ADXRS453Z
// Tyler Robbins & Conlon Meek - 2-16-15 - Added method to check if done calibrating.

#ifndef _ADXRS453Z_H_
#define _ADXRS453Z_H_

#include "WPILib.h"

class ADXRS453Z {
public:
	ADXRS453Z();
	virtual ~ADXRS453Z();
	void Start();
	void Stop();
	float GetRate();
	float GetAngle();
	float GetOffset();
	void Reset();
	void Update();
	void Offset();
	bool HasFinishedCalibrating();

	static void DataAssemblyTest();

private:
	int GetData();
	void UpdateData();
	void Calibrate();

	static void check_parity(unsigned char* command); // requires odd parity for command
	static int bits(unsigned char val);					// return number of on bits in a byte

	static short assemble_sensor_data(unsigned char * data); // takes the sensor data from the data array and puts it into an int

	static const unsigned char DATA_SIZE = 4; //4 bytes = 32 bits
	static const unsigned char PARITY_BIT = 1; //parity check on first bit
	static const unsigned char FIRST_BYTE_DATA = 0x3; //mask to find sensor data bits on first byte: X X X X X X D D
	static const unsigned char THIRD_BYTE_DATA = 0xFC; //mask to find sensor data bits on third byte: D D D D D D X X

	static const unsigned char READ_COMMAND = 0x20; //0010 0000 for first byte

	float accumulated_angle;
	Timer* update_timer;
	Timer* calibration_timer;
	float current_rate;
	float accumulated_offset;
	float rate_offset;

	unsigned char command[4];
	unsigned char data[4];
	SPI *spi;

	char sensor_output_1[9];
	char sensor_output_2[9];
	char sensor_output_3[9];
	char sensor_output_4[9];

	bool task_started;
	Task *update_task;

	int calibration_count;

	float this_time;
	float last_time;

	static int id;
};



#endif
