//*******************
// Golden Gears Robotics
// 2/1/15
// ADXRS453Z MAIN
// http://www.chiefdelphi.com/forums/showthread.php?p=1426540
// http://www.chiefdelphi.com/forums/showthread.php?t=130538&highlight=Best+Gyro
// http://www.chiefdelphi.com/forums/showthread.php?p=1430798
// https://github.com/FRC830/2015Robot
//*******************

// Tyler Robbins - 2-7-15 - Added a class for handling the digital gyro: ADXRS453Z
// Tyler Robbins & Conlon Meek - 2-16-15 - Added method and call to method to check if done calibrating.

// Special thanks to team 1296

//"
#include "ADXRS453Z.h"

int ADXRS453Z::id = 0;

int ADXRS453ZUpdateFunction(int pointer_val){
	ADXRS453Z* conlon = (ADXRS453Z*) pointer_val;
	Timer *petey = new Timer();
	petey->Start();
	while(true){
		conlon->Update();
		SmartDashboard::PutNumber("Gyro Loop Time", petey->Get());
		petey->Reset();
	}
	return 0;
}

ADXRS453Z::ADXRS453Z(){
	spi = new SPI(SPI::Port::kOnboardCS0);
	spi->SetClockRate(4000000); //4 MHz
	spi->SetClockActiveHigh();
	spi->SetChipSelectActiveLow();
	spi->SetMSBFirst();

	command[0] = READ_COMMAND;
	command[1] = 0;
	command[2] = 0;
	command[3] = 0;

	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;

	accumulated_angle = 0.0;
	current_rate = 0.0;
	accumulated_offset = 0.0;
	rate_offset = 0.0;
	calibration_count = 0;
	update_timer = new Timer();
	calibration_timer = new Timer();

	update_timer->Start();
	calibration_timer->Start();
	char buffer[50];
	sprintf(buffer,"adxrs453zupdate_%d",id);
	update_task = new Task(buffer,(FUNCPTR)&ADXRS453ZUpdateFunction);
	task_started = false;

	this_time = 0.0;
	last_time = 0.0;
	id++;
}

ADXRS453Z::~ADXRS453Z(){}

void ADXRS453Z::Start(){
	if(task_started){
		update_task->Resume();
	}else{
		update_task->Start((int) this);
		task_started = true;
//		Calibrate();
	}
}

void ADXRS453Z::Stop(){
	if(task_started)
		update_task->Suspend();
}

int ADXRS453Z::GetData(){
	check_parity(command);
	spi->Transaction(command,data,DATA_SIZE);

	return assemble_sensor_data(data);
}

void ADXRS453Z::Update(){
//	calibration_timer->Start();
	check_parity(command);
	spi->Transaction(command,data,DATA_SIZE);

	if(calibration_timer->Get() < 5.0){
		last_time = this_time = update_timer->Get();
		return;
	}
	else if(calibration_timer->Get() < 15.0)
//	else if(calibration_count < 6000)
		Calibrate();
	else if(calibration_timer->Get() < 15.01)
		printf(HasFinishedCalibrating()?"Done\n":"");
	else
		UpdateData();

//	to_binary_string(data[0],sensor_output_1);
//	to_binary_string(data[0],sensor_output_2);
//	to_binary_string(data[0],sensor_output_3);
//	to_binary_string(data[0],sensor_output_4);
//
//	SmartDashboard::PutString("gyro sensor data 1",sensor_output_1);
//	SmartDashboard::PutString("gyro sensor data 2",sensor_output_2);
//	SmartDashboard::PutString("gyro sensor data 3",sensor_output_3);
//	SmartDashboard::PutString("gyro sensor data 4",sensor_output_4);
}

void ADXRS453Z::UpdateData(){
	int sensor_data = GetData();

	float rate = ((float)sensor_data)/80.0;

	current_rate = rate;
	current_rate -= rate_offset;
//	current_rate -= 6.9;
//	update_timer->Start();
	this_time = update_timer->Get();

	accumulated_offset += rate*(this_time - last_time);
	accumulated_angle += (this_time-last_time)*current_rate;

	last_time = this_time;
//	update_timer->Reset();
}

void ADXRS453Z::Calibrate(){
	int sensor_data = GetData();
	float rate = ((float)sensor_data)/80.0;
	update_timer->Start();
	calibration_timer->Start();
	this_time = update_timer->Get();
	accumulated_offset += rate*(this_time-last_time);
	last_time = this_time;
	rate_offset = accumulated_offset/(calibration_timer->Get()-5.0);
//	update_timer->Reset();
	calibration_count ++;
}

float ADXRS453Z::GetRate(){
	return current_rate;
}

float ADXRS453Z::GetAngle(){
	return accumulated_angle;
}

float ADXRS453Z::GetOffset(){
	return rate_offset;
}

void ADXRS453Z::Reset(){
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	current_rate = 0.0;
	calibration_count = 0;
	accumulated_angle = 0.0;
	rate_offset = 0.0;
	accumulated_offset = 0.0;
	calibration_timer->Reset();
//	update_timer->Stop();
	update_timer->Reset();
}

bool ADXRS453Z::HasFinishedCalibrating(){
	return calibration_timer->Get() > 15.0;
}

void ADXRS453Z::DataAssemblyTest(){
	unsigned char data[4] = {0x00,0x00,0x00,0xFF};
	SmartDashboard::PutNumber("gyro test",assemble_sensor_data(data));
}

short ADXRS453Z::assemble_sensor_data(unsigned char* data){
	//cast to short to make space for shifts
	//the 16 bits from the gyro are a 2's complement short
	//so we just cast it to a C++ short
	//the data is split across the output like this (MSB first): (D = data bit, X = not data)
	// X X X X X X D D | D D D D D D D D | D D D D D D X X | X X X X X X X X X
	return ((short) (data[0] & FIRST_BYTE_DATA)) << 14
			| ((short) data[1]) << 6
			| ((short) (data[2] & THIRD_BYTE_DATA)) << 2;
}

void ADXRS453Z::check_parity(unsigned char * command){
	int num_bits = bits(command[0]) + bits(command[2]) + bits(command[3]);
	if (num_bits % 2 == 0)
		command[3] |= PARITY_BIT;
}

int ADXRS453Z::bits(unsigned char val){
	int n = 0;
	while (val){
		val &= val-1;
		n+=1;
	}

	return n;
}
//" (FRC 830)(FRC 1296).

/*******************************
 * Works Cited
 * "FRC830/2015Robot." GitHub. Team 830, 2 Feb. 15. Web. 04 Feb. 2015. <https://github.com/FRC830/2015Robot/tree/master/util>.
 * "FRC1296/RHSRobot2015." GitHub. Team 1296, 07 Feb. 2015. Web. 07 Feb. 2015. <https://github.com/FRC1296/RHSRobot2015>.
 *******************************/
