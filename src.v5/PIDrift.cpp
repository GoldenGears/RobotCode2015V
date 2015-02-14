//*************
// Golden Gears Robotics
// 2/8/15
// PIDrift Main
//*************

// Tyler Robbins - 2-8-15 - Added definition of PIDrift class.
// Tyler Robbins & Conlon Meek - 2-12-15 - Added a method to get the PID error from the PIDController.
// Tyler Robbins - 2-13-15 - Removed commented out code.

#include "PIDrift.h"

PIDrift::PIDrift(double p, double i, double d, ADXRS453Z *gyro)
	: PIDSubsystem("PIDrift Subsystem", p,i,d)
{
	m_gyro = gyro;
	output = 0;
	InitPIDrift();
}

void PIDrift::InitPIDrift(){
	SetPIDSourceParameter(kAngle);
}

double PIDrift::GetOutput(){
	return output;
}

void PIDrift::SetPIDSourceParameter(PIDSourceParameter param){
	m_param = param;
}

double PIDrift::ReturnPIDInput(){
	switch(m_param){
		case kRate:
			return m_gyro->GetRate();
		case kAngle:
			return m_gyro->GetAngle();
		default:
			return 0;
	}
}

void PIDrift::UsePIDOutput(double out){
	output = out;
}

float PIDrift::GetPIDError(){
	return GetPIDController()->GetError();
}
