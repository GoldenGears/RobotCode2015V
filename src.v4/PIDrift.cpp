//*************
// Golden Gears Robotics
// 2/8/15
// PIDrift Main
//*************

// Tyler Robbins - 2-8-15 - Added definition of PIDrift class.

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
//	if(param == 0 or param > 2){
//		wpi_setWPIErrorWithContext(ParameterOutOfRange,"Gyro PIDrift");
//		return;
//	}

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
