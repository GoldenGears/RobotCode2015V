//*************
// Golden Gears Robotics
// 2/8/15
// PIDrift Main
//*************

/*Class to handle drifting using the ADXRS453Z gyro
	Copyright (C) 2015  Golden Gears Robotics

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


// Tyler Robbins - 2-8-15 - Added definition of PIDrift class.
// Tyler Robbins & Conlon Meek - 2-12-15 - Added a method to get the PID error from the PIDController.
// Tyler Robbins - 2-13-15 - Removed commented out code.
// Tyler Robbins - 4-20-15 - Added GPL header.

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
