//*************
// Golden Gears Robotics
// 2/8/15
// PIDrift H
//*************

/*Header file for the PIDrift class
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


// Tyler Robbins - 2-8-15 - Added PID Control Loop subsystem class to deal with motor drift.
// Tyler Robbins & Conlon Meek - 2-13-15 - Added a method to get the PID error from the PIDController.
// Tyler Robbins - 4-20-15 - Added GPL header.

#ifndef _PIDRIFT_H_
#define _PIDRIFT_H_

#include "Commands/PIDSubsystem.h"
#include "PIDSource.h"

#include "ADXRS453Z.h"

class PIDrift : public PIDSubsystem{
public:
	PIDrift(double p, double i, double d, ADXRS453Z *gyro);
	double GetOutput();
	float GetPIDError();
	virtual void SetPIDSourceParameter(PIDSourceParameter param);
protected:
	double ReturnPIDInput();
	void UsePIDOutput(double out);
private:
	void InitPIDrift();
	ADXRS453Z *m_gyro;
	double output;
	PIDSourceParameter m_param;
};

#endif
