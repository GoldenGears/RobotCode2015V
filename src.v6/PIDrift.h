//*************
// Golden Gears Robotics
// 2/8/15
// PIDrift H
//*************

// Tyler Robbins - 2-8-15 - Added PID Control Loop subsystem class to deal with motor drift.
// Tyler Robbins & Conlon Meek - 2-13-15 - Added a method to get the PID error from the PIDController.

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
