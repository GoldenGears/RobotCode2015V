//*************
// Golden Gears Robotics
// 2/8/15
// PIDrift H
//*************

// Tyler Robbins - 2-8-15 - Added PID Control Loop subsystem class to deal with motor drift.

#ifndef _PIDRIFT_H_
#define _PIDRIFT_H_

#include "Commands/PIDSubsystem.h"
#include "PIDSource.h"

#include "ADXRS453Z.h"

class PIDrift : public PIDSubsystem{
public:
	PIDrift(double p, double i, double d, ADXRS453Z *gyro);
	double GetOutput();
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

#endif /* SRC_PIDRIFT_H_ */
