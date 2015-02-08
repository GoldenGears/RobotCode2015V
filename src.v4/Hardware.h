//**************
// Golden Gears Robotics
// 2/7/15
// Hardware H
//**************

// Tyler Robbins - 2-7-15 - Added Hardware class definition.
// Tyler Robbins - 2-8-15 - Added PIDrift object.

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "WPILib.h"
#include "ADXRS453Z.h"
#include "PIDrift.h"

enum Talons{FRONT_LEFT_WHEEL,FRONT_RIGHT_WHEEL,BACK_LEFT_WHEEL,BACK_RIGHT_WHEEL,ELEVATOR_MOTOR};

class Hardware{
public:
	~Hardware();

	void move(Joystick *joy);

	void ElevatorPeriodic(bool up=false, bool down=false);

	void toggleSolenoid(Solenoid *sol);
	void setSolenoid(Solenoid *sol, bool value);

	float shiftThrottle(float throt);
	float filterJoyValue(float val);

	ADXRS453Z* GetGyro(){ return ggnore; };
	PIDrift* GetPIDrift(){ return drift; };
	RobotDrive* GetDrive(){ return drive; };
	TalonSRX* GetTalon(Talons tal){
		switch(tal){
			case Talons::FRONT_RIGHT_WHEEL: return frontRight;
			case Talons::FRONT_LEFT_WHEEL:  return frontLeft;
			case Talons::BACK_RIGHT_WHEEL:  return backRight;
			case Talons::BACK_LEFT_WHEEL:   return backLeft;
			case Talons::ELEVATOR_MOTOR:    return elevator;
		}
	};

	static Hardware* GetInstance();

	double GetGyroRate() { return ggnore->GetRate(); };
	double GetGyroAngle() { return ggnore->GetAngle(); };
	double GetGyroOffset() { return ggnore->GetOffset(); };

private:
	Hardware();
	void moveElevator(float pwr);
	void ElevatorUp(float percent);
	void ElevatorDown(float percent);

	TalonSRX *frontLeft;
	TalonSRX *frontRight;
	TalonSRX *backLeft;
	TalonSRX *backRight;
	TalonSRX *elevator;

	RobotDrive *drive;
	ADXRS453Z *ggnore;
	PIDrift *drift;

	static Hardware* m_instance;
};

#endif
