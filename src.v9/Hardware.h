//**************
// Golden Gears Robotics
// 2/7/15
// Hardware H
//**************

/*Header file for the Hardware class
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

// Tyler Robbins - 2-7-15 - Added Hardware class definition.
// Tyler Robbins - 2-8-15 - Added PIDrift object.
// Tyler Robbins - 2-14-15 - Added potentiometer code. Made TalonSRX's CANTalons. Added solenoid code.
/* Tyler Robbins - 4-20-15 - Added GPL header. Removed toggleArms method. Added ArmsPeriodic method.
 *  Made ElevatorUp and ElevatorDown public. Added HasPassed method. Added m_timer, lastTime, and hasArmsBeenToggled members.
*/

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "WPILib.h"
#include "ADXRS453Z.h"
#include "PIDrift.h"

enum Talons{FRONT_LEFT_WHEEL,FRONT_RIGHT_WHEEL,BACK_LEFT_WHEEL,BACK_RIGHT_WHEEL,ELEVATOR_MOTOR};

const float SPOOL_RADIUS = 1.25; // inches

class Hardware{
	friend class AutonomousDrive;

public:
	~Hardware();

	void move(Joystick *joy);

	void ElevatorPeriodic(bool up=false, bool down=false, int gotop=-1);

//	void toggleArms();
	void ArmsPeriodic(bool value);

	void toggleSolenoid(Solenoid *sol);
	void setSolenoid(Solenoid *sol, bool value);
	void setGabe(bool value);
	void resetDrift();

	float shiftThrottle(float throt);
	float filterJoyValue(float val);
	void SetTalon(Talons tal,float pwr);

	ADXRS453Z* GetGyro(){ 	return ggnore; 	};
	PIDrift* GetPIDrift(){ 	return drift; 	};
	RobotDrive* GetDrive(){ return drive; 	};
	CANTalon* GetTalon(Talons tal){
		switch(tal){
			case Talons::FRONT_RIGHT_WHEEL: return frontRight;
			case Talons::FRONT_LEFT_WHEEL:  return frontLeft;
			case Talons::BACK_RIGHT_WHEEL:  return backRight;
			case Talons::BACK_LEFT_WHEEL:   return backLeft;
			case Talons::ELEVATOR_MOTOR:    return elevator;
			default: 						return frontRight;
		}
	};
	Solenoid* GetGabe(){ return gabe; };

	AnalogPotentiometer* GetPotentiometer(){ return poten; }

	double GetGyroRate(){ 	return ggnore->GetRate(); 	};
	double GetGyroAngle(){ 	return ggnore->GetAngle(); 	};
	double GetGyroOffset(){ return ggnore->GetOffset(); };
	float GetPotenValue(){ 	return poten->Get(); 		};

	static Hardware* GetInstance();

	void ElevatorIncrement();
	void ElevatorDecrement();
	void ElevatorUp(float percent);
	void ElevatorDown(float percent);
private:
	Hardware();
	void moveElevator(float pwr);
	void ElevatorGoTo(int pos);
	float stripDecimals(float num,int place);
	float convertDisToVolt(float dist);
	float convertVolToDist(float volt);
	float GetNextPosition(JoystickButton* buttons);
	bool HasPassed(float sec,float offset=0);

	CANTalon *frontLeft;
	CANTalon *frontRight;
	CANTalon *backLeft;
	CANTalon *backRight;
	CANTalon *elevator;

	AnalogPotentiometer *poten;

	Solenoid *gabe;
	RobotDrive *drive;
	ADXRS453Z *ggnore;
	PIDrift *drift;
	Timer *m_timer;

	int lastTime;
	bool hasArmsBeenToggled;

	static Hardware* m_instance;
};

#endif
