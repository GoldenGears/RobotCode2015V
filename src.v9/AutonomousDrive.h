//***********
// Golden Gears Robotics
// 2/14/15
// Autonomous Drive H
//***********

/*Header for the AutonomousDrive class.
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

// Tyler Robbins - 4-20-15 - Added GPL header. Added offset parameter to HasPassed method.

#ifndef _AUTONOMOUS_DRIVE_H_
#define _AUTONOMOUS_DRIVE_H_

#include "WPILib.h"

#include "Hardware.h"

enum AutoModes{MODE1,MODE2,MODE3,MODE4,MODE5,MODE6};

class AutonomousDrive : public Command{
public:
	AutonomousDrive(Hardware *hard,double seconds, float pwr, AutoModes mode = MODE2);
	~AutonomousDrive();

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

	void SetSol(bool state);
	void driveY(float pwr,bool useGyro=true);
	void driveX(float pwr,bool useGyro=true);
	void turn(float pwr);

	void SetSetpoint();
	void SetMode(AutoModes mode);

	void mode1();
	void mode2();
	void mode3();
	void mode4();
	void mode5();
	void mode6();

	double GetSeconds(){ return m_seconds; };
	float GetPower(){ return m_pwr; };
	Timer* GetTimer(){ return m_timer; };
	RobotDrive* GetDrive(){ return m_drive; };

private:
	float GetPIDrift();
	bool HasPassed(float sec,float offset=0);
	
	RobotDrive *m_drive;
	Hardware *m_hard;
	double m_seconds;
	float m_pwr;
	Timer *m_timer;
	int m_mode;
	int m_turnCounter;
};



#endif
