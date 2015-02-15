//***********
// Golden Gears Robotics
// 2/14/15
// Autonomous Drive H
//***********

#ifndef _AUTONOMOUS_DRIVE_H_
#define _AUTONOMOUS_DRIVE_H_

#include "WPILib.h"

#include "Hardware.h"

enum AutoModes{MODE1,MODE2};

class AutonomousDrive : public Command{
public:
	AutonomousDrive(Hardware *hard,double seconds, float pwr, AutoModes mode = MODE2);
	~AutonomousDrive();

	void Initialize() override;
	void Execute() override;
	bool IsFinished() override;
	void End() override;
	void Interrupted() override;

	void driveY(float pwr);
	void driveX(float pwr);
	void turn(float pwr);

	void mode1();
	void mode2();

	double GetSeconds(){ return m_seconds; };
	float GetPower(){ return m_pwr; };
	Timer* GetTimer(){ return m_timer; };
	RobotDrive* GetDrive(){ return m_drive; };

private:
	RobotDrive *m_drive;
	Hardware *m_hard;
	double m_seconds;
	float m_pwr;
	Timer *m_timer;
	int m_mode;
};



#endif
