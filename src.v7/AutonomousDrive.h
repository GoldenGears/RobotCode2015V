//***********
// Golden Gears Robotics
// 2/14/15
// Autonomous Drive H
//***********

#ifndef _AUTONOMOUS_DRIVE_H_
#define _AUTONOMOUS_DRIVE_H_

#include "WPILib.h"

#include "Hardware.h"

enum AutoModes{MODE1,MODE2,MODE3,MODE4,MODE5};

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

	void SetSetpoint();
	void SetMode(AutoModes mode);

	void mode1();
	void mode2();
	void mode3();
	void mode4();
	void mode5();

	double GetSeconds(){ return m_seconds; };
	float GetPower(){ return m_pwr; };
	Timer* GetTimer(){ return m_timer; };
	RobotDrive* GetDrive(){ return m_drive; };

private:
	float GetPIDrift();
	bool HasPassed(float sec);
	
	RobotDrive *m_drive;
	Hardware *m_hard;
	double m_seconds;
	float m_pwr;
	Timer *m_timer;
	int m_mode;
	int m_turnCounter;
};



#endif
