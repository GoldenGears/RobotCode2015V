//***********
// Golden Gears Robotics
// 2/14/15
// Autonomous Drive MAIN
//***********

// Tyler Robbins - 2-14-15 - Added autonomous drive class. Has two possible autonomous modes, for breaking apart and picking up totes.

#include "AutonomousDrive.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

AutonomousDrive::AutonomousDrive(Hardware *hard, double seconds, float pwr, AutoModes mode)
	: Command("Autonomous Drive")
{
	m_hard = hard;
	m_drive = hard->GetDrive();
	m_seconds = seconds;
	m_pwr = pwr;
	m_timer = new Timer();
	m_mode = mode;

	srand(time(NULL));
}

AutonomousDrive::~AutonomousDrive(){
	delete m_timer;
}

void AutonomousDrive::Initialize(){
	m_timer->Start();
}

void AutonomousDrive::Execute(){
	switch(m_mode){
		case AutoModes::MODE1:
			mode1();
			break;
		case AutoModes::MODE2:
			mode2();
			break;
	}

}

bool AutonomousDrive::IsFinished(){
	bool finished = m_timer->HasPeriodPassed(m_seconds);
	printf("Autonomous is %s.", (finished ? "finished":"not finished"));
	return finished;
}

void AutonomousDrive::End(){
	m_timer->Stop();
	m_timer->Reset();
}

void AutonomousDrive::Interrupted(){
	printf("Interrupted.");
	End();
}

void AutonomousDrive::driveY(float pwr){
	m_drive->MecanumDrive_Cartesian(0,pwr/100,0,0.0);
}

void AutonomousDrive::driveX(float pwr){
	m_drive->MecanumDrive_Cartesian(pwr/100,0,0,0.0);
}

void AutonomousDrive::turn(float pwr){
	m_drive->MecanumDrive_Cartesian(0,0,pwr/100,0.0);
}

void AutonomousDrive::mode1(){
	if(m_timer->HasPeriodPassed(5))
		switch(rand()%2){
			case 0:
				turn(20);
				break;
			case 1:
				driveY(20);
				break;
			case 2:
				driveX(20);
				break;
			default:
				driveY(0);
				break;
		}
	else
		driveY(50);
}

void AutonomousDrive::mode2(){
	if(m_timer->HasPeriodPassed(4)){
		m_hard->setSolenoid(m_hard->GetGabe(),false);
	}else
		m_hard->setSolenoid(m_hard->GetGabe(),true);
		driveY(50);
}
