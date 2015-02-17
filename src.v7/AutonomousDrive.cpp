//***********
// Golden Gears Robotics
// 2/14/15
// Autonomous Drive MAIN
//***********

// Tyler Robbins - 2-14-15 - Added autonomous drive class. Has two possible autonomous modes, for breaking apart and picking up totes.
// Tyler Robbins & Conlon Meek - 2-16-15 - Added three more Autonomous modes. Added gyro correction. Fixed Timer issues.

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
	m_turnCounter = 0;

	srand(time(NULL));
}

AutonomousDrive::~AutonomousDrive(){
	delete m_timer;
}

void AutonomousDrive::Initialize(){
	printf("Initializing Autonomous in mode %d.\n",m_mode+1);
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
		case AutoModes::MODE3:
			mode3();
			break;
		case AutoModes::MODE4:
			mode4();
			break;
		case AutoModes::MODE5:
			mode5();
			break;
		default:
			printf("Invalid Mode %d",m_mode);
			break;
	}

}

bool AutonomousDrive::IsFinished(){
	// bool finished = m_timer->HasPeriodPassed(m_seconds);
	bool finished = HasPassed(m_seconds);
	// printf("Autonomous is %s.", (finished ? "finished":"not finished"));
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
	if(m_turnCounter == 0) SetSetpoint();
	m_turnCounter += 1;
	m_drive->MecanumDrive_Cartesian(0,pwr/100,GetPIDrift(),0.0);
}

void AutonomousDrive::driveX(float pwr){
	if(m_turnCounter == 0) SetSetpoint();
	m_turnCounter += 1;
	m_drive->MecanumDrive_Cartesian(pwr/100,0,GetPIDrift(),0.0);
}


void AutonomousDrive::turn(float pwr){
	m_turnCounter = 0;
	m_drive->MecanumDrive_Cartesian(0,0,pwr/100,0.0);
}

float AutonomousDrive::GetPIDrift(){
	// hard->GetDrift()->SetSetpoint(hard->GetGyroAngle());
	printf("%f\t\t",m_hard->GetPIDrift()->GetOutput());
	return m_hard->GetPIDrift()->GetOutput();
}

void AutonomousDrive::SetSetpoint(){
	m_hard->GetPIDrift()->SetSetpoint(m_hard->GetGyroAngle());
}

void AutonomousDrive::SetMode(AutoModes mode){
	m_mode = mode;
}

void AutonomousDrive::mode1(){
	// Designed for breaking apart the totes.
	if(HasPassed(5))
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

	if(HasPassed(4)){
		m_hard->setSolenoid(m_hard->GetGabe(),false);
	}else
		m_hard->setSolenoid(m_hard->GetGabe(),true);
		driveY(50);
}

void AutonomousDrive::mode3(){
	printf("%f\t\t",m_hard->GetGyroAngle());
	printf("%f\n",m_timer->Get());
	if(HasPassed(6)){
		driveX(0);
		driveY(0);
		turn(0);
	}
	else if(HasPassed(5))
		driveX(60);
	else if(HasPassed(4))
		driveX(-60);
	else if(HasPassed(3))
		driveY(60);
	else if(HasPassed(2))
		driveY(-60);
//	else if(m_timer->HasPeriodPassed(1))
	else if(HasPassed(1))
		turn(60);
//	else if(m_timer->HasPeriodPassed(0))
	else if(HasPassed(0))
		turn(-60);
	else{
		driveX(0);
		driveY(0);
		turn(0);
	}
}

void AutonomousDrive::mode4(){
//	printf("%f\t\t",m_hard->GetGyroAngle());
	if(HasPassed(3))
		driveY(0);
	else
		driveY(40);
	printf("%f\t\t",m_hard->GetGyroAngle());
	printf("%f\n",m_timer->Get());
}

void AutonomousDrive::mode5(){
	if(HasPassed(3))
		driveX(0);
	else
		driveX(-40);
	printf("%f\t\t",m_hard->GetGyroAngle());
	printf("%f\n",m_timer->Get());
}

bool AutonomousDrive::HasPassed(float sec){
	return m_timer->Get() >= sec;
}
