//***********
// Golden Gears Robotics
// 2/14/15
// Autonomous Drive MAIN
//***********

/*Defines what to do during Autonomous mode
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


// Tyler Robbins - 2-14-15 - Added autonomous drive class. Has two possible autonomous modes, for breaking apart and picking up totes.
// Tyler Robbins & Conlon Meek - 2-16-15 - Added three more Autonomous modes. Added gyro correction. Fixed Timer issues.
// Tyler Robbins - 4-20-15 - Added GPL header. Decreased mode4 times by .5 seconds. Changed mode5. Added offset to HasPassed method.

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
		case AutoModes::MODE6:
			mode6();
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

void AutonomousDrive::driveY(float pwr,bool useGyro){
	if(m_turnCounter == 0) SetSetpoint();
	m_turnCounter += 1;
	if(useGyro)
		m_drive->MecanumDrive_Cartesian(0,pwr/100,GetPIDrift(),0.0);
	else
		m_drive->MecanumDrive_Cartesian(0,pwr/100,0,0.0);
}

void AutonomousDrive::driveX(float pwr,bool useGyro){
	if(m_turnCounter == 0) SetSetpoint();
	m_turnCounter += 1;
	if(useGyro)
		m_drive->MecanumDrive_Cartesian(pwr/100,0,GetPIDrift(),0.0);
	else
		m_drive->MecanumDrive_Cartesian(0,pwr/100,0,0.0);
}


void AutonomousDrive::turn(float pwr){
	m_turnCounter = 0;
	m_drive->MecanumDrive_Cartesian(0,0,pwr/100,0.0);
}

void AutonomousDrive::SetSol(bool state){
	m_hard->setSolenoid(m_hard->GetGabe(),state);
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
		switch(rand()%4){
			case 0:
				turn(20);
				break;
			case 1:
				driveY(20);
				break;
			case 2:
				driveX(20);
				break;
			case 3:
				turn(-20);
				break;
			case 4:
				driveX(-20);
				break;
			case 5:
				driveY(-20);
				break;
			default:
				driveY(0);
				break;
		}
	else
		driveY(50);
}

void AutonomousDrive::mode2(){
	if(HasPassed(15)){
		driveY(0);
		m_hard->setSolenoid(m_hard->GetGabe(),false);
	}else if(HasPassed(14))
		driveY(-50);
	else if(HasPassed(12))
		m_hard->setSolenoid(m_hard->GetGabe(),true);
	else if(HasPassed(10)){
		driveY(0);
		m_hard->ElevatorDown(20);
	}else if(HasPassed(8)){
		m_hard->ElevatorUp(10);
		driveY(50);
	}else if(HasPassed(6))
		m_hard->ElevatorUp(50);
	else if(HasPassed(5))
		driveY(0);
	else if(HasPassed(4)){
		driveY(-5);
		m_hard->setSolenoid(m_hard->GetGabe(),false);
	}else{
		m_hard->setSolenoid(m_hard->GetGabe(),true);
		driveY(50);
	}
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
	if(HasPassed(12.30))
		turn(0);
	else if(HasPassed(9.7)){
		driveY(0);
		turn(-30);
	}else if(HasPassed(8.7))
		driveY(5);
	else if(HasPassed(4.5))
		driveY(-30,false);
	else if(HasPassed(3.7)){
		turn(0);
	}else if(HasPassed(1.5))
		turn(-30);
	else if(HasPassed(1))
		m_hard->ElevatorUp(10);
	else if(HasPassed(0))
		m_hard->ElevatorUp(50);
	else{
		driveY(0);
		driveX(0);
		turn(0);
	}
}

void AutonomousDrive::mode5(){
	if(HasPassed(5))
		driveY(0);
	else if(HasPassed(1)){
		driveY(-50);
		m_hard->ElevatorUp(10);
	}else
//		driveY(40);
		m_hard->ElevatorUp(50);
	printf("%f\t\t",m_hard->GetGyroAngle());
	printf("%f\n",m_timer->Get());
}

void AutonomousDrive::mode6(){
	/*Null*/
}



bool AutonomousDrive::HasPassed(float sec,float offset){
	return (m_timer->Get()-offset) >= sec;
}
