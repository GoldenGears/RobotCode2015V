//*************
// Golden Gears Robotics
// 1/31/15
// Robot MAIN
//*************

/*Entry point for the Robot Code
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

// Tyler Robbins - 1-11-15 - First Commit of Robot class. Wheels, compressor, and solenoid.
// Conlon Meek - 1-14-15 - Added individual Talon objects to pass to the RobotDrive classes.
// Tyler Robbins - 1-31-15 - Redid version control.
// Tyler Robbins - 1-31-15 - Added Joystick deadband for turning. Limited all joystick values to 2 decimal places.
// Tyler Robbins - 1-31-15 - Added elevator code (buttons, up and down movement).
// Tyler Robbins - 2-1-15 - Fixed turning.
// Tyler Robbins - 2-7-15 - Added methods for fixing drift with offset (not in use). Added gyro variable and methods. Added some printing methods. Used gyro to fix drift while driving.
// Tyler Robbins - 2-8-15 - Moved all methods relating to hardware (with the exception of the Compressor) to the Hardware class. Added hardware object.
// Tyler Robbins - 2-11-15 - Removed commented out code. Added goto joystick buttons. Added more print methods. Added elevator goto call.
// Tyler Robbins - 2-13-15 - Added check for if competition bot.
// Tyler Robbins - 2-14-15 - Removed check for if competition bot. Moved solenoid to Hardware class. Added code for test mode.
/* Tyler Robbins & Conlon Meek - 2-16-15 - Added AutonomousDrive object. Mapped a button to reset gyro. Integrated Autonomous code.
 *	Added code to reset gyro during disabled. Rearranged gyro method calls.
 */
// Tyler Robbins - 3-27-15 - Added code for an xbox controller and moved all controls for everything except driving to it from the normal joystick. Removed config methods.
/* Tyler Robbins - 4-20-15 - Added GPL header. Added mode times. Modified control system to support two controllers. Made all members but lw into a pointer. Removed stick.
 *  Replaced all instances of stick with driver1 or driver2 where appropriate. Made variable elevator speeds. Removed readConfig and configure methods. Added HasPassed method.
 */

#define AutoBots_RollOut TeleopInit

#include <math.h>
#include <vector>

#include <iostream>
#include <fstream>
#include <stdio.h>

#include "WPILib.h"

#include "ADXRS453Z.h"
#include "Hardware.h"
#include "AutonomousDrive.h"

const float TURN_OFFSET = 0.04;

enum modeTimes{AUTO=15,TELEOP=135}; // Seconds

class Robot: public IterativeRobot
{
	Timer *printTimer;
	// Joystick stick; // only joystick
	Joystick *driver1;
	Joystick *driver2;
	LiveWindow *lw;
	JoystickButton *solButt;
	JoystickButton *elevatorUpButt;
	JoystickButton *elevatorDownButt;
	JoystickButton *resetGyroButt;
	JoystickButton *armsButton;
	std::vector<JoystickButton*> *elevatorGoToButts;
	std::vector<JoystickButton*> *testButts;
	Hardware* hard = Hardware::GetInstance();
	AutonomousDrive* autod;
	Timer* matchTimer;
	bool competition_bot;

public:
	Compressor *c = new Compressor(0);
	Robot() :
		// stick(0),		// as they are declared above.
		lw(NULL)
//		gearShift(0),
//		solButt(driver2,1),
//		elevatorUpButt(driver2,5),
//		elevatorDownButt(driver2,6)
//		competition_bot(false)
	{
//		competition_bot = false;
		// New Control system so that one driver controls the driving, while the other controls the elevator and claw
		driver1 = new Joystick(0); // Normal joystick
		driver2 = new Joystick(1); // Xbox Controller

		solButt = new JoystickButton(driver2,1);
		elevatorUpButt = new JoystickButton(driver2,5);
		elevatorDownButt = new JoystickButton(driver2,6);

		elevatorGoToButts = new std::vector<JoystickButton*>();

		// List buttons in order of priority
		elevatorGoToButts->push_back(new JoystickButton(driver2,10));
		elevatorGoToButts->push_back(new JoystickButton(driver2,11));
		elevatorGoToButts->push_back(new JoystickButton(driver2,12));

		testButts = new std::vector<JoystickButton*>();
		testButts->push_back(new JoystickButton(driver1,10));
		testButts->push_back(new JoystickButton(driver1,11));
		testButts->push_back(new JoystickButton(driver1,12));
		testButts->push_back(new JoystickButton(driver1,9));

		resetGyroButt = new JoystickButton(driver1,1);
		autod = new AutonomousDrive(hard,15,2,AutoModes::MODE4);

		armsButton = new JoystickButton(driver2,1);

		printTimer = new Timer();
		matchTimer = new Timer();
		c->Start();

		printf(IsUserAGoat()?"The user is a goat.":"The user is a goat.");
	}

private:
	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		printf("Version 7.2\n");
		//c->Start();
		hard->GetGyro()->Start();
		hard->GetGyro()->Reset();
		hard->GetPIDrift()->Enable();
		hard->GetDrive()->SetSafetyEnabled(false);
	}

	void AutonomousInit()
	{
		//autoLoopCounter = 0;
		//c->Stop();
		hard->GetGyro()->Reset();
		autod->Start();
		c->Start();
//		ggnore->Start();
//		ggnore->Reset();
	}

	void AutonomousPeriodic()
	{
		Scheduler::GetInstance()->Run();
	}

	void AutoBots_RollOut()
	{
		printf((c->Enabled()?"Compressor On":"Compressor Off"));
		printf("\n");
	}

	void TeleopPeriodic()
	{
		hard->move(driver1);
		printGyro();
//		printJoyVal();

//		if(hard->GetGyroAngle() >= 360 or hard->GetGyroAngle() <= -360){
//			hard->GetGyro()->Reset();
//		}

//		hard->setSolenoid(&gearShift,solButt.Get());
//		hard->setSolenoid(hard->GetGabe(),solButt.Get());
		hard->setGabe(driver2->GetRawButton(1));

		// Uses the right-trigger on an xbox controller
//		hard->setGabe(driver2->GetRawAxis(3) > 0.5);

//		hard->ElevatorPeriodic(	elevatorUpButt->Get(),
//								elevatorDownButt->Get(),
//								getNextPosition(elevatorGoToButts));

		// Use triggers to give variable speeds now
		// Triggers are axises 2 and 3
		if(driver2->GetRawAxis(2) > 0)
			hard->ElevatorUp(driver2->GetRawAxis(2)*100);
		else if(driver2->GetRawAxis(3) > 0)
			hard->ElevatorDown(driver2->GetRawAxis(3)*100);
		else
			hard->ElevatorUp(10);

		// Uses the left-most joystick on an xbox controller
//		hard->ElevatorPeriodic( driver2->GetRawAxis(2) < -0.5,
//								driver2->GetRawAxis(2) >  0.5,
//								getNextPosition(elevatorGoToButts) );
//		hard->ArmsPeriodic(driver2->GetRawAxis(3) > 0.5);
//		hard->ArmsPeriodic(driver2->GetRawButton(1));
//		hard->ArmsPeriodic(armsButton->Get());

//		if(HasPassed(modeTimes::TELEOP-0.5)) // Half a second before teleop ends, drop whatever is in the claw
//			hard->setGabe(true);
	}

	void TestInit(){
		hard->GetGyro()->Reset();
	}

	void TestPeriodic()
	{
		lw->Run();
		printGyro();

		for(unsigned int i = 0; i < testButts->size(); i++){
			if(testButts->at(i)->Get())
				switch(i){
					case Talons::BACK_LEFT_WHEEL:
						hard->SetTalon(Talons::BACK_LEFT_WHEEL,50);
						break;
					case Talons::BACK_RIGHT_WHEEL:
						hard->SetTalon(Talons::BACK_RIGHT_WHEEL,50);
						break;
					case Talons::FRONT_LEFT_WHEEL:
						hard->SetTalon(Talons::FRONT_LEFT_WHEEL,50);
						break;
					case Talons::FRONT_RIGHT_WHEEL:
						hard->SetTalon(Talons::FRONT_RIGHT_WHEEL,50);
						break;
				}
//				hard->GetTalon(Talons::BACK_LEFT_WHEEL)->Set(.5);
//				hard->SetTalon(i,50.0);
		}
//		hard->GetTalon(Talons::BACK_RIGHT_WHEEL);
//		hard->GetTalon(Talons::FRONT_LEFT_WHEEL);
//		hard->GetTalon(Talons::FRONT_RIGHT_WHEEL);
		hard->setSolenoid(hard->GetGabe(),false);
	}

	void printLineSeparator(int length){
		for(int i = 0; i < length; i++){
			printf("=");
		}
	}

	void printGyro(){
		float div = fmod(printTimer->Get(),10);
		if(.1 > div and div > -.1){
			printf("Angle:%f\t\t",hard->GetGyroAngle());
			printf("Rate:%f\t\t",hard->GetGyroRate());
			printf("Offset:%f\n",hard->GetGyroOffset());
		}
	}

	void printJoy(){
		float div = fmod(printTimer->Get(),10);
		if(.1 > div and div < -.1){
			printf("X=%f\t\t",(driver1)->GetX());
			printf("Y=%f\t\t",(driver1)->GetY());
			printf("Twist=%f\t\t",(driver1)->GetTwist());
			printf("Throt=%f\n",(driver1)->GetThrottle());
		}
	}

	void SystemCheck(){	}

	bool IsUserAGoat(){
		return true;
	}

	int getNextPosition(std::vector<JoystickButton*> *buttons){
		// Return first button in list that returns true
		for(unsigned int i = 0; i < buttons->size(); i++){
			if(buttons->at(i)->Get())
				return i;
		}

		return -1;
	}

	void DisabledInit(){
		hard->GetGyro()->Reset();
	}

	void DisabledPeriodic(){
		if(resetGyroButt->Get())
			hard->GetGyro()->Reset();
	}

	bool isCompetitionRobot(){
		return competition_bot;
	}

	bool HasPassed(float seconds){
		return matchTimer->Get() >= seconds;
	}
};

START_ROBOT_CLASS(Robot);
