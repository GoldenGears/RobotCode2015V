//*************
// Golden Gears Robotics
// 1/31/15
// Robot MAIN
//*************

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

// const float matchTimes[] = {};

class Robot: public IterativeRobot
{
	Timer *printTimer;
	Joystick stick; // only joystick
	LiveWindow *lw;
//	Solenoid *gearShift;	// There is no more gearshift, this controls the claw.
	JoystickButton solButt;
	JoystickButton elevatorUpButt;
	JoystickButton elevatorDownButt;
	JoystickButton *resetGyroButt;
//	JoystickButton *elevatorGoToButts[];
	std::vector<JoystickButton*> *elevatorGoToButts;
	std::vector<JoystickButton*> *testButts;
	Hardware* hard = Hardware::GetInstance();
	AutonomousDrive* autod;
	Timer* matchTimer;
	bool competition_bot;

public:
	Compressor *c = new Compressor(0);
	Robot() :
		stick(0),		// as they are declared above.
		lw(NULL),
//		gearShift(0),
		solButt(&stick,2),
		elevatorUpButt(&stick,5),
		elevatorDownButt(&stick,3),
		competition_bot(false)
	{
		elevatorGoToButts = new std::vector<JoystickButton*>();

		// List buttons in order of priority
		elevatorGoToButts->push_back(new JoystickButton(&stick,10));
		elevatorGoToButts->push_back(new JoystickButton(&stick,11));
		elevatorGoToButts->push_back(new JoystickButton(&stick,12));

		testButts = new std::vector<JoystickButton*>();
		testButts->push_back(new JoystickButton(&stick,10));
		testButts->push_back(new JoystickButton(&stick,11));
		testButts->push_back(new JoystickButton(&stick,12));
		testButts->push_back(new JoystickButton(&stick,9));

		resetGyroButt = new JoystickButton(&stick,1);
		autod = new AutonomousDrive(hard,15,2,AutoModes::MODE4);

		printTimer = new Timer();
		matchTimer = new Timer();
		c->Start();

		printf(IsUserAGoat()?"The user is a goat.":"The user is a goat.");

		//*************************************
		// REMOVE ME AFTER I RUN ON REAL_ROBOT!
		//*************************************
//		configure();

//		readConfig();
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
		hard->move(&stick);
		printGyro();
//		printJoyVal();

//		if(hard->GetGyroAngle() >= 360 or hard->GetGyroAngle() <= -360){
//			hard->GetGyro()->Reset();
//		}

//		hard->setSolenoid(&gearShift,solButt.Get());
//		hard->setSolenoid(hard->GetGabe(),solButt.Get());
		hard->setGabe(solButt.Get());

		hard->ElevatorPeriodic(	elevatorUpButt.Get(),
								elevatorDownButt.Get(),
								getNextPosition(elevatorGoToButts));
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
			printf("X=%f\t\t",(&stick)->GetX());
			printf("Y=%f\t\t",(&stick)->GetY());
			printf("Twist=%f\t\t",(&stick)->GetTwist());
			printf("Throt=%f\n",(&stick)->GetThrottle());
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

	void readConfig(){
//		std::ifstream config_read("CONFIG.TXT");
		FILE * config_read = std::fopen("CONFIG.TXT","r");
//		config_read.open();
		char *contents = new char[16];
//		config_read.getline(contents,16);
//		config_read.read(contents,16);
		std::fscanf(config_read,"%s",contents);
		if(strcmp(contents, "COMPETITION") == 0)
			competition_bot = true;
		else{
			printf(contents);
			competition_bot = false;
		}
		std::fclose(config_read);

		delete [] contents;
		delete config_read;
	}

	void configure(){
		std::ofstream config_write;
		config_write.open("CONFIG.TXT");
		config_write << "COMPETITION";
		config_write.close();
	}
};

START_ROBOT_CLASS(Robot);
