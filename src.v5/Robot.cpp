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
// Tyler Robbins - 2-13-15 - Added

#define AutoBots_RollOut TeleopInit

#include <math.h>
#include <vector>

#include <iostream>
#include <fstream>

#include "WPILib.h"

#include "ADXRS453Z.h"
#include "Hardware.h"

const float TURN_OFFSET = 0.04;

class Robot: public IterativeRobot
{
	Timer *printTimer;
	Joystick stick; // only joystick
	LiveWindow *lw;
	Solenoid gearShift;	// There is no more gearshift, this controls the claw.
	JoystickButton solButt;
	JoystickButton elevatorUpButt;
	JoystickButton elevatorDownButt;
//	JoystickButton *elevatorGoToButts[];
	std::vector<JoystickButton*> *elevatorGoToButts;
	Hardware* hard = Hardware::GetInstance();
	bool competition_bot;

public:
	Compressor *c = new Compressor(0);
	Robot() :
		stick(0),		// as they are declared above.
		lw(NULL),
		gearShift(0),
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

		printTimer = new Timer();
		c->Start();

		//*************************************
		// REMOVE ME AFTER I RUN ON REAL_ROBOT!
		//*************************************
		configure();

		readConfig();
	}

private:
	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		printf("Version 3.2\n");
		//c->Start();
		hard->GetGyro()->Start();
		hard->GetGyro()->Reset();
		hard->resetDrift();
		hard->GetPIDrift()->Enable();
	}

	void AutonomousInit()
	{
		//autoLoopCounter = 0;
		//c->Stop();
		hard->GetGyro()->Reset();
//		ggnore->Start();
//		ggnore->Reset();
	}

	void AutonomousPeriodic()
	{

	}

	void AutoBots_RollOut()
	{
		printf((c->Enabled()?"Compressor On":"Compressor Off"));
		printf("\n");
		hard->GetGyro()->Reset();
	}

	void TeleopPeriodic()
	{
		hard->move(&stick);
		printGyro();
//		printJoyVal();

//		if(hard->GetGyroAngle() >= 360 or hard->GetGyroAngle() <= -360){
//			hard->GetGyro()->Reset();
//		}

		hard->setSolenoid(&gearShift,solButt.Get());

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
		// hard->resetDrift();
	}

	bool isCompetitionRobot(){
		return competition_bot;
	}

	void readConfig(){
		std::ifstream config_read("CONFIG.TXT");
//		config_read.open();
		char *contents = new char[16];
		do{
//			config_read.getline(contents,16);
			config_read.read(contents,16);
			if(strcmp(contents, "COMPETITION") == 0)
				competition_bot = true;
			else
				competition_bot = false;
		} while(false);
		delete [] contents;
	}

	void configure(){
		std::ofstream config_write;
		config_write.open("CONFIG.TXT");
		config_write << "COMPETITION";
		config_write.close();
	}
};

START_ROBOT_CLASS(Robot);
