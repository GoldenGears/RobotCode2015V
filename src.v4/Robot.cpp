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

#include <math.h>

#include "WPILib.h"

#include "ADXRS453Z.h"
#include "Hardware.h"

const float TURN_OFFSET = 0.04;

class Robot: public IterativeRobot
{
//	Talon elevatorMotor;
//	Talon frontLeft;
//	Talon frontRight;
//	Talon backLeft;
//	TalonSRX backRight;
//	RobotDrive drive; // robot drive system
	Timer *printTimer;
	//RobotDrive driveFront;
	//RobotDrive driveMiddle;
	//RobotDrive driveBack;
	Joystick stick; // only joystick
	LiveWindow *lw;
	Solenoid gearShift;
	JoystickButton solButt;
	JoystickButton elevatorUpButt;
	JoystickButton elevatorDownButt;
//	Gyro* ggnore;
//	ADXRS453Z* ggnore;
	Hardware* hard = Hardware::GetInstance();
	//Compressor *c;
	//int autoLoopCounter;

public:
	Compressor *c = new Compressor(0);
	Robot() :
//		elevatorMotor(9),
//		frontLeft(0),
//		frontRight(2),
//		//midLeft(4),
//		//midRight(1),
//		backLeft(1),
//		backRight(3),
		//driveFront(frontRight,frontLeft),
		//driveMiddle(midRight,midLeft),
		//driveBack(backRight,backLeft),
//		drive(frontLeft, backLeft, frontRight, backRight),
		stick(0),		// as they are declared above.
		lw(NULL),
		gearShift(0),
		solButt(&stick,2),
		elevatorUpButt(&stick,3),
		elevatorDownButt(&stick,5)
	{
		printTimer = new Timer();
//		ggnore = new ADXRS453Z();
//		drive.SetExpiration(0.1);
		c->Start();

//		drive.SetInvertedMotor(RobotDrive::MotorType::kFrontLeftMotor,true);
//		drive.SetInvertedMotor(RobotDrive::MotorType::kRearLeftMotor,true);
		// drive.SetInvertedMotor(RobotDrive::MotorType::kFrontRightMotor,true);
		// drive.SetInvertedMotor(RobotDrive::MotorType::kRearRightMotor,true);
	}

private:
	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		printf("Version 3.2\n");
		//c->Start();
		hard->GetGyro()->Start();
		hard->GetGyro()->Reset();
//		hard->GetPIDrift()->PIDSubsystem::Enable();
		hard->GetPIDrift()->Enable();
//		ggnore->Start();
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

	void TeleopInit()
	{
		//printf("Hello World 4413!");
		//c->Stop();
		//driveFront.SetInvertedMotor(RobotDrive::kFrontLeftMotor,true);
		//driveFront.SetInvertedMotor(RobotDrive::kRearRightMotor,true);
		//driveMiddle.SetInvertedMotor(RobotDrive::kFrontLeftMotor,true);
		//driveMiddle.SetInvertedMotor(RobotDrive::kFrontRightMotor,true);
		//driveMiddle.SetInvertedMotor(RobotDrive::kFrontRightMotor,true);
		printf((c->Enabled()?"Compressor On":"Compressor Off"));
		printf("\n");
		hard->GetGyro()->Reset();
//		hard->GetGyro()->Start();
//		ggnore->Reset();
//		ggnore->Start();
	}

	void TeleopPeriodic()
	{
		//myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
		//driveFront.ArcadeDrive(stick);
		//driveMiddle.ArcadeDrive(stick);
		//driveBack.ArcadeDrive(stick);
//		move();
		hard->move(&stick);
//		printGyro();
//		printJoyVal();

		if(hard->GetGyroAngle() >= 360 or hard->GetGyroAngle() <= -360){
//		if(ggnore->GetAngle() >= 360 or ggnore->GetAngle() <= -360){
			hard->GetGyro()->Reset();
//			ggnore->Reset();
		}

//		printf("PIDrift=%f\t\t",hard->GetPIDrift()->GetOutput());
		printGyro();

//		if(solButt.Get() && false){
//			//toggleSolenoid(&gearShift);
//			setSolenoid(&gearShift,solButt.Get());
//		}
		hard->setSolenoid(&gearShift,solButt.Get());

		hard->ElevatorPeriodic(elevatorUpButt.Get(),elevatorDownButt.Get());
	}

	void TestInit(){
		hard->GetGyro()->Reset();
//		ggnore->Reset();
	}

	void TestPeriodic()
	{
		lw->Run();
//		ggnore->Update();
		printGyro();
	}

//	void printGyro(){
//		printf("Angle:%f\t\t",ggnore->GetAngle());
////		printf("\n%f",ggnore->GetData());
//		printf("Rate:%f\t\t",ggnore->GetRate());
////		printf("Turn:%f\n",filterJoyValue(stick.GetTwist()));
//		printf("Offset:%f\n",ggnore->GetOffset());
//	}
	void printGyro(){
		printf("Angle:%f\t\t",hard->GetGyroAngle());
		printf("Rate:%f\t\t",hard->GetGyroRate());
		printf("Offset:%f\n",hard->GetGyroOffset());
	}
//
//	void printJoyVal(){
//		printf("X=%f\t\t",filterJoyValue(stick.GetX()));
//		printf("Y=%f\t\t",filterJoyValue(stick.GetY()));
//		printf("Turn=%f\t\t",filterJoyValue(stick.GetTwist()));
//		printf("Throttle=%f\n",
//				(-filterJoyValue(stick.GetThrottle()))/2);
//		printf("Offset=%f\n",getOffset(stick.GetX(),shiftThrottle(stick.GetThrottle())));
//	}

//	float shiftThrottle(float throttle){
//		return -filterJoyValue(throttle)/2;
//	}
//
//	void DisabledInit(){
//		//c->Start();
//		ggnore->Reset();
//	}
//
//	float filterJoyValue(float val){
//		return ((int)(val*100))/100.0;
//	}

//	float getSpeed(float y, float throttle){
//		return y*throttle;
//	}

//	float getOffset(float y, float throttle){
//		return 0.195*getSpeed(y,throttle)+0.01;
//	}

//	void move(){
//		float x = stick.GetX();
//		float y = stick.GetY();
//		float z = stick.GetZ();
//		float turn = stick.GetTwist();
//		float throttle = stick.GetThrottle();
//
//		throttle = (-throttle + 1.0)/2.0; // Shift the throttle value from -1-1 to 0-1
//
//		throttle = ((int)(throttle*100))/100.0;
//		x = ((int)(x*100))/100.0;
//		y = ((int)(y*100))/100.0;
//		z = ((int)(z*100))/100.0;
//		turn = ((int)(turn*100))/100.0;
//
//
////		turn -= TURN_OFFSET;
////		turn -= getOffset(stick.GetY(),shiftThrottle(stick.GetThrottle()));
////		printf("%f",turn);
//
//		x*=throttle;
//		y*=throttle;
//		z*=throttle;
//		turn*=throttle;
//
//		//driveFront.ArcadeDrive(y,turn);
//		//driveMiddle.ArcadeDrive(y,turn);
//		//driveBack.ArcadeDrive(y,turn);
//
////		if(turn <= 0.2 or turn >= -0.2)
////			printf("Turn to low! (%f)",turn);
////			turn = 0;
////
////		printf("Turn=%f\n",turn);
//
//		drive.MecanumDrive_Cartesian(x,y,turn,getGyroAngle());
//
//	}

//	void moveElevator(float pwr){
//		elevatorMotor.Set(pwr);
//	}
//
//	void ElevatorUp(float pwr){
//		moveElevator(pwr/100);
//	}
//
//	void ElevatorDown(float pwr){
//		moveElevator(-pwr/100);
//	}
//
//	void ElevatorPeriodic(bool up=false,bool down=false){
//		if(up)
//			ElevatorUp(80);
//		else if(down)
//			ElevatorDown(80);
//		else
//			ElevatorDown(0);
//	}

	void SystemCheck(){	}

//	void toggleSolenoid(Solenoid *sol){
//		//sol->Set(!sol->Get());
//		setSolenoid(sol,!sol->Get());
//	}
//
//	void setSolenoid(Solenoid *sol,bool value){
//		sol->Set(value);
//	}
//
//	double getGyroRate(){
//		return ggnore->GetRate();
//	}
//
//	double getGyroAngle(){
//		return ggnore->GetAngle();
//	}
//
//	double getGyroOffset(){
//		return ggnore->GetOffset();
//	}

//	bool isStopped(){
//		float x = filterJoyValue(stick.GetX());
//		float y = filterJoyValue(stick.GetY());
//		float turn = filterJoyValue(stick.GetTwist());
//
//		return (x==0 && y==0 && turn==0);
//	}

	bool IsUserAGoat(){
		return true;
	}
};

START_ROBOT_CLASS(Robot);
