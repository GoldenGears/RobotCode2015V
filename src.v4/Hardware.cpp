//************
// Golden Gears Robotics
// 2/7/15
// Hardware MAIN

// Tyler Robbins - 2-8-15 - Added hardware class with PID Control Loop tm.

#include "Hardware.h"

Hardware* Hardware::m_instance = NULL;

Hardware::Hardware(){
	frontLeft = new TalonSRX(0);
	frontRight = new TalonSRX(2);
	backLeft = new TalonSRX(1);
	backRight = new TalonSRX(3);
	elevator = new TalonSRX(9);

	drive = new RobotDrive(frontLeft,backLeft,frontRight,backRight);
	ggnore = new ADXRS453Z();
	drift = new PIDrift(0.05,0,0,ggnore);
//
//	ggnore->Start();

	drive->SetExpiration(0.1);

	drive->SetInvertedMotor(RobotDrive::MotorType::kFrontLeftMotor,true);
	drive->SetInvertedMotor(RobotDrive::MotorType::kRearLeftMotor,true);
}

Hardware::~Hardware(){ }

void Hardware::move(Joystick *joy){
	float x = joy->GetX();
	float y = joy->GetY();
	float z = joy->GetZ();
	float turn = joy->GetTwist();
	float throttle = joy->GetThrottle();

	if(turn <= 0.05 and turn >= -0.05)
		drift->SetSetpoint(GetGyroAngle());

	float turn_drift = drift->GetOutput();

//	throttle = shiftThrottle(throttle);
	throttle = (-throttle+1.0)/2.0;

	throttle = ((int)(throttle*100))/100.0;

//	x = filterJoyValue(x);
	x = ((int)(x*100))/100.0;
//	y = filterJoyValue(y);
	y = ((int)(y*100))/100.0;
//	z = filterJoyValue(z);
	z = ((int)(z*100))/100.0;
//	turn = filterJoyValue(turn);
	turn = ((int)(turn*100))/100.0;
	turn_drift = ((int)(turn*100))/100.0;

	x*=throttle;
	y*=throttle;
	z*=throttle;
	turn*=throttle;
	turn_drift*=throttle;

	printf("PIDrift Formatted=%f\t\t",turn_drift);

	if(turn <= 0.05 and turn >= -0.05){
		drive->MecanumDrive_Cartesian(x,y,turn_drift,0.0);
	}else
		drive->MecanumDrive_Cartesian(x,y,turn,0.0);
}

float Hardware::shiftThrottle(float throt){
	return filterJoyValue((-throt+1.0)/2.0);
}

float Hardware::filterJoyValue(float val){
	return ((int)(val*100))/100.0;
}

void Hardware::moveElevator(float pwr){
	elevator->Set(pwr);
}

void Hardware::ElevatorUp(float percent){
	moveElevator(percent/100);
}

void Hardware::ElevatorDown(float percent){
	moveElevator(-percent/100);
}

void Hardware::ElevatorPeriodic(bool up, bool down){
	if(up)
		ElevatorUp(80.0);
	else if(down)
		ElevatorDown(80.0);
	else
		ElevatorUp(0.0);
}

void Hardware::toggleSolenoid(Solenoid *sol){
	setSolenoid(sol,!sol->Get());
}

void Hardware::setSolenoid(Solenoid *sol, bool value){
	sol->Set(value);
}


Hardware* Hardware::GetInstance(){
	if(m_instance == NULL)
		m_instance = new Hardware();

	return m_instance;
}
