//************
// Golden Gears Robotics
// 2/7/15
// Hardware MAIN

/* Hardware class
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


// Tyler Robbins - 2-8-15 - Added hardware class with PID Control Loop tm.
// Tyler Robbins - 2-11-15 - Added potentiometer code. Added elevator goto methods.
/* Tyler Robbins & Conlon Meek - 2-12-15 - Fixed how PIDrift output is gotten. Robot now moves mostly straight (a bit sensitive).
 * 		Gyro correction disabled for testing on new robot. Changed all TalonSRXs to CANTalons.
 */
// Tyler Robbins - 2-13-15 - Reversed the right motors. Changed elevator percentage speed to 50.
// Tyler Robbins - 2-14-15 - Added potentiometer code. Made TalonSRX's CANTalons. Added solenoid code.
// Tyler Robbins & Conlon Meek - 2-16-15 - Found good PID constants.
/* Tyler Robbins - 4-20-15 - Added GPL header. Added Timer declaration and appropriate method calls. Added hasArmsBeenToggled member.
 *  	Increased elevatorUp and elevatorDown speeds to 80. Added ArmsPeriodic method to add toggleability to arms. Added HasPassed method.
 * */

#include "Hardware.h"

Hardware* Hardware::m_instance = NULL;

Hardware::Hardware(){
	hasArmsBeenToggled = false;

	frontLeft = new CANTalon(0);
	frontRight = new CANTalon(2);
	backLeft = new CANTalon(1);
	backRight = new CANTalon(3);
	elevator = new CANTalon(9);
	m_timer = new Timer();

	gabe = new Solenoid(0);
	drive = new RobotDrive(frontLeft,backLeft,frontRight,backRight);
	ggnore = new ADXRS453Z();
	drift = new PIDrift(.05,0.00000,0.1,ggnore); // 0.05,0,0
	poten = new AnalogPotentiometer(0);

	drive->SetExpiration(0.1);
	m_timer->Start();

//	drive->SetInvertedMotor(RobotDrive::MotorType::kFrontRightMotor,true);
//	drive->SetInvertedMotor(RobotDrive::MotorType::kRearRightMotor,true);
	drive->SetInvertedMotor(RobotDrive::MotorType::kFrontLeftMotor,true);
	drive->SetInvertedMotor(RobotDrive::MotorType::kRearLeftMotor,true);

	frontLeft->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	frontRight->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	backLeft->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
	backRight->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
}

Hardware::~Hardware(){ }

void Hardware::move(Joystick *joy){
	float x = joy->GetX();
	float y = joy->GetY();
	float z = joy->GetZ();
	float turn = joy->GetTwist();
	float throttle = joy->GetThrottle();
	float turn_drift;

	throttle = (-throttle+1.0)/2.0;

	throttle = ((int)(throttle*100))/100.0;

	x = ((int)(x*100))/100.0;
	y = ((int)(y*100))/100.0;
	z = ((int)(z*100))/100.0;
	turn = ((int)(turn*100))/100.0;
	// turn_drift = ((int)(turn*100))/100.0;

	x*=throttle;
	y*=throttle;
	z*=throttle;
	turn*=throttle;
	// turn_drift*=throttle;

	printf("PIDrift Formatted=%f\t\t",turn_drift);

	if(turn <= 0.1 and turn >= -0.1 and false){ // Remove false condition to re-enable gyro correction.
		drift->SetSetpoint(GetGyroAngle());
		turn_drift = drift->GetOutput();
		drive->MecanumDrive_Cartesian(x, y, turn_drift, 0.0);
	}else
		drive->MecanumDrive_Cartesian(-x,y,-turn,0.0);
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

void Hardware::ElevatorIncrement(){
	float currentPos = convertVolToDist(GetPotenValue());
	float volt = convertDisToVolt(currentPos++);
	if(GetPotenValue() < volt and !(currentPos >= 59)) // 1 inch deadband
		ElevatorUp(50.0);
	else if(GetPotenValue() > volt and !(currentPos >= 59))
		ElevatorDown(50.0);
	else
		ElevatorUp(10.0);
}

void Hardware::ElevatorDecrement(){
	float currentPos = convertVolToDist(GetPotenValue());
	float volt = convertDisToVolt(currentPos--);
	if(GetPotenValue() < volt and !(currentPos <= 1))
		ElevatorUp(50.0);
	else if(GetPotenValue() > volt and !(currentPos <= 1))
		ElevatorDown(50.0);
	else
		ElevatorUp(10.0);
}

void Hardware::ElevatorGoTo(int pos){
	float realPos;

	switch(pos){
		case 0:
			realPos = 0;
			break;
		case 2:
			realPos = 36; // inches (3 feet)
			break;
		case 3:
			realPos = 72; // inches (6 feet)
			break;
		default:
			printf("[ERROR] - Invalid elevator position: %d",pos);
			return;
	}

	float volt = convertDisToVolt(realPos);
	volt = stripDecimals(volt,10);

	if(GetPotenValue() < volt)
		ElevatorUp(50.0);
	else if(GetPotenValue() > volt)
		ElevatorDown(50.0);
	else
		ElevatorUp(10.0);
}

void Hardware::ElevatorPeriodic(bool up, bool down, int gotop){
	if(up)
		 ElevatorUp(80.0);
//		ElevatorIncrement();
	else if(down)
		 ElevatorDown(80.0);
//		ElevatorDecrement();
	else if(gotop >= 0 and false)
		ElevatorGoTo(gotop);
	else
		ElevatorUp(10);
}

void Hardware::ArmsPeriodic(bool value){
	if(!hasArmsBeenToggled and value){
		if(HasPassed(0.2,lastTime)){
			lastTime = m_timer->Get();
			hasArmsBeenToggled = true;
			gabe->Set(!gabe->Get());
		}
	}
	else
		hasArmsBeenToggled = false;
}

void Hardware::toggleSolenoid(Solenoid *sol){
	setSolenoid(sol,!sol->Get());
}

void Hardware::setSolenoid(Solenoid *sol, bool value){
	sol->Set(value);
}

void Hardware::setGabe(bool value){
	if(value) printf("Gabe has been called.");
	gabe->Set(value);
}

Hardware* Hardware::GetInstance(){
	if(m_instance == NULL)
		m_instance = new Hardware();

	return m_instance;
}

float Hardware::convertDisToVolt(float dist){
	return dist/SPOOL_RADIUS;
}

float Hardware::convertVolToDist(float volt){
	return volt*SPOOL_RADIUS;
}

float Hardware::stripDecimals(float num, int place){
	return ((int)(num*place))/place;
}

void Hardware::resetDrift(){

	float p = SmartDashboard::GetNumber(std::string("Proportion"),0);
	float i = SmartDashboard::GetNumber(std::string("Integral"),0);
	float d = SmartDashboard::GetNumber(std::string("Derivative"),0);
	drift = new PIDrift(p,i,d,ggnore);
}

void Hardware::SetTalon(Talons tal,float pwr){
	CANTalon *talon = GetTalon(tal);
	talon->Set(pwr/100);
}

bool Hardware::HasPassed(float sec,float offset){
	return (m_timer->Get()-offset) >= sec;
}
