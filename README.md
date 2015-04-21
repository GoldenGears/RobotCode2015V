Golden Gears Robotics 2015
==========================
#Description
Robot code for FRC Team 4413 (Golden Gears Robotics). It is written in C++ and compiled using [Eclipse](http://www.eclipse.org/downloads/). Instructions for setting up Eclipse to work with FRC Robotics Code can be found [here](http://wpilib.screenstepslive.com/s/4485/m/13809/l/145002-installing-eclipse-c-java).

#Programming Team Members#
Jacob Charles Peterson

Joseph Conlon Meek

Tyler Clay Robbins

#Deploying#
1) Have Eclipse running

2) Make sure your computer is connected to the robot while it is on

- This can be either physically with an ethernet cable, or wirelessly to the router
	+ An ethernet connection can be hooked up either directly into the RoboRio, or into the router

3) Right click on your project (Note: Project, not source code), and hover over the option that says "Run as"

4) Click on "WPILib C++ Deploy"

#Controlling the Robot#
> WARNING!
> --------
> The FRC Driver Station software is a Windows-only application. It will not run on either OSX or Linux.

1) Make sure you have the [FRC 2015 Update Suite](http://www.ni.com/download/first-robotics-software-2015/5112/en/) installed.

- The password is: R3C3CL3RU$H2015

2) Connect your computer to the robot (Either physically or wirelessly, as specified in Deploying)

3) Open the Driver Station software and netconsole

- These are located in "C:\Program Files (x86)\FRC Driver Station" and "C:\Program Files (x86)\NetConsole for cRIO" respectively.

4) In DriverStation.exe, click Enable when the "Robot Code" and "Communications" lights are green.

##Structure
- src/
	+ Robot
		- Entry point
		- Controls the entire lifecycle of the robot
	+ Hardware
		- Contains methods for controlling the hardware of the robot
	+ ADXRS453Z
		- Class for controlling the ADXRS453Z gyro
	+ PIDrift
		- Class to handle drifting that occurs while using the gyro
	+ AutonomousDrive
		- Determines what the robot does during Autonomous mode.