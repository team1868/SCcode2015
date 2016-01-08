#ifndef AUTOCOMMAND_H
#define AUTOCOMMAND_H

#include "WPILib.h"
#include "Debugging.h"
#include "RobotModel.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include "ElevatorController.h"
#include "ControlBoard.h"
#include <vector>
#include <string>
#include <iostream>
#include "PIDControlLoop.h"
#include "DriveController.h"
#include "AutoControlBoard.h"
#include "ElevatorController.h"

using namespace std;

class AutoCommand {
public:
	AutoCommand() {
	}
	virtual ~AutoCommand() {
	}
	virtual void Init() = 0;
	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;
	virtual bool IsDone() = 0;
	virtual AutoCommand* GetNextCommand() = 0;
};



class SimpleAutoCommand : public AutoCommand {
public:
	SimpleAutoCommand() {
		nextCommand = NULL;
	}
	virtual ~SimpleAutoCommand() {}
	virtual void SetNextCommand(AutoCommand* myNextCommand) {
		nextCommand = myNextCommand;
	}
	virtual AutoCommand* GetNextCommand() {
		return nextCommand;
	}
private:
	AutoCommand *nextCommand;
};

class ConditionalAutoCommand : public AutoCommand {
public:
	ConditionalAutoCommand() {
		trueNextCommand = NULL;
		falseNextCommand = NULL;
		condition = false;
	}
	virtual ~ConditionalAutoCommand() {}
	virtual void SetTrueNextCommand(AutoCommand* myTrueNextCmd) {
		trueNextCommand = myTrueNextCmd;
	}
	virtual void SetFalseNextCommand(AutoCommand* myFalseNextCmd) {
		falseNextCommand = myFalseNextCmd;
	}
	virtual AutoCommand* GetNextCommand() {
		if (condition) {
			return trueNextCommand;
		} else {
			return falseNextCommand;
		}
	}
private:
	AutoCommand *trueNextCommand;
	AutoCommand *falseNextCommand;
	bool condition;
};

class ParallelAutoCommand : public AutoCommand {
public:
	ParallelAutoCommand(SimpleAutoCommand* myFirst, SimpleAutoCommand* mySecond) {
		first = myFirst;
		second = mySecond;
		nextCommand = NULL;
		firstDone = false;
		secondDone = false;
		done = false;
	}
	virtual void Init() {
		first->Init();
		second->Init();
	}
	virtual void Update(double currTimeSec, double deltaTimeSec) {
		firstDone = first->IsDone();
		secondDone = second->IsDone();
		if (firstDone && secondDone) {
			done = true;
		} else if (firstDone && !secondDone) {
			second->Update(currTimeSec, deltaTimeSec);
		} else if (!firstDone && secondDone) {
			first->Update(currTimeSec, deltaTimeSec);
		} else {
			first->Update(currTimeSec, deltaTimeSec);
			second->Update(currTimeSec, deltaTimeSec);
		}
	}
	virtual bool IsDone() {
		return done;
	}
	virtual ~ParallelAutoCommand() {}
	virtual void SetNextCommand(AutoCommand* myNextCommand) {
		nextCommand = myNextCommand;
	}
	virtual AutoCommand* GetNextCommand() {
		return nextCommand;
	}
private:
	SimpleAutoCommand* first;
	SimpleAutoCommand* second;
	AutoCommand* nextCommand;
	bool firstDone;
	bool secondDone;
	bool done;
};

class WaitingCommand: public SimpleAutoCommand {
public:
	WaitingCommand(double myWaitTimeSec);
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();

private:
	double waitTimeSec;
	Timer *timer;
};

class DriveCommand : public SimpleAutoCommand {
public:
	/**
	 * All-encompassing drive command. Don't want turn and strafe at the same
	 * time because strafing finishes slower than all else because strafing
	 * speed is less than forward speed(used for turn). All other combinations ok.
	 */
	DriveCommand(RobotModel* myRobot,
				double myDesiredX, double myDesiredY, double myDesiredR);
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	static double xPFac, xIFac, xDFac, xDesiredAccuracy, xMaxAbsDiffError, xMaxAbsError,
		xMaxAbsITerm, xMaxAbsOutput;
	static double yPFac, yIFac, yDFac, yDesiredAccuracy, yMaxAbsDiffError, yMaxAbsError,
			yMaxAbsITerm, yMaxAbsOutput;
	static double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsDiffError, rMaxAbsError,
			rMaxAbsITerm, rMaxAbsOutput;
private:
	PIDConfig* CreateXPIDConfig();
	PIDConfig* CreateYPIDConfig();
	PIDConfig* CreateRPIDConfig();
	double Abs(double x);
	double one;
	RobotModel* robot;

	double desiredX;
	double desiredY;
	double desiredR;
	double initialX;
	double initialY;
	double initialR;

	PIDConfig* xPIDConfig;
	PIDControlLoop* xPID;
	double xOutput;

	PIDConfig* yPIDConfig;
	PIDControlLoop* yPID;
	double yOutput;

	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	double rOutput;

	bool driveDone;
};

class PivotCommand : public SimpleAutoCommand {
public:
	/**
	 * This command is DriveCommand with just an r input, but was created
	 * before DriveCommand. It only PIDs the r axis while DriveCommand can
	 * PID x and y too if given tiny inputs so they aren't both 0.0
	 */
	PivotCommand(RobotModel* myRobot, double desiredAngleTurn);
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	virtual ~PivotCommand() {}
	static double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsOutput, rMaxAbsDiffError,
		rMaxAbsError, rMaxAbsITerm;
private:
	PIDConfig* CreateRPIDConfig();
	double Abs(double x);
	double one;
	RobotModel* robot;

	double initialR;
	double desiredR;
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	double rOutput;

	bool isDone;
};

class StrafeCommand : public SimpleAutoCommand {
public:
	/**
	 * StrafeCommand is for strafing only. Beware, inputting an amount in
	 * StrafeCommand doesn't mean the robot will strafe that amount, rather
	 * it will strafe an amount that is proportional to the input. Doesn't
	 * strafe in a straight line without y PID.
	 */
	StrafeCommand(RobotModel* myRobot, double desiredDistance);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	~StrafeCommand() {}
	static double xPFac, xIFac, xDFac, xDesiredAccuracy, xMaxAbsOutput, xMaxAbsDiffError,
		xMaxAbsError, xMaxAbsITerm;
	static double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsOutput, rMaxAbsDiffError,
		rMaxAbsError, rMaxAbsITerm;
private:
	double Abs(double x);
	PIDConfig* CreateXPIDConfig();
	PIDConfig* CreateRPIDConfig();
	RobotModel* robot;
	double desiredX;
	double initialX;
	double initialR;
	bool isDone;
	double one;
	PIDConfig* xPIDConfig;
	PIDControlLoop* xPID;
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;

};
/*
 * Same as DriveCommand but without PIDing X axis so less accurate.
 */
class DriveStraightCommand : public SimpleAutoCommand {
public:
	DriveStraightCommand(RobotModel* robot, DriveController* drive, double desiredDistance);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	~DriveStraightCommand() {}
	static double yPFac, yIFac, yDFac, yDesiredAccuracy, yMaxAbsOutput, yMaxAbsDiffError,
		yMaxAbsITerm, yMaxAbsError;
	static double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsOutput, rMaxAbsDiffError,
		rMaxAbsITerm, rMaxAbsError;
private:
	double Abs(double x);
	PIDConfig* CreateYPIDConfig();
	PIDConfig* CreateRPIDConfig();
	RobotModel* robot;
	DriveController* drive;
	double encoderDesired;
	double encoderInit;
	double gyroInit;
	bool isDone;
	double one;

	PIDConfig* yPIDConfig;
	PIDConfig* rPIDConfig;
	PIDControlLoop* yPID;
	PIDControlLoop* rPID;
};

class ServoCommand : public SimpleAutoCommand {
public:
	/**
	 * Moves the Servo to a desired angle, useful when resetting the servo with
	 * the ultrasonic sensor on it.
	 */
	ServoCommand(RobotModel* myRobot, float myDesiredDegrees) {
		robot = myRobot;
		degreeReached = false;
		desiredDegrees = myDesiredDegrees;
	}
	virtual void Init() {
		robot->ResetServoAngle();
	}
	virtual void Update(double currTimeSec, double deltaTimeSec) {
		if (robot->GetServoAngle() >= desiredDegrees) {
			degreeReached = true;
		} else {
			float currAngle = robot->GetServoAngle();
			robot->SetServoAngle(currAngle + 5);
			degreeReached = false;
		}
	}
	virtual bool IsDone() {
		return degreeReached;
	}
	~ServoCommand() {}
private:
	RobotModel* robot;
	bool degreeReached;
	float desiredDegrees;
};
class UltrasonicSweepCommand : public SimpleAutoCommand {
public:
	/**
	 * Sweeps ultrasonic sensor until it sees something within a certain range.
	 */
	UltrasonicSweepCommand(RobotModel* myRobot, float myDesiredDistance, float myDesiredAccuracy) {
		robot = myRobot;
		distanceReached = false;
		desiredDistance = myDesiredDistance;
		accuracy = myDesiredAccuracy;
		diffAngle = 2;
		desiredTrials = 10;
		stage = "Slope";
		delta = 0.0;
		old = robot->GetServoAngle();
	}
	virtual void Init() {
	}

	virtual void Update(double currTimeSec, double deltaTimeSec) {
		float currDistance = robot->GetUltrasonicDistance();
		/*
		 * using stage to store what we stage we are at, should switch to state machine if time.
		 */
		if (stage == "Slope")
		{
			/*
			 * testing for when the slope drastically decreases, signifying that the sensor is seeing an object
			 */
			delta = currDistance - old;
			if (delta/deltaTimeSec < -50) {
				stage = "Platform";
			} else {
				/*
				 * keep sweeping by moving the servo
				 */
				float currAngle = robot->GetServoAngle();
				if (currAngle >= 180) {
					/*
					 * finished sweep
					 */
					distanceReached = true;
				}
				robot->SetServoAngle(currAngle + diffAngle);
				stage = "Slope";
			}
			old = currDistance;
		} else if (stage == "Platform") {
			/*
			 * if the sensor sees a tote, the distance should platform
			 */
			delta = currDistance - old;
			if (abs(delta/deltaTimeSec) < 0.1) {
				/*
				 * are we platforming?
				 */
				stage = "Distance verify";
			} else {
				stage = "Platform";
				float currAngle = robot->GetServoAngle();
				if (currAngle >= 180) {
					distanceReached = true;
				}
				robot->SetServoAngle(currAngle + diffAngle);
			}
			old = currDistance;
		} else if (stage == "Distance verify") {
			/*
			 * checking to make sure it is the right estimated distance
			 */
			float diffDistance = abs(desiredDistance - currDistance);
			if (diffDistance < accuracy) {
				distanceReached = true;
				double angle = robot->GetServoAngle();
				robot->SetServoAngle(angle); // adding extra because senses edge before points at edge
			} else {
				/*
				 * go back to sweeping
				 */
				stage = "Slope";
			}
			old = currDistance;
		}
	}

	virtual bool IsDone() {
		return distanceReached;
	}

	~UltrasonicSweepCommand() {}

private:
	RobotModel* robot;
	bool distanceReached;
	float desiredDistance;
	float diffAngle;
	float accuracy;
	string stage;
	int desiredTrials;
	double delta;
	double old;
};

class GyroLineUpToteCommand : public SimpleAutoCommand {
public:
	/**
	 * When robot is close to the tote and facing in approximately the right
	 * direction, use GyroLineUpToteCommand to align.
	 */
	GyroLineUpToteCommand(RobotModel* myRobot);
	~GyroLineUpToteCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();

	static double gyroPFac, gyroIFac, gyroDFac, gyroDesiredAccuracy,gyroMaxAbsOutput,
		gyroMaxAbsDiffError, gyroMaxAbsError, gyroMaxAbsITerm;
private:
	virtual PIDConfig* CreateRPIDConfig();
	double Abs(double x);
	void ResetGyro();
	double desiredR;
	double initialR;
	RobotModel* robot;
	DriveController* driveController;
	double one = (double) 1.0;
	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	bool rDone;
	double rOutput;
};

class UltrasonicLineUpToteCommand : public SimpleAutoCommand {
public:
	UltrasonicLineUpToteCommand(RobotModel* myRobot,
			DriveController* myDriveController, double myDesiredX, double myDesiredY, double myDesiredR);
	~UltrasonicLineUpToteCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	static double xPFac, xIFac, xDFac, xDesiredAccuracy, xMaxAbsDiffError,
			xMaxAbsError, xMaxAbsITerm, xMaxAbsOutput;
	static double yPFac, yIFac, yDFac, yDesiredAccuracy, yMaxAbsDiffError,
			yMaxAbsError, yMaxAbsITerm, yMaxAbsOutput;
	static double rPFac, rIFac, rDFac, rDesiredAccuracy, rMaxAbsDiffError,
			rMaxAbsError, rMaxAbsITerm, rMaxAbsOutput;

private:
	virtual PIDConfig* CreateXPIDConfig();
	virtual PIDConfig* CreateYPIDConfig();
	virtual PIDConfig* CreateRPIDConfig();
	double Abs(double x) {
		if (x > 0.0) {
			return x;
		} else {
			return -x;
		}
	}
	RobotModel* robot;
	DriveController* driveController;
	double one = (double)1.0;
	double desiredX;
	double desiredY;
	double desiredR;
	double initialX;
	double initialY;
	double initialR;
	double initialEncoderX;
	double initialEncoderY;
	double initialGyroR;
	bool linedUpDone;

	PIDConfig* xPIDConfig;
	PIDControlLoop* xPID;
	bool xDone;
	double xOutput;

	PIDConfig* yPIDConfig;
	PIDControlLoop* yPID;
	bool yDone;
	double yOutput;

	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	bool rDone;
	double rOutput;
};

class UltrasonicLineUpBinCommand : public SimpleAutoCommand {
public:
	UltrasonicLineUpBinCommand(RobotModel* myRobot,
			DriveController* myDriveController, double myDesiredX, double myDesiredY,
			double myDesiredR);
	~UltrasonicLineUpBinCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
private:
	virtual PIDConfig* CreateXPIDConfig();
	virtual PIDConfig* CreateYPIDConfig();
	virtual PIDConfig* CreateRPIDConfig();
	RobotModel* robot;
	DriveController* driveController;
	double desiredX;
	double desiredY;
	double desiredR;
	double initialX;
	double initialY;
	double initialR;
	double initialEncoderX;
	double initialEncoderY;
	double initialGyroR;
	bool linedUpDone;

	PIDConfig* xPIDConfig;
	PIDControlLoop* xPID;
	bool xDone;
	double xOutput;

	PIDConfig* yPIDConfig;
	PIDControlLoop* yPID;
	bool yDone;
	double yOutput;

	PIDConfig* rPIDConfig;
	PIDControlLoop* rPID;
	bool rDone;
	double rOutput;

	double initialDistanceFromBin;
	double servoAngle;
};

class PlaybackAutoCommand : public SimpleAutoCommand {
public:
	/**
	 * Robot duplicates actions from files recorded by turning isRecording to
	 * true in ControlBoard. Specify the path name.
	 */
	PlaybackAutoCommand(RobotModel* myRobot, const std::string& playbackFileName);
	~PlaybackAutoCommand() {};
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
	void SetValue(std::string name, double val);
	// void SetValue(std::string name, bool val);
private:
	std::string playbackFilePath;
	RobotModel* robot;

	double currTime, oldTime;
	std::ifstream playbackFile;
	std::string buffer;
	std::string varName;
	double varVal;

	AutoControlBoard* autoControl;
	DriveController* autoDriveControl;
	ElevatorController* autoElevControl;

	void RefreshValues();
	bool Contains(std::string bigString, std::string lilString);
};

class RollOutCommand : public SimpleAutoCommand {
public:
	RollOutCommand(ElevatorController* myElevator, double myTime);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	~RollOutCommand() {}
private:
	ElevatorController* elevator;
	Timer* timer;
	double time;
	bool rollOutDone;
};

class ClampCommand : public SimpleAutoCommand {
public:
	ClampCommand(ElevatorController* myElevator, bool myClamp);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	~ClampCommand() {}
private:
	ElevatorController* elevator;
	bool clamp;
	bool clampDone;
};

class RampCommand {
public:
	RampCommand(ElevatorController* elevator, bool rampOut);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	~RampCommand() {}
private:
	ElevatorController* elevator;
	bool desiredRampOut;
	bool rampOutDone;
};

class IntakingCommand {
public:
	IntakingCommand(ElevatorController* elevator);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	~IntakingCommand() {}
private:
	ElevatorController* elevator;
	bool intakingDone;
};

class OuttakingCommand {
public:
	OuttakingCommand(ElevatorController* elevator);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
	~OuttakingCommand() {}
private:
	ElevatorController* elevator;
	bool outtakingDone;
};
/*
class GrabBinCommand : public SimpleAutoCommand{
public:
	GrabBinCommand(ElevatorController* elevator) {
		this->elevator = elevator;
		grabDone = false;
	}
	void Init() {
		elevator->SetAutoIntaking();
	}
	void Update(double currTimeSec, double deltaTimeSec) {
		if (elevator->AutoIntakingDone()) {
			grabDone = true;
		} else {
			grabDone = false;
		}
	}
	bool IsDone() {
		return grabDone;
	}
	void ~GrabBinCommand() {}
private:
	ElevatorController* elevator;
	bool grabDone;
};
*/

#endif
