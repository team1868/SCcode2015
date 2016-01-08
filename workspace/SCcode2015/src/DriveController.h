#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H

#include "WPILib.h"
#include "RobotModel.h"
#include "RemoteControl.h"
#include "Debugging.h"
#include "PIDControlLoop.h"

class DriveController {
public:
	enum DriveStates {
		kInitialize,
		kReset,
		kTeleopDrive,
		kEnterControlledTeleop,
		kControlledTeleop,
		kButtonStrafe
	};

	DriveController(RobotModel* myRobot, RemoteController* myHumanControl);
	~DriveController() {
	}
	void Update(double currTimeSec, double deltaTimeSec);
	void RefreshIni();
	void Reset();
	void ControlledMecanumDrive(double joyX, double joyY, double joyR, double gyroAngle, double currTimeSec, double deltaTimeSec);
	void ControlledMecanumDrive(double joyX, double joyY, double joyR, double currTimeSec, double deltaTimeSec);
	void MecanumDrive(double myX, double myY, double myRotate);
	void MecanumDrive(double myX, double myY, double myRotate, double myAngle);
	int DriveDirection();
private:
	double Abs(double x);
	void DriveLength();
	void DriveWidth();
	PIDConfig* CreateFLPIDConfig();
	PIDConfig* CreateRLPIDConfig();
	PIDConfig* CreateFRPIDConfig();
	PIDConfig* CreateRRPIDConfig();
	double EncoderToMotorSpeed(double encoderSpeed);
	double NavxToMotorSpeed(double navxSpeed);
	RobotModel* robot;
	RemoteController* humanControl;
	uint32_t m_stateVal;
	uint32_t nextState;

	double joyX, joyY, joyRotate;
	double tolerance;

	double oldAngle;
	PIDConfig *flPIDConfig, *rlPIDConfig, *frPIDConfig, *rrPIDConfig;
	PIDControlLoop *flPID, *rlPID, *frPID, *rrPID;
	double oldFL, oldRL, oldFR, oldRR;
	double currFL, currRL, currFR, currRR;
	double flSpeed, rlSpeed, frSpeed, rrSpeed;
	double flPFac, flIFac, flDFac, flDesiredAccuracy, flMaxAbsDiffError,
			flMaxAbsError, flMaxAbsITerm, flMaxAbsOutput;
	double rlPFac, rlIFac, rlDFac, rlDesiredAccuracy, rlMaxAbsDiffError,
			rlMaxAbsError, rlMaxAbsITerm, rlMaxAbsOutput;
	double frPFac, frIFac, frDFac, frDesiredAccuracy, frMaxAbsDiffError,
			frMaxAbsError, frMaxAbsITerm, frMaxAbsOutput;
	double rrPFac, rrIFac, rrDFac, rrDesiredAccuracy, rrMaxAbsDiffError,
			rrMaxAbsError, rrMaxAbsITerm, rrMaxAbsOutput;
	Timer *timer;
	bool isFirstStrafeIteration;
	double timeOfFirstStrafeIteration;
	int strafeDirection;
	int numTimesStrafeButtonPressed;
	double xSpeedStrafeButton, ySpeedStrafeButton, strafeButtonTime;
};

#endif
