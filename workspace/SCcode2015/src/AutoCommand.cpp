#include "AutoCommand.h"
#include <math.h>
#include "ini.h"
#include <iostream>
#include <string>

#define PI 3.14159265358979

WaitingCommand::WaitingCommand(double myWaitTimeSec) {
	waitTimeSec = myWaitTimeSec;
	timer = new Timer();
}

void WaitingCommand::Init() {
	timer->Start();
}

void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
}

bool WaitingCommand::IsDone() {
	return (timer->Get() >= waitTimeSec);
}

DriveCommand::DriveCommand(RobotModel* myRobot,
		double myDesiredX, double myDesiredY, double myDesiredR) {
	robot = myRobot;
	desiredX = myDesiredX;
	desiredY = myDesiredY;
	desiredR = myDesiredR;
	driveDone = false;
	one = (double) 1.0; //using 1.0 makes things round weirdly so we have to say it is a double
}

void DriveCommand::Init() {
	/*
	 * creating the PIDConfigs has to happen before creating and initializing the PIDs themselves
	 */
	xPIDConfig = CreateXPIDConfig();
	yPIDConfig = CreateYPIDConfig();
	rPIDConfig = CreateRPIDConfig();

	xPID = new PIDControlLoop(xPIDConfig);
	yPID = new PIDControlLoop(yPIDConfig);
	rPID = new PIDControlLoop(rPIDConfig);

	initialX = robot->GetXEncoderVal();
	initialY = robot->GetYEncoderVal();
	initialR = robot->GetNavxAngle();
	/*
	 * desired value for the PID is initial + desired because the input desired for the drivecommand is
	 * the desired amount driven, not where to drive to.
	 */
	xPID->Init(initialX, initialX + desiredX);
	yPID->Init(initialY, initialY + desiredY);
	rPID->Init(0.0, desiredR);
}

void DriveCommand::Update(double currTimeSec, double deltaTimeSec) {
	double xVal = robot->GetXEncoderVal();
	double yVal = robot->GetYEncoderVal();
	double rVal = robot->GetNavxAngle();
	//printf("rVal init %f\n", rVal - initialR);
	if (rVal > 0.0 && initialR > 0.0) {
		rVal = rVal - initialR;
	} else if (rVal > 0.0 && initialR < 0.0) {
		rVal = rVal - initialR;
		if (rVal > 180.0) {
			rVal = -360 + rVal;
		}
	} else if (rVal < 0.0 && initialR > 0.0) {
		rVal = rVal - initialR;
		if (rVal < -180.0) {
			rVal = 360 + rVal;
		}
	} else {
		rVal = rVal - initialR;
	}

	//printf("rVal %f\n", rVal);
	bool xDone = xPID->ControlLoopDone(xVal);
	bool yDone = yPID->ControlLoopDone(yVal);
	bool rDone = rPID->ControlLoopDone(rVal);


	if (desiredX == 0.0 && desiredY == 0.0) {
		/*
		 * we want to PID r only if we only want to turn because the quickturn motion affects encoder
		 * distances (aka x and y) and it won't exit the PID.
		 */
		xDone = true;
		yDone = true;
	}
	if (xDone && yDone && rDone) {
		driveDone = true;
		/*
		 * set all speeds to 0 to stop the robot from moving
		 */
		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, 0.0);
	} else {
		/*
		 * get proposed motor speeds (similar to joystick inputs)
		 */
		xOutput = xPID->Update(xVal);
		yOutput = yPID->Update(yVal);
		rOutput = rPID->Update(rVal);
		//printf("Navx Angle: %f \n", robot->GetNavxAngle());
		//printf("rPID Value: %f \n", rOutput);
		if (desiredX == 0.0 && desiredY == 0.0) {
			xOutput = 0.0;
			yOutput = 0.0;
			/*
			 * again, do not want quickturning affecting x and y and getting stuck in the PID
			 */
		}
		/*
		 * mecanum calculation equations, see drivecontroller for explanation.
		 */
		double frontLeft = xOutput + yOutput + rOutput;
		double rearLeft = -xOutput + yOutput + rOutput;
		double frontRight = -xOutput + yOutput - rOutput;
		double rearRight = xOutput + yOutput - rOutput;

		double maxAbsMagnitude = 0.0;

		if (maxAbsMagnitude < Abs(rearLeft)) {
			maxAbsMagnitude = Abs(rearLeft);
		}
		if (maxAbsMagnitude < Abs(frontLeft)) {
			maxAbsMagnitude = Abs(frontLeft);
		}
		if (maxAbsMagnitude < Abs(frontRight)) {
			maxAbsMagnitude = Abs(frontRight);
		}
		if (maxAbsMagnitude < Abs(rearRight)) {
			maxAbsMagnitude = Abs(rearRight);
		}

		if (maxAbsMagnitude < one) {
			maxAbsMagnitude = one;
		}

		frontLeft = frontLeft / maxAbsMagnitude;
		rearLeft = rearLeft / maxAbsMagnitude;
		frontRight = frontRight / maxAbsMagnitude;
		rearRight = rearRight / maxAbsMagnitude;

		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, frontLeft);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rearLeft);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frontRight);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, rearRight);
	}
}

bool DriveCommand::IsDone() {
	return driveDone;
}

double DriveCommand::xPFac = 0.0;
double DriveCommand::xIFac = 0.0;
double DriveCommand::xDFac = 0.0;
double DriveCommand::xDesiredAccuracy = 0.0;
double DriveCommand::xMaxAbsDiffError = 0.0;
double DriveCommand::xMaxAbsError = 0.0;
double DriveCommand::xMaxAbsITerm = 0.0;
double DriveCommand::xMaxAbsOutput = 0.0;

double DriveCommand::yPFac = 0.0;
double DriveCommand::yIFac = 0.0;
double DriveCommand::yDFac = 0.0;
double DriveCommand::yDesiredAccuracy = 0.0;
double DriveCommand::yMaxAbsDiffError = 0.0;
double DriveCommand::yMaxAbsError = 0.0;
double DriveCommand::yMaxAbsITerm = 0.0;
double DriveCommand::yMaxAbsOutput = 0.0;

double DriveCommand::rPFac = 0.0;
double DriveCommand::rIFac = 0.0;
double DriveCommand::rDFac = 0.0;
double DriveCommand::rDesiredAccuracy = 0.0;
double DriveCommand::rMaxAbsDiffError = 0.0;
double DriveCommand::rMaxAbsError = 0.0;
double DriveCommand::rMaxAbsITerm = 0.0;
double DriveCommand::rMaxAbsOutput = 0.0;

PIDConfig* DriveCommand::CreateXPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = xPFac;
	pidConfig->iFac = xIFac;
	pidConfig->dFac = xDFac;
	pidConfig->desiredAccuracy = xDesiredAccuracy;
	pidConfig->maxAbsOutput = xMaxAbsOutput;
	pidConfig->maxAbsDiffError = xMaxAbsDiffError;
	pidConfig->maxAbsError = xMaxAbsError;
	pidConfig->maxAbsITerm = xMaxAbsITerm;
	return pidConfig;
}

PIDConfig* DriveCommand::CreateYPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = yPFac;
	pidConfig->iFac = yIFac;
	pidConfig->dFac = yDFac;
	pidConfig->desiredAccuracy = yDesiredAccuracy;
	pidConfig->maxAbsOutput = yMaxAbsOutput;
	pidConfig->maxAbsDiffError = yMaxAbsDiffError;
	pidConfig->maxAbsError = yMaxAbsError;
	pidConfig->maxAbsITerm = yMaxAbsITerm;
	return pidConfig;
}

PIDConfig* DriveCommand::CreateRPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = rPFac;
	pidConfig->iFac = rIFac;
	pidConfig->dFac = rDFac;
	pidConfig->desiredAccuracy = rDesiredAccuracy;
	pidConfig->maxAbsOutput = rMaxAbsOutput;
	pidConfig->maxAbsDiffError = rMaxAbsDiffError;
	pidConfig->maxAbsError = rMaxAbsError;
	pidConfig->maxAbsITerm = rMaxAbsITerm;
	return pidConfig;
}

double DriveCommand::Abs(double x) {
	if (x > 0.0) {
		return x;
	} else {
		return -x;
	}
}

PivotCommand::PivotCommand(RobotModel* myRobot, double desiredAngleTurn) {
	robot = myRobot;
	desiredR = desiredAngleTurn;
	isDone = false;
	one = (double) 1.0;
}

void PivotCommand::Init() {
	rPIDConfig = CreateRPIDConfig();
	initialR = robot->GetGyroAngle();
	rPID = new PIDControlLoop(rPIDConfig);
	rPID->Init(initialR, initialR + desiredR);
}

double PivotCommand::rPFac = 0.0;
double PivotCommand::rIFac = 0.0;
double PivotCommand::rDFac = 0.0;
double PivotCommand::rDesiredAccuracy = 0.0;
double PivotCommand::rMaxAbsOutput = 0.0;
double PivotCommand::rMaxAbsDiffError = 0.0;
double PivotCommand::rMaxAbsError = 0.0;
double PivotCommand::rMaxAbsITerm = 0.0;

PIDConfig* PivotCommand::CreateRPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = rPFac;
	pidConfig->iFac = rIFac;
	pidConfig->dFac = rDFac;
	pidConfig->desiredAccuracy = rDesiredAccuracy;
	pidConfig->maxAbsOutput = rMaxAbsOutput;
	pidConfig->maxAbsDiffError = rMaxAbsDiffError;
	pidConfig->maxAbsError = rMaxAbsError;
	pidConfig->maxAbsITerm = rMaxAbsITerm;
	return pidConfig;
}

void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {
	bool pidDone = rPID->ControlLoopDone(robot->GetGyroAngle());

	if (pidDone) {
		isDone = true;
		/*
		 * Set wheel speed to 0 so that we don't keep turning after exiting the command
		 */
		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel,0.0);
	} else {
		rOutput = rPID->Update(robot->GetGyroAngle());

		double frontLeft = rOutput;
		double rearLeft = rOutput;
		double frontRight = -rOutput;
		double rearRight = -rOutput;

		double maxAbsMagnitude = 0.0;

		if (maxAbsMagnitude < Abs(rearLeft)) {
			maxAbsMagnitude = Abs(rearLeft);
		}
		if (maxAbsMagnitude < Abs(frontLeft)) {
			maxAbsMagnitude = Abs(frontLeft);
		}
		if (maxAbsMagnitude < Abs(frontRight)) {
			maxAbsMagnitude = Abs(frontRight);
		}
		if (maxAbsMagnitude < Abs(rearRight)) {
			maxAbsMagnitude = Abs(rearRight);
		}

		if (maxAbsMagnitude < one) {
			maxAbsMagnitude = one;
		}

		frontLeft = frontLeft / maxAbsMagnitude;
		rearLeft = rearLeft / maxAbsMagnitude;
		frontRight = frontRight / maxAbsMagnitude;
		rearRight = rearRight / maxAbsMagnitude;

		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, frontLeft);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rearLeft);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frontRight);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, rearRight);
	}
}

bool PivotCommand::IsDone() {
	return isDone;
}

double PivotCommand::Abs(double x) {
	if (x > 0.0) {
		return x;
	} else {
		return -x;
	}
}

StrafeCommand::StrafeCommand(RobotModel* myRobot, double desiredDistance) {
	robot = myRobot;
	desiredX = desiredDistance;
	isDone = false;
	one = (double) 1.0;
}

/**
 * we want an x and an r PID because an x PID does x distance and r PID keeps it from turning.
 * Still goes diagonally though.
 */
void StrafeCommand::Init() {
	xPIDConfig = CreateXPIDConfig();
	rPIDConfig = CreateRPIDConfig();

	xPID = new PIDControlLoop(xPIDConfig);
	rPID = new PIDControlLoop(rPIDConfig);

	initialX = robot->GetXEncoderVal();
	initialR = robot->GetGyroAngle();
	xPID->Init(initialX, initialX + desiredX);
	rPID->Init(initialR, initialR);
}

void StrafeCommand::Update(double currTimeSec, double deltaTimeSec) {
	double newX = robot->GetXEncoderVal();// * cos(robot->GetGyroAngle() - initialR); <- not sure why but isn't in big DriveCommand

	bool xDone = xPID->ControlLoopDone(newX);
	bool rDone = rPID->ControlLoopDone(robot->GetGyroAngle());

	if (xDone && rDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, 0.0);
	} else {
		double x = xPID->Update(newX);
		double r = rPID->Update(robot->GetGyroAngle());

		double frontLeft = x + r;
		double rearLeft = -x + r;
		double frontRight = -x - r;
		double rearRight = x - r;

		double maxAbsMagnitude = 0.0;

		if (maxAbsMagnitude < Abs(rearLeft)) {
			maxAbsMagnitude = Abs(rearLeft);
		}
		if (maxAbsMagnitude < Abs(frontLeft)) {
			maxAbsMagnitude = Abs(frontLeft);
		}
		if (maxAbsMagnitude < Abs(frontRight)) {
			maxAbsMagnitude = Abs(frontRight);
		}
		if (maxAbsMagnitude < Abs(rearRight)) {
			maxAbsMagnitude = Abs(rearRight);
		}

		if (maxAbsMagnitude < one) {
			maxAbsMagnitude = one;
		}

		frontLeft = frontLeft / maxAbsMagnitude;
		rearLeft = rearLeft / maxAbsMagnitude;
		frontRight = frontRight / maxAbsMagnitude;
		rearRight = rearRight / maxAbsMagnitude;

		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, frontLeft);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rearLeft);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frontRight);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, rearRight);
	}
}

bool StrafeCommand::IsDone() {
	return isDone;
}

double StrafeCommand::xPFac = 0.0;
double StrafeCommand::xIFac = 0.0;
double StrafeCommand::xDFac = 0.0;
double StrafeCommand::xDesiredAccuracy = 0.0;
double StrafeCommand::xMaxAbsOutput = 0.0;
double StrafeCommand::xMaxAbsDiffError = 0.0;
double StrafeCommand::xMaxAbsError = 0.0;
double StrafeCommand::xMaxAbsITerm = 0.0;

double StrafeCommand::rPFac = 0.0;
double StrafeCommand::rIFac = 0.0;
double StrafeCommand::rDFac = 0.0;
double StrafeCommand::rDesiredAccuracy = 0.0;
double StrafeCommand::rMaxAbsOutput = 0.0;
double StrafeCommand::rMaxAbsDiffError = 0.0;
double StrafeCommand::rMaxAbsError = 0.0;
double StrafeCommand::rMaxAbsITerm = 0.0;

PIDConfig* StrafeCommand::CreateXPIDConfig() {
	PIDConfig* pid = new PIDConfig();
	pid->pFac = xPFac;
	pid->iFac = xIFac;
	pid->dFac = xDFac;
	pid->desiredAccuracy = xDesiredAccuracy;
	pid->maxAbsOutput = xMaxAbsOutput;
	pid->maxAbsDiffError = xMaxAbsDiffError;
	pid->maxAbsError = xMaxAbsError;
	pid->maxAbsITerm = xMaxAbsITerm;
	return pid;
}

PIDConfig* StrafeCommand::CreateRPIDConfig() {
	PIDConfig* pid = new PIDConfig();
	pid->pFac = rPFac;
	pid->iFac = rIFac;
	pid->dFac = rDFac;
	pid->desiredAccuracy = rDesiredAccuracy;
	pid->maxAbsOutput = rMaxAbsOutput;
	pid->maxAbsDiffError = rMaxAbsDiffError;
	pid->maxAbsError = rMaxAbsError;
	pid->maxAbsITerm = rMaxAbsITerm;
	return pid;
}

double StrafeCommand::Abs(double x) {
	if (x > 0.0) {
		return x;
	} else {
		return -x;
	}
}

DriveStraightCommand::DriveStraightCommand(RobotModel* myRobot,
		DriveController* myDriveController, double desiredDistance) {
	robot = myRobot;
	drive = myDriveController;
	encoderDesired = desiredDistance;
	isDone = false;
	one = (double) 1.0;
}

void DriveStraightCommand::Init() {
	encoderInit = robot->GetYEncoderVal();
	gyroInit = robot->GetGyroAngle();

	yPIDConfig = CreateYPIDConfig();
	rPIDConfig = CreateRPIDConfig();

	yPID = new PIDControlLoop(yPIDConfig);
	rPID = new PIDControlLoop(rPIDConfig);

	yPID->Init(encoderInit, encoderDesired + encoderInit);
	rPID->Init(gyroInit, gyroInit);
}

void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) {
	bool yDone = yPID->ControlLoopDone(robot->GetYEncoderVal());
	bool rDone = rPID->ControlLoopDone(robot->GetGyroAngle());

	if (yDone && rDone) {
		isDone = true;
		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel,0.0);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel,0.0);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel,0.0);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel,0.0);
	} else {
		double yOutput = yPID->Update(robot->GetYEncoderVal());
		double rOutput = rPID->Update(robot->GetGyroAngle());

		double frontLeft = yOutput + rOutput;
		double rearLeft = yOutput + rOutput;
		double frontRight = yOutput - rOutput;
		double rearRight = yOutput - rOutput;

		double maxAbsMagnitude = 0.0;

		if (maxAbsMagnitude < Abs(rearLeft)) {
			maxAbsMagnitude = Abs(rearLeft);
		}
		if (maxAbsMagnitude < Abs(frontLeft)) {
			maxAbsMagnitude = Abs(frontLeft);
		}
		if (maxAbsMagnitude < Abs(frontRight)) {
			maxAbsMagnitude = Abs(frontRight);
		}
		if (maxAbsMagnitude < Abs(rearRight)) {
			maxAbsMagnitude = Abs(rearRight);
		}

		if (maxAbsMagnitude < one) {
			maxAbsMagnitude = one;
		}

		frontLeft = frontLeft / maxAbsMagnitude;
		rearLeft = rearLeft / maxAbsMagnitude;
		frontRight = frontRight / maxAbsMagnitude;
		rearRight = rearRight / maxAbsMagnitude;

		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, frontLeft);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rearLeft);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frontRight);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, rearRight);
	}

}
double DriveStraightCommand::Abs(double x) {
	if (x > 0.0) {
		return x;
	} else {
		return -x;
	}
}
bool DriveStraightCommand:: IsDone() {
	return isDone;
}

double DriveStraightCommand::yPFac = 0.0;
double DriveStraightCommand::yIFac = 0.0;
double DriveStraightCommand::yDFac = 0.0;
double DriveStraightCommand::yDesiredAccuracy = 0.0;
double DriveStraightCommand::yMaxAbsOutput = 0.0;
double DriveStraightCommand::yMaxAbsDiffError = 0.0;
double DriveStraightCommand::yMaxAbsError = 0.0;
double DriveStraightCommand::yMaxAbsITerm = 0.0;

double DriveStraightCommand::rPFac = 0.0;
double DriveStraightCommand::rIFac = 0.0;
double DriveStraightCommand::rDFac = 0.0;
double DriveStraightCommand::rDesiredAccuracy = 0.0;
double DriveStraightCommand::rMaxAbsDiffError = 0.0;
double DriveStraightCommand::rMaxAbsOutput = 0.0;
double DriveStraightCommand::rMaxAbsError = 0.0;
double DriveStraightCommand::rMaxAbsITerm = 0.0;

PIDConfig* DriveStraightCommand::CreateYPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = yPFac;
	pidConfig->iFac = yIFac;
	pidConfig->dFac = yDFac;
	pidConfig->desiredAccuracy = yDesiredAccuracy;
	pidConfig->maxAbsOutput = yMaxAbsOutput;
	pidConfig->maxAbsDiffError = yMaxAbsDiffError;
	pidConfig->maxAbsError = yMaxAbsError;
	pidConfig->maxAbsITerm = yMaxAbsITerm;
	return pidConfig;
}

PIDConfig* DriveStraightCommand::CreateRPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = rPFac;
	pidConfig->iFac = rIFac;
	pidConfig->dFac = rDFac;
	pidConfig->desiredAccuracy = rDesiredAccuracy;
	pidConfig->maxAbsOutput = rMaxAbsOutput;
	pidConfig->maxAbsDiffError = rMaxAbsDiffError;
	pidConfig->maxAbsError = rMaxAbsError;
	pidConfig->maxAbsITerm = rMaxAbsITerm;
	return pidConfig;
}

/**
 * Straightens out the robot so that it is lined up for the tote.
 */
GyroLineUpToteCommand::GyroLineUpToteCommand(RobotModel* myRobot) {
	initialR = 0.0;
	robot = myRobot;
}

void GyroLineUpToteCommand::Init() {
	/*
	 * want to turn to face the tote with the shortest rotation needed, same angle each time
	 */
	rPIDConfig = CreateRPIDConfig();
	rPID = new PIDControlLoop(rPIDConfig);
	initialR = robot->GetGyroAngle();

	double floorR = ((floor)(initialR/360)) * 360.0;
	double ceilR = ((ceil)(initialR/360)) * 360;
	if (fabs(floorR-initialR) < fabs(ceilR - initialR)) {
		desiredR = floorR;
	} else {
		desiredR = ceilR;
	}
	rPID->Init(initialR, desiredR);
}

double GyroLineUpToteCommand::gyroPFac = 0.0;
double GyroLineUpToteCommand::gyroIFac = 0.0;
double GyroLineUpToteCommand::gyroDFac = 0.0;
double GyroLineUpToteCommand::gyroDesiredAccuracy = 0.0;
double GyroLineUpToteCommand::gyroMaxAbsOutput = 0.0;
double GyroLineUpToteCommand::gyroMaxAbsDiffError = 0.0;
double GyroLineUpToteCommand::gyroMaxAbsError = 0.0;
double GyroLineUpToteCommand::gyroMaxAbsITerm = 0.0;

PIDConfig* GyroLineUpToteCommand::CreateRPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = gyroPFac;
	pidConfig->iFac = gyroIFac;
	pidConfig->dFac = gyroDFac;
	pidConfig->desiredAccuracy = gyroDesiredAccuracy;
	pidConfig->maxAbsOutput = gyroMaxAbsOutput;
	pidConfig->maxAbsDiffError = gyroMaxAbsDiffError;
	pidConfig->maxAbsError = gyroMaxAbsError;
	pidConfig->maxAbsITerm = gyroMaxAbsITerm;
	return pidConfig;
}

void GyroLineUpToteCommand::Update(double currTimeSec, double deltaTimeSec) {
	double currR = robot->GetGyroAngle();
	rDone = rPID->ControlLoopDone(currR);
	if (!rDone) {
		double rOutput = rPID->Update(currR);

		double frontLeft = rOutput;
		double rearLeft = rOutput;
		double frontRight = rOutput;
		double rearRight = rOutput;

		double maxAbsMagnitude = 0.0;

		if (maxAbsMagnitude < Abs(rearLeft)) {
			maxAbsMagnitude = Abs(rearLeft);
		}
		if (maxAbsMagnitude < Abs(frontLeft)) {
			maxAbsMagnitude = Abs(frontLeft);
		}
		if (maxAbsMagnitude < Abs(frontRight)) {
			maxAbsMagnitude = Abs(frontRight);
		}
		if (maxAbsMagnitude < Abs(rearRight)) {
			maxAbsMagnitude = Abs(rearRight);
		}

		if (maxAbsMagnitude < one) {
			maxAbsMagnitude = one;
		}

		frontLeft = frontLeft / maxAbsMagnitude;
		rearLeft = rearLeft / maxAbsMagnitude;
		frontRight = frontRight / maxAbsMagnitude;
		rearRight = rearRight / maxAbsMagnitude;

		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, frontLeft);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rearLeft);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frontRight);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, rearRight);
	} else {
		rDone = true;
		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, 0.0);
	}
}

bool GyroLineUpToteCommand::IsDone() {
	return rDone;
}

double GyroLineUpToteCommand::Abs(double x) {
	if (x > 0.0) {
		return x;
	} else {
		return -x;
	}
}

/**
 * Sets values to variables desiredX, desiredY, desiredR, robot, and
 * driveController from given parameters. Also, sets lineUpDone to false.
 */
UltrasonicLineUpToteCommand::UltrasonicLineUpToteCommand(RobotModel* myRobot,
		DriveController* myDriveController, double myDesiredX,
		double myDesiredY, double myDesiredR) {
	initialEncoderX = 0.0;
	initialEncoderY = 0.0;
	initialX = 0.0;
	initialY = 0.0;
	initialR = 0.0;
	desiredX = myDesiredX;
	desiredY = myDesiredY;
	robot = myRobot;
	driveController = myDriveController;
	desiredR = myDesiredR;
	linedUpDone = false;

}
void UltrasonicLineUpToteCommand::Init() {
	/**
	 * Inits PID Loop for x, y, and rotation. Gets the distance from the ultrasonic
	 * sensor and angle from the servo motor to calculate the initialX, initialY,
	 * and get gyro value for initialR variables. Also, gets values from the
	 * encoders for the initialEncoderX and initialEncoderY variables and gets values
	 * from the gyro sensor for the intialGyroR. These to be used for comparison for
	 * update. Also creates all of the PIDConfigs and PIDControlLoops for the
	 * update method
	 */

	xPIDConfig = CreateXPIDConfig();
	yPIDConfig = CreateYPIDConfig();
	rPIDConfig = CreateRPIDConfig();
	xPID = new PIDControlLoop(xPIDConfig);
	yPID = new PIDControlLoop(yPIDConfig);
	rPID = new PIDControlLoop(rPIDConfig);
	double initialDistance = robot->ultrasonic->GetRangeInFeet();
	double initialAngle = PI - (robot->GetServoAngle() * PI / 180);
	initialX = initialDistance * cos(initialAngle);
	initialY = initialDistance * sin(initialAngle);
	initialR = robot->GetNavxAngle();
	xPID->Init(0.0, initialX - desiredX);
	yPID->Init(0.0, initialY - desiredY);
	rPID->Init(initialR, initialR);
	initialGyroR = initialR;
	initialEncoderX = robot->GetXEncoderVal();
	initialEncoderY = robot->GetYEncoderVal();
	if (robot->GetServoAngle() > 90.0) {
		linedUpDone = true;
		//SmartDashboard::PutString("Servo Too Far!", "NOOO!");
	}
	if (Abs(initialY - desiredY) > 3.0 || Abs(initialX - desiredX) > 3.0) {
		linedUpDone = true;
	}

	//SmartDashboard::PutNumber("InitialX", initialX);
	//SmartDashboard::PutNumber("InitialY", initialY);
}

double UltrasonicLineUpToteCommand::xPFac = 0.0;
double UltrasonicLineUpToteCommand::xIFac = 0.0;
double UltrasonicLineUpToteCommand::xDFac = 0.0;
double UltrasonicLineUpToteCommand::xDesiredAccuracy = 0.0;
double UltrasonicLineUpToteCommand::xMaxAbsDiffError = 0.0;
double UltrasonicLineUpToteCommand::xMaxAbsError = 0.0;
double UltrasonicLineUpToteCommand::xMaxAbsITerm = 0.0;
double UltrasonicLineUpToteCommand::xMaxAbsOutput = 0.0;

double UltrasonicLineUpToteCommand::yPFac = 0.0;
double UltrasonicLineUpToteCommand::yIFac = 0.0;
double UltrasonicLineUpToteCommand::yDFac = 0.0;
double UltrasonicLineUpToteCommand::yDesiredAccuracy = 0.0;
double UltrasonicLineUpToteCommand::yMaxAbsDiffError = 0.0;
double UltrasonicLineUpToteCommand::yMaxAbsError = 0.0;
double UltrasonicLineUpToteCommand::yMaxAbsITerm = 0.0;
double UltrasonicLineUpToteCommand::yMaxAbsOutput = 0.0;

double UltrasonicLineUpToteCommand::rPFac = 0.0;
double UltrasonicLineUpToteCommand::rIFac = 0.0;
double UltrasonicLineUpToteCommand::rDFac = 0.0;
double UltrasonicLineUpToteCommand::rDesiredAccuracy = 0.0;
double UltrasonicLineUpToteCommand::rMaxAbsDiffError = 0.0;
double UltrasonicLineUpToteCommand::rMaxAbsError = 0.0;
double UltrasonicLineUpToteCommand::rMaxAbsITerm = 0.0;
double UltrasonicLineUpToteCommand::rMaxAbsOutput = 0.0;

PIDConfig* UltrasonicLineUpToteCommand::CreateXPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = xPFac;
	pidConfig->iFac = xIFac;
	pidConfig->dFac = xDFac;
	pidConfig->desiredAccuracy = xDesiredAccuracy;
	pidConfig->maxAbsOutput = xMaxAbsOutput;
	pidConfig->maxAbsDiffError = xMaxAbsDiffError;
	pidConfig->maxAbsError = xMaxAbsError;
	pidConfig->maxAbsITerm = xMaxAbsITerm;
	return pidConfig;
}

PIDConfig* UltrasonicLineUpToteCommand::CreateYPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = yPFac;
	pidConfig->iFac = yIFac;
	pidConfig->dFac = yDFac;
	pidConfig->desiredAccuracy = yDesiredAccuracy;
	pidConfig->maxAbsOutput = yMaxAbsOutput;
	pidConfig->maxAbsDiffError = yMaxAbsDiffError;
	pidConfig->maxAbsError = yMaxAbsError;
	pidConfig->maxAbsITerm = yMaxAbsITerm;
	return pidConfig;
}

PIDConfig* UltrasonicLineUpToteCommand::CreateRPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = rPFac;
	pidConfig->iFac = rIFac;
	pidConfig->dFac = rDFac;
	pidConfig->desiredAccuracy = rDesiredAccuracy;
	pidConfig->maxAbsOutput = rMaxAbsOutput;
	pidConfig->maxAbsDiffError = rMaxAbsDiffError;
	pidConfig->maxAbsError = rMaxAbsError;
	pidConfig->maxAbsITerm = rMaxAbsITerm;
	return pidConfig;
}

/**
 * Gets the current sensor values from the various sensors (encoders and gyro)
 * and finds out if all (and only all) the loops are done, in which case IsDone()'s
 * bool variable is true. Otherwise, uses MecanumDrive() from driveController
 * to drive with the updated PID values.
 */
void UltrasonicLineUpToteCommand::Update(double currTimeSec,
		double deltaTimeSec) {

	double currX = robot->GetXEncoderVal() - initialEncoderX;
	double currY = robot->GetYEncoderVal() - initialEncoderY;
	double currR = robot->GetNavxAngle();

	bool xDone = xPID->ControlLoopDone(currX);
	bool yDone = yPID->ControlLoopDone(currY);
	bool rDone = rPID->ControlLoopDone(currR);

	bool allDone = (xDone && yDone && rDone);

	if (allDone || linedUpDone) {
		linedUpDone = true;
		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, 0.0);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, 0.0);
	} else {
		xOutput = xPID->Update(currX);
		yOutput = yPID->Update(currY);
		rOutput = rPID->Update(currR);

		double frontLeft = xOutput + yOutput + rOutput;
		double rearLeft = -xOutput + yOutput + rOutput;
		double frontRight = -xOutput + yOutput - rOutput;
		double rearRight = xOutput + yOutput - rOutput;
		double maxAbsMagnitude = 0.0;

		if (maxAbsMagnitude < Abs(rearLeft)) {
			maxAbsMagnitude = Abs(rearLeft);
		}
		if (maxAbsMagnitude < Abs(frontLeft)) {
			maxAbsMagnitude = Abs(frontLeft);
		}
		if (maxAbsMagnitude < Abs(frontRight)) {
			maxAbsMagnitude = Abs(frontRight);
		}
		if (maxAbsMagnitude < Abs(rearRight)) {
			maxAbsMagnitude = Abs(rearRight);
		}

		if (maxAbsMagnitude < one) {
			maxAbsMagnitude = one;
		}

		frontLeft = frontLeft / maxAbsMagnitude;
		rearLeft = rearLeft / maxAbsMagnitude;
		frontRight = frontRight / maxAbsMagnitude;
		rearRight = rearRight / maxAbsMagnitude;

		robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, frontLeft);
		robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rearLeft);
		robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frontRight);
		robot->SetWheelSpeed(RobotModel::kRearRightWheel, rearRight);

	}
}
bool UltrasonicLineUpToteCommand::IsDone() {
	if (linedUpDone) {
		printf("Line up done");
	}
	return linedUpDone;
}
/**
 * Sets values to variables desiredX, desiredY, desiredR, robot, and
 * driveController from given parameters. Also, sets lineUpDone to false.
 */
UltrasonicLineUpBinCommand::UltrasonicLineUpBinCommand(RobotModel* myRobot,
		DriveController* myDriveController, double myDesiredX,
		double myDesiredY, double myDesiredR) {
	initialEncoderX = 0.0;
	initialEncoderY = 0.0;
	initialX = 0.0;
	initialY = 0.0;
	initialR = 0.0;

	desiredX = myDesiredX;
	desiredY = myDesiredY;
	desiredR = myDesiredY;
	robot = myRobot;
	driveController = myDriveController;
	linedUpDone = false;
}

	/**
	 * Inits PID Loop for x, y, and rotation. Gets the distance from the ultrasonic
	 * sensor and angle from the servo motor to calculate the initialX, initialY,
	 * and get gyro value for initialR variables. Also, gets values from the
	 * encoders for the initialEncoderX and initialEncoderY variables and gets values
	 * from the gyro sensor for the intialGyroR. These to be used for comparison for
	 * update. Also creates all of the PIDConfigs and PIDControlLoops for the
	 * update method
	 */
void UltrasonicLineUpBinCommand::Init() {
	xPIDConfig = CreateXPIDConfig();
	yPIDConfig = CreateYPIDConfig();
	rPIDConfig = CreateRPIDConfig();
	xPID = new PIDControlLoop(xPIDConfig);
	yPID = new PIDControlLoop(yPIDConfig);
	rPID = new PIDControlLoop(rPIDConfig);

	initialDistanceFromBin = robot->GetUltrasonicDistance();
	servoAngle = robot->GetServoAngle() * PI / 180;
	initialX = initialDistanceFromBin * cos(servoAngle);
	initialY = initialDistanceFromBin * sin(servoAngle);
	initialR = robot->GetGyroAngle();
	xPID->Init(initialX, desiredX);
	yPID->Init(initialY, desiredY);
	rPID->Init(initialR, initialR);
	initialGyroR = initialR;
	initialEncoderX = robot->GetXEncoderVal();
	initialEncoderY = robot->GetYEncoderVal();
}

PIDConfig* UltrasonicLineUpBinCommand::CreateXPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = SmartDashboard::GetNumber("xPID PFAC: ");
	pidConfig->iFac = SmartDashboard::GetNumber("xPID IFAC: ");
	pidConfig->dFac = SmartDashboard::GetNumber("xPID DFAC: ");
	pidConfig->desiredAccuracy = SmartDashboard::GetNumber("xPID Accuracy: ");
	pidConfig->maxAbsOutput = SmartDashboard::GetNumber("xPID maxOutput: ");
	pidConfig->maxAbsDiffError = SmartDashboard::GetNumber("xPID maxDiffError: ");
	pidConfig->maxAbsError = SmartDashboard::GetNumber("xPID maxError: ");
	pidConfig->maxAbsITerm = SmartDashboard::GetNumber("xPID maxITerm: ");
	return pidConfig;
}

PIDConfig* UltrasonicLineUpBinCommand::CreateYPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = SmartDashboard::GetNumber("yPID PFAC: ");
	pidConfig->iFac = SmartDashboard::GetNumber("yPID IFAC: ");
	pidConfig->dFac = SmartDashboard::GetNumber("yPID DFAC: ");
	pidConfig->desiredAccuracy = SmartDashboard::GetNumber("yPID Accuracy: ");
	pidConfig->maxAbsOutput = SmartDashboard::GetNumber("yPID maxOutput: ");
	pidConfig->maxAbsDiffError = SmartDashboard::GetNumber("yPID maxDiffError: ");
	pidConfig->maxAbsError = SmartDashboard::GetNumber("yPID maxError: ");
	pidConfig->maxAbsITerm = SmartDashboard::GetNumber("yPID maxITerm: ");
	return pidConfig;
}

PIDConfig* UltrasonicLineUpBinCommand::CreateRPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = SmartDashboard::GetNumber("rPID PFAC: ");
	pidConfig->iFac = SmartDashboard::GetNumber("rPID IFAC: ");
	pidConfig->dFac = SmartDashboard::GetNumber("rPID DFAC: ");
	pidConfig->desiredAccuracy = SmartDashboard::GetNumber("rPID Accuracy: ");
	pidConfig->maxAbsOutput = SmartDashboard::GetNumber("rPID maxOutput: ");
	pidConfig->maxAbsDiffError = SmartDashboard::GetNumber(
			"rPID maxDiffError: ");
	pidConfig->maxAbsError = SmartDashboard::GetNumber("rPID maxError: ");
	pidConfig->maxAbsITerm = SmartDashboard::GetNumber("rPID maxITerm: ");
	return pidConfig;
}

/**
	 * Gets the current sensor values from the various sensors (encoders and gyro)
	 * and finds out if all (and only all) the loops are done, in which case IsDone()'s
	 * bool variable is true. Otherwise, uses MecanumDrive() from driveController
	 * to drive with the updated PID values.
	 */
void UltrasonicLineUpBinCommand::Update(double currTimeSec,
		double deltaTimeSec) {

	double currX = robot->GetXEncoderVal() - initialEncoderX + initialX;
	double currY = robot->GetYEncoderVal() - initialEncoderY + initialY;
	double currR = robot->GetGyroAngle() - initialGyroR;

	xDone = yPID->ControlLoopDone(currX);
	yDone = yPID->ControlLoopDone(currY);
	rDone = rPID->ControlLoopDone(currR);

	bool allDone = (xDone && yDone && rDone);

	if (allDone) {
		linedUpDone = true;
	} else {
		double xVal = xPID->Update(currX);
		double yVal = yPID->Update(currY);
		double rVal = rPID->Update(currR);

		driveController->MecanumDrive(xVal, yVal, rVal);
	}
}
bool UltrasonicLineUpBinCommand::IsDone() {
	return linedUpDone;
}

RollOutCommand::RollOutCommand(ElevatorController* myElevator, double myTime) {
	elevator = myElevator;
	time = myTime;
	rollOutDone = false;
}

void RollOutCommand::Init() {
	timer = new Timer();
	timer->Start();
	elevator->SetRollOutDesired(true);
}

void RollOutCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (timer->Get() > time) {
		elevator->SetRollOutDesired(false);
		rollOutDone = true;
	}
}

bool RollOutCommand::IsDone() {
	if (rollOutDone) {
		elevator->SetRollOutDone(true);
	}
	return rollOutDone;
}

ClampCommand::ClampCommand(ElevatorController* myElevator, bool myClamp) {
	elevator = myElevator;
	clamp = myClamp;
	clampDone = false;
}

void ClampCommand::Init() {

	elevator->SetAutoClamp(clamp);
}

void ClampCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (elevator->AutoClampDone()) {
		clampDone = true;
	} else {
		clampDone = false;
	}
}

bool ClampCommand::IsDone() {
	return clampDone;
}

RampCommand::RampCommand(ElevatorController* elevator, bool rampOut) {
	this->elevator = elevator;
	desiredRampOut = rampOut;
	rampOutDone = false;
}

void RampCommand::Init() {
	elevator->SetAutoRampOutDesired(desiredRampOut);
}

void RampCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (elevator->AutoRampOutDone()) {
		rampOutDone = true;
	} else {
		rampOutDone = false;
	}
}

bool RampCommand::IsDone() {
	return rampOutDone;
}

IntakingCommand::IntakingCommand(ElevatorController* elevator) {
	this->elevator = elevator;
	intakingDone = false;
}

void IntakingCommand::Init() {
	elevator->SetAutoIntaking();
}

void IntakingCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (elevator->AutoIntakingDone()) {
		intakingDone = true;
	} else {
		intakingDone = false;
	}
}

bool IntakingCommand::IsDone() {
	return intakingDone;
}

OuttakingCommand::OuttakingCommand(ElevatorController* elevator) {
	this->elevator = elevator;
	outtakingDone = false;
}

void OuttakingCommand::Init() {
	elevator->SetAutoOuttaking();
}

void OuttakingCommand::Update(double currTimeSec, double deltaTimeSec) {
	if (elevator->AutoOuttakingDone()) {
		outtakingDone = true;
	} else {
		outtakingDone = false;
	}
}

bool OuttakingCommand::IsDone() {
	return outtakingDone;
}

/**
 * @param const std::string& playbackFileName is the name of the file to be
 * read and fed into the robot during autonomous. It must be created by turning
 * isRecording on in ControlBoard in order to have the correct format.
 * @see ControlBoard.h
 */
PlaybackAutoCommand::PlaybackAutoCommand(RobotModel* myRobot, const std::string&
		playbackFileName) {
	robot = myRobot;
	currTime = 0.0;
	oldTime = 0.0;
	buffer = "";
	varName = "";
	varVal = 0.0;
	playbackFilePath = playbackFileName;
	playbackFile.open(playbackFilePath, std::ifstream::in);
	autoControl = new AutoControlBoard(robot, playbackFilePath);
	autoDriveControl = new DriveController(robot, autoControl);
	autoElevControl = new ElevatorController(robot, autoControl);

}

/**
 * PlaybackAutoCommand works on the principle of creating a facade ControlBoard
 * that feeds all the controllers (DriveController, ElevatorController, etc) values
 * from the playback file instead of from the joystick. New instances of the controllers
 * are created just for the purpose of autonomous; these are reset in Init()
 * @see ControlBoard.h
 */
void PlaybackAutoCommand::Init() {
	autoDriveControl->Reset();
	autoElevControl->Reset();
	autoDriveControl->RefreshIni();
	autoElevControl->RefreshIni();

}

/**
 * Every loop through the code reads a new line from the playback file, which
 * is in the format of loopNumber,"variableName","variableValue" eg.
 * 223,"mIntakeJoyX","0" where 223 is the 223rd iteration through the code.
 * This line is parsed to extract the variable name and value, which are updated,
 * ie. the variable in the line is set to the value given in the line.
 * Each new loop through the code is demarcated by ###,"Timeslice","0.0" at the
 * start and ###,"Timeslice","1.0" at the end. This method will update all variables
 * listed between these two lines.
 * @see RefreshValues() for how the parsing works.
 */
void PlaybackAutoCommand::Update(double currTimeSec, double deltaTimeSec) {
	oldTime = currTime;
	std::getline(playbackFile, buffer);
	if (!IsDone()) {
		if (Contains(buffer, "\"Timeslice\",\"0.0\"")) {
			std::getline(playbackFile, buffer);
			while (!(Contains(buffer, "\"Timeslice\",\"1.0\"")) && !IsDone()) {
				RefreshValues(); // only refreshes one line
				SetValue(varName, varVal);
				oldTime = currTime;
				std::getline(playbackFile, buffer);
			}
		}
	}
	autoDriveControl->Update(currTimeSec, deltaTimeSec);
	autoElevControl->Update(currTimeSec, deltaTimeSec);
	return;
}

/**
 * When the file has no more lines or is corrupted, PlaybackAutoCommand is
 * finished and the file is closed.
 * @return true if the file has ended, and false if not
 */
bool PlaybackAutoCommand::IsDone() {
	if (playbackFile.eof() || !playbackFile.good()) {
		printf("finished reading file\n");
		playbackFile.close();
		return true;
	} else {
		return false;
	}
}

/**
 * Parses the format of each line to extract the variable name and its value.
 * @see Update(double currTimeSec, double DeltaTimeSec) for where it's used.
 */
void PlaybackAutoCommand::RefreshValues() {
	// updates member variables varName, varVal for current line
	int position1 = buffer.find_first_of(',');
	if (position1 < 0) { // if end of file basically
		return;
	} else {
		int position2 = buffer.find_last_of(',');
		varName = buffer.substr(position1 + 2, position2 - position1 - 3);
		varVal = std::stod(buffer.substr(position2 + 2, buffer.length() - position2 - 3));
	}
}

/**
 * A whole big list of possible variable strings found in the playback file,
 * and their corresponding variables. Whenever variables potentially useful
 * for autonomous are added to any controller, this method should be updated
 * as well as ControlBoard.h and .cpp
 * @see DriveController.h
 * @see ElevatorController.h
 * @see ControlBoard.h
 * @param std::string name is the string version of the name, double val is
 * the value of the variable
 */
void PlaybackAutoCommand::SetValue(std::string name, double val) {
	printf("Entering Set Value %s %f\n", name.c_str(), val);
	if (name.compare("mLeftJoyX") == 0) {
		autoControl->SetJoystickValue(RemoteController::kLeftJoy, RemoteController::kX, val);
	}
	if (name.compare("mLeftJoyY") == 0) {
		autoControl->SetJoystickValue(RemoteController::kLeftJoy, RemoteController::kY, val);
	}
	if (name.compare("mRightJoyX") == 0) {
		autoControl->SetJoystickValue(RemoteController::kRightJoy, RemoteController::kX, val);
	}
	if (name.compare("mRightJoyY") == 0) {
		autoControl->SetJoystickValue(RemoteController::kRightJoy, RemoteController::kY, val);
	}
	if (name.compare("mIntakeJoyX") == 0) {
		autoControl->SetJoystickValue(RemoteController::kIntakeJoy, RemoteController::kX, val);
	}
	if (name.compare("mIntakeJoyY") == 0) {
		autoControl->SetJoystickValue(RemoteController::kIntakeJoy, RemoteController::kY, val);
	}
	if (name.compare("moveElevatorUpSpeed") == 0) {
		autoControl->SetElevatorUpSpeedAdjuster(val);
	}
	if (name.compare("moveElevatorDownSpeed") == 0) {
		autoControl->SetElevatorDownSpeedAdjuster(val);
	}
	bool bVal = ( val == 0.0 ) ? false : true;
	if (name.compare("mReverseDriveDesired") == 0) {
		autoControl->SetReverseDriveDesired(bVal);
	}
	if (name.compare("fieldCentricDesired") == 0) {
		autoControl->SetFieldCentricDesired(bVal);
	}
	if (name.compare("outtakeToPlatformDesired") == 0) {
		autoControl->SetOuttakeToPlatformDesired(bVal);
	}
	if (name.compare("rampDesired") == 0) {
		autoControl->SetRampDesired(bVal);
	}
	if (name.compare("intakeDesired") == 0) {
		autoControl->SetIntakeDesired(bVal);
//		printf("intakeDesired: %d\n", bVal);
	}
	if (name.compare("moveElevatorUpStartDesired") == 0) {
		autoControl->SetMoveElevatorUpStartDesired(bVal);
		//printf("moveElevatorStartDesired: %d\n", bVal);
		//printf("control board moveElevatorStart %d\n", autoControl->MoveElevatorUpStartDesired());
	}
	if (name.compare("moveElevatorUpEndDesired") == 0) {
		autoControl->SetMoveElevatorUpEndDesired(bVal);
		//printf("moveElevatorEndDesired %d\n", bVal);
	}
	if (name.compare("moveElevatorUpDesired") == 0) {
		autoControl->SetMoveElevatorUpDesired(bVal);
		printf("moveElevatorUpDesired %d\n", bVal);
	}
	if (name.compare("moveElevatorDownStartDesired") == 0) {
		autoControl->SetMoveElevatorDownStartDesired(bVal);
	}
	if (name.compare("moveElevatorDownEndDesired") == 0) {
		autoControl->SetMoveElevatorDownEndDesired(bVal);
	}
	if (name.compare("moveElevatorDownDesired") == 0) {
		autoControl->SetMoveElevatorDownDesired(bVal);
		printf("moveElevatorDownDesired %d\n", bVal);
	}
}

/**
 * Helper function to detect whether the first string contains the second.
 * @param std::string bigString is the string that contains others, and
 * std::string lilString is the string to be found in bigString.
 * @return true if lilString is found, but false if not.
 */
bool PlaybackAutoCommand::Contains(std::string bigString, std::string lilString) {
	for (int i = 0; i < (bigString.length() - lilString.length()); i++) {
		if (bigString.substr(i, lilString.length()).compare(lilString) == 0) {
			return true;
		}
	}
	return false;
}
