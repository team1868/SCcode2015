#include "DriveController.h"
#include "RobotPorts2015.h"
//#include "RemoteControl.h"
#include "WPILib.h"
#include <math.h>
#include <algorithm>

DriveController::DriveController(RobotModel *myRobot, RemoteController *myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;
	nextState = kInitialize;
	m_stateVal = kInitialize;
	joyX = 0;
	joyY = 0;
	joyRotate = 0;
	tolerance = 0.15;
	oldFL = 0.0;
	oldRL = 0.0;
	oldFR = 0.0;
	oldRR = 0.0;
	timer = new Timer();
	timer->Start();
	isFirstStrafeIteration = true;
	timeOfFirstStrafeIteration = 0.0;
	strafeDirection = 0;
	numTimesStrafeButtonPressed = 0;
	oldAngle = robot->GetNavxAngle();
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	switch (m_stateVal) {
	case (kInitialize):
		nextState = kEnterControlledTeleop;
		break;
	case (kReset):
		nextState = kTeleopDrive;
		break;
	case (kTeleopDrive):

		joyX = DriveDirection() * humanControl->GetJoystickValue(RemoteController::kLeftJoy,
				RemoteController::kX);
		joyY = DriveDirection() * humanControl->GetJoystickValue(RemoteController::kLeftJoy,
			RemoteController::kY);
		joyRotate = humanControl->GetJoystickValue(
				RemoteController::kRightJoy, RemoteController::kX);
		/*
		if (humanControl->XValDesired()) {
			joyY = 0.0;
		}
		if (humanControl->YValDesired()) {
			joyX = 0.0;
		} // todo what happens if someone presses both buttons? you can't move?
		*/
		// testing
		/*
		 * 3point turn clause
		 */
		if (Abs(joyX) < tolerance) {
			joyX = 0.0;
		}
		if (Abs(joyY) < tolerance) {
			joyY = 0.0;
		}
		if (Abs(joyRotate) < tolerance) {
			joyRotate = 0.0;
		}
		if (joyY < -0.1) {
			joyRotate = -joyRotate;
		}


		if (humanControl->FieldCentricDesired()) {
			MecanumDrive(joyX, joyY, joyRotate, robot->GetNavxAngle());
		} else {
			MecanumDrive(joyX, joyY, joyRotate);
		}
		nextState = kTeleopDrive;
		break;
	case (kEnterControlledTeleop):
		oldFL = robot->GetFrontLeftEncoderVal();
		oldRL = robot->GetRearLeftEncoderVal();
		oldFR = robot->GetFrontRightEncoderVal();
		oldRR = robot->GetRearRightEncoderVal();
		flPIDConfig = CreateFLPIDConfig();
		rlPIDConfig = CreateRLPIDConfig();
		frPIDConfig = CreateFRPIDConfig();
		rrPIDConfig = CreateRRPIDConfig();
		flPID = new PIDControlLoop(flPIDConfig);
		rlPID = new PIDControlLoop(rlPIDConfig);
		frPID = new PIDControlLoop(frPIDConfig);
		rrPID = new PIDControlLoop(rrPIDConfig);
		nextState = kControlledTeleop;
		break;
	case (kControlledTeleop):

		joyX = DriveDirection()
				* humanControl->GetJoystickValue(RemoteController::kLeftJoy,
						RemoteController::kX);
		joyY = DriveDirection()
				* humanControl->GetJoystickValue(RemoteController::kLeftJoy,
						RemoteController::kY);
		joyRotate = humanControl->GetJoystickValue(RemoteController::kRightJoy,
						RemoteController::kX);

		if (Abs(joyX) < tolerance) {
			joyX = 0.0;
		}
		if (Abs(joyY) < tolerance) {
			joyY = 0.0;
		}
		if (Abs(joyRotate) < tolerance) {
			joyRotate = 0.0;
		}
		if (joyY < -0.1) {
			joyRotate = -joyRotate;
		}
		// ControlledMecanumDrive(joyX, joyY, joyRotate, currTimeSec, deltaTimeSec);
		// temporarily changing to use version with navx/gyro angle
		ControlledMecanumDrive(joyX, joyY, joyRotate, robot->GetNavxAngle(), currTimeSec, deltaTimeSec);
		printf("navx angle: %f\n", robot->GetNavxAngle());
		nextState = kControlledTeleop;

		if (humanControl->LeftDesired()) {
			nextState = kButtonStrafe;
			strafeDirection = -1;
			numTimesStrafeButtonPressed++;
		}

		if (humanControl->RightDesired()) {
			nextState = kButtonStrafe;
			strafeDirection = 1;
			numTimesStrafeButtonPressed++;
		}

		break;

	case (kButtonStrafe):
		nextState = kButtonStrafe;
		if (humanControl->RightDesired() || humanControl->LeftDesired()) {
			numTimesStrafeButtonPressed++;
		} else if (isFirstStrafeIteration) {
			timeOfFirstStrafeIteration = timer->Get();
			isFirstStrafeIteration = false;
		} else if (timer->Get() - timeOfFirstStrafeIteration < strafeButtonTime * numTimesStrafeButtonPressed) {
			// ControlledMecanumDrive(xSpeedStrafeButton * strafeDirection, ySpeedStrafeButton * -strafeDirection, 0, currTimeSec, deltaTimeSec);
			// temporarily changing to version w gyro/navx angle
			ControlledMecanumDrive(xSpeedStrafeButton * strafeDirection, ySpeedStrafeButton * -strafeDirection, 0, robot->GetNavxAngle(), currTimeSec, deltaTimeSec);
			printf("navx angle: %f\n", robot->GetNavxAngle());
		} else {
			// ControlledMecanumDrive(0, 0, 0, currTimeSec, deltaTimeSec);
			// temporarily changing to version w gyro/navx angle
			ControlledMecanumDrive(0, 0, 0, robot->GetNavxAngle(), currTimeSec, deltaTimeSec);
			printf("navx angle: %f\n", robot->GetNavxAngle());
			isFirstStrafeIteration = true;
			numTimesStrafeButtonPressed = 0;
			nextState = kControlledTeleop;
		}
		break;
	}

	m_stateVal = nextState;
}

void DriveController::RefreshIni() {
	flPFac = robot->pini->getf("DRIVE", "flPFac", 0.7);
	flIFac = robot->pini->getf("DRIVE", "flIFac", 0.002);
	flDFac = robot->pini->getf("DRIVE", "flDFac", 1.3);
	flDesiredAccuracy = robot->pini->getf("DRIVE", "flDesiredAccuracy", 0.15);
	flMaxAbsError = robot->pini->getf("DRIVE", "flMaxAbsError", 1.0);
	flMaxAbsDiffError = robot->pini->getf("DRIVE", "flMaxAbsDiffError", 3.0);
	flMaxAbsITerm = robot->pini->getf("DRIVE", "flMaxAbsITerm", 0.3);
	flMaxAbsOutput = robot->pini->getf("DRIVE", "flMaxAbsOutput", 1.0);

	rlPFac = robot->pini->getf("DRIVE", "rlPFac", 0.7);
	rlIFac = robot->pini->getf("DRIVE", "rlIFac", 0.002);
	rlDFac = robot->pini->getf("DRIVE", "rlDFac", 1.3);
	rlDesiredAccuracy = robot->pini->getf("DRIVE", "rlDesiredAccuracy", 0.15);
	rlMaxAbsError = robot->pini->getf("DRIVE", "rlMaxAbsError", 1.0);
	rlMaxAbsDiffError = robot->pini->getf("DRIVE", "rlMaxAbsDiffError", 3.0);
	rlMaxAbsITerm = robot->pini->getf("DRIVE", "rlMaxAbsITerm", 0.3);
	rlMaxAbsOutput = robot->pini->getf("DRIVE", "rlMaxAbsOutput", 1.0);

	frPFac = robot->pini->getf("DRIVE", "frPFac", 0.7);
	frIFac = robot->pini->getf("DRIVE", "frIFac", 0.002);
	frDFac = robot->pini->getf("DRIVE", "frDFac", 1.3);
	frDesiredAccuracy = robot->pini->getf("DRIVE", "frDesiredAccuracy", 0.15);
	frMaxAbsError = robot->pini->getf("DRIVE", "frMaxAbsError", 1.0);
	frMaxAbsDiffError = robot->pini->getf("DRIVE", "frMaxAbsDiffError", 3.0);
	frMaxAbsITerm = robot->pini->getf("DRIVE", "frMaxAbsITerm", 0.3);
	frMaxAbsOutput = robot->pini->getf("DRIVE", "frMaxAbsOutput", 1.0);

	rrPFac = robot->pini->getf("DRIVE", "rrPFac", 0.7);
	rrIFac = robot->pini->getf("DRIVE", "rrIFac", 0.002);
	rrDFac = robot->pini->getf("DRIVE", "rrDFac", 1.3);
	rrDesiredAccuracy = robot->pini->getf("DRIVE", "rrDesiredAccuracy", 0.15);
	rrMaxAbsError = robot->pini->getf("DRIVE", "rrMaxAbsError", 1.0);
	rrMaxAbsDiffError = robot->pini->getf("DRIVE", "rrMaxAbsDiffError", 3.0);
	rrMaxAbsITerm = robot->pini->getf("DRIVE", "rrMaxAbsITerm", 0.3);
	rrMaxAbsOutput = robot->pini->getf("DRIVE", "rrMaxAbsOutput", 1.0);
	strafeButtonTime = robot->pini->getf("DRIVE", "strafeButtonTime", 0.25);
	xSpeedStrafeButton = robot->pini->getf("DRIVE", "xSpeedStrafeButton", 0.9);
	ySpeedStrafeButton = robot->pini->getf("DRIVE", "ySpeedStrafeButton", 0.13);
}

void DriveController::Reset() {
	oldFL = 0.0;
	oldRL = 0.0;
	oldFR = 0.0;
	oldRR = 0.0;
}

// created 7/31/2015 by chloe as a copy of mecanum drive (cartesian) from wpi that includes gyro

void DriveController::ControlledMecanumDrive(double joyX, double joyY, double joyR, double gyroAngle,
		double currTimeSec, double deltaTimeSec) {
	printf("joyX: %f    ", joyX);
	double x = joyX;
	double y = 1.3*joyY; // might need to negate y
	double r = joyR;
	double angle = gyroAngle;
	double dAngle = 0;

	if (angle < 0)
		angle += 360;

	// Following lines in hope of stopping field centric
	angle = angle * (3.14159 / 180.0); // convert to radians
	printf("r: %f   ", r);
	printf("fabs(x): %f   ", fabs(x));
	printf("x: %f    ", x);

	double desiredFL = x+y+r;
	double desiredRL = -x+y+r;
	double desiredFR = -x+y-r;
	double desiredRR = x+y-r;

	if (fabs(r) < 0.1 && fabs(x) > 0.1) { // do not keep a consistent angle when wanting to change angle
		printf("HELLO   ");
		if (angle != oldAngle) {
			dAngle = (angle - oldAngle) / deltaTimeSec; // dAngle measures speed, the same units as r
			printf("dAngle in loop: %f   ", dAngle);
			r -= dAngle; // counteracting the change in r
		}
		desiredFL = x+y+r;
		desiredRL = -x+y+r;
		desiredFR = -x+y-r;
		desiredRR = x+y-r;
	}

	oldAngle = angle;

	double maxAbsMagnitude = 0.0;

	if (maxAbsMagnitude < Abs(desiredFL)) {
		maxAbsMagnitude = Abs(desiredFL);
	}
	if (maxAbsMagnitude < Abs(desiredRL)) {
		maxAbsMagnitude = Abs(desiredRL);
	}
	if (maxAbsMagnitude < Abs(desiredFR)) {
		maxAbsMagnitude = Abs(desiredFR);
	}
	if (maxAbsMagnitude < Abs(desiredRR)) {
		maxAbsMagnitude = Abs(desiredRR);
	}

	if (maxAbsMagnitude < 1.0) {
		maxAbsMagnitude = 1.0;
	}

	desiredFL = desiredFL / maxAbsMagnitude;
	desiredRL = desiredRL / maxAbsMagnitude;
	desiredFR = desiredFR / maxAbsMagnitude;
	desiredRR = desiredRR / maxAbsMagnitude;

	printf("Desired fl %f\n", desiredFL);
	printf("Desired rl %f\n", desiredRL);
	printf("Desired fr %f\n", desiredFR);
	printf("Desired rr %f\n", desiredRR);

	currFL = robot->GetFrontLeftEncoderVal();
	currRL = robot->GetRearLeftEncoderVal();
	currFR = robot->GetFrontRightEncoderVal();
	currRR = robot->GetRearRightEncoderVal();

	flSpeed = (currFL - oldFL) / deltaTimeSec;
	rlSpeed = (currRL - oldRL) / deltaTimeSec;
	frSpeed = (currFR - oldFR) / deltaTimeSec;
	rrSpeed = (currRR - oldRR) / deltaTimeSec;

	flSpeed = EncoderToMotorSpeed(flSpeed);
	rlSpeed = EncoderToMotorSpeed(rlSpeed);
	frSpeed = EncoderToMotorSpeed(frSpeed);
	rrSpeed = EncoderToMotorSpeed(rrSpeed);

//	printf("FL Speed %f\n", flSpeed);
//	printf("RL Speed %f\n", rlSpeed);
//	printf("FR Speed %f\n", frSpeed);
//	printf("RR Speed %f\n", rrSpeed);

//	printf("FL \n");
	double flOutput = flPID->Update(flSpeed, desiredFL);
//	printf("RL \n");
	double rlOutput = rlPID->Update(rlSpeed, desiredRL);
//	printf("FR \n");
	double frOutput = frPID->Update(frSpeed, desiredFR);
//	printf("RR \n");
	double rrOutput = rrPID->Update(rrSpeed, desiredRR);

	printf("Front Left Output %f\n", flSpeed + flOutput);
	printf("Rear Left Output %f\n", rlSpeed + rlOutput);
	printf("Front Right Output %f\n", frSpeed + frOutput);
	printf("Rear Right Output %f\n", rrSpeed + rrOutput);

	robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, flSpeed + flOutput);
	robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rlSpeed + rlOutput);
	robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frSpeed + frOutput);
	robot->SetWheelSpeed(RobotModel::kRearRightWheel, rrSpeed + rrOutput);
	//robot->SetWheelSpeed(RobotModel::kRearRightWheel, 0.0);

	oldFL = currFL;
	oldRL = currRL;
	oldFR = currFR;
	oldRR = currRR;
}


// original controllmedcanumdrive w/o gyro
void DriveController::ControlledMecanumDrive(double joyX, double joyY, double joyR, double currTimeSec, double deltaTimeSec) {
	double x = joyX;
	double y = joyY;
	double r = joyR;
	double desiredFL = x+y+r;
	double desiredRL = -x+y+r;
	double desiredFR = -x+y-r;
	double desiredRR = x+y-r;

	double maxAbsMagnitude = 0.0;

	if (maxAbsMagnitude < Abs(desiredFL)) {
		maxAbsMagnitude = Abs(desiredFL);
	}
	if (maxAbsMagnitude < Abs(desiredRL)) {
		maxAbsMagnitude = Abs(desiredRL);
	}
	if (maxAbsMagnitude < Abs(desiredFR)) {
		maxAbsMagnitude = Abs(desiredFR);
	}
	if (maxAbsMagnitude < Abs(desiredRR)) {
		maxAbsMagnitude = Abs(desiredRR);
	}

	if (maxAbsMagnitude < 1.0) {
		maxAbsMagnitude = 1.0;
	}

	desiredFL = desiredFL / maxAbsMagnitude;
	desiredRL = desiredRL / maxAbsMagnitude;
	desiredFR = desiredFR / maxAbsMagnitude;
	desiredRR = desiredRR / maxAbsMagnitude;

//	printf("Desired fl %f\n", desiredFL);
//	printf("Desired rl %f\n", desiredRL);
//	printf("Desired fr %f\n", desiredFR);
//	printf("Desired rr %f\n", desiredRR);

	currFL = robot->GetFrontLeftEncoderVal();
	currRL = robot->GetRearLeftEncoderVal();
	currFR = robot->GetFrontRightEncoderVal();
	currRR = robot->GetRearRightEncoderVal();

	flSpeed = (currFL - oldFL) / deltaTimeSec;
	rlSpeed = (currRL - oldRL) / deltaTimeSec;
	frSpeed = (currFR - oldFR) / deltaTimeSec;
	rrSpeed = (currRR - oldRR) / deltaTimeSec;

	flSpeed = EncoderToMotorSpeed(flSpeed);
	rlSpeed = EncoderToMotorSpeed(rlSpeed);
	frSpeed = EncoderToMotorSpeed(frSpeed);
	rrSpeed = EncoderToMotorSpeed(rrSpeed);

	//printf("FL Speed %f\n", flSpeed);
	//printf("RL Speed %f\n", rlSpeed);
	//printf("FR Speed %f\n", frSpeed);
	//printf("RR Speed %f\n", rrSpeed);

	//printf("FL \n");
	double flOutput = flPID->Update(flSpeed, desiredFL);
	//printf("RL \n");
	double rlOutput = rlPID->Update(rlSpeed, desiredRL);
	//printf("FR \n");
	double frOutput = frPID->Update(frSpeed, desiredFR);
	//printf("RR \n");
	double rrOutput = rrPID->Update(rrSpeed, desiredRR);


	//printf("Front Left Output %f\n", flSpeed + flOutput);
	//printf("Rear Left Output %f\n", rlSpeed + rlOutput);
	//printf("Front Right Output %f\n", frSpeed + frOutput);
	//printf("Rear Right Output %f\n", rrSpeed + rrOutput);
	robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, flSpeed + flOutput);
	robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rlSpeed + rlOutput);
	robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frSpeed + frOutput);
	robot->SetWheelSpeed(RobotModel::kRearRightWheel, rrSpeed + rrOutput);

	oldFL = currFL;
	oldRL = currRL;
	oldFR = currFR;
	oldRR = currRR;
}

void DriveController::MecanumDrive(double myX, double myY, double myRotate) {
	//DO_PERIODIC(20, printf("joy R %f\n", myRotate));
	double x = myX;
	double y = myY;
	double rotate = myRotate;
	double frontLeft = x + y + rotate;
	double rearLeft = -x + y + rotate;
	double frontRight = -x + y - rotate;
	double rearRight = x + y - rotate;
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

	if (maxAbsMagnitude < 1.0) {
		maxAbsMagnitude = 1.0;
	}

	frontLeft = frontLeft / maxAbsMagnitude;
	rearLeft = rearLeft / maxAbsMagnitude;
	frontRight = frontRight / maxAbsMagnitude;
	rearRight = rearRight / maxAbsMagnitude;

	robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, frontLeft);
	robot->SetWheelSpeed(RobotModel::kRearLeftWheel, rearLeft);
	robot->SetWheelSpeed(RobotModel::kFrontRightWheel, frontRight);
	robot->SetWheelSpeed(RobotModel::kRearRightWheel, rearRight);
/*
	DO_PERIODIC(20, printf("Front Left %f\n", frontLeft));
	DO_PERIODIC(20, printf("Rear Left %f\n", rearLeft));
	DO_PERIODIC(20, printf("Front Right %f\n", frontRight));
	DO_PERIODIC(20, printf("Rear right %f\n", rearRight));
	*/
}

void DriveController::MecanumDrive(double myX, double myY, double myRotate, double myAngle) {
	double x = myX;
	double y = myY;
	double r = myRotate;
	double a = myAngle;
	double newX, newY, newR;

	double cosA = cos(a * (3.14159 / 180.0));
	double sinA = sin(a * (3.14159 / 180.0));
	newX = x * cosA - y * sinA;
	newY = x * sinA + y * cosA;
	newR = r;

	MecanumDrive(newX, newY, newR);
}

void DriveController::DriveLength() {
}

void DriveController::DriveWidth() {
}

int DriveController::DriveDirection() {
	if (humanControl->ReverseDriveDesired())
		return -1;
	else
		return 1;
}

double DriveController::Abs(double x) {
	if (x > 0.0) {
		return x;
	} else {
		return -x;
	}
}

PIDConfig* DriveController::CreateFLPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = flPFac;
	pidConfig->iFac = flIFac;
	pidConfig->dFac = flDFac;
	pidConfig->desiredAccuracy = flDesiredAccuracy;
	pidConfig->maxAbsOutput = flMaxAbsOutput;
	pidConfig->maxAbsDiffError = flMaxAbsDiffError;
	pidConfig->maxAbsError = flMaxAbsError;
	pidConfig->maxAbsITerm = flMaxAbsITerm;
	pidConfig->minAbsError = 0.1;
	return pidConfig;
}

PIDConfig* DriveController::CreateRLPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = rlPFac;
	pidConfig->iFac = rlIFac;
	pidConfig->dFac = rlDFac;
	pidConfig->desiredAccuracy = rlDesiredAccuracy;
	pidConfig->maxAbsOutput = rlMaxAbsOutput;
	pidConfig->maxAbsDiffError = rlMaxAbsDiffError;
	pidConfig->maxAbsError = rlMaxAbsError;
	pidConfig->maxAbsITerm = rlMaxAbsITerm;
	pidConfig->minAbsError = 0.1;
	return pidConfig;
}

PIDConfig* DriveController::CreateFRPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = frPFac;
	pidConfig->iFac = frIFac;
	pidConfig->dFac = frDFac;
	pidConfig->desiredAccuracy = frDesiredAccuracy;
	pidConfig->maxAbsOutput = frMaxAbsOutput;
	pidConfig->maxAbsDiffError = frMaxAbsDiffError;
	pidConfig->maxAbsError = frMaxAbsError;
	pidConfig->maxAbsITerm = frMaxAbsITerm;
	pidConfig->minAbsError = 0.1;
	return pidConfig;
}

PIDConfig* DriveController::CreateRRPIDConfig() {
	PIDConfig* pidConfig = new PIDConfig();
	pidConfig->pFac = rrPFac;
	pidConfig->iFac = rrIFac;
	pidConfig->dFac = rrDFac;
	pidConfig->desiredAccuracy = rrDesiredAccuracy;
	pidConfig->maxAbsOutput = rrMaxAbsOutput;
	pidConfig->maxAbsDiffError = rrMaxAbsDiffError;
	pidConfig->maxAbsError = rrMaxAbsError;
	pidConfig->maxAbsITerm = rrMaxAbsITerm;
	pidConfig->minAbsError = 0.1;
	return pidConfig;
}

double DriveController::EncoderToMotorSpeed(double encoderSpeed) {
	return encoderSpeed / 7.9;
}

double DriveController::NavxToMotorSpeed(double navxSpeed) {
	return navxSpeed / 200.0;
}
