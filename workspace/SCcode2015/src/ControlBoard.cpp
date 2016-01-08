#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotPorts2015.h"
#include <iostream>
#include <cstring>
#include <stdlib.h>

ControlBoard::ControlBoard() {
	mLeftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	mRightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	mOperatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);
	mIntakeJoy = new Joystick(INTAKE_JOY_USB_PORT);

	mReverseDriveDesired = false;
	leftDesired = false;/**<corresponds to button for incrementally strafing left */
	rightDesired = false;/**<corresponds to button for incrementally strafing right */
	rampDesired = false;
	intakeDesired = false;
	outtakeToPlatformDesired = false;
	outtakeToStepDesired = false;
	moveElevatorUpStartDesired = false;
	moveElevatorUpEndDesired = false;
	moveElevatorUpDesired = false;
	moveElevatorDownStartDesired = false;
	moveElevatorDownEndDesired = false;
	moveElevatorDownDesired = false;
	binsUpDesired = false;
	binsOutDesired = false;
	fieldCentricDesired = true;
	intakeSequenceDesired = false;

	fieldRobotButton = new ButtonReader(mRightJoy, FIELD_ROBOT_BUTTON_PORT);
	joyRampButton = new ButtonReader(mLeftJoy, JOY_RAMP_BUTTON_PORT);
	leftButton = new ButtonReader(mIntakeJoy, LEFT_BUTTON_PORT);
	rightButton = new ButtonReader(mIntakeJoy, RIGHT_BUTTON_PORT);
	rampButton = new ButtonReader(mOperatorJoy, RAMP_BUTTON_PORT);
	intakeButton = new ButtonReader(mOperatorJoy, INTAKE_BUTTON_PORT);
	outtakeToPlatformButton = new ButtonReader(mOperatorJoy, OUTTAKE_TO_PLATFORM_BUTTON_PORT);
	outtakeToStepButton = new ButtonReader(mOperatorJoy, OUTTAKE_TO_STEP_BUTTON_PORT);
	moveElevatorUpButton = new ButtonReader(mOperatorJoy, MOVE_ELEVATOR_UP_BUTTON_PORT);
	moveElevatorDownButton = new ButtonReader(mOperatorJoy, MOVE_ELEVATOR_DOWN_BUTTON_PORT);
	binsOutButton = new ButtonReader(mOperatorJoy, BINS_OUT_BUTTON_PORT);
	mDriveDirectionButton = new ButtonReader(mOperatorJoy, REVERSE_DRIVE_BUTTON_PORT);
	secondIntakeButton = new ButtonReader(mOperatorJoy, SECOND_INTAKE_BUTTON_PORT);
	//ultrasonicButton = new ButtonReader(mOperatorJoy, ULTRASONIC_BUTTON_PORT);
	//intakeSequenceButton = new ButtonReader(mOperatorJoy, INTAKE_SEQUENCE_BUTTON_PORT);

	mLeftJoyX = 0.0;
	mLeftJoyY = 0.0;
	mRightJoyX = 0.0;
	mRightJoyY = 0.0;
	mIntakeJoyX = 0.0;
	mIntakeJoyY = 0.0;
	elevatorUpSpeedAdjustVal = 0.7;
	elevatorDownSpeedAdjustVal = 0.35;

	isRecording = true;/**<If true, every time the robot is enabled it will record a file of
	every action taken*/
	readControlsCounter = 0;
};

void ControlBoard::ReadControls() {
	ReadAllButtons();
	if (fieldRobotButton->IsDown()) {
		SetFieldCentricDesired(true);
	} else {
		SetFieldCentricDesired(false);
	}
/*
	if (intakeSequenceButton->WasJustPressed()) {
		intakeSequenceDesired = true;
	} else {
		intakeSequenceDesired = false;
	}
	*/
	/*
	if (ultrasonicButton->WasJustPressed()) {
		ultrasonicDesired = true;
	} else {
		ultrasonicDesired = false;
	}
	*/
	if (mDriveDirectionButton->GetState()) {
		SetReverseDriveDesired(true);
	} else {
		SetReverseDriveDesired(false);
	}
	leftDesired = leftButton->WasJustPressed();	// we should compact all of the if statements to be like this (later)
	rightDesired = rightButton->WasJustPressed();

	if (rampButton->WasJustPressed() || joyRampButton->WasJustPressed()) {
		SetRampDesired(true);
	} else if (rampButton->WasJustReleased() || joyRampButton->WasJustReleased()) {
		SetRampDesired(false);
	}

	if (intakeButton->WasJustPressed() || secondIntakeButton->WasJustPressed()) {
		SetIntakeDesired(true);
	} else {
		SetIntakeDesired(false);
	}
	if (outtakeToPlatformButton->WasJustPressed()) {
		SetOuttakeToPlatformDesired(true);
	} else {
		SetOuttakeToPlatformDesired(false);
	}
	if (outtakeToStepButton->WasJustPressed()) {
		SetOuttakeToStepDesired(true);
	} else {
		SetOuttakeToStepDesired(false);
	}
	if(moveElevatorUpButton->WasJustPressed()) {
		moveElevatorUpStartDesired = true;
	} else {
		moveElevatorUpStartDesired = false;
	}
	if (moveElevatorUpButton->WasJustReleased()) {
		moveElevatorUpEndDesired = true;
	} else {
		moveElevatorUpEndDesired = false;
	}
	if (moveElevatorUpButton->IsDown()) {
		moveElevatorUpDesired = true;
	} else {
		moveElevatorUpDesired = false;
	}
	if (moveElevatorDownButton->WasJustPressed()) {
		moveElevatorDownStartDesired = true;
	} else {
		moveElevatorDownStartDesired = false;
	}
	if (moveElevatorDownButton->WasJustReleased()) {
		moveElevatorDownEndDesired = true;
	} else {
		moveElevatorDownEndDesired = false;
	}
	if (moveElevatorDownButton->IsDown()) {
		moveElevatorDownDesired = true;
	} else {
		moveElevatorDownDesired = false;
	}

	if (binsOutButton->WasJustPressed()) {
		binsOutDesired = true;
	} else {
		binsOutDesired = false;
	}

	elevatorUpSpeedAdjustVal = mOperatorJoy->GetX(); // left dial
	elevatorDownSpeedAdjustVal = mOperatorJoy->GetY(); // right dial
	mLeftJoyX = mLeftJoy->GetX();
	mLeftJoyY = -mLeftJoy->GetY();
	mRightJoyX = mRightJoy->GetX();
	mRightJoyY = -mRightJoy->GetY();
	mIntakeJoyX = mIntakeJoy->GetX();
	mIntakeJoyY = -mIntakeJoy->GetY();
	if (isRecording) {
		outFile << readControlsCounter << ",\"Timeslice\",\"0.0\"" << "\r\n";
		outFile << readControlsCounter << ",\"mReverseDriveDesired\",\"" << mReverseDriveDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"fieldCentricDesired\",\"" << fieldCentricDesired << "\"\r\n";
//		if (!xValDesired && !yValDesired) {
//			outFile << readControlsCounter << ",\"mLeftJoyX\",\"" << mLeftJoyX << "\"\r\n";
//			outFile << readControlsCounter << ",\"mLeftJoyY\",\"" << mLeftJoyY << "\"\r\n";
//			outFile << readControlsCounter << ",\"mRightJoyX\",\"" << mRightJoyX << "\"\r\n";
//		} else if (!xValDesired && yValDesired) {
//			outFile << readControlsCounter << ",\"mLeftJoyY\",\"" << mLeftJoyY << "\"\r\n";
//		} else if (xValDesired && !yValDesired) {
//			outFile << readControlsCounter << ",\"mLeftJoyX\",\"" <<  mLeftJoyX << "\"\r\n";
//		}
		outFile << readControlsCounter << ",\"mLeftJoyX\",\"" << mLeftJoyX << "\"\r\n";
		outFile << readControlsCounter << ",\"mLeftJoyY\",\"" << mLeftJoyY << "\"\r\n";
		outFile << readControlsCounter << ",\"mRightJoyX\",\"" << mRightJoyX << "\"\r\n";

		outFile << readControlsCounter << ",\"mIntakeJoyX\",\"" << mIntakeJoyX << "\"\r\n";
		outFile << readControlsCounter << ",\"mIntakeJoyY\",\"" << mIntakeJoyY << "\"\r\n";

		outFile << readControlsCounter << ",\"moveElevatorUpSpeed\",\"" << elevatorUpSpeedAdjustVal << "\"\r\n";
		outFile << readControlsCounter << ",\"moveElevatorDownSpeed\",\"" << elevatorDownSpeedAdjustVal << "\"\r\n";

		outFile << readControlsCounter << ",\"moveElevatorUpStartDesired\",\"" << moveElevatorUpStartDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"moveElevatorUpEndDesired\",\"" << moveElevatorUpEndDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"moveElevatorUpDesired\",\"" << moveElevatorUpDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"moveElevatorDownStartDesired\",\"" << moveElevatorDownStartDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"moveElevatorDownEndDesired\",\"" << moveElevatorDownEndDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"moveElevatorDownDesired\",\"" << moveElevatorDownDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"outtakeToPlatformDesired\",\"" << outtakeToPlatformDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"rampDesired\",\"" << rampDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"intakeDesired\",\"" << intakeDesired << "\"\r\n";
		outFile << readControlsCounter << ",\"Timeslice\",\"1.0\"" << "\r\n";
		readControlsCounter++;
	}
	//printf("after writing moveElevatorUpStart %d\n", moveElevatorUpStartDesired);
}

void ControlBoard::OpenFile() {
	if (isRecording) {
		outFile.open("/home/lvuser/playbackWRITE.txt", std::ofstream::out);
	}
}

void ControlBoard::CloseFile() {
	if (isRecording) {
		outFile.close();
	}
}

double ControlBoard::GetJoystickValue(Joysticks j, Axes a) /*const*/ {
	switch (j) {
		case (kLeftJoy):
			if (a == kX) {
				return mLeftJoyX;
			}
			if (a == kY) {
				return mLeftJoyY;
			}
			break;
		case (kRightJoy):
			if (a == kX) {
				return mRightJoyX;
			}
			if (a == kY) {
				return mRightJoyY;
			}
			break;
		case (kIntakeJoy):
			if (a == kX) {
				return mIntakeJoyX;
			}
			if (a == kY) {
				return mIntakeJoyY;
			}
		break;
		default:
			return 0.0;
			break;
	}
	return 0.0;
}

void ControlBoard::SetJoystickValue(Joysticks j, Axes a, double value) {
	switch (j) {
		case (kLeftJoy):
			if (a == kX) {
				mLeftJoyX = value;
			}
			if (a == kY) {
				mLeftJoyY = value;
			}
		break;
		case (kRightJoy):
			if (a == kX) {
				mRightJoyX = value;
			}
			if (a == kY) {
				mRightJoyY = value;
			}
		break;
		case (kIntakeJoy):
			if (a == kX) {
				mIntakeJoyX = value;
			}
			if (a == kY) {
				mIntakeJoyY = value;
			}
		break;
	}
}

bool ControlBoard::IntakeSequenceDesired() {
	//return intakeSequenceDesired;
	return false;
}

bool ControlBoard::ReverseDriveDesired() {
	return mReverseDriveDesired;
}
/*
bool ControlBoard::UltrasonicDesired() {
	return ultrasonicDesired;
}
*/
bool ControlBoard::LeftDesired() {
	return leftDesired;
}

bool ControlBoard::RightDesired() {
	return rightDesired;
}

bool ControlBoard::RampDesired() {
	return rampDesired;
}

bool ControlBoard::IntakeDesired() {
//	printf("Control Board's intake %d\n", intakeDesired);
	return intakeDesired;
}

bool ControlBoard::OuttakeToPlatformDesired() {
	return outtakeToPlatformDesired;
}

bool ControlBoard::OuttakeToStepDesired() {
	return outtakeToStepDesired;
}

bool ControlBoard::MoveElevatorUpStartDesired() {
	//SmartDashboard::PutBoolean("Giving value break start", moveElevatorUpStartDesired);
	//printf("Control Board's moveELevatorUpStart %d\n", moveElevatorUpStartDesired);
	return moveElevatorUpStartDesired;
}

bool ControlBoard::MoveElevatorUpEndDesired() {
	//SmartDashboard::PutBoolean("Giving value break end", moveElevatorUpEndDesired);
	return moveElevatorUpEndDesired;
}

bool ControlBoard::MoveElevatorUpDesired() {
	return moveElevatorUpDesired;
}

bool ControlBoard::MoveElevatorDownStartDesired() {
	return moveElevatorDownStartDesired;
}

bool ControlBoard::MoveElevatorDownEndDesired() {
	return moveElevatorDownEndDesired;
}

bool ControlBoard::MoveElevatorDownDesired() {
	return moveElevatorDownDesired;
}

bool ControlBoard::BinsOutDesired() {
	return binsOutDesired;
}

bool ControlBoard::FieldCentricDesired() {
	return fieldCentricDesired;
}

bool ControlBoard::RobotCentricDesired() {
	return !fieldCentricDesired;
}

void ControlBoard::SetRampDesired(bool rampDes) {
	rampDesired = rampDes;
}

void ControlBoard::SetReverseDriveDesired(bool desired) {
	mReverseDriveDesired = desired;
}

void ControlBoard::SetIntakeDesired(bool intakeDes) {
//	printf("Setting Intake Desired %d\n", intakeDes);
	intakeDesired = intakeDes;
}

void ControlBoard::SetOuttakeToPlatformDesired(bool outtakeDes) {
	outtakeToPlatformDesired = outtakeDes;
}

void ControlBoard::SetOuttakeToStepDesired(bool outtakeDes) {
	outtakeToStepDesired = outtakeDes;
}

void ControlBoard::SetMoveElevatorUpStartDesired(bool startDes) {
	//printf("Setting Move Elevator Up Start %d\n", startDes);
	moveElevatorUpStartDesired = startDes;
}
void ControlBoard::SetMoveElevatorUpEndDesired(bool endDes) {
	//printf("Setting Move Elevator Up End %d\n", endDes);
	moveElevatorUpEndDesired = endDes;
}
void ControlBoard::SetMoveElevatorUpDesired(bool upDes) {
	//printf("Setting Move Elevator Up %d\n", upDes);
	moveElevatorUpDesired = upDes;
}
void ControlBoard::SetMoveElevatorDownStartDesired(bool startDes) {
	moveElevatorDownStartDesired = startDes;
}
void ControlBoard::SetMoveElevatorDownEndDesired(bool endDes) {
	moveElevatorDownEndDesired = endDes;
}
void ControlBoard::SetMoveElevatorDownDesired(bool downDes) {
	moveElevatorDownDesired = downDes;
}

void ControlBoard::SetBinsOutDesired(bool outDes) {
	binsOutDesired = outDes;
}

void ControlBoard::SetBinsUpDesired(bool upDes) {
	binsUpDesired = upDes;
}

void ControlBoard::SetFieldCentricDesired(bool des) {
	fieldCentricDesired = des;
}

double ControlBoard::GetElevatorUpSpeedAdjuster() {
	return elevatorUpSpeedAdjustVal;
}

double ControlBoard::GetElevatorDownSpeedAdjuster() {
	return elevatorDownSpeedAdjustVal;
}

void ControlBoard::SetElevatorUpSpeedAdjuster(double upSpeed) {
	elevatorUpSpeedAdjustVal = upSpeed;
}

void ControlBoard::SetElevatorDownSpeedAdjuster(double downSpeed) {
	elevatorDownSpeedAdjustVal = downSpeed;
}

/*
void ControlBoard::SetUltrasonicDesired(bool des) {
	ultrasonicDesired = des;
}
*/
void ControlBoard::ReadAllButtons() {
	fieldRobotButton->ReadValue();
	joyRampButton->ReadValue();
	leftButton->ReadValue();
	rightButton->ReadValue();
	rampButton->ReadValue();
	intakeButton->ReadValue();
	outtakeToPlatformButton->ReadValue();
	outtakeToStepButton->ReadValue();
	moveElevatorUpButton->ReadValue();
	moveElevatorDownButton->ReadValue();
	//binsUpButton->ReadValue();
	binsOutButton->ReadValue();
	mDriveDirectionButton->ReadValue();
	secondIntakeButton->ReadValue();
	//intakeSequenceButton->ReadValue();
	//ultrasonicButton->ReadValue();
}
