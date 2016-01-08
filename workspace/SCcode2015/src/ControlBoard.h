#ifndef CONTROLBOARD_H
#define CONTROLBOARD_H

#include "WPILib.h"
#include "RemoteControl.h"
#include "ButtonReader.h"
#include <iostream>
#include <fstream> // file streams defined in this library
#include <cstring>
#include <stdlib.h>

class ControlBoard : public RemoteController {
public:
	ControlBoard();
	~ControlBoard() {};

	virtual void ReadControls();
	void OpenFile();
	void CloseFile();

	double GetJoystickValue(Joysticks j, Axes a) /*const*/;
	void SetJoystickValue(Joysticks j, Axes a, double value); // used to be in acb
	bool ReverseDriveDesired();

	/**
	 * Puts robot on an absolute orientation so that it moves relative to the
	 * field, not the direction it's facing.
	 */
	bool FieldCentricDesired();
	/**
	 * Puts robot on an orientation relative to itself, ie turning left means
	 * the robot will turn in the direction that is left from its point of view.
	 */
	bool RobotCentricDesired();
	//bool UltrasonicDesired();
	/**
	 * Corresponds to buttons on driver station; for as long as the left button is
	 * held down, the robot will strafe to the left.
	 */
	bool LeftDesired();
	/**
	 * Right counterpart to LeftDesired().
	 */
	bool RightDesired();
	/**
	 * Used for telling controller if the ramp button is triggered
	 */
	bool RampDesired();
	/**
	 * Tells controller if button to trigger opening and closing of intake
	 * arms is pressed
	 */
	bool IntakeDesired();
	bool OuttakeToPlatformDesired();
	bool OuttakeToStepDesired();
	bool MoveElevatorUpStartDesired();
	bool MoveElevatorUpEndDesired();
	bool MoveElevatorUpDesired();
	bool MoveElevatorDownStartDesired();
	bool MoveElevatorDownEndDesired();
	bool MoveElevatorDownDesired();

	bool IntakeSequenceDesired();
	bool BinsOutDesired();

	void SetReverseDriveDesired(bool desired);
	void SetOuttakeToPlatformDesired(bool outtakeToPlatformDesired);
	void SetOuttakeToStepDesired(bool outtakeToStepDesired);
	void SetRampDesired(bool rampDesired);
	void SetIntakeDesired(bool intakeDesired);
	void SetMoveElevatorUpStartDesired(bool startDes);
	void SetMoveElevatorUpEndDesired(bool endDes);
	void SetMoveElevatorUpDesired(bool upDes);
	void SetMoveElevatorDownStartDesired(bool startDes);
	void SetMoveElevatorDownEndDesired(bool endDes);
	void SetMoveElevatorDownDesired(bool downDes);

	void SetBinsOutDesired(bool outDes);
	void SetBinsUpDesired(bool upDes);

	void SetFieldCentricDesired(bool des);
	//void SetUltrasonicDesired(bool des);
	double GetElevatorUpSpeedAdjuster();
	double GetElevatorDownSpeedAdjuster();
	void SetElevatorUpSpeedAdjuster(double upSpeed);
	void SetElevatorDownSpeedAdjuster(double downSpeed);

protected:
	bool fieldCentricDesired, leftDesired, rightDesired, rampDesired, intakeDesired,
			outtakeToPlatformDesired, outtakeToStepDesired,
			moveElevatorUpStartDesired, moveElevatorUpEndDesired,moveElevatorUpDesired,
			moveElevatorDownStartDesired, moveElevatorDownEndDesired, moveElevatorDownDesired,
			binsOutDesired, binsUpDesired, intakeSequenceDesired;// ultrasonicDesired;
	bool mReverseDriveDesired;
	double mLeftJoyX, mLeftJoyY, mRightJoyX, mRightJoyY, mIntakeJoyX, mIntakeJoyY,
		   elevatorUpSpeedAdjustVal, elevatorDownSpeedAdjustVal;

private:
	Joystick *mLeftJoy, *mRightJoy, *mOperatorJoy, *mIntakeJoy;

	ButtonReader *leftButton, *rightButton, *rampButton, *intakeButton, *outtakeToPlatformButton,
				 *outtakeToStepButton, *moveElevatorUpButton,
				 *moveElevatorDownButton, *mDriveDirectionButton, *fieldRobotButton,
				 *binsOutButton, *secondIntakeButton, *joyRampButton;// *ultrasonicButton;

	bool isRecording;
	int readControlsCounter;

	void ReadAllButtons();

	std::ofstream outFile;
};

#endif
