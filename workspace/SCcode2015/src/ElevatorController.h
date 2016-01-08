#ifndef ELEVATORCONTROLLER_H
#define ELEVATORCONTROLLER_H

#include "Debugging.h"
#include "RemoteControl.h"
#include "RobotModel.h"
#include "PIDControlLoop.h"

class ElevatorController {

  public:
	ElevatorController();
	~ElevatorController() {}
	ElevatorController(RobotModel* myRobot, RemoteController* myHumanControl);
	void Reset();
	void RefreshIni();
	void Update(double currTimeSec, double deltaTimeSec);

	bool AutoOuttakingDone();
	void SetAutoOuttaking();
	bool AutoIntakingDone();
	void SetAutoIntaking();
	bool AutoRampOutDone();
	void SetAutoRampOutDesired(bool outDesired);
	void SetAutoClamp(bool clamp);
	bool AutoClampDone();
	void SetRollOutDesired(bool desired);
	void SetRollOutDone(bool done);

  private:
	void ResetAllInstanceFields();
	void InitCarriageMovement(PIDConfig *posPIDConfig, PIDConfig *velPIDConfig,
							  int desiredEncoderVal, double desiredCarriageSpeed);
	double UpdateCarriageMoveUp();
	double UpdateCarriageMoveDown();
	bool CarriageMoveDone();
	void InitVelPIDOnly(PIDConfig *velPIDConfig, double desiredCarriageSpeed);
	double UpdateVelPIDOnly();
	bool IntakeDone(double currTimeSec);
	bool EncoderZeroingDone();
	bool RampOutDesired();
	bool OuttakeOutDesired();

	double TransformUpSpeed(double original);
	double TransformDownSpeed(double original);

	enum ElevatorState {
		kInit, kIdle, kZeroEncoder,
		kTest, kInitIntaking, kRaiseCarriageToIntake,
		kIntake, kLowerCarriageToBottom, kRaiseCarriageToOuttakeToPlatform,
		kRaiseCarriageToOuttakeToStep, kOuttake, kIntakeSequence
	};

	RobotModel* robot;
	RemoteController* humanControl;

	uint32_t m_stateVal;
	uint32_t nextState;
	uint32_t stateAfterZeroingEncoder;

	double lastEncoderVal;
	double currEncoderVal;
	double deltaEncoderVal;
	double carriageSpeed;
	double lastVelOutput;
	double inOrOuttakeStartTime;
	double rollerSpeed;
	double moveElevatorUpSpeed;
	double moveElevatorDownSpeed;
	double intakeStartTime;
	double elevatorSpeedAdjuster;

	double intakeX, intakeY;

	bool encoderZeroed;
	bool encoderZeroingStarted;
	bool rampOut;
	bool outtakeOut;
	bool carriageMoveStarted;
	bool intakeStarted;
	bool outtakeStarted;
	bool testOuttake;
	bool intakeTimerStarted;

	bool autoRampOutDesired,autoRampChangeDesired,autoRampDone,autoIntakeDesired,
	autoIntakingDone,autoOuttakeDesired,autoOuttakingDone;

	bool rollOutDesired;
	double rollOutVal;
	bool rollOutDone;

	bool autoClampDesired;
	bool autoClampVal;
	bool autoClampDone;

	PIDConfig* defaultPosPIDConfig;
	PIDConfig* defaultVelPIDConfig;

	PIDControlLoop* posPID;
	PIDControlLoop* velPID;
	Timer* timer;
	bool intakeSequenceDesired, movedElevatorUpInIntakeSequence, movedRollerArmsOutInIntakeSequence;
	int lowestEncoderValToIntake;
	int endIntakeSequenceElevatorEncoderVal;
};

#endif
