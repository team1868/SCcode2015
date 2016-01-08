#include "ElevatorController.h"
#include <math.h>

ElevatorController::ElevatorController(RobotModel* myRobot, RemoteController* myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;

	ResetAllInstanceFields();

	defaultPosPIDConfig = new PIDConfig();
	defaultVelPIDConfig = new PIDConfig();

	posPID = new PIDControlLoop(defaultPosPIDConfig);
	velPID = new PIDControlLoop(defaultVelPIDConfig);
	timer = new Timer();
}

void ElevatorController::Reset() {
	RefreshIni();
	ResetAllInstanceFields();

	robot->SetElevatorSpeed(0.0);
	robot->SetRollerSpeed(0.0);
}

void ElevatorController::RefreshIni() {
	//Update all ini values and store them in the member variables
	defaultPosPIDConfig->pFac =  robot->pini->getf("ELEVATORCONTROLLER", "defPosPFac", 0.1);
	defaultPosPIDConfig->iFac =  robot->pini->getf("ELEVATORCONTROLLER", "defPosIFac", 0.0);
	defaultPosPIDConfig->dFac =  robot->pini->getf("ELEVATORCONTROLLER", "defPosDFac", 0.0);
	defaultPosPIDConfig->desiredAccuracy = robot->pini->getf("ELEVATORCONTROLLER",
															 "defPosDesiredAcuracy", 0.1);
	defaultPosPIDConfig->maxAbsOutput = robot->pini->getf("ELEVATORCONTROLLER",
														  "defPosMaxAbsOutput", 0.5);
	defaultPosPIDConfig->maxAbsDiffError = robot->pini->getf("ELEVATORCONTROLLER",
															 "defPosMaxAbsDiffError", 12.0);
	defaultPosPIDConfig->maxAbsError = robot->pini->getf("ELEVATORCONTROLLER", "defPosMaxAbsError",
														 12.0);
	defaultPosPIDConfig->maxAbsITerm = robot->pini->getf("ELEVATORCONTROLLER", "defPosMaxAbsITerm", 0.1);
	rollerSpeed = robot->pini->getf("ELEVATORCONTROLLER", "rollerSpeed", 0.8);
	moveElevatorUpSpeed = robot->pini->getf("ELEVATORCONTROLLER", "moveElevatorUpSpeed", 0.7);
	moveElevatorDownSpeed = robot->pini->getf("ELEVATORCONTROLLER", "moveElevatorDownSpeed", 0.2);
	defaultVelPIDConfig->pFac = robot->pini->getf("ELEVATORCONTROLLER", "defVelPFac", 0.5);
	defaultVelPIDConfig->iFac = robot->pini->getf("ELEVATORCONTROLLER", "defVelIFac", 0.0);
	defaultVelPIDConfig->dFac = robot->pini->getf("ELEVATORCONTROLLER", "defVelDFac", 0.0);
	defaultVelPIDConfig->desiredAccuracy = robot->pini->getf("ELEVATORCONTROLLER",
															 "defVelDesiredAccuracy", 1.0);
	defaultVelPIDConfig->maxAbsOutput = robot->pini->getf("ELEVATORCONTROLLER", "defVelMaxAbsOutput",
														  0.5);
	defaultVelPIDConfig->maxAbsDiffError = robot->pini->getf("ELEVATORCONTROLLER",
															 "defVelMaxAbsDiffError", 5.0);
	defaultVelPIDConfig->maxAbsError = robot->pini->getf("ELEVATORCONTROLLER", "defVelMaxAbsError", 5.0);
	defaultVelPIDConfig->maxAbsITerm = robot->pini->getf("ELEVATORCONTROLLER", "defVelMaxAbsITerm", 0.1);
	//todo Change to false before competition
	testOuttake = robot->pini->getbool("ELEVATORCONTROLLER", "testOuttake", true);
	lowestEncoderValToIntake = robot->pini->getf("ELEVATORCONTROLLER", "lowestEncoderValToIntake", 20);
	endIntakeSequenceElevatorEncoderVal = robot->pini->getf("ELEVATORCONTROLLER", "endIntakeSequenceElevatorEncoderVal", 4);
}

void ElevatorController::Update(double currTimeSec, double deltaTimeSec) {
	lastEncoderVal = currEncoderVal;
	currEncoderVal = robot->GetElevatorEncoderVal();
	deltaEncoderVal = currEncoderVal - lastEncoderVal;
	carriageSpeed = deltaEncoderVal / deltaTimeSec;


//	SmartDashboard::PutNumber("CurrEncVal", currEncoderVal);
	//SmartDashboard::PutNumber("CarriageSpeed", carriageSpeed);

	switch(m_stateVal) {
	case (kInit):
		//Initialize everything that is not reset in Reset() (called in teleop init)
		//printf("kInit\n");
		robot->MoveRampIn();

		rampOut = robot->GetRampPos();
		outtakeOut = robot->GetOuttakePos();

		robot->SetElevatorSpeed(0.0);
		robot->SetRollerSpeed(0.0);
		timer->Start();
		nextState = kIdle;
		break;

	case (kIdle):
		printf("kIdle\n");
		/*For any piston movements that are not included in the state machine and conditionals for
		 * buttons that allow you to move to other states*/
		nextState = kIdle;
		//SmartDashboard::PutNumber("Roller speed a", rollerSpeed);
	/*
		printf("Ramp Piston %d\n", robot->rampSolenoid->Get());
	//	printf("Intake Piston %d\n", robot->rollerArmSolenoid->Get());
		printf("Outtake Piston %d\n", robot->outtakeSolenoid->Get());
		printf("Break Piston %d\n", robot->breakSolenoid->Get());
		*/



		if(humanControl->RampDesired()) {
			robot->MoveRampOut();
			rampOut = true;
			autoRampDone = true;
		} else {
			robot->MoveRampIn();
			rampOut = false;
			autoRampDone = true;
		}
//		printf("elevator intake %d\n", humanControl->IntakeDesired());
		if (autoClampDesired) {
			printf("In autoClamp in Elevator Controller \n");
			if (autoClampVal) {
				robot->MoveRollerArmsIn();
			} else {
				robot->MoveRollerArmsOut();
			}
			autoClampDesired = false;
			autoClampDone = true;
		}


		if (humanControl->IntakeDesired() /*|| autoIntakeDesired*/) {
//			printf("Intake Position ");

				if (robot->GetRollerArmPos() == 1) {
	//				printf("moving out \n");
				//	SmartDashboard::PutBoolean("State", true);
					robot->MoveRollerArmsOut();
				} else {
	//				printf("moving in \n");
					//SmartDashboard::PutBoolean("State", false);
					robot->MoveRollerArmsIn();
				}
				autoIntakingDone = true;
				//nextState = kLowerCarriageToBottom;

		}

		if (humanControl->OuttakeToPlatformDesired() /*|| autoOuttakeDesired*/) {
			if (testOuttake) {
				if(OuttakeOutDesired()) {
					robot->MoveOuttakeOut();
					outtakeOut = true;
				} else {
					robot->MoveOuttakeIn();
					outtakeOut = false;
				}
			} else {
				nextState = kRaiseCarriageToOuttakeToPlatform;
			}
		}

		if (humanControl->OuttakeToStepDesired()) {
			//nextState = kRaiseCarriageToOuttakeToStep;
			nextState = kIdle;
		}
		moveElevatorUpSpeed = humanControl->GetElevatorUpSpeedAdjuster();
		moveElevatorDownSpeed = humanControl->GetElevatorDownSpeedAdjuster();
		moveElevatorUpSpeed = TransformUpSpeed(moveElevatorUpSpeed);
		moveElevatorDownSpeed = TransformUpSpeed(moveElevatorDownSpeed);
		//printf("Elevator up knob %f\n", humanControl->GetElevatorUpSpeedAdjuster());
		//printf("Elevator down knob: %f\n", humanControl->GetElevatorDownSpeedAdjuster());

		if (moveElevatorDownSpeed > 0.5) {
			moveElevatorDownSpeed = 0.5;
		}

		if (!robot->GetElevatorLowerLimitSwitchVal()) {
			robot->ResetElevatorEncoder();// idk if we should go to kResetEncoder
			moveElevatorDownSpeed = 0.0;
			//printf("reset elevator encoder\n");
		}

		//moveElevatorUpSpeed = 0.7;
		//moveElevatorDownSpeed = 0.35;
//		printf("transformed up %f\n", TransformUpSpeed(humanControl->GetElevatorUpSpeedAdjuster()));
//		printf("transformed down %f\n", TransformDownSpeed(humanControl->GetElevatorDownSpeedAdjuster()));
		if (humanControl->MoveElevatorUpStartDesired()) {
			printf("in Move Elevator Up Start \n");
		} else if (humanControl->MoveElevatorUpEndDesired()) {
			robot->SetElevatorSpeed(0.0);
			robot->EngageBrake();
		} else if (humanControl->MoveElevatorDownStartDesired()) {
			//robot->DisengageBreak();
		} else if (humanControl->MoveElevatorDownEndDesired()) {
			robot->SetElevatorSpeed(0.0);
			robot->EngageBrake();
		} else if (humanControl->MoveElevatorUpDesired()) {
			robot->DisengageBrake();
			robot->SetElevatorSpeed(moveElevatorUpSpeed);
			printf("movingUp elevatorEncoderVal: %f\n", robot->GetElevatorEncoderVal());
		} else if (humanControl->MoveElevatorDownDesired()) {
			robot->DisengageBrake();
			robot->SetElevatorSpeed(-moveElevatorDownSpeed);

			printf("movingDown elevatorEncoderVal: %f\n", robot->GetElevatorEncoderVal());
		} else {
			robot->SetElevatorSpeed(0.0);
		}

		// below is for mini intake joystick
		intakeY = humanControl->GetJoystickValue(
			RemoteController::kIntakeJoy, RemoteController::kY);
		// printf("intakeY: %f\n", intakeY);
		intakeX = humanControl->GetJoystickValue(
			RemoteController::kIntakeJoy, RemoteController::kX);

		if (abs(intakeY) < 0.0) {
		// printf("intakeX: %f\n", intakeX);
			intakeY = 0.0;
		}
		if (abs(intakeX) < 0.0) {
			intakeX = 0.0;
		}


		if (humanControl->IntakeSequenceDesired()) {
			nextState = kIntakeSequence;
		}

		//printf("intakeX %f /n", intakeX);
		//printf("intakeY %f /n", intakeY);

		if (abs(intakeY) >= abs(intakeX)) {
			robot->SetRollerSpeed(-intakeY); // strict intake/outtake motion
		} else {
			robot->SetLeftRollerSpeed(intakeX); // strict left / right motion
			robot->SetRightRollerSpeed(intakeX);
		}

		if (rollOutDesired) {
			robot->SetRollerSpeed(rollOutVal);
		}

		break;

	case (kZeroEncoder):
		printf("kZeroEncoder\n");
		/*Zero encoders the first time you try to move the elevator, may or may not have to be included
		 * in the auto sequence*/
		if (EncoderZeroingDone()) {
			encoderZeroingStarted = false;
			robot->EngageBrake();
			robot->ResetElevatorEncoder();
			encoderZeroed = true;
			nextState = stateAfterZeroingEncoder;
		} else if (!encoderZeroingStarted) {
			InitVelPIDOnly(defaultVelPIDConfig, -10.0);
			robot->DisengageBrake();
		} else {
			robot->SetElevatorSpeed(UpdateVelPIDOnly());
			nextState = kZeroEncoder;
		}

		break;

	case (kInitIntaking):
		printf("kInitIntaking\n");
		if (!encoderZeroed) {
			stateAfterZeroingEncoder = kInitIntaking;
			nextState = kZeroEncoder;
		} else {
			carriageMoveStarted = false;
			intakeStarted = false;
			nextState = kRaiseCarriageToIntake;
		}
		break;

	case (kRaiseCarriageToIntake):
		printf("kRaiseCarriageToIntake\n");
		if (!carriageMoveStarted) {
			//todo Change distance and speed carriage needs to raise
			InitCarriageMovement(defaultPosPIDConfig, defaultVelPIDConfig, 24.0, 0.0);
			robot->ResetElevatorEncoder();
			nextState = kRaiseCarriageToIntake;
			carriageMoveStarted = true;
		} else if (CarriageMoveDone() || robot->GetElevatorUpperLimitSwitchVal()) {
			robot->SetElevatorSpeed(0.0);
			carriageMoveStarted = false;
			nextState = kIdle;
		} else {
			robot->SetElevatorSpeed(UpdateCarriageMoveUp());
			nextState = kRaiseCarriageToIntake;
		}
		break;

	case (kIntake):
		printf("kIntake\n");
		if (!intakeStarted) {
			robot->MoveRollerArmsIn();
			robot->DisengageBrake();
			intakeStarted = true;
			inOrOuttakeStartTime = currTimeSec;
			nextState = kIntake;
		} else if (IntakeDone(currTimeSec)) {
			robot->EngageBrake();
			robot->SetRollerSpeed(0.0);
			robot->MoveRollerArmsOut();
			intakeStarted = false;
			nextState = kLowerCarriageToBottom;
		} else {
			robot->SetRollerSpeed(rollerSpeed);
			nextState = kIntake;
		}
		break;

	case (kLowerCarriageToBottom):
		printf("kLowerCarriageToBottom\n");
		//todo Change speed carriage needs to lower
		if (!carriageMoveStarted) {
			//SmartDashboard::PutString("State:", "Starting");
			robot->DisengageBrake();
			InitCarriageMovement(defaultPosPIDConfig, defaultVelPIDConfig, -24.0, -10.0);
			nextState = kLowerCarriageToBottom;
			carriageMoveStarted = true;
		} else if (CarriageMoveDone() || robot->GetHallEffectsSensorVal()) {
			robot->EngageBrake();
			//SmartDashboard::PutString("State:", "Stopping");
			robot->SetElevatorSpeed(0.0);
			carriageMoveStarted = false;
			nextState = kIdle;
		} else {
			//SmartDashboard::PutString("State:", "Moving");
			robot->SetElevatorSpeed(UpdateCarriageMoveDown());
			nextState = kLowerCarriageToBottom;
		}
		break;

	case (kRaiseCarriageToOuttakeToPlatform):
		printf("kRaiseCarriageToOuttakeToPlatform\n");
		if (!encoderZeroed) {
			stateAfterZeroingEncoder = kRaiseCarriageToOuttakeToPlatform;
			nextState = kZeroEncoder;
		} else if (!carriageMoveStarted) {
			//todo Change distance and speed carriage needs to raise
			InitCarriageMovement(defaultPosPIDConfig, defaultVelPIDConfig, 0, 0.0);
			carriageMoveStarted = true;
			nextState = kRaiseCarriageToOuttakeToPlatform;
		} else if (CarriageMoveDone() || robot->GetElevatorUpperLimitSwitchVal()) {
			robot->SetElevatorSpeed(0.0);
			carriageMoveStarted = false;
			nextState = kOuttake;
		} else {
			robot->SetElevatorSpeed(UpdateCarriageMoveUp());
			nextState = kRaiseCarriageToOuttakeToPlatform;
		}
		break;

	case (kRaiseCarriageToOuttakeToStep):
		printf("kRaiseCarriageToOuttakeToStep\n");
		if (!encoderZeroed) {
			stateAfterZeroingEncoder = kRaiseCarriageToOuttakeToStep;
			nextState = kZeroEncoder;
		} else if (!carriageMoveStarted) {
			//todo Change distance and speed carriage needs to raise
			InitCarriageMovement(defaultPosPIDConfig, defaultVelPIDConfig, 0, 0.0);
			carriageMoveStarted = true;
			nextState = kRaiseCarriageToOuttakeToStep;
		} else if (CarriageMoveDone() || robot->GetElevatorUpperLimitSwitchVal()) {
			robot->SetElevatorSpeed(0.0);
			carriageMoveStarted = false;
			nextState = kOuttake;
		} else {
			robot->SetElevatorSpeed(UpdateCarriageMoveUp());
			nextState = kRaiseCarriageToOuttakeToStep;
		}
		break;

	case (kOuttake):
		printf("kOuttake\n");
		if(!outtakeStarted) {
			robot->MoveOuttakeOut();
			InitCarriageMovement(defaultPosPIDConfig, defaultVelPIDConfig, 0, 0.0);
			outtakeStarted = true;
			nextState = kOuttake;
		} else if (CarriageMoveDone() || robot->GetHallEffectsSensorVal()) {
			robot->SetElevatorSpeed(0.0);
			outtakeStarted = false;
			robot->MoveOuttakeIn();
			nextState = kLowerCarriageToBottom;
		} else {
			robot->SetElevatorSpeed(UpdateCarriageMoveDown());
			nextState = kOuttake;
		}
		break;

	case (kTest):
		//To be used as a test bed
		printf("kTest\n");
		nextState = kIdle;
		break;

	case (kIntakeSequence):
		printf("kIntakeSequence\n");
		nextState = kIntakeSequence;
		if (!movedElevatorUpInIntakeSequence && robot->GetElevatorEncoderVal() < lowestEncoderValToIntake) {
			robot->DisengageBrake();
			robot->SetElevatorSpeed(0.8);
//			printf("movingUp encoderValue: %f\n", robot->GetElevatorEncoderVal());
			robot->MoveRampOut();
			printf("movingRampOut\n");
		} else if (!intakeTimerStarted) {
			intakeStartTime = timer->Get();
			intakeTimerStarted = true;
			movedElevatorUpInIntakeSequence = true;
			robot->MoveRollerArmsIn();
//			printf("movedRollerArmsIn\n");
//			printf("intakeStartTime: %f\n", intakeStartTime);
		} else if (timer->Get() - intakeStartTime < 1.0) {
			robot->SetElevatorSpeed(0.0);
			robot->EngageBrake();
			intakeY = -0.9;
			printf("intaking\n");
		} else if (!movedRollerArmsOutInIntakeSequence) {
			//robot->MoveRollerArmsOut();
			printf("movedRollerArmsOut\n");
			movedRollerArmsOutInIntakeSequence = true;
		} else if (robot->GetElevatorEncoderVal() > endIntakeSequenceElevatorEncoderVal){
			intakeY = 0.0;
			robot->DisengageBrake();
			robot->SetElevatorSpeed(-0.5);
			robot->MoveRampIn();
			printf("movingDown encoderValue: %f\n", robot->GetElevatorEncoderVal());
		} else if (robot->GetElevatorEncoderVal() <= endIntakeSequenceElevatorEncoderVal) {
			robot->SetElevatorSpeed(0.0);
			robot->EngageBrake();
			movedElevatorUpInIntakeSequence = false;
			intakeTimerStarted = false;
			movedRollerArmsOutInIntakeSequence = false;
			nextState = kIdle;
			printf("done with intakeSequence");
		}

		if (abs(intakeY) >= abs(intakeX)) {
			robot->SetRollerSpeed(-intakeY); // strict intake/outtake motion
		} else {
			robot->SetLeftRollerSpeed(intakeX); // strict left / right motion
			robot->SetRightRollerSpeed(intakeX);
		}

		break;

	default:
		/*Program should never come here, put print statement alerting programmer if it does*/
		nextState = kIdle;
		break;
	}
	m_stateVal = nextState;
}

void ElevatorController::SetAutoRampOutDesired(bool outDesired) {
	autoRampOutDesired = outDesired;
	autoRampChangeDesired = true;
	autoRampDone = false;
}

bool ElevatorController::AutoRampOutDone() {
	return autoRampDone;
}

void ElevatorController::SetAutoIntaking() {
	autoIntakeDesired = true;
	autoIntakingDone = false;
}

bool ElevatorController::AutoIntakingDone() {
	return autoIntakingDone;
}

void ElevatorController::SetAutoOuttaking() {
	autoOuttakeDesired = true;
	autoOuttakingDone = false;
}

bool ElevatorController::AutoOuttakingDone() {
	return autoOuttakingDone;
}

void ElevatorController::SetAutoClamp(bool clamp) {

	autoClampDesired = true;
	autoClampVal = clamp;
	autoClampDone = false;
}

bool ElevatorController::AutoClampDone() {

	return autoClampDone;
}

void ElevatorController::SetRollOutDesired(bool desired) {
	rollOutDesired = desired;
	if (desired) {
		rollOutVal = -1.0;
	} else {
		rollOutVal = 0.0;
	}
}

void ElevatorController::SetRollOutDone(bool done) {
	rollOutDone = done;
	if (done) {
		rollOutDesired = false;
	}
}

void ElevatorController::ResetAllInstanceFields() {
	m_stateVal = kInit;
	nextState = kInit;
	stateAfterZeroingEncoder = kIdle;

	lastEncoderVal = 0.0;
	currEncoderVal = 0.0;
	deltaEncoderVal = 0.0;
	lastVelOutput = 0.0;
	carriageSpeed = 0.0;
	inOrOuttakeStartTime = 0.0;
	rollerSpeed = 0.0;
	moveElevatorUpSpeed = 0.0;
	moveElevatorDownSpeed = 0.0;

	intakeX = 0.0;
	intakeY = 0.0;

	encoderZeroed = false;
	encoderZeroingStarted = false;
	rampOut = false;
	outtakeOut = false;
	carriageMoveStarted = false;
	intakeStarted = false;
	outtakeStarted = false;
	//todo Change to false before competition and to test sequences
	testOuttake = true;
	intakeTimerStarted = false;
	intakeStartTime = 0.0;
	movedElevatorUpInIntakeSequence = false;
	movedRollerArmsOutInIntakeSequence = false;
}

void ElevatorController::InitCarriageMovement(PIDConfig* posPIDConfig, PIDConfig* velPIDConfig,
											  int desiredEncoderVal, double desiredCarriageSpeed) {
	posPID->Init(posPIDConfig, currEncoderVal, desiredEncoderVal);
	velPID->Init(velPIDConfig, carriageSpeed, desiredCarriageSpeed);
	lastVelOutput = 0.0;
}

double ElevatorController::UpdateCarriageMoveUp() {
	double posOutput = posPID->Update(currEncoderVal);
	double velOutput = lastVelOutput + velPID->Update(carriageSpeed);
	velOutput = PIDControlLoop::Saturate(velOutput, velPID->GetPIDConfig()->maxAbsOutput);
	//SmartDashboard::PutNumber("Error", 24.0 - currEncoderVal);
	//SmartDashboard::PutNumber("Output", posOutput);
	lastVelOutput = velOutput;
	return fmin(posOutput, velOutput);
}

double ElevatorController::UpdateCarriageMoveDown() {
	double posOutput = posPID->Update(currEncoderVal);
	double velOutput = lastVelOutput + velPID->Update(carriageSpeed);
	velOutput = PIDControlLoop::Saturate(velOutput, velPID->GetPIDConfig()->maxAbsOutput);
	posPID->PrintPIDValues("posPID");
	posPID->PrintPIDValues("velPID");
	//SmartDashboard::PutNumber("Error", currEncoderVal);
	//SmartDashboard::PutNumber("PosOutput", posOutput);
	//SmartDashboard::PutNumber("VelOutput", velOutput);
	lastVelOutput = velOutput;
	return fmax(posOutput, velOutput);
}

bool ElevatorController::CarriageMoveDone() {
	return posPID->ControlLoopDone(currEncoderVal);
}

void ElevatorController::InitVelPIDOnly(PIDConfig* velPIDConfig, double desiredCarriageSpeed) {
	velPID->Init(velPIDConfig, carriageSpeed, desiredCarriageSpeed);
	lastVelOutput = 0.0;
}

double ElevatorController::UpdateVelPIDOnly() {
	double velOutput = lastVelOutput + velPID->Update(carriageSpeed);
	lastVelOutput = velOutput;
	return velOutput;
}

bool ElevatorController::IntakeDone(double currTimeSec) {
	// todo Confirm that the true and false of the light sensor are correct
	// return robot->GetLightSensorVal();
	// todo Change back to light sensor
	return ((inOrOuttakeStartTime - currTimeSec) > 3.0);
}

bool ElevatorController::EncoderZeroingDone() {
	encoderZeroed = robot->GetHallEffectsSensorVal();
	return encoderZeroed;
}

bool ElevatorController::RampOutDesired() {
	//NOTE TO KATY: When you add your auto bools, make this into a conditional
	if (autoRampChangeDesired) {
		return autoRampOutDesired;
	}
	return !rampOut;
}

bool ElevatorController::OuttakeOutDesired() {
	return !outtakeOut;
}

double ElevatorController::TransformUpSpeed(double original) {
	double modified;
	modified = original - (-1); // original - minimum
	modified *= 0.7; // 1 - 0.3 is the new range
	modified /= 2; // 1- (-1) is the old range
	modified += 0.3; // new mininum
	return modified;
}

//right now trasnformup and down are the same

double ElevatorController::TransformDownSpeed(double original) {
	double modified;
	modified = original - (-1); // original - minimum
	modified *= 0.7; // 1 - 0.3 is the new range
	modified /= 2; // 1- (-1) is the old range
	modified += 0.3; // new mininum
	return modified;
}
