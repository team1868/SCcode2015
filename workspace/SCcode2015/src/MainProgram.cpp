#include "WPILib.h"
#include "ControlBoard.h"
#include "RobotModel.h"
#include "DriveController.h"
#include "ElevatorController.h"
#include "AutonomousController.h"
#include "Debugging.h"
#include "AutoControlBoard.h"
#include <string.h>
#include "BinController.h"

class MainProgram : public IterativeRobot {
	LiveWindow *lw;
	RobotModel *robot;
	ControlBoard *humanControl;
	DriveController *driveController;
	AutonomousController *autonomousController;
	ElevatorController* elevatorController;
	BinController* binController;
	Timer* timer;

	double currTimeSec;
	double lastTimeSec;
	double deltaTimeSec;

public:
	MainProgram(void) {
		lw = LiveWindow::GetInstance();
		robot = new RobotModel();
		humanControl = new ControlBoard();
		driveController = new DriveController(robot, humanControl);
		elevatorController = new ElevatorController(robot, humanControl);
		autonomousController = new AutonomousController(robot, driveController, elevatorController);
		binController = new BinController(robot, humanControl);
		timer = new Timer();

		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;
	}
private:
	void RobotInit() {
		robot->EnableCompressor();
		robot->ResetTimer();
		robot->ResetGyro();
		robot->ResetDriveEncoders();
		robot->ResetElevatorEncoder();
		printf("robot init\n");
		driveController->RefreshIni();
		autonomousController->RefreshIni();
		//elevatorController->RefreshIni();
		timer->Start();
	}

	void AutonomousInit() {
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;

		robot->SetServoAngle(0.0);
		printf("auto init\n");

		robot->ResetDriveEncoders();
		robot->ResetElevatorEncoder();
		binController->Reset();

		RefreshAllIni();

		autonomousController->StartAutonomous();

	}

	void AutonomousPeriodic() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;

		autonomousController->Update(currTimeSec, deltaTimeSec);
		binController->Update(currTimeSec, deltaTimeSec);
		elevatorController->Update(currTimeSec, deltaTimeSec);

		//printf("auto periodic\n");
	}

	void TeleopInit() {
		//humanControl->CloseFile();
		//robot->DisableCompressor();
		RefreshAllIni();


		humanControl->OpenFile();
		currTimeSec = 0.0;
		lastTimeSec = 0.0;
		deltaTimeSec = 0.0;

		//driveController->Reset();
		//autonomousController->Reset();
		elevatorController->Reset();
		//binController->Reset();

		driveController->RefreshIni();
		autonomousController->RefreshIni();
		elevatorController->RefreshIni();

		robot->ResetDriveEncoders();
		robot->ResetElevatorEncoder();
/*
		if (robot->GetBinIn()) {
			robot->BinIn();
		} else {
			robot->BinOut();
		}
*/
		if (autonomousController->autoMode == AutonomousController::k1Tote3BinAuto) {
			robot->BinIn();
		}

		robot->ResetNavxDisplacement();
		robot->ResetNavxRotation();
	}

	void TeleopPeriodic() {
		lastTimeSec = currTimeSec;
		currTimeSec = robot->timer->Get();
		deltaTimeSec = currTimeSec - lastTimeSec;

		humanControl->ReadControls();
		driveController->Update(currTimeSec, deltaTimeSec);
		elevatorController->Update(currTimeSec, deltaTimeSec);

		binController->Update(currTimeSec, deltaTimeSec);

		if (robot->GetVoltage() < 9.5) {
			printf("LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS LOW VOLTS \n");
		}
		//DO_PERIODIC(20,printf("Front Left current %f\n", robot->pdp->GetCurrent(7)));
		//DO_PERIODIC(20,printf("Rear Left current %f\n", robot->pdp->GetCurrent(4)));
		//DO_PERIODIC(20,printf("Front Right current %f\n", robot->pdp->GetCurrent(3)));
		//DO_PERIODIC(20,printf("Rear Right current %f\n", robot->pdp->GetCurrent(0)));
		printf("Bin In/Out %d\n", robot->GetBinIn());
	}

	void TestPeriodic() {
		robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
		// lw->Run();
		printf("fleft encoder: %f\n", robot->GetFrontLeftEncoderVal());
		printf("fright encoder: %f\n", robot->GetFrontRightEncoderVal()); // in meters
		printf("rleft encoder: %f\n", robot->GetRearLeftEncoderVal());
		printf("rright encoder: %f\n", robot->GetRearRightEncoderVal());

		printf("ELEV encoder: %f\n", robot->GetElevatorEncoderVal());

	}

	void DisabledInit() {
		humanControl->CloseFile();
		robot->DisableCompressor();
		driveController->Reset();
		autonomousController->Reset();
		binController->Reset();
	}

	void RefreshAllIni() {
		robot->RefreshIniFile();
		autonomousController->RefreshIni();
		driveController->RefreshIni();
		elevatorController->RefreshIni();
	}
};

START_ROBOT_CLASS(MainProgram);

