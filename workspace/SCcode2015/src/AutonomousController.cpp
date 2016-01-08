#include "AutonomousController.h"
#include "AutoCommand.h"
#include "Debugging.h"

AutonomousController::AutonomousController(RobotModel* myRobot,	DriveController* myDriver,
		ElevatorController* myElevator) {
	robot = myRobot;
	firstCommand = NULL;
	nextCommand = NULL;
	currentCommand = NULL;
	drive = myDriver;
	elevator = myElevator;
	autoMode = 0;
	autoStart = 0;
}

/**
 * Creates the queue of AutoCommand instances
 */
void AutonomousController::StartAutonomous() {
	printf("Starting auto \n");
	CreateQueue();
	currentCommand = firstCommand;
	if (currentCommand != NULL) {
		currentCommand->Init();
	}
}

/**
 * Calls the Init(), then Update(double currTimeSec, double deltaTimeSec) of the current
 * AutoCommand until IsDone() returns true.
 */
void AutonomousController::Update(double currTimeSec, double deltaTimeSec) {
	if (currentCommand != NULL) {
		if (currentCommand->IsDone()) {
			DO_PERIODIC(1, printf("Command Complete at: %f \n", currTimeSec));
			currentCommand = currentCommand->GetNextCommand();
			if (currentCommand != NULL) {
				currentCommand->Init();
			}
		} else {
			currentCommand->Update(currTimeSec, deltaTimeSec);
		}
	} else {
		DO_PERIODIC(100, printf("Queue finished at: %f \n", currTimeSec));
	}
}

/**
 * Stops the robot's movement and the whole queue.
 */
void AutonomousController::Reset() {
	// firstCommand->IsDone();
	// currentCommand->IsDone();
	firstCommand = NULL;
	currentCommand = NULL;
	robot->SetWheelSpeed(RobotModel::kFrontLeftWheel, 0.0);
	robot->SetWheelSpeed(RobotModel::kRearLeftWheel, 0.0);
	robot->SetWheelSpeed(RobotModel::kFrontRightWheel, 0.0);
	robot->SetWheelSpeed(RobotModel::kRearRightWheel, 0.0);
}

void AutonomousController::RefreshIni() {
	autoMode = robot->pini->geti("AUTONOMOUS","AUTOMODE",0);
	DriveCommand::xPFac = robot->pini->getf("DRIVECOMMAND", "xPFac", 0.5);
	DriveCommand::xIFac = robot->pini->getf("DRIVECOMMAND", "xIFac", 0.01);
	DriveCommand::xDFac = robot->pini->getf("DRIVECOMMAND", "xDFac", 1.0);
	DriveCommand::xDesiredAccuracy = robot->pini->getf("DRIVECOMMAND", "xDesiredAccuracy", 0.05);
	DriveCommand::xMaxAbsError = robot->pini->getf("DRIVECOMMAND", "xMaxAbsError", 3.0);
	DriveCommand::xMaxAbsDiffError = robot->pini->getf("DRIVECOMMAND", "xMaxAbsDiffError", 1.0);
	DriveCommand::xMaxAbsITerm = robot->pini->getf("DRIVECOMMAND", "xMaxAbsITerm", 0.1);
	DriveCommand::xMaxAbsOutput = robot->pini->getf("DRIVECOMMAND", "xMaxAbsOutput", 0.7);

	DriveCommand::yPFac = robot->pini->getf("DRIVECOMMAND", "yPFac", 0.65);
	DriveCommand::yIFac = robot->pini->getf("DRIVECOMMAND", "yIFac", 0.001);
	DriveCommand::yDFac = robot->pini->getf("DRIVECOMMAND", "yDFac", 1.0);
	DriveCommand::yDesiredAccuracy = robot->pini->getf("DRIVECOMMAND", "yDesiredAccuracy", 0.05);
	DriveCommand::yMaxAbsError = robot->pini->getf("DRIVECOMMAND", "yMaxAbsError", 3.0);
	DriveCommand::yMaxAbsDiffError = robot->pini->getf("DRIVECOMMAND", "yMaxAbsDiffError", 1.0);
	DriveCommand::yMaxAbsITerm = robot->pini->getf("DRIVECOMMAND", "yMaxAbsITerm", 0.1);
	DriveCommand::yMaxAbsOutput = robot->pini->getf("DRIVECOMMAND", "yMaxAbsOutput", 0.7);

	DriveCommand::rPFac = robot->pini->getf("DRIVECOMMAND", "rPFac", 0.07);
	DriveCommand::rIFac = robot->pini->getf("DRIVECOMMAND", "rIFac", 0.0015);
	DriveCommand::rDFac = robot->pini->getf("DRIVECOMMAND", "rDFac", 0.1);
	DriveCommand::rDesiredAccuracy = robot->pini->getf("DRIVECOMMAND", "rDesiredAccuracy", 1.0);
	DriveCommand::rMaxAbsError = robot->pini->getf("DRIVECOMMAND", "rMaxAbsError", 10.0);
	DriveCommand::rMaxAbsDiffError = robot->pini->getf("DRIVECOMMAND", "rMaxAbsDiffError", 2.0);
	DriveCommand::rMaxAbsITerm = robot->pini->getf("DRIVECOMMAND", "rMaxAbsITerm", 0.1);
	DriveCommand::rMaxAbsOutput = robot->pini->getf("DRIVECOMMAND", "rMaxAbsOutput", 0.7);

	DriveStraightCommand::yPFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND","yPFac", 0.6);
	DriveStraightCommand::yIFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "yIFac", 0.002);
	DriveStraightCommand::yDFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "yDFac", 1.0);
	DriveStraightCommand::yDesiredAccuracy = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "yDesiredAccuracy", 0.05);
	DriveStraightCommand::yMaxAbsOutput = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "yMaxAbsOutput", 0.7);
	DriveStraightCommand::yMaxAbsDiffError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "yMaxAbsDiffError", 1.0);
	DriveStraightCommand::yMaxAbsError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "yMaxAbsError", 5.0);
	DriveStraightCommand::yMaxAbsITerm = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "yMaxAbsITerm", 0.2);

	DriveStraightCommand::rPFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rPFac", 0.002);
	DriveStraightCommand::rIFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rIFac", 0.0);
	DriveStraightCommand::rDFac = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rDFac", 0.1);
	DriveStraightCommand::rDesiredAccuracy = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rDesiredAccuracy", 0.1);
	DriveStraightCommand::rMaxAbsOutput = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsOutput", 0.2);
	DriveStraightCommand::rMaxAbsDiffError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsDiffError", 10.0);
	DriveStraightCommand::rMaxAbsError = robot->pini->getf("DRIVESTRAIGHTCOMMAND", "rMaxAbsError", 10.0);
	DriveStraightCommand::rMaxAbsITerm = robot->pini->getf("DRIVESTRAIGHTCOMMAND","rMaxAbsITerm", 0.1);

	GyroLineUpToteCommand::gyroPFac = robot->pini->getf("GYROLINEUPCOMMAND", "gyroPFac", 0.05);
	GyroLineUpToteCommand::gyroIFac = robot->pini->getf("GYROLINEUPCOMMAND", "gyroIFac", 0.002);
	GyroLineUpToteCommand::gyroDFac = robot->pini->getf("GYROLINEUPCOMMAND", "gyroDFac", 0.15);
	GyroLineUpToteCommand::gyroDesiredAccuracy = robot->pini->getf("GYROLINEUPCOMMAND", "gyroDesiredAccuracy", 0.1);
	GyroLineUpToteCommand::gyroMaxAbsDiffError = robot->pini->getf("GYROLINEUPCOMMAND", "gyroMaxAbsDiffError", 10.0);
	GyroLineUpToteCommand::gyroMaxAbsError = robot->pini->getf("GYROLINEUPCOMMAND", "gyroMaxAbsError", 10.0);
	GyroLineUpToteCommand::gyroMaxAbsOutput = robot->pini->getf("GYROLINEUPCOMMAND", "gyroMaxAbsOutput", 0.7);
	GyroLineUpToteCommand::gyroMaxAbsITerm = robot->pini->getf("GYROLINEUPCOMMAND", "gyroMaxAbsITerm", 0.1);

	PivotCommand::rPFac = robot->pini->getf("PIVOTCOMMAND", "pFac", 0.03);
	PivotCommand::rIFac = robot->pini->getf("PIVOTCOMMAND", "iFac", 0.00);
	PivotCommand::rDFac = robot->pini->getf("PIVOTCOMMAND", "dFac", 0.1);
	PivotCommand::rDesiredAccuracy = robot->pini->getf("PIVOTCOMMAND", "desiredAccuracy", 0.1);
	PivotCommand::rMaxAbsOutput = robot->pini->getf("PIVOTCOMMAND", "maxAbsOutput", 0.7);
	PivotCommand::rMaxAbsDiffError = robot->pini->getf("PIVOTCOMMAND", "maxAbsDiffError", 10.0);
	PivotCommand::rMaxAbsError = robot->pini->getf("PIVOTCOMMAND", "maxAbsError", 10.0);
	PivotCommand::rMaxAbsITerm = robot->pini->getf("PIVOTCOMMAND", "maxAbsITerm", 0.1);

	StrafeCommand::xPFac = robot->pini->getf("STRAFECOMMAND", "xPFac", 0.5);
	StrafeCommand::xIFac = robot->pini->getf("STRAFECOMMAND", "xIFac", 0.001);
	StrafeCommand::xDFac = robot->pini->getf("STRAFECOMMAND", "xDFac", 1.0);
	StrafeCommand::xDesiredAccuracy = robot->pini->getf("STRAFECOMMAND",
			"xDesiredAccuracy", 0.025);
	StrafeCommand::xMaxAbsError = robot->pini->getf("STRAFECOMMAND",
			"xMaxAbsError", 3.0);
	StrafeCommand::xMaxAbsDiffError = robot->pini->getf("STRAFECOMMAND",
			"xMaxAbsDiffError", 1.0);
	StrafeCommand::xMaxAbsITerm = robot->pini->getf("STRAFECOMMAND",
			"xMaxAbsITerm", 0.1);
	StrafeCommand::xMaxAbsOutput = robot->pini->getf("STRAFECOMMAND",
			"xMaxAbsOutput", 0.7);

	StrafeCommand::rPFac = robot->pini->getf("STRAFECOMMAND", "rPFac", 0.08);
	StrafeCommand::rIFac = robot->pini->getf("STRAFECOMMAND", "rIFac", 0.0001);
	StrafeCommand::rDFac = robot->pini->getf("STRAFECOMMAND", "rDFac", 0.5);
	StrafeCommand::rDesiredAccuracy = robot->pini->getf("STRAFECOMMAND",
			"rDesiredAccuracy", 0.25);
	StrafeCommand::rMaxAbsError = robot->pini->getf("STRAFECOMMAND",
			"rMaxAbsError", 0.3);
	StrafeCommand::rMaxAbsDiffError = robot->pini->getf("STRAFECOMMAND",
			"rMaxAbsDiffError", 2.0);
	StrafeCommand::rMaxAbsITerm = robot->pini->getf("STRAFECOMMAND",
			"rMaxAbsITerm", 0.1);
	StrafeCommand::rMaxAbsOutput = robot->pini->getf("STRAFECOMMAND",
			"rMaxAbsOutput", 0.34);
	desiredXDis = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "desiredX", 0.9);
	desiredYDis = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "desiredY", 0.8);
	UltrasonicLineUpToteCommand::xPFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "xPFac", 0.3);
	UltrasonicLineUpToteCommand::xIFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "xIFac", 0.0015);
	UltrasonicLineUpToteCommand::xDFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "xDFac", 0.8);
	UltrasonicLineUpToteCommand::xDesiredAccuracy = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"xDesiredAccuracy", 0.05);
	UltrasonicLineUpToteCommand::xMaxAbsError = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"xMaxAbsError", 3.0);
	UltrasonicLineUpToteCommand::xMaxAbsDiffError = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"xMaxAbsDiffError", 1.0);
	UltrasonicLineUpToteCommand::xMaxAbsITerm = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"xMaxAbsITerm", 0.2);
	UltrasonicLineUpToteCommand::xMaxAbsOutput = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"xMaxAbsOutput", 0.7);

	UltrasonicLineUpToteCommand::yPFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "yPFac", 0.6);
	UltrasonicLineUpToteCommand::yIFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "yIFac", 0.003);
	UltrasonicLineUpToteCommand::yDFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "yDFac", 1.3);
	UltrasonicLineUpToteCommand::yDesiredAccuracy = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"yDesiredAccuracy", 0.05);
	UltrasonicLineUpToteCommand::yMaxAbsError = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"yMaxAbsError", 3.0);
	UltrasonicLineUpToteCommand::yMaxAbsDiffError = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"yMaxAbsDiffError", 1.0);
	UltrasonicLineUpToteCommand::yMaxAbsITerm = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"yMaxAbsITerm", 0.2);
	UltrasonicLineUpToteCommand::yMaxAbsOutput = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"yMaxAbsOutput", 0.7);

	UltrasonicLineUpToteCommand::rPFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "rPFac", 0.04);
	UltrasonicLineUpToteCommand::rIFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "rIFac", 0.0005);
	UltrasonicLineUpToteCommand::rDFac = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND", "rDFac", 0.07);
	UltrasonicLineUpToteCommand::rDesiredAccuracy = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"rDesiredAccuracy", 0.5);
	UltrasonicLineUpToteCommand::rMaxAbsError = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"rMaxAbsError", 10.0);
	UltrasonicLineUpToteCommand::rMaxAbsDiffError = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"rMaxAbsDiffError", 2.0);
	UltrasonicLineUpToteCommand::rMaxAbsITerm = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"rMaxAbsITerm", 0.2);
	UltrasonicLineUpToteCommand::rMaxAbsOutput = robot->pini->getf("ULTRASONICLINEUPTOTECOMMAND",
			"rMaxAbsOutput", 0.7);
}

/**
 * Pushes new AutoCommand objects into the queue.
 * @param AutoCommand* myNewAutoCommand is the command to be pushed in, and
 * SimpleAutoCommand* myLastAutoCommand is the command that comes before it
 */
void AutonomousController::AddtoQueue(AutoCommand* myNewAutoCommand,
		SimpleAutoCommand* myLastAutoCommand) {
	myLastAutoCommand->SetNextCommand(myNewAutoCommand);
}

/**
 * Contains many different states for different versions of autonomous, all
 * of which have different AutoCommand objects in their queues.
 */
void AutonomousController::CreateQueue() {
	firstCommand = NULL;
	printf("AutoMode: %i \n", autoMode);

	switch (autoMode) {

	case (kTestAuto): {

		DriveCommand* drive1 = new DriveCommand(robot, 0.0, 1.0, 0.0);
		firstCommand = drive1;

		ClampCommand* clamp1 = new ClampCommand(elevator, true);
		drive1->SetNextCommand(clamp1);

		WaitingCommand* wc = new WaitingCommand(0.5);
		clamp1->SetNextCommand(wc);

		RollOutCommand* roc = new RollOutCommand(elevator, 1.0);
		wc->SetNextCommand(roc);

		DriveCommand* backwardsturn1 = new DriveCommand(robot, 0.0, -1.0, 85.0);
		roc->SetNextCommand(backwardsturn1);

		DriveCommand* forward1 = new DriveCommand(robot, 0.0, 1.50, 0.0);
		ClampCommand* unclamp1 = new ClampCommand(elevator, false);
		ParallelAutoCommand* pac = new ParallelAutoCommand(forward1, unclamp1);
		backwardsturn1->SetNextCommand(pac);

		DriveCommand* turndrive = new DriveCommand(robot, 0.0, 2.0, -80.0);
		pac->SetNextCommand(turndrive);

		ClampCommand* clamp2 = new ClampCommand(elevator, true);
		turndrive->SetNextCommand(clamp2);

		WaitingCommand* wc2 = new WaitingCommand(0.5);
		clamp2->SetNextCommand(wc2);

		RollOutCommand* roc2 = new RollOutCommand(elevator, 1.0);
		wc2->SetNextCommand(roc2);

		DriveCommand* backwardsturn2 = new DriveCommand(robot, 0.0, -1.0, 85.0);
		roc2->SetNextCommand(backwardsturn2);

		DriveCommand* forward2 = new DriveCommand(robot, 0.0, 1.50, 0.0);
		ClampCommand* unclamp2 = new ClampCommand(elevator, false);
		ParallelAutoCommand* pac2 = new ParallelAutoCommand(forward2, unclamp2);
		backwardsturn2->SetNextCommand(pac2);

		DriveCommand* turndrive2 = new DriveCommand(robot, 0.0, 2.0, -80.0);
		pac2->SetNextCommand(turndrive2);

		ClampCommand* clamp3 = new ClampCommand(elevator, true);
		turndrive2->SetNextCommand(clamp3);

		WaitingCommand* wc3 = new WaitingCommand(0.5);
		clamp3->SetNextCommand(wc3);

		RollOutCommand* roc3 = new RollOutCommand(elevator, 1.0);
		wc3->SetNextCommand(roc3);

		DriveCommand* backwardsturn3 = new DriveCommand(robot, 0.0, -1.0, 85.0);
		roc3->SetNextCommand(backwardsturn3);

		DriveCommand* forward3 = new DriveCommand(robot, 0.0, 1.50, 0.0);
		backwardsturn3->SetNextCommand(forward3);

		DriveCommand* turn = new DriveCommand(robot, 0.0, 0.0, -80.0);
		forward3->SetNextCommand(turn);
		DriveCommand* forward4 = new DriveCommand(robot, 0.0, 4.0, 0.0);
		turn->SetNextCommand(forward4);


/*
 //START
		std::string intake1 = "/home/lvuser/intake.txt";
		PlaybackAutoCommand* firstintake = new PlaybackAutoCommand(robot, intake1);
		firstCommand = firstintake;

		std::string drivetitanium1 = "/home/lvuser/driveandtitanium.txt";
		PlaybackAutoCommand* firstdrive = new PlaybackAutoCommand(robot, drivetitanium1);
		AddtoQueue(firstdrive, firstintake);

		/*
		ServoTestAutoCommand* stc1 = new ServoTestAutoCommand(robot, 18.68);
		AddtoQueue(stc1, firstdrive);

		WaitingCommand* wc1 = new WaitingCommand(0.0);
		AddtoQueue(wc1, stc1);

		UltrasonicSweepAutoCommand* usa1 = new UltrasonicSweepAutoCommand(robot,
				20.0, 5.0);
		AddtoQueue(usa1, wc1);

		UltrasonicLineUpToteCommand* ulut1 = new UltrasonicLineUpToteCommand(
				robot, drive, -desiredXDis, desiredYDis, 0.0);
		AddtoQueue(ulut1, usa1);
		*/
/*
		std::string intake2 = "/home/lvuser/intake.txt";
		PlaybackAutoCommand* secondintake = new PlaybackAutoCommand(robot,
				intake2);
		AddtoQueue(secondintake, firstdrive);

		std::string drivetitanium2 = "/home/lvuser/driveandtitanium.txt";
		PlaybackAutoCommand* seconddrive = new PlaybackAutoCommand(robot,
				drivetitanium2);
		AddtoQueue(seconddrive, secondintake);
/*
		ServoTestAutoCommand* stc2 = new ServoTestAutoCommand(robot, 18.68);
		AddtoQueue(stc2, seconddrive);

		WaitingCommand* wc2 = new WaitingCommand(0.0);
		AddtoQueue(wc2, stc2);

		UltrasonicSweepAutoCommand* usa2 = new UltrasonicSweepAutoCommand(robot,
				20.0, 5.0);
		AddtoQueue(usa2, wc2);

		UltrasonicLineUpToteCommand* ulut2 = new UltrasonicLineUpToteCommand(
				robot, drive, -desiredXDis, desiredYDis, 0.0);
		AddtoQueue(ulut2, usa2);
*/
/*
		std::string intake3 = "/home/lvuser/intake.txt";
		PlaybackAutoCommand* thirdintake = new PlaybackAutoCommand(robot,
				intake3);
		AddtoQueue(thirdintake, seconddrive);

		std::string outtake1 = "/home/lvuser/outtake.txt";
		PlaybackAutoCommand* firstouttake = new PlaybackAutoCommand(robot,
				outtake1);
		AddtoQueue(firstouttake, thirdintake);

//END



		/*
		GrabBinCommand* g = new GrabBinCommand(elevator);
		firstCommand = g;
*/
/* Playback testing w/ ultra line up */
/*
		std::string file1 = "/home/lvuser/firsttote.txt";

		std::string file2 = "/home/lvuser/secondtote.txt";


		PlaybackAutoCommand* pac = new PlaybackAutoCommand(robot, file1);

		firstCommand = pac;
		*/
/*
		ServoTestAutoCommand* stc = new ServoTestAutoCommand(robot, 18.68);
		AddtoQueue(stc, pac);

		WaitingCommand* wc = new WaitingCommand(0.5);
		AddtoQueue(wc, stc);

		UltrasonicSweepAutoCommand* usa = new UltrasonicSweepAutoCommand(robot,
				20.0, 5.0);
		AddtoQueue(usa, wc);

		UltrasonicLineUpToteCommand* ulut = new UltrasonicLineUpToteCommand(
				robot, drive, -desiredXDis, desiredYDis, 0.0);
		AddtoQueue(ulut, usa);

		PlaybackAutoCommand* pac2 = new PlaybackAutoCommand(robot, file2);

		AddtoQueue(pac2, ulut);
		*/

		/*
		DriveCommand* dc = new DriveCommand(robot, 0.0, 23.0, 0.0);
		firstCommand = dc;
		*/


/*
		DriveCommand* spin = new DriveCommand(robot, drive, 0.0, 0.0, 180.0);
		AddtoQueue(spin, dc);
		*/
/*
		std::string o = "/home/lvuser/outtake.txt";
		PlaybackAutoCommand* outtake = new PlaybackAutoCommand(robot,
				o);
		firstCommand = outtake;
*/
	/*
		std::string knockoffbin = "/home/lvuser/knockoffbin.txt";
		PlaybackAutoCommand* kob = new PlaybackAutoCommand(robot, knockoffbin);
		firstCommand = kob;

		std::string intake1txt = "/home/lvuser/intake.txt";
		PlaybackAutoCommand* intake1 = new PlaybackAutoCommand(robot, intake1txt);
		AddtoQueue(intake1, kob);

		std::string turntopickuptxt = "/home/lvuser/turntopickup.txt";
		PlaybackAutoCommand* ttpu = new PlaybackAutoCommand(robot, turntopickuptxt);
		AddtoQueue(ttpu, intake1);

		std::string intake2txt = "/home/lvuser/intake.txt";
		PlaybackAutoCommand* intake2 = new PlaybackAutoCommand(robot, intake2txt);
		AddtoQueue(intake2, ttpu);
*/
		break;
	}

	case (kBlankAuto): {

		/*
		 * Do absolutely nothing.
		 */
		break;
	}
	case (kDriveAuto): {
		DriveCommand* driveforwardtozone = new DriveCommand(robot, 0.0, 8.5, 0.0);
		firstCommand = driveforwardtozone;
		break;}
	case (k1ToteAuto): {
		DriveCommand* driveFromLandfill = new DriveCommand(robot, 0.0, 3.5, 0.0);
		firstCommand = driveFromLandfill;

		break;}
	case (k1BinAuto): {

		break;}
	case (k1Tote1BinAuto): {
		DriveCommand* drive2 = new DriveCommand(robot, 0.0, 2.0, 0.0);
		firstCommand = drive2;

		break;}
	case (k1Tote2BinAuto): {
		ClampCommand* clampbin1 = new ClampCommand(elevator, true);
		firstCommand = clampbin1;

		DriveCommand* turn12 = new DriveCommand(robot, 0.0,0.0,180.0);
		clampbin1->SetNextCommand(turn12);

		RollOutCommand* roc12 = new RollOutCommand(elevator, 0.5);
		turn12->SetNextCommand(roc12);

		DriveCommand* turn13 = new DriveCommand(robot, 0.0, 0.0, -150.0);
		ClampCommand* unclamp12 = new ClampCommand(elevator, false);
		ParallelAutoCommand* pac12 = new ParallelAutoCommand(turn13, unclamp12);
		roc12->SetNextCommand(pac12);

		break;}
	case (k1Tote3BinAuto): {

		robot->BinIn();

		WaitingCommand* wc1 = new WaitingCommand(0.5);
		firstCommand = wc1;

		DriveCommand* driving1 = new DriveCommand(robot, 0.0, 2.0, 0.0);
		AddtoQueue(driving1, wc1);
		/*
		 * Intake one bin and one tote and then drive to a point 6 feet away from
		 * the other two bins. Pick up one bin and place it in autozone and
		 * repeate with the last bin and then drive to the autozone to drop the
		 * stack
		 */
		break;}
	case (k2ToteAuto): {
		/*
		 * Intake one tote and then drive to the next tote to intake it. Then, move
		 * to autozone to drop the stack.
		 */
		switch (autoStart) {
		case (kA): {
			break;}
		case (kB): {
			break;}
		case (kC): {
			break;}
		}
		break;}
	case (k2BinAuto): {
		/*
		 * Be 6 feet away from the two bins and pick up one bin and place in the
		 * autozone and repeate with the other bin. Drive towards the autozone.
		 */

		break;}
	case (k2Tote1BinAuto): {
		/*
		 * Intake bin then a tote. Drive towards next tote and intake it. Drive to
		 * autozone to drop the stack.
		 */
		switch (autoStart) {
		case (kA): {
			break;}
		case (kB): {
			break;}
		case (kC): {
			break;}
		}
		break;}
	case (k2Tote2BinAuto): {
		/*
		 * Intake bin then a tote. Drive towards next tote and intake it. Drive 6 feet
		 * away from the bin and pick it up and pivot to place into the autozone.
		 * Then, drive towards the autozone to drop the stack.
		 */
		break;}
	case (k2Tote3BinAuto): {
		/*
		 * Intake bin and tote. Drive towards the next tote and intake it. Drive 6
		 * feet away from the last two bins. Pick up one bin to place into the
		 * autozone and repeat for the other bin. Then, drive towards the autozoen
		 * to drop the stack.
		 */
		break;}
	case (kStackedToteAuto): {
		/*
		 * Intake tote. Drive towards the next tote and intake it. Drive towards
		 * last tote and intake it. Drive towards autozone and drop the stack.
		 */
		break;}
	case (k3Tote1BinAuto): {
		/*
		 * Intake bin and tote. Drive towards the next tote and intake it. Drive
		 * towards last tote and intake it. Drive towards autozone and drop the
		 * stack.
		 */
		break;}
	case (k3Tote2BinAuto): {
		/*
		 * Intake bin and tote. Drive towards next tote and intake it. Drive towards
		 * last tote and intake it. Drive 6 feet away from the bin and pick it up
		 * to drop in autozone. Drive to autozone to drop the stack
		 */
		break;}
	case (k3Tote3BinAuto): {
		/*
		 * Intake bin and tote. Drive towards the next tote and intake it.
		 * Then drive towards the next tote and intake it too. Drive 6
		 * feet away from the last two bins. Pick up one bin to place into the
		 * autozone and repeat for the other bin. Then, drive towards the autozoen
		 * to drop the stack.
		 */
		break;}
	case (k4BinAuto): {
		/*
		 * This is for getting bins from the step.
		 * Position the robot so that the arm can extend to 2 bins from this position
		 * Grab a bin and pivot and drop it off in the auto zone.
		 * Pivot and grab the other bin and pivot and drop it off in the auto zone
		 * Drive (PID) forward to the next such spot for the next 2 bins and repeat
		 * Drive to the autozone
		 */
		break;}
	}
}
