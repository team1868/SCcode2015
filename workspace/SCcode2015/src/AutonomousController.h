#ifndef AUTONOMOUSCONTROLLER_H
#define AUTONOMOUSCONTROLLER_H

#include "WPILib.h"
#include "RobotModel.h"
#include "AutoCommand.h"
#include <vector>
#include <string>
#include <iostream>

//include the controllers as we create their auto implementations as #include s

class AutonomousController {
public:
	enum AutoMode {kTestAuto = 0,
					kBlankAuto = 1,
					kDriveAuto = 2,
					k1ToteAuto = 3,
					k1BinAuto = 4,
					k1Tote1BinAuto = 5,
					k1Tote2BinAuto = 6,
					k1Tote3BinAuto = 7,
					k2ToteAuto = 8,
					k2BinAuto = 9,
					k2Tote2BinAuto = 10,
					k2Tote1BinAuto = 11,
					k2Tote3BinAuto = 12,
					k3BinAuto = 13,
					kStackedToteAuto = 14,
					k3Tote1BinAuto = 15,
					k3Tote2BinAuto = 16,
					k3Tote3BinAuto = 17,
					k4BinAuto = 18
					};
	enum StartingPosition {
		kA = 0, kB = 1, kC = 2
	};

	AutonomousController(RobotModel* myRobot, DriveController* myDriver, ElevatorController* myElevator); //add controllers as we create them as parameters of the constructor
	~AutonomousController() {}

	void StartAutonomous();
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();
	void RefreshIni();

	unsigned int autoMode;
private:
	void CreateQueue();
	void AddtoQueue(AutoCommand* myNewAutoCommand, SimpleAutoCommand* myLastAutoCommand);
	AutoCommand* firstCommand;
	AutoCommand* nextCommand;
	AutoCommand* currentCommand;
	RobotModel* robot;
	DriveController* drive;
	ElevatorController* elevator;

	unsigned int autoStart;
	double desiredXDis;
	double desiredYDis;

};



#endif
