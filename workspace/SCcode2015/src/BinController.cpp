#include "BinController.h"

BinController::BinController(RobotModel* myRobot, RemoteController* myHumanControl) {
	robot = myRobot;
	humanControl = myHumanControl;
}

/**
 * Extends bin arms
 */
void BinController::Reset() {
	robot->BinOut(); //hooks bin mechanism on bin handle using piston
}

/**
 * If the button on the driver station is pressed, the arms will toggle
 * their position.
 */
void BinController::Update(double currTimeSec, double deltaTimeSec) {
	if (humanControl->BinsOutDesired()) {
		/*
		 * toggle button to retract and extend bin mechanism
		 */
		if (robot->GetBinIn()) {
			robot->BinOut();
		} else {
			robot->BinIn();
		}
	}
}
