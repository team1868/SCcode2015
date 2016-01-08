#ifndef BINCONTROLLER_H
#define BINCONTROLLER_H

#include "RobotModel.h"
#include "RemoteControl.h"

class BinController {
public:
	BinController();
	~BinController() {}
	/**
	 * Used to control the two hooks that grab bins from the step during
	 * the autonomous period.
	 */
	BinController(RobotModel* myRobot, RemoteController* myHumanControl);

	void Reset();
	void Update(double currTimeSec, double deltaTimeSec);
private:
	RobotModel* robot;
	RemoteController* humanControl;
};
#endif
