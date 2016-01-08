#ifndef AUTOCONTROLBOARD_H
#define AUTOCONTROLBOARD_H

#include "WPILib.h"
#include "RemoteControl.h"
#include "ControlBoard.h"
#include "ButtonReader.h"
#include "RobotModel.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
class AutoControlBoard : public ControlBoard {
public:
	/**
	 * This should never be used after one instance has been created.
	 * @see AutoCommand.cpp
	 */
	AutoControlBoard(RobotModel* myRobot, const std::string& playbackFileName);
	~AutoControlBoard() {};
	void ReadControls();
	bool IsDone();
	void Reset();

private:
	RobotModel* robot;
	std::string playbackFilePath;
	// double varVal;
	//bool mReverseDriveDesired;
};
#endif
