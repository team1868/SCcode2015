#include "AutoControlBoard.h"
#include "RobotPorts2015.h"
#include "WPILib.h"

/**
 * The whole point of AutoControlBoard is to pretend to be a child of ControlBoard,
 * but instead do nothing. Values are not given from ReadControls(), rather they
 * are read from the playback file.
 * @see AutoCommand.cpp
 */
AutoControlBoard::AutoControlBoard(RobotModel* myRobot, const std::string& playbackFileName) : ControlBoard() {
	robot = myRobot;
	playbackFilePath = playbackFileName;
}

/**
 * ControlBoard has a method called ReadControls() that reads values from the
 * driver station and sends it to all the controllers. AutoControlBoard, however,
 * does nothing in ReadControls() because all the values in the auto controllers
 * come from PlaybackAutoCommand.
 * @see ControlBoard.cpp
 * @see AutoCommand.cpp
 */
void AutoControlBoard::ReadControls() {
	// explicitly does nothing. overrides ControlBoard's readcontrols
}
