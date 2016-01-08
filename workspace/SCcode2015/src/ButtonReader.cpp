#include "ButtonReader.h"

ButtonReader::ButtonReader(Joystick* myJoystick, int myButtonNum) {
	joystick = myJoystick;
	buttonNum = myButtonNum;
	currState = joystick->GetRawButton(buttonNum);
	lastState = currState;
}

void ButtonReader::ReadValue() {
	lastState = currState;
	currState = joystick->GetRawButton(buttonNum);
}

bool ButtonReader::WasJustPressed() {
	return (lastState == false && currState == true);
}

bool ButtonReader::WasJustReleased() {
	return (lastState == true && currState == false);
}

bool ButtonReader::StateJustChanged() {
	return (lastState != currState);
}

bool ButtonReader::IsDown() {
	return currState;
}

bool ButtonReader::GetState() {
	return currState;
}
ButtonReader::~ButtonReader() {
}

ToggleButtonReader::ToggleButtonReader(Joystick *joy, int buttonNum) :
	ButtonReader(joy, buttonNum) {
	currToggleState = false;
}

bool ToggleButtonReader::GetState() {
	if (WasJustReleased()) {
		currToggleState = !currToggleState;
	}
	return (currToggleState);
}

ToggleButtonReader::~ToggleButtonReader() {
}

SwitchReader::SwitchReader(Joystick *myJoy, int upButton, int downButton) {
	joy = myJoy;
	upB = upButton;
	downB = downButton;
}

SwitchState SwitchReader::GetSwitchState() {
	if (joy->GetRawButton(upB))
		return kUp;
	if (joy->GetRawButton(downB))
		return kDown;
	return kNeutral;
}

