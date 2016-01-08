#ifndef BUTTONREADER_H_
#define BUTTONREADER_H_

#include "WPILib.h"
class ButtonReader {
public:
	ButtonReader(Joystick *joy, int buttonNum);
	virtual ~ButtonReader();
	void ReadValue();
	bool IsDown();
	bool WasJustPressed();
	bool WasJustReleased();
	bool StateJustChanged();
	virtual bool GetState();

private:
	Joystick *joystick;
	int buttonNum;
	bool lastState;
	bool currState;
};

class ToggleButtonReader : public ButtonReader
{
public:
	ToggleButtonReader(Joystick *joy, int buttonNum);
	virtual ~ToggleButtonReader();
	virtual bool GetState();


private:
	bool currToggleState;
};

enum SwitchState {
	kUp = 1,
	kNeutral = 0,
	kDown = -1,
};

class SwitchReader {
public:
	SwitchReader(Joystick *myJoy, int upButton, int downButton);
	SwitchState GetSwitchState();

private:
	Joystick *joy;
	int upB;
	int downB;
};



#endif /*BUTTONREADER_H_*/
