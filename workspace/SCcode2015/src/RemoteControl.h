#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H

class RemoteController {
public:
	enum Joysticks {kLeftJoy, kRightJoy, kIntakeJoy};
	typedef enum Axes {kX, kY} uint32_t;

	virtual void ReadControls() = 0;

	virtual double GetJoystickValue(Joysticks j, Axes a) = 0;

	virtual bool FieldCentricDesired() = 0;

	virtual bool RobotCentricDesired() = 0;

	virtual bool IntakeSequenceDesired() = 0;

	//virtual bool UltrasonicDesired() = 0;

	virtual bool ReverseDriveDesired() = 0;

	virtual bool LeftDesired() = 0;

	virtual bool RightDesired() = 0;

	virtual bool RampDesired() = 0;

	virtual bool IntakeDesired() = 0;

	virtual bool OuttakeToPlatformDesired() = 0;

	virtual bool OuttakeToStepDesired() = 0;

	virtual bool MoveElevatorUpStartDesired() = 0;

	virtual bool MoveElevatorUpEndDesired() = 0;

	virtual bool MoveElevatorUpDesired() = 0;

	virtual bool MoveElevatorDownStartDesired() = 0;

	virtual bool MoveElevatorDownEndDesired() = 0;

	virtual bool MoveElevatorDownDesired() = 0;

	virtual double GetElevatorUpSpeedAdjuster() = 0;
	virtual double GetElevatorDownSpeedAdjuster() = 0;

	virtual bool BinsOutDesired() = 0;

	virtual ~RemoteController() {}
};

#endif
