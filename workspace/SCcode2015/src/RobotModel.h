#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include "WPILib.h"
#include "RobotPorts2015.h"
#include "ini.h"
#include "Debugging.h"
#include "UltrasonicSensor.h"
#include "navx/AHRS.h"
#include <iostream>
#include <fstream>

class RobotModel {
public:
	enum Wheels {kFrontLeftWheel, kRearLeftWheel, kFrontRightWheel, kRearRightWheel,
				 kAllWheels};

	RobotModel();
	~RobotModel() {}
	void SetWheelSpeed(Wheels w, double speed);
	float GetWheelSpeed(Wheels w);
	void SetElevatorSpeed(double speed);
	void SetRollerSpeed(double speed);
	void SetLeftRollerSpeed(double speed);
	void SetRightRollerSpeed(double speed);

	double GetVoltage();

	void EnableCompressor();
	void DisableCompressor();
	void ResetCompressor();
	bool GetCompressorState();

	double GetXEncoderVal(); // for omni wheels
	double GetYEncoderVal();
	double GetFrontLeftEncoderVal();
	double GetFrontRightEncoderVal();
	double GetRearLeftEncoderVal();
	double GetRearRightEncoderVal();
	double GetElevatorEncoderVal();
	void ResetDriveEncoders();
	void ResetElevatorEncoder();

	void RefreshIniFile();

	bool GetLightSensorVal();
	bool GetElevatorUpperLimitSwitchVal();
	bool GetElevatorLowerLimitSwitchVal();
	bool GetHallEffectsSensorVal();

	void ResetGyro();
	float GetGyroAngle();

	void ResetNavxRotation();
	void ResetNavxDisplacement();
	float GetNavxAngle();
	float GetNavxX();
	float GetNavxY();

	void ResetTimer();

	void SetServoAngle(float angle);
	float GetServoAngle();
	void ResetServoAngle();

	void MoveRampOut();
	void MoveRampIn();
	bool GetRampPos();
	void MoveRollerArmsIn();
	void MoveRollerArmsOut();
	void DisengageRollerArms();
	int GetRollerArmPos();
	void MoveOuttakeIn();
	void MoveOuttakeOut();
	bool GetOuttakePos();
	void EngageBrake();
	void DisengageBrake();

	void BinIn();
	void BinOut();
	bool GetBinIn();

	float GetUltrasonicDistance();

	bool GetIsRecording();
;

	Timer *timer;
	Ini* pini;

	UltrasonicSensor* ultrasonic;

	Solenoid *rampSolenoid, *outtakeSolenoid, *brakeSolenoid, *binSolenoid;
	/*
	DoubleSolenoid *rollerArmSolenoid;
	*/
	Solenoid* rollerArmSolenoid1, *rollerArmSolenoid2;
	PowerDistributionPanel* pdp;
private:

	Compressor *compressor;
	//Actuators
	Victor *frontLeft, *rearLeft, *frontRight, *rearRight, *elevatorMotor1, *elevatorMotor2,
		  *leftRollerMotor, *rightRollerMotor;
	Servo *servo;

	//Sensors
	Encoder *frontLeftEncoder, *rearLeftEncoder, *frontRightEncoder, *rearRightEncoder, *elevatorEncoder;
	Gyro *gyro;
	AHRS *navx;
	SerialPort *serialPort;
	DigitalInput *hallEffectsSensor, *elevatorLowerLimitSwitch; /*lightSensor, *upperLimitSwitch;*/
	bool isRecording; // for playback auto testing. by competition this should not exist
	// probably should be in the ini file actually
};

#endif
