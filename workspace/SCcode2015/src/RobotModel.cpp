#include "RobotModel.h"

#define PI 3.141592653589

#include "Debugging.h"

RobotModel::RobotModel() {
	pini = new Ini("/home/lvuser/robot.ini");

	pdp = new PowerDistributionPanel();

	frontLeft = new Victor(FRONT_LEFT_MOTOR_PWM_PORT);
	rearLeft = new Victor(REAR_LEFT_MOTOR_PWM_PORT);
	frontRight = new Victor(FRONT_RIGHT_MOTOR_PWM_PORT);
	rearRight = new Victor(REAR_RIGHT_MOTOR_PWM_PORT);
	elevatorMotor1 = new Victor(ELEVATOR_MOTOR_1_PWM_PORT);
	elevatorMotor2 = new Victor(ELEVATOR_MOTOR_2_PWM_PORT);
	leftRollerMotor = new Victor(LEFT_ROLLER_MOTOR_PWM_PORT);
	rightRollerMotor = new Victor(RIGHT_ROLLER_MOTOR_PWM_PORT);

	frontLeft->SetExpiration(0.5);
	rearLeft->SetExpiration(0.5);
	frontRight->SetExpiration(0.5);
	rearRight->SetExpiration(0.5);
	elevatorMotor1->SetExpiration(0.5);
	elevatorMotor2->SetExpiration(0.5);
	leftRollerMotor->SetExpiration(0.5);
	rightRollerMotor->SetExpiration(0.5);

	frontLeft->SetSafetyEnabled(false);
	rearLeft->SetSafetyEnabled(false);
	frontRight->SetSafetyEnabled(false);
	rearRight->SetSafetyEnabled(false);
	elevatorMotor1->SetSafetyEnabled(false);
	elevatorMotor2->SetSafetyEnabled(false);
	leftRollerMotor->SetSafetyEnabled(false);
	rightRollerMotor->SetSafetyEnabled(false);

	rampSolenoid = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, RAMP_SOLENOID_PORT);
	outtakeSolenoid = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, OUTTAKE_SOLENOID_PORT);
	brakeSolenoid = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, BRAKE_SOLENOID_PORT);
/*
	rollerArmSolenoid = new DoubleSolenoid(PNEUMATICS_CONTROL_MODULE_ID,ROLLER_ARM_SOLENOID_A_PORT,
										   ROLLER_ARM_SOLENOID_B_PORT);
										   */
	rollerArmSolenoid1 = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, ROLLER_ARM_SOLENOID_A_PORT);
	rollerArmSolenoid2 = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, ROLLER_ARM_SOLENOID_B_PORT);

	binSolenoid = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, BIN_SOLENOID_PORT);

	frontLeftEncoder = new Encoder(LEFT_ENCODER_A_PWM_PORT,
				LEFT_ENCODER_B_PWM_PORT, true);
	frontRightEncoder = new Encoder(RIGHT_ENCODER_A_PWM_PORT,
				RIGHT_ENCODER_B_PWM_PORT, true);
	rearLeftEncoder = new Encoder(REAR_LEFT_ENCODER_A_PORT, REAR_LEFT_ENCODER_B_PORT, true);
	rearRightEncoder = new Encoder(REAR_RIGHT_ENCODER_A_PORT, REAR_RIGHT_ENCODER_B_PORT, true);
	elevatorEncoder = new Encoder(ELEVATOR_ENCODER_A_PWM_PORT,
								  ELEVATOR_ENCODER_B_PWM_PORT, true);
	frontLeftEncoder->SetDistancePerPulse(-(PI / 2.0) / 256.0);
	// 6 inch wheels (1/2 ft), 256 tics per rotation
	frontRightEncoder->SetDistancePerPulse((PI / 2.0) / 256.0);
	rearLeftEncoder->SetDistancePerPulse(-(PI / 2.0) / 256.0);
	rearRightEncoder->SetDistancePerPulse((PI / 2.0) / 256.0);
	elevatorEncoder->SetDistancePerPulse((50.0 * PI * (18.0 / 60.0) / 25.4) / 128.0); // check math
	// 50mm diameter pulley, 18/60 = ratio of encoder to pulley turn, 25.4mm/in, 256 tics per rotation
	compressor = new Compressor(COMPRESSOR_PORT);

	gyro = new Gyro(GYRO_PORT);
	gyro->Reset();
	gyro->SetSensitivity(0.007);

	uint8_t update_rate_hz = 50;
	serialPort = new SerialPort(57600, SerialPort::kMXP);
	navx = new AHRS(serialPort, update_rate_hz);

	timer = new Timer();
	timer->Start();

	servo = new Servo(SERVO_PWM_PORT);

	ultrasonic = new UltrasonicSensor(ULTRASONIC_PORT);

	//hallEffectsSensor = new DigitalInput(HALL_EFFECTS_SENSOR_PWM_PORT);
	/*lightSensor = new DigitalInput(LIGHT_SENSOR_PWM_PORT);
	upperLimitSwitch = new DigitalInput(UPPER_LIMIT_SWITCH_PWM_PORT);*/
	elevatorLowerLimitSwitch = new DigitalInput(ELEVATOR_LOWER_LIMIT_SWITCH_PWM_PORT);

	isRecording = pini->getbool("PLAYBACKAUTO", "isRecording", true);
}

void RobotModel::SetWheelSpeed(Wheels w, double speed) {
/*
 * This is given that we will need to negate the speed for the two different sides.
 */

	switch (w) {
	case (kFrontLeftWheel):
#if COMP_BOT
		frontLeft->Set(speed);
#else
		frontLeft->Set(-speed);
#endif
		//SmartDashboard::PutNumber("frontLeft victor",
		//frontLeft->Get());
		break;
	case (kRearLeftWheel):
#if COMP_BOT
		rearLeft->Set(speed);
#else
		rearLeft->Set(-speed);
#endif
		//SmartDashboard::PutNumber("rearLeft victor", rearLeft->Get());
		break;
	case (kFrontRightWheel):
#if COMP_BOT
		frontRight->Set(-speed);
#else
		frontRight->Set(speed);
#endif
		//SmartDashboard::PutNumber("frontRight victor", frontRight->Get());
		break;
	case (kRearRightWheel):
#if COMP_BOT
		rearRight->Set(-speed);
#else
		rearRight->Set(speed);
#endif
		//SmartDashboard::PutNumber("rearRight victor", rearRight->Get());
		break;
	case (kAllWheels):
		frontLeft->Set(speed);
		rearLeft->Set(speed);
		frontRight->Set(speed);
		rearRight->Set(speed);
		break;
	}
}

float RobotModel::GetWheelSpeed(Wheels w) {
	switch(w) {
	case (kFrontLeftWheel):
		return frontLeft->Get();
		break;
	case (kRearLeftWheel):
		return rearLeft->Get();
		break;
	case (kFrontRightWheel):
		return frontRight->Get();
		break;
	case (kRearRightWheel):
		return rearRight->Get();
		break;
	}
	return false;
}

void RobotModel::SetElevatorSpeed(double speed) {
	elevatorMotor1->Set(-speed);
	elevatorMotor2->Set(speed);
}

void RobotModel::SetRollerSpeed(double speed) {
	leftRollerMotor->Set(speed);
	rightRollerMotor->Set(-speed);
}

void RobotModel::SetLeftRollerSpeed(double speed) {
	leftRollerMotor->Set(speed);
}

void RobotModel::SetRightRollerSpeed(double speed) {
	rightRollerMotor->Set(speed);
}

double RobotModel::GetVoltage() {
	return pdp->GetVoltage();
}

void RobotModel::EnableCompressor() {
	compressor->Start();
}

void RobotModel::DisableCompressor() {
	compressor->Stop();
}

void RobotModel::ResetCompressor() {
	compressor->Stop();
	compressor->Start();
}

bool RobotModel::GetCompressorState() {
	return (compressor->Enabled());
}

void RobotModel::ResetGyro() {
	gyro->Reset();
}

void RobotModel::ResetTimer() {
	timer->Reset();
}

void RobotModel::ResetNavxRotation() {
	//navx->ZeroYaw();
}

void RobotModel::ResetNavxDisplacement() {
	//navx->ResetDisplacement();
}

float RobotModel::GetGyroAngle() {
	return gyro->GetAngle();
}

float RobotModel::GetNavxAngle() {
	return navx->GetYaw();
	//return 0.0;
}

float RobotModel::GetNavxX() {
	//return navx->GetDisplacementX();
	return 0.0;
}

float RobotModel::GetNavxY() {
	//return navx->GetDisplacementY();
	return 0.0;
}

double RobotModel::GetXEncoderVal() {
//#if COMP_BOT
	double fleft = GetFrontLeftEncoderVal();
	double fright = -GetFrontRightEncoderVal();
	double rleft = -GetRearLeftEncoderVal();
	double rright = GetRearRightEncoderVal();
//#else
	//SmartDashboard::PutNumber("left encoder", frontLeftEncoder->GetDistance());
	//SmartDashboard::PutNumber("right encoder", frontRightEncoder->GetDistance());
	//double left = leftEncoder->GetDistance();
	//double right = -rightEncoder->GetDistance();
//#endif
	double val = (fleft + fright + rleft + rright)/((double)4.0);
	return val;
}

double RobotModel::GetYEncoderVal() {
//#if COMP_BOT
	double fleft = GetFrontLeftEncoderVal();
	double fright = GetFrontRightEncoderVal();
	double rleft = GetRearLeftEncoderVal();
	double rright = GetRearRightEncoderVal();
//#else
	//double left = -leftEncoder->GetDistance();
	//double right = -rightEncoder->GetDistance();
//#endif
	double val = (fleft + fright + rleft + rright)/((double)4.0);
	return val;
}

double RobotModel::GetFrontLeftEncoderVal() {
	// from comp? return frontLeftEncoder->GetDistance();
	return frontLeftEncoder->GetDistance();
}

double RobotModel::GetFrontRightEncoderVal() {
	// from comp? return frontRightEncoder->GetDistance();
	return frontRightEncoder->GetDistance();
	//return -1.0 * frontRightEncoder->GetDistance();
}

double RobotModel::GetRearLeftEncoderVal() {
	return -rearLeftEncoder->GetDistance();
}

double RobotModel::GetRearRightEncoderVal() {
	return -rearRightEncoder->GetDistance();
}

double RobotModel::GetElevatorEncoderVal() {
	return -elevatorEncoder->GetDistance();
}

void RobotModel::ResetDriveEncoders() {
	frontLeftEncoder->Reset();
	frontRightEncoder->Reset();
	rearLeftEncoder->Reset();
	rearRightEncoder->Reset();

}

void RobotModel::ResetElevatorEncoder() {
	elevatorEncoder->Reset();
}

bool RobotModel::GetLightSensorVal() {
	//return lightSensor->Get();
	return false;
}

bool RobotModel::GetElevatorUpperLimitSwitchVal() {
	//return upperLimitSwitch->Get();
	return false;
}

bool RobotModel::GetElevatorLowerLimitSwitchVal() {
	return elevatorLowerLimitSwitch->Get();
}

bool RobotModel::GetHallEffectsSensorVal() {
	return !hallEffectsSensor->Get();
}

void RobotModel::SetServoAngle(float angle){
	servo->SetAngle(angle);
}

float RobotModel::GetServoAngle() {
	return servo->GetAngle();
}

void RobotModel::ResetServoAngle() {
	servo->SetAngle(0.0);
}

float RobotModel::GetUltrasonicDistance() {
	return ultrasonic->GetRangeInInches();
}

void RobotModel::MoveRampIn() {
	rampSolenoid->Set(false);
}

void RobotModel::MoveRampOut() {
	rampSolenoid->Set(true);
}

bool RobotModel::GetRampPos() {
	return rampSolenoid->Get();
}

void RobotModel::MoveRollerArmsIn() {
	/*
	rollerArmSolenoid->Set(DoubleSolenoid::kForward);
	*/
	rollerArmSolenoid1->Set(true);
	rollerArmSolenoid2->Set(false);
}

void RobotModel::MoveRollerArmsOut() {
	/*
	rollerArmSolenoid->Set(DoubleSolenoid::kReverse);
	*/
	rollerArmSolenoid1->Set(false);
	rollerArmSolenoid2->Set(true);
}

void RobotModel::DisengageRollerArms() {
	//rollerArmSolenoid->Set(DoubleSolenoid::kOff);
}

int RobotModel::GetRollerArmPos() {
	//return rollerArmSolenoid->Get();
	if (rollerArmSolenoid1->Get() && !rollerArmSolenoid2->Get()) {
		return 1;
	} else {
		return 2;
	}
}

void RobotModel::MoveOuttakeIn() {
	outtakeSolenoid->Set(false);
}

void RobotModel::MoveOuttakeOut() {
	outtakeSolenoid->Set(true);
}

bool RobotModel::GetOuttakePos() {
	return outtakeSolenoid->Get();
}

void RobotModel::EngageBrake() {
	brakeSolenoid->Set(false);
}

void RobotModel::DisengageBrake() {
	brakeSolenoid->Set(true);
}

void RobotModel::BinIn() {
	binSolenoid->Set(true);
}

void RobotModel::BinOut() {
	binSolenoid->Set(false);
}

bool RobotModel::GetBinIn() {
	return binSolenoid->Get();
}

bool RobotModel::GetIsRecording() {
	return isRecording;
}

void RobotModel::RefreshIniFile() {
	delete pini;
	pini = new Ini("/home/lvuser/robot.ini");
}
