#ifndef ROBOTPORTS2014_H
#define ROBOTPORTS2014_H

//********PWM PORTS***********************************
static const int CATAPULT_MOTOR_A_PWM_PORT			= 8;
static const int CATAPULT_MOTOR_B_PWM_PORT			= 9;
static const int INTAKE_MOTOR_A_PWM_PORT			= 10;
static const int INTAKE_MOTOR_B_PWM_PORT			= 5;

static const int LEFT_MOTOR_A_PWM_PORT				= 3;
static const int LEFT_MOTOR_B_PWM_PORT				= 4;
static const int RIGHT_MOTOR_A_PWM_PORT				= 1;
static const int RIGHT_MOTOR_B_PWM_PORT				= 2;
//***********DIGITAL I/O PORTS************************
static const int GYRO_PORT 							= 1;

static const int RIGHT_WHEEL_ENCODER_A_PWM_PORT		= 6;
static const int RIGHT_WHEEL_ENCODER_B_PWM_PORT		= 7;
static const int LEFT_WHEEL_ENCODER_A_PWM_PORT		= 2;
static const int LEFT_WHEEL_ENCODER_B_PWM_PORT		= 3;
static const int CATAPULT_ENCODER_A_PWM_PORT		= 9;
static const int CATAPULT_ENCODER_B_PWM_PORT		= 10;

static const int LIMIT_SWITCH_PWM_PORT				= 8;
static const int OPTICAL_SENSOR_PWM_PORT			= 12;

//**************MISC**********************************
static const int COMPRESSOR_PRESSURE_SWITCH_CHAN 	= 1;
static const int COMPRESSOR_RELAY_CHAN 			 	= 1;
//*********SOLENOID PORTS****************************
static const int SHIFTER_SOLENOID_CHAN          	= 5;
static const int CATAPULT_SHIFTER_SOLENOID_CHAN		= 6;
static const int LATCH_SOLENOID_CHAN				= 7;
static const int POP_BALL_SOLENOID_CHAN				= 8;
static const int INTAKE_UPPER_ARM_SOLENOID_CHAN_A	= 1;
static const int INTAKE_UPPER_ARM_SOLENOID_CHAN_B	= 2;
static const int INTAKE_LOWER_ARM_SOLENOID_CHAN_A	= 3;
static const int INTAKE_LOWER_ARM_SOLENOID_CHAN_B	= 4;
//**********JOYSTICK USB PORTS************************

static const int LEFT_JOY_USB_PORT 					= 1;
static const int RIGHT_JOY_USB_PORT 				= 2;
static const int OPERATOR_JOY_USB_PORT 				= 3;

//***************BUTTON PORTS****************

//operator
static const int OPERATOR_JOY_INTAKE_BACKWARD_PORT	= 3; // intake motor, ball eject
static const int OPERATOR_JOY_INTAKE_FORWARD_PORT	= 4; // intake motor, ball intake
static const int OPERATOR_JOY_ARMING_PORT			= 2; // drive station: "RETRACT"
static const int OPERATOR_JOY_SHOOTER_PORT			= 9; // drive station: "FIRE"
static const int OPERATOR_JOY_POP_BALL_PORT			= 6;
static const int OPERATOR_JOY_INTAKE_START_PORT		= 8; // drive station: "ARM"
static const int OPERATOR_JOY_INTAKE_SHOOT_PORT		= 7; // drive station: "SAFE"
static const int OPERATOR_JOY_INTAKING_PORT			= 1; // drive station: "PICK UP"
static const int OPERATOR_JOY_HOLD_BALL_PORT		= 5; // drive station: "HOLD BALL"
static const int OPERATOR_JOY_REVERSE_DRIVE_PORT	= 12;// drive station: near joysticks, reverse drive motors

//right
static const int RIGHT_JOY_QUICK_TURN_PORT			= 1;
static const int RIGHT_JOY_HIGH_LOW_GEAR_PORT		= 3;
static const int LEFT_JOY_AUTO_MAN_GEAR_PORT		= 3;

//left
static const int LEFT_JOY_CAMERA_PORT				= 2;
static const int LEFT_JOY_AUTO_DRIVE_PORT			= 1;
#endif
