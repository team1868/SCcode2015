#ifndef ROBOTPORTS2015_H
#define ROBOTPORTS2015_H
#include "Debugging.h"

//********PWM PORTS***********************************
#if COMP_BOT
static const int FRONT_LEFT_MOTOR_PWM_PORT 			= 7;
static const int REAR_LEFT_MOTOR_PWM_PORT			= 4;
static const int FRONT_RIGHT_MOTOR_PWM_PORT			= 0;
static const int REAR_RIGHT_MOTOR_PWM_PORT			= 3;
static const int ELEVATOR_MOTOR_2_PWM_PORT			= 5;
#else
static const int FRONT_LEFT_MOTOR_PWM_PORT 			= 4;
static const int REAR_LEFT_MOTOR_PWM_PORT			= 5;
static const int FRONT_RIGHT_MOTOR_PWM_PORT			= 7;
static const int REAR_RIGHT_MOTOR_PWM_PORT			= 8;
static const int ELEVATOR_MOTOR_2_PWM_PORT			= 0;
// https://code.google.com/p/navx-mxp/wiki/MXPIOExpansion documentation wrong
#endif
static const int SERVO_PWM_PORT						= 6;
static const int ELEVATOR_MOTOR_1_PWM_PORT			= 2;
static const int LEFT_ROLLER_MOTOR_PWM_PORT			= 9;
static const int RIGHT_ROLLER_MOTOR_PWM_PORT		= 1;

//***********DIGITAL I/O PORTS************************
#if COMP_BOT

static const int LEFT_ENCODER_A_PWM_PORT 			= 1;
static const int LEFT_ENCODER_B_PWM_PORT			= 0;
//FROM COMP BOT FOR REAL!!
static const int RIGHT_ENCODER_A_PWM_PORT			= 3;
static const int RIGHT_ENCODER_B_PWM_PORT			= 2;

static const int REAR_LEFT_ENCODER_A_PORT			= 8;
static const int REAR_LEFT_ENCODER_B_PORT			= 9;

static const int REAR_RIGHT_ENCODER_A_PORT			= 6;
static const int REAR_RIGHT_ENCODER_B_PORT			= 7;

/*
static const int LEFT_ENCODER_A_PWM_PORT 			= 3;
static const int LEFT_ENCODER_B_PWM_PORT			= 2;

static const int RIGHT_ENCODER_A_PWM_PORT			= 0;
static const int RIGHT_ENCODER_B_PWM_PORT			= 1;

static const int REAR_LEFT_ENCODER_A_PORT			= 6;
static const int REAR_LEFT_ENCODER_B_PORT			= 7;

static const int REAR_RIGHT_ENCODER_A_PORT			= 8;
static const int REAR_RIGHT_ENCODER_B_PORT			= 9;
*/
#else
static const int LEFT_ENCODER_A_PWM_PORT			= 18;
static const int LEFT_ENCODER_B_PWM_PORT			= 19;
static const int RIGHT_ENCODER_A_PWM_PORT			= 20;
static const int RIGHT_ENCODER_B_PWM_PORT			= 21;
#endif
static const int ELEVATOR_ENCODER_A_PWM_PORT 		= 4;
static const int ELEVATOR_ENCODER_B_PWM_PORT 		= 5;
//static const int HALL_EFFECTS_SENSOR_PWM_PORT		= 9;
// static const int LIGHT_SENSOR_PWM_PORT 			= 11;
// static const int UPPER_LIMIT_SWITCH_PWM_PORT		= 11;
static const int ELEVATOR_LOWER_LIMIT_SWITCH_PWM_PORT=11;

//**************MISC**********************************

static const int COMPRESSOR_PORT					= 1;
static const int PNEUMATICS_CONTROL_MODULE_ID		= 0;
static const int GYRO_PORT 							= 0;
static const int ULTRASONIC_PORT					= 2;
static const int ELEVATOR_UP_ADJUST_SPEED_AXIS		= 3;
static const int ELEVATOR_DOWN_ADJUST_SPEED_AXIS	= 2;

//*********SOLENOID PORTS****************************

static const int RAMP_SOLENOID_PORT					= 5;
static const int ROLLER_ARM_SOLENOID_A_PORT			= 0;
static const int ROLLER_ARM_SOLENOID_B_PORT			= 1;
static const int OUTTAKE_SOLENOID_PORT				= 4;
static const int BRAKE_SOLENOID_PORT				= 7;
static const int BIN_SOLENOID_PORT					= 6;

//**********JOYSTICK USB PORTS************************

static const int LEFT_JOY_USB_PORT						= 0;
static const int RIGHT_JOY_USB_PORT						= 1;
static const int OPERATOR_JOY_USB_PORT					= 2;
static const int INTAKE_JOY_USB_PORT					= 3;

//***************BUTTON PORTS****************
static const int FIELD_ROBOT_BUTTON_PORT				= 3;
static const int JOY_RAMP_BUTTON_PORT					= 3;
static const int REVERSE_DRIVE_BUTTON_PORT				= 12;
static const int LEFT_BUTTON_PORT						= 1;
static const int RIGHT_BUTTON_PORT						= 2;
static const int RAMP_BUTTON_PORT						= 2;
static const int INTAKE_BUTTON_PORT						= 7;
static const int OUTTAKE_TO_PLATFORM_BUTTON_PORT		= 9;
static const int OUTTAKE_TO_STEP_BUTTON_PORT			= 8;
static const int MOVE_ELEVATOR_UP_BUTTON_PORT			= 4;
static const int MOVE_ELEVATOR_DOWN_BUTTON_PORT			= 3;
static const int BINS_OUT_BUTTON_PORT					= 5;
static const int SECOND_INTAKE_BUTTON_PORT				= 1;
//static const int INTAKE_SEQUENCE_BUTTON_PORT			= 1;
//static const int ULTRASONIC_BUTTON_PORT					= 19;

//operator

//left

//right

#endif
