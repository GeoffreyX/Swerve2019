/**
 *  Const.h
 *  Date: 5/21/19
 *  Last Edited By: Geoffrey Xue
 *  Huge credit to team 2910 for the entirety of this code system
 */


#ifndef SRC_CONST_H_
#define SRC_CONST_H_


extern bool Debug;                  // Set to true to enable additional debugging

// Robot
#define WHEELBASE 15
#define TRACKWIDTH 15

// OperatorInputs
//   Controllers
#define INP_DUAL 1
#define INP_JOYSTICK -1
#define INP_XBOX_1 0
#define INP_XBOX_2 1
//   Set to 1.0 or -1.0
#define INVERT_Y_AXIS 1.0
#define INVERT_X_AXIS -1.0
//   XBox Controller Buttons
#define A_BUTTON  1
#define B_BUTTON  2
#define X_BUTTON  3
#define Y_BUTTON  4
#define LEFT_BUMPER  5
#define RIGHT_BUMPER  6
#define BACK_BUTTON  7
#define START_BUTTON  8
#define L3_BUTTON 9
#define R3_BUTTON 10
//   XBox Triggers -- Changed for 2016, previously XBOX triggers were both on a single axis
#define XBOX_LEFT_TRIGGER_AXIS  12
#define XBOX_RIGHT_TRIGGER_AXIS  13
#define LEFT_TRIGGER_MIN  0.5
#define LEFT_TRIGGER_MAX  1.0
#define RIGHT_TRIGGER_MIN  0.5
#define RIGHT_TRIGGER_MAX  1.0
#define JOYSTICK_X_AXIS  0
#define JOYSTICK_Y_AXIS  1
#define AXIS0_LEFT_MIN -1.0
#define AXIS0_LEFT_MAX -0.75
#define AXIS0_RIGHT_MIN 0.75
#define AXIS0_RIGHT_MAX 1.0
#define AXIS1_BACK_MIN -1.0
#define AXIS1_BACK_MAX -0.75
#define AXIS1_FORWARD_MIN 0.75
#define AXIS1_FORWARD_MAX 1.0
//	 Controller Dead Zones
#define DEADZONE_Y  0.18
#define DEADZONE_X  0.18
#define DEADZONE_Z  0.18


// Drivetrain
//   Talons ports
#define CAN_FRONT_LEFT_MOV 1
#define CAN_FRONT_LEFT_ANG 2

#define CAN_FRONT_RIGHT_MOV 3
#define CAN_FRONT_RIGHT_ANG 4

#define CAN_BACK_LEFT_MOV 5
#define CAN_BACK_LEFT_ANG 6

#define CAN_BACK_RIGHT_MOV 7
#define CAN_BACK_RIGHT_ANG 8

//   Talon parameters
#define MOTOR_DRIVE_CURRENT_LIMIT 25
#define MOTOR_ANGLE_CURRENT_LIMIT 30
#define MOTOR_CURRENT_DURATION 100

//   Encoders
#define ENC_TYPE FeedbackDevice::QuadEncoder

#define TICKS_PER_REV 1382.0
#define TICKS_PER_INCH 73.317
#define WHEEL_DIAMETER 6.0
#define WHEEL_TRACK 23.50


// Compressor
#define PCM_COMPRESSOR_SOLENOID 0
#define CAN_POWER_DISTRIBUTION_PANEL 0
#define PNE_CURRENT_DRAW 80
#define PNE_VOLTAGE_DROP 10
#define PNE_WAITTIME 1.0


// Gyro
#define CAN_GYRO1 0
#define CAN_GYRO2 -1


// Math
#define PI 3.141592

#endif /* SRC_CONST_H_ */