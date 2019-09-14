/*
 *
 * Created on September 4 2015
 * by Poramate Manoonpong
 *
 *
 */

#ifndef CDBOTSENSMOTDEF
#define CDBOTSENSMOTDEF

enum cdbotSensorName
{

	//  \IR_LEFT2\   /IR_LEFT1/      \IR_RIGHT2\   /IR_RIGHT1/ //
	//														   //
	//														   //
	//													       //


	IR_RIGHTOUT = 0,
	IR_RIGHTIN = 1,
	IR_LEFTOUT = 2,
	IR_LEFTIN = 3,
	LIGHT_RIGHT = 4,
	LIGHT_LEFT = 5,
	SOUND_RIGHT = 6, // Not use
	SOUND_LEFT = 7, // Not use
	CDBOT_SENSOR_MAX=8

};

enum cdbotMotorName
{
	MOTOR_RIGHT = 0,
	MOTOR_LEFT = 1,
	CDBOT_MOTOR_MAX = 2

};


#endif
