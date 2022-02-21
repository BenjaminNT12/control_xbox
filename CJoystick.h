#ifndef CJoystick_H
#define CJoystick_H

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

using std::string;

class CJoystick
{
public:
	CJoystick();  //Contrusctor
	~CJoystick();  //Destructor
	void chatterCallback(const sensor_msgs::Joy& msg); //suscribe to function of the joystick
	void publishMessage(ros::Publisher *const); //publish information to other topic
	void ComandoControl();	

//-------------------Botones-----------------------------//
//private:
	bool a;
	bool b;
	bool x;
	bool y;
	bool lb;
	bool fl;
        bool b5;
        bool b6;
        bool b7;
        bool b8;
	int cOp;

	float JYaw;
	float JGaz;
	float JRoll;
	float JPitch;
};
#endif

