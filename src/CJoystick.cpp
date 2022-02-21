#include "CJoystick.h"

CJoystick::CJoystick(){a=b=x=y=lb=fl=false,cOp=1,JYaw=JGaz=JRoll=JPitch=0;}

CJoystick::~CJoystick(){}

void CJoystick::chatterCallback(const sensor_msgs::Joy& msg)
{

//Assign Buttons
  a = msg.buttons[0]; //button a Takeoff
  b = msg.buttons[1]; //button b Emergency
  x = msg.buttons[2]; //button x Option
  y = msg.buttons[3]; //button y Landing
//  lb = msg.buttons[4]; //button lb
  b5 = msg.buttons[4];
  b6 = msg.buttons[5];
  b7 = msg.buttons[6];
  b8 = msg.buttons[7];

//Assign sticks
// this-> JYaw = floor(msg.axes[0]*10)/10; //Left/Right Axis stick left
// this-> JGaz = floor((msg.axes[1]*10)/2)/10; //Up/Down Axis stick left
// this-> JRoll = floor((msg.axes[3]*10)/4)/10; //Left/Right Axis stick right
// this-> JPitch = floor((msg.axes[4]*10)/4)/10;//Up/Down Axis stick right
 this-> JYaw = msg.axes[0]; //Left/Right Axis stick left
 this-> JGaz = msg.axes[1]; //Up/Down Axis stick left
 this-> JRoll = msg.axes[3]; //Left/Right Axis stick right
 this-> JPitch = msg.axes[2];//Up/Down Axis stick right
}

//function to change case for different flight modes
void CJoystick::ComandoControl()
{
 if(x==true){
if(x==true && fl==false){
	if(cOp>=3){
	 cOp=0;	
	}
	cOp=cOp+1;
	fl=true;
}
}
if(x==false){
 fl=false;
}
}

/*------------------publish -------------*/
void CJoystick::publishMessage(ros::Publisher *const x)
{
	std_msgs::Empty msg;  
	x->publish(msg);		
}
/*------------------finish publishing-------------*/
