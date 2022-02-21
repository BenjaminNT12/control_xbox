#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h" 
#include "geometry_msgs/PoseStamped.h"
#include "bebop_msgs/CommonCommonStateBatteryStateChanged.h"
#include "CJoystick.h"
#include "CJoystick.cpp"
#include "CLevant.h"
#include "CLevant.cpp"
#include <sstream>
#include <iostream>
#include <iomanip> 
#include <fstream>
#include <tf/transform_datatypes.h>

#define M_PI  3.14159265358979323846

using namespace std;

float saturacion(float a);
float saturacion_Z(float a);

double roll, pitch;

// POSICIONES UAV1
double x1_UAV,y1_UAV,z1_UAV,yaw1_UAV,yaw1_UAV_rad;
// POSICIONES UAV2
double x2_UAV,y2_UAV,z2_UAV,yaw2_UAV,yaw2_UAV_rad;

// NIVEL DE BATERIA DE LOS UAV
int batpor_UAV1, batpor_UAV2,batpor_UAV3,batpor_UAV4;
					  
// NIVEL DE BATERIA DEL UAV1
void bat_UAV1(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr& msg)
{
	batpor_UAV1 = msg->percent;
}		
// NIVEL DE BATERIA DEL UAV2	
void bat_UAV2(const bebop_msgs::CommonCommonStateBatteryStateChanged::ConstPtr& msg)
{
	batpor_UAV2 = msg->percent;
}

// LEER DATOS DE POSICION (OPTITRACK) DEL VEHICULO 1
void poseCallback_UAV1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw1_UAV_rad);
  yaw1_UAV = yaw1_UAV_rad*(180/M_PI);
  x1_UAV = msg->pose.position.x*1000;
  y1_UAV = msg->pose.position.y*1000;
  z1_UAV = msg->pose.position.z*1000;
}

// LEER DATOS DE POSICION (OPTITRACK) DEL VEHICULO 1
void poseCallback_UAV2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Quaternion q(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw2_UAV_rad);
  yaw2_UAV = yaw2_UAV_rad*(180/M_PI);
  x2_UAV = msg->pose.position.x*1000;
  y2_UAV = msg->pose.position.y*1000;
  z2_UAV = msg->pose.position.z*1000;
}

int main(int argc, char **argv)
	{
		// VARIABLES DEL JOYSTICK, CONTROL MANUAL
		float axis_X,axis_Y,axis_yaw;
		// BOTONES PARA DESPEGUE, ATERRIZAJE Y ACTIVACION DEL CONTROL
		int bot_toff,bot_land,boy_cyaw_on,boy_cyaw_off;
  
		// SENALES DE REFEENCIA
		float ref_X1 = 0, ref_Y1 = 0, ref_X2 = 0, ref_Y2 = 0, ref_Z1 = 1000, ref_Z2 = 1000;
  
		// VARIABLES DE CONTROL UAV1
		float control_X1 = 0, control_X1_rot = 0, control_Y1 = 0, control_Y1_rot = 0, control_Z1 = 0, control_yaw1 = 0;
		// VARIABLES DE CONTROL UAV2
		float control_X2 = 0, control_X2_rot = 0, control_Y2 = 0, control_Y2_rot = 0, control_Z2 = 0, control_yaw2 = 0;
  
		// VARIABLES VELOCIDAD UAV 1
		float x1_vel,y1_vel,z1_vel,yaw1_vel;
		// VARIABLES VELOCIDAD UAV 2
		float x2_vel,y2_vel,z2_vel,yaw2_vel;

		// GANANCIAS DE LOS CONTROLADORES (YAW)
		float kp_yaw = 0.02, kd_yaw=0.005;
		// GANANCIAS DE LOS CONTROLADORES (Z)
		float kp_Z = 0.0015, kd_Z=0.0001;
		// GANANCIAS DE LOS CONTROLADORES (Y)
		//float kp_Y = 0.0005, kd_Y=0.0006;
		float kp_Y = 0.00048, kd_Y=0.0006;
		// GANANCIAS DE LOS CONTROLADORES (Y)
		//float kp_X = 0.0005, kd_X=0.0006;  
		float kp_X = 0.00048, kd_X=0.0006;  

		// BANDERAS
		int b1=0,b2=0;
		// BANDERAS PARA DESPEGUE Y ATERRIZAJE
		char cmd[50],aa,pos;
		aa = '0';
		pos = 'l';
 
		// VARIABLE DE ARCHIVO		
		int archivo;
		ifstream last_file;
		last_file.open ("src/controllers/src/datos/last.txt");
		last_file >> archivo;
		last_file.close();	
		
		char str1[80],str2[80];		
		snprintf (str1,80,"src/controllers/src/datos/datos%d_UAV1.txt",archivo);
		snprintf (str2,80,"src/controllers/src/datos/datos%d_UAV2.txt",archivo);
		
		ofstream last_file1;
		last_file1.open ("src/controllers/src/datos/last.txt");		
		archivo++;
		last_file1 << archivo;
		last_file1.close();
								
		ofstream myfile_UAV1,myfile_UAV2; 
		myfile_UAV1.open (str1); // ARCHIVO DONDE SE GUARDAN LOS DATOS DEL UAV1
		myfile_UAV2.open (str2); // ARCHIVO DONDE SE GUARDAN LOS DATOS DEL UAV2
	
		CJoystick *J = new CJoystick();  // OBJETO DEL JOYSTICK
		CLevant Levant; // VARIABLE PARA EL DIFERENCIADOR DE LEVANT

		// INICIALIZACION DE ROS
		ros::init(argc, argv, "cuadro2UAVs");
		ros::NodeHandle n;
       
		// PUBLISHER FOR BEBOP1 IP 192.168.1.5 = UAV_IP5 = UAV 1
		ros::Publisher takeOff_b1 = n.advertise<std_msgs::Empty>("/bebop_IP5/takeoff",1000);
		ros::Publisher land_b1 = n.advertise<std_msgs::Empty>("/bebop_IP5/land",1000);
		ros::Publisher control_u_b1 = n.advertise<geometry_msgs::Twist>("/bebop_IP5/cmd_vel",2);

		// PUBLISHER FOR BEBOP2 IP 192.168.1.6 = UAV_IP6 = UAV 2
		ros::Publisher takeOff_b2 = n.advertise<std_msgs::Empty>("/bebop_IP6/takeoff",1000);
		ros::Publisher land_b2 = n.advertise<std_msgs::Empty>("/bebop_IP6/land",1000);
		ros::Publisher control_u_b2 = n.advertise<geometry_msgs::Twist>("/bebop_IP6/cmd_vel",2);

		// SUSCRIPTOR PARA EL JOYSTICK
		ros::Subscriber Joystick = n.subscribe("joy", 1000, &CJoystick::chatterCallback, J); //read Joystick
  
		// SUSCRIPTOR PARA LOS UAV
		ros::Subscriber sub_UAV1 = n.subscribe("/UAV_IP5/pose",1000, poseCallback_UAV1); // SUSCRIPTOR UAV1
		ros::Subscriber sub_UAV2 = n.subscribe("/UAV_IP6/pose",1000, poseCallback_UAV2); // SUSCRIPTOR UAV2		
		
		// SUSCRIPTOR PARA OBTENER LOS NIVELES DE BATERIA DE LOS UAVs
		ros::Subscriber bateria_UAV1 = n.subscribe("/bebop_IP5/states/common/CommonState/BatteryStateChanged",1000, bat_UAV1); // BATERIA UAV1
		ros::Subscriber bateria_UAV2 = n.subscribe("/bebop_IP6/states/common/CommonState/BatteryStateChanged",1000, bat_UAV2); // BATERIA UAV2				
    
		// DECLARAR MENSAJES PARA DESPEGAR, ATERRIZAR Y CONTROL
		std_msgs::Empty msg_takeoff_b1,msg_takeoff_b2;
		std_msgs::Empty msg_land_b1,msg_land_b2;
		geometry_msgs::Twist msg_control1, msg_control2;
  
		// FRECUENCIA DE EJECUCION DEL CICLO PRINCIPAL (T = 0.02 SEG)
		ros::Rate loop_rate(50);
		float T = 0.02;
		
		// VARIABLES PARA TIMERS		
		float tiempo = 0;
		float timer_p1 = 0;

		int count = 0;
		cout.precision(2);
		while (ros::ok())
			{		
				ROS_INFO("UAV1: %d, UAV2: %d",batpor_UAV1,batpor_UAV2);						
				
				// LEER JOYSTICK
				axis_X = J->JYaw/2;
				axis_Y = J->JGaz/2;
				axis_yaw = J->JPitch/2;
				bot_toff = J->a; // Takeoff Boton 1
				bot_land = J->x; // Landing Boton 3 
				
				// OBTENER VELOCIDADES UAV 1
				x1_vel = Levant.x1_vel(x1_UAV);		
				y1_vel = Levant.y1_vel(y1_UAV);	
				z1_vel = Levant.z1_vel(z1_UAV);	
				yaw1_vel = Levant.yaw1_vel(yaw1_UAV);		
				
				// OBTENER VELOCIDADES UAV 2
				x2_vel = Levant.x2_vel(x2_UAV);		
				y2_vel = Levant.y2_vel(y2_UAV);	
				z2_vel = Levant.z2_vel(z2_UAV);	
				yaw2_vel = Levant.yaw2_vel(yaw2_UAV);						
    		    
				if ( bot_toff == 1 ) // DESPEGAR LOS VEHICULOS
					{
						std::cout << "Despegar \n";
						takeOff_b1.publish(msg_takeoff_b1);
						takeOff_b2.publish(msg_takeoff_b2);
						pos = 't';
						b1 = 1;
					}
      	
				if ( bot_land == 1) // ATERRIZAR LOS VEHICULOS
					{
						std::cout << "Aterrizar \n";
						land_b1.publish(msg_land_b1);
						land_b2.publish(msg_land_b2);
						timer_p1 = 0;
						b1 = 0;
						pos = 'l';
					}

				if (pos == 't') // VOLANDO, ENVIAR LAS SENALES DE CONTROL
					{	   
						if (boy_cyaw_on == 1)
							b1 = 1;
					
						if (boy_cyaw_off == 1)
							{ 
								b1 = 0;
								timer_p1 = 0;
							}	

						if (b1 == 1)  
							{       				  								
								if (timer_p1 < 8)
									{
										// O1(500,0)
										ref_X1 = 750;
										ref_Y1 = 0;											
										// O2(-500,0)
										ref_X2 = -750;
										ref_Y2 = 0;
									}
								else if ( (timer_p1 > 8) && (timer_p1 <= 14) )
									{
										// P1_UAV1(1000,-1000)
										ref_X1 = 1000;
										ref_Y1 = -1000;	
										// P1_UAV2(-1000,1000)
										ref_X2 = -1000;
										ref_Y2 = 1000;											
									}
									else if ( (timer_p1 > 14) && (timer_p1 <= 20) )
											{
												// P2_UAV1(1000,1000)
												ref_X1 = 1000;
												ref_Y1 = 1000;
												// P2_UAV2(-1000,-1000)
												ref_X2 = -1000;
												ref_Y2 = -1000;												
											}
										else if ( (timer_p1 > 20) && (timer_p1 <= 26) )	
												{
													// P3_UAV1(-1000,1000)
													ref_X1 = -1000;
													ref_Y1 = 1000;											
													// P3_UAV2(1000,-1000)
													ref_X2 = 1000;
													ref_Y2 = -1000;																								
												}
											else if ( (timer_p1 > 26) && (timer_p1 <= 32) )
													{
														// P4_UAV1(-1000,-1000)
														ref_X1 = -1000;
														ref_Y1 = -1000;	
														// P4_UAV2(1000,1000)
														ref_X2 = 1000;
														ref_Y2 = 1000;																																					
													}
												else if ( (timer_p1 > 32) && (timer_p1 <= 38) )	
														{
															// P1_UAV1(1000,-1000)
															ref_X1 = 1000;
															ref_Y1 = -1000;	
															// P1_UAV2(-1000,1000)
															ref_X2 = -1000;
															ref_Y2 = 1000;																
														}
													else if ( (timer_p1 > 38) && (timer_p1 <= 42) )	
															{
																// 01(500,0)
																ref_X1 = 750;
																ref_Y1 = 0;	
																// 01(-500,0)
																ref_X2 = -750;
																ref_Y2 = 0;																	
															}
														else if	(timer_p1 > 42)	
																{

																	ref_Z1 = 500;
																	ref_Z2 = 500;
																	//b1 = 0;
																	timer_p1 = 0;
																	//pos = 'l';
																}
															
						// PD CONTROL UAV 1									
						control_X1 = -saturacion_Z( kp_X*(ref_X1 - x1_UAV) + kd_X*(0 - x1_vel) );                           
						control_Y1 = saturacion_Z( kp_Y*(ref_Y1 - y1_UAV) + kd_Y*(0 - y1_vel) );
						control_Z1 = saturacion_Z( kp_Z*(ref_Z1 - z1_UAV) + kd_Z*(0 - z1_vel) );
						control_yaw1 = saturacion( kp_yaw*(0 - yaw1_UAV) + kd_yaw*(0 - yaw1_vel) );
						control_X1_rot =  control_X1*cos(yaw1_UAV_rad) + control_Y1*sin(yaw1_UAV_rad); 
						control_Y1_rot = -control_X1*sin(yaw1_UAV_rad) + control_Y1*cos(yaw1_UAV_rad); 			  
						
						// PD CONTROL UAV 2									
						control_X2 = -saturacion_Z( kp_X*(ref_X2 - x2_UAV) + kd_X*(0 - x2_vel) );                           
						control_Y2 = saturacion_Z( kp_Y*(ref_Y2 - y2_UAV) + kd_Y*(0 - y2_vel) );
						control_Z2 = saturacion_Z( kp_Z*(ref_Z2 - z2_UAV) + kd_Z*(0 - z2_vel) );
						control_yaw2 = saturacion( kp_yaw*(0 - yaw2_UAV) + kd_yaw*(0 - yaw2_vel) );
						control_X2_rot =  control_X2*cos(yaw2_UAV_rad) - control_Y2*sin(yaw2_UAV_rad); 
						control_Y2_rot =  control_X2*sin(yaw2_UAV_rad) + control_Y2*cos(yaw2_UAV_rad); 			 
						
						timer_p1 = timer_p1 + 0.02;
					}
				else
					{												
						control_X2_rot = axis_X;
						control_Y2_rot = axis_Y;
						control_Z2 = 0;
						control_yaw2 = axis_yaw;	
					}		    
				
				// ASIGNAR SENALES DE CONTROL A UAV 1
				msg_control1.linear.x = control_Y1_rot;
				msg_control1.linear.y = control_X1_rot;
				msg_control1.linear.z = control_Z1;
				msg_control1.angular.z = control_yaw1;
				// ENVIAR SENALES DE CONTROL A UAV 1
				control_u_b1.publish(msg_control1);				

				// ASIGNAR SENALES DE CONTROL A UAV 1
				msg_control2.linear.x = control_Y2_rot;
				msg_control2.linear.y = control_X2_rot;
				msg_control2.linear.z = control_Z2;
				msg_control2.angular.z = control_yaw2;
				// ENVIART SENALES DE CONTROL A UAV 1
				control_u_b2.publish(msg_control2);				
        }
		
    myfile_UAV1 << std::setprecision(5) << tiempo << "\t" << x1_UAV << "\t" << y1_UAV << "\t" << z1_UAV << "\t" << x1_vel << "\t" << y1_vel << "\t" << z1_vel<< "\t" << control_X1 << "\t" << control_Y1 << "\t" << control_Z1 << "\n"; 
	myfile_UAV2 << std::setprecision(5) << tiempo << "\t" << x2_UAV << "\t" << y2_UAV << "\t" << z2_UAV << "\t" << x2_vel << "\t" << y2_vel << "\t" << z2_vel<< "\t" << control_X2 << "\t" << control_Y2 << "\t" << control_Z2 << "\n"; 
	
    ros::spinOnce();
    loop_rate.sleep();

    ++count;
    tiempo = tiempo + T;
  }

  delete J;
  myfile_UAV1.close();
  myfile_UAV2.close();
  return 0;
}

float saturacion(float a)
{
  if (a > 0.7)
     a = 0.7;
  if (a < -0.7)
     a = -0.7;	
  return a;
}

float saturacion_Z(float a)
{
  if (a > 0.7)
     a = 0.7;
  if (a < -0.7)
     a = -0.7;
  return a;
}



