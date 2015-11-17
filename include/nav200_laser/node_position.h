#include <string.h>
#include <stdint.h>
#include <math.h>
#include <ros/ros.h>
#include <nav200_laser/Pose2D_nav.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include "SerialDevice.h"

//ERROR FLAGS
#define ERROR_OPEN						1
#define ERROR_READ 						2	
#define ERROR_WRITE 					3
#define ERROR_ACTIVATE					4
#define ERROR_SEND						5
#define ERROR_RECEIVE 					6
#define NAV200_MIN_ESTIMATION_QUALITY	70.0

#define PI  							3.14159265359

namespace nav{
	
	
	
	//Data structure for receiving message
	class message{
		public:
			float x;
			float y;
			float theta;
			int quality;
			int refnumb;
			message(){
				x=0;
				y=0;
				theta=0;
				quality=0;
				refnumb=0;
			}
	};

	class vel_msg{
		public:
			int16_t x,y,theta;
			vel_msg(){
				x=0;
				y=0;
				theta=0;
			}

	};

	//Defines return values for methods and functions
	enum ReturnValue{
			OK = 0,
			ERROR = -1,
	};

	//States of nav200
	enum States{
		INIT_STATE,
		ACTIVE_STATE,
		SHUTDOWN_STATE,
		FAILURE_STATE
	};

	//Global variable
	message msg;
	message quality_msg;
	message odom_pos;
	message pos_wrt_odom;
	vel_msg velocity;
	double min_estimation_quality_;

	//Main Class
	class robotnik_nav200{

		private:
			int iState;
			int iErrorType;
			SerialDevice *serial;
			
		public:
			//Constructer	
			robotnik_nav200(std::string port);
			//Destructer
			~robotnik_nav200();
			void StateMachine();
			void SwitchToState(States new_state);
			int Open();
			int Close();
			int GetState();
			char* GetStateString();
		private:
			void InitState();
			void ActiveState();
			void FailureState();
			unsigned char BlockCheck(int bLong,char *nMsg);
			
	};

}

