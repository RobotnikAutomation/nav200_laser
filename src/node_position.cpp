#include "nav200_laser/node_position.h"


using namespace nav;

std::string port;
std::string mode;
std::string nav200_laser_frame_id_;
std::string base_frame_id_;
std::string map_frame_id_;
std::string odom_frame_id_;
std::string odom_topic_;
robotnik_nav200 * rbk_nav;


/*!	\fn robotnik_nav200::robotnik_nav200()
 * 	\brief Public constructor
*/
robotnik_nav200::robotnik_nav200(std::string port){
    	
	// Create serial port
	serial= new SerialDevice(port.c_str(), 19200,"even",8);
	
}

/*!	\fn robotnik_nav200::~robotnik_nav200()
 * 	\brief Public destructor
*/
robotnik_nav200::~robotnik_nav200(){

	// Close serial port
	if (serial!=NULL) serial->ClosePort();

        // Delete serial port
        if (serial!=NULL) delete serial;
}



/*!	\fn void robotnik_nav200::StateMachine()
 * 	function that manages internal states of the component 
*/

void robotnik_nav200::StateMachine(){

   switch (iState){
        case INIT_STATE:
                InitState();
        break;
	case ACTIVE_STATE:
		ActiveState();
	break;
        case FAILURE_STATE:
                FailureState();
        break;
	default:
	break;
	}
}


/*!	\fn void robotnik_nav200::SwitchToState(States new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void robotnik_nav200::SwitchToState(States new_state){

	if(new_state == iState){
		return;
	}
	iState = new_state;
	switch (new_state) {
		case INIT_STATE:
			ROS_INFO("robotnik_nav200:: switching to INIT state");
			break;
		case ACTIVE_STATE:
			ROS_INFO("robotnik_nav200:: switching to ACTIVE state");
			break;
		case FAILURE_STATE:
			ROS_INFO("robotnik_nav200:: switching to FAILURE state");
			break;
		case SHUTDOWN_STATE:
			ROS_INFO("robotnik_nav200:: switching to SHUTDOWN state");
		    break;

		default:
			ROS_ERROR("robotnik_nav200:: Switching to UNKNOWN state");
			break;
	}
}

/*!	\fn int robotnik_nav200::Open(char *dev)
 * 	\brief Open serial port
 * 	\returns -1 Error
 * 	\returns 0 Ok
*/
int robotnik_nav200::Open(){

	// Setup serial device
	if (this->serial->OpenPort2() == SERIAL_ERROR) {
          ROS_ERROR("robotnik_nav200::Open: Error Opening Serial Port");
	  SwitchToState(FAILURE_STATE);
	  iErrorType =ERROR_OPEN;
	  return ERROR;
          }
	ROS_INFO("robotnik_nav200::Open: serial port opened at %s", serial->GetDevice());
        usleep(50000);
	SwitchToState(INIT_STATE);
	
	return OK;
}


/*!	\fn int robotnik_nav200::Close()
 * 	\brief Closes serial port
 * 	\returns ERROR
 * 	\returns OK
*/
int robotnik_nav200::Close(){

	if (serial!=NULL) serial->ClosePort();
	return OK;
}


void robotnik_nav200::InitState(){
	char nMsg[64]="\0", nReadBuffer[64]="\0";
	int written_bytes=0;
	int read_bytes=0;
	int iTry=0, max_iTRY=1000, ret=0;
	nMsg[0]=2;
	nMsg[1]=5;
	nMsg[2]='P';
	nMsg[3]='A';
	nMsg[4]=BlockCheck(5,nMsg);
	// Send message to serial port in order to activate nav200 positioning mode	
	if(serial->WritePort(nMsg, &written_bytes, strlen(nMsg)) != OK) {
		ROS_ERROR("robotnik_nav200::InitState: Error sending message");
		SwitchToState(FAILURE_STATE);
		iErrorType=ERROR_SEND;
        }
	
	//Waits for response
	usleep(10000);
	//Reading data from nav200
        while ((iTry<max_iTRY) && (read_bytes==0)) {
        	ret = serial->ReadPort(nReadBuffer, &read_bytes, sizeof(nReadBuffer));
        	iTry++;
       	 	usleep(10000);
   	}


	/*std::cout<<"Writing messsage: "<<int(nMsg[0])<<int(nMsg[1])<<int(nMsg[2])<<int(nMsg[3])<<int(nMsg[4])<<std::endl;
	std::cout<<"Reading message: "<<int(nReadBuffer[0])<<int(nReadBuffer[1])<<int(nReadBuffer[2])<<int(nReadBuffer[3])<<int(nReadBuffer[4])<<std::endl;
	std::cout<<"read_bytes: "<<read_bytes<<std::endl;*/

	//Receiving error message from nav200
	if(nReadBuffer[3]=='E'){
		ROS_ERROR("robotnik_nav200::InitState: Error identifier");
		iErrorType=ERROR_ACTIVATE;
		SwitchToState(FAILURE_STATE);
	}
	//Reading message wrong 
	else if(nMsg[0]!=nReadBuffer[0] || nMsg[1]!=nReadBuffer[1] || nMsg[2]!=nReadBuffer[2] || nMsg[3]!=nReadBuffer[3] || nMsg[4]!=nReadBuffer[4]){
		ROS_ERROR("robotnik_nav200::InitState: Error activate nav200");
		iErrorType=ERROR_ACTIVATE;
		SwitchToState(FAILURE_STATE);
	}
	//Standby mode is activated successfully
	else{	
		SwitchToState(ACTIVE_STATE);
	}
}

/* /robotnik_nav200::ActiveState
    Send data to nav200 and read data from nav200 
    Position mode of nav200
*/
void robotnik_nav200::ActiveState(){
	char nMsg[64]="\0", nReadBuffer[64]="\0";
	int written_bytes=0;
	int read_bytes=0;
	int iTry=0, max_iTRY=200,ret=0;
	int numb_bytes;
	static long count=0;
	position_ok = false;
	
	if(mode=="automatic_speed"){
		numb_bytes=5;		
		nMsg[0]=2;
		nMsg[1]=5;
		nMsg[2]='P';
		nMsg[3]='P';
		nMsg[4]=BlockCheck(5,nMsg);
	}
	else{
		numb_bytes=11;
		nMsg[0]=2;
		nMsg[1]=11;
		nMsg[2]='P';
		nMsg[3]='w';
		nMsg[4]=(velocity.x & 0xFF);
		nMsg[5]=((velocity.x & 0xFF00) >> 8);
		nMsg[6]=(velocity.y & 0xFF);
		nMsg[7]=((velocity.y & 0xFF00) >> 8);
		nMsg[8]=(velocity.theta & 0xFF);
		nMsg[9]=((velocity.theta & 0xFF00) >> 8);
		nMsg[10]=BlockCheck(11,nMsg);			
		//ROS_INFO("Feeding speed = (%d, %d, %d)", velocity.x, velocity.y, velocity.theta);
	}

	/*std::cout<<"velocity_x: "<<velocity.x<<" , "<<(velocity.x & 0xFF)<<" , "<<(velocity.x & 0xFF00)<<std::endl;
	std::cout<<"velocity_y: "<<velocity.y<<" , "<<(velocity.y & 0xFF)<<" , "<<(velocity.y & 0xFF00) <<std::endl;
	std::cout<<"velocity_theta: "<<velocity.theta<<" , "<<(velocity.theta & 0xFF)<<" , "<<(velocity.theta & 0xFF00)<<std::endl;
	std::cout<<static_cast<float>(nMsg[4]+nMsg[5]*256)<<" , "<<static_cast<float>(nMsg[6]+nMsg[7]*256)<<std::endl;
	std::cout<<"count: "<<count<<std::endl;
	std::cout<<"writing*"<<std::endl;
	std::cout<<"nMsg: "<<static_cast<int>(nMsg[0])<<static_cast<int>(nMsg[1])<<static_cast<int>(nMsg[2])<<static_cast<int>(nMsg[3])<<static_cast<int>(nMsg[4])<<static_cast<int>(nMsg[5])<<static_cast<int>(nMsg[6])<<static_cast<int>(nMsg[7])<<static_cast<int>(nMsg[8])<<static_cast<int>(nMsg[9])<<static_cast<int>(nMsg[10])<<std::endl;
	*/	

	// Send message to serial port in order to activate nav200 positioning mode	
	if(serial->WritePort(nMsg, &written_bytes, numb_bytes) != OK) {
		ROS_ERROR("robotnik_nav200::ActiveState: Error writing message");
		SwitchToState(FAILURE_STATE);
		iErrorType=ERROR_WRITE;
	}
		
	//Waits for response
	usleep(50000);
	// Read nav200 messages
	while ((iTry<max_iTRY) && (read_bytes==0)) {
		ret = serial->ReadPort(nReadBuffer, &read_bytes, 17);
		iTry++;
		usleep(10000);
		//std::cout<<"iTry:"<<iTry<<" , "<<"read_bytes:"<<read_bytes<<std::endl;
	}
	if((iTry==max_iTRY) && (read_bytes==0)){
		ROS_ERROR("robotnik_nav200::ActiveState: Error receiving message.");
		serial->Flush();
		SwitchToState(FAILURE_STATE);
		count=0;
		iErrorType=ERROR_RECEIVE;			
	}
		
	//nav200 send error message		
	if(nReadBuffer[3]=='E'){
		ROS_INFO("robotnik_nav200::ActiveState: Error message from nav200.");
		ROS_ERROR("Error type: F0 = %d, F1 = %d, F2 = %d, F3 = %d", static_cast<int>(nReadBuffer[4]), static_cast<int>(nReadBuffer[5]), static_cast<int>(nReadBuffer[6]), static_cast<int>(nReadBuffer[7]));			
	}
	//succesfull reading data 
	else if(read_bytes==17){
		msg.x = static_cast<float>(static_cast<unsigned char>(nReadBuffer[7])*16777216 + static_cast<unsigned char>(nReadBuffer[6])*65536 + static_cast<unsigned char>(nReadBuffer[5])*256 + static_cast<unsigned char>(nReadBuffer[4]))/1000;
		msg.y = static_cast<float>(static_cast<unsigned char>(nReadBuffer[11])*16777216 + static_cast<unsigned char>(nReadBuffer[10])*65536 + static_cast<unsigned char>(nReadBuffer[9])*256 + static_cast<unsigned char>(nReadBuffer[8]))/1000;
		msg.theta = static_cast<float>(static_cast<unsigned char>(nReadBuffer[13])*256 + static_cast<unsigned char>(nReadBuffer[12]))*0.0055f;
		//Angle conversion
		if(msg.theta>180){
			msg.theta=msg.theta-360;				
		}
		//Quality and number of reading reflector
		msg.quality=static_cast<int>(nReadBuffer[14]);
		msg.refnumb=static_cast<int>(nReadBuffer[15]);
		
		position_ok = true;
		//ROS_INFO("OK: iTry = %d", iTry);
/*

		std::cout<<"x_bytes:"<<static_cast<int>(nReadBuffer[7])<<static_cast<int>(nReadBuffer[6])<<static_cast<int>(nReadBuffer[5])<<static_cast<int>(nReadBuffer[4])<<std::endl;
		std::cout<<"x:"<<msg.x<<std::endl;
		std::cout<<"y_bytes:"<<static_cast<int>(nReadBuffer[11])<<static_cast<int>(nReadBuffer[10])<<static_cast<int>(nReadBuffer[9])<<static_cast<int>(nReadBuffer[8])<<std::endl;
		std::cout<<"y:"<<msg.y<<std::endl;
		std::cout<<"theta:"<<msg.theta<<std::endl;

		std::cout<<"nReadBuffer: "<<static_cast<int>(nReadBuffer[0])<<static_cast<int>(nReadBuffer[1])<<static_cast<int>(nReadBuffer[2])<<static_cast<int>(nReadBuffer[3])<<static_cast<int>(nReadBuffer[14])<<static_cast<int>(nReadBuffer[15])<<std::endl;
		std::cout<<"------------"<<std::endl;
*/		
		count++;
	}else{
		ROS_WARN("robotnik_nav200::ActiveState: read_bytes = %d, iTry = %d", read_bytes, iTry);
	}
}	




/*!	\fn void robotnik_nav200::FailureState()
 * 	\brief Actions in Failure State
*/
void robotnik_nav200::FailureState(){
	int timer = 25000; //microseconds
	static int recovery_cycles = 0;

	recovery_cycles++;
	if(recovery_cycles >= 50){ //Try to recover every 'second'
		switch(iErrorType)	{
			ROS_INFO("robotnik_nav200::FailureState: Trying to recover..");
			case ERROR_OPEN://Try to recover
				ROS_ERROR("robotnik_nav200::FailureState: Recovering from failure state (ERROR_OPEN.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case ERROR_READ:
				ROS_ERROR("robotnik_nav200::FailureState: Recovering from failure state (ERROR_READ.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case ERROR_WRITE:
				ROS_ERROR("robotnik_nav200::FailureState: Recovering from failure state (ERROR_WRITE.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case ERROR_ACTIVATE:
				ROS_ERROR("robotnik_nav200::FailureState: Recovering from failure state (ERROR_ACTIVATE.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case ERROR_RECEIVE:
				ROS_ERROR("robotnik_nav200::FailureState: Recovering from failure state (ERROR_RECEIVE.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case ERROR_SEND:
				ROS_ERROR("robotnik_nav200::FailureState: Recovering from failure state (ERROR_SEND.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
		}
		recovery_cycles = 0;
	}
}

//Calculate "BlockCheck" byte to send 
unsigned char robotnik_nav200::BlockCheck(int bLong,char *nMsg)
{ 
    	int i;
        unsigned char LRC,ch;

        LRC = 0x02;
        for (i=1;i<(bLong-1);i++) {
        	ch = nMsg[i];
                LRC ^= ch;
                }
	return LRC;
}

int robotnik_nav200::GetState(){
	return iState;
}

std::string robotnik_nav200::GetStateString(){
	switch (iState) {
		case INIT_STATE:
			return "INIT";
			break;
		case ACTIVE_STATE:
			return "ACTIVE";
			break;
		case FAILURE_STATE:
			return "FAILURE";
			break;
		
		default:
			return "UNKNOWN";	
		break;
	}
}
			

//callback function for feeeding_speed mode
void callback1(const nav_msgs::Odometry::ConstPtr& msg_odom){
		velocity.x=static_cast<int>(msg_odom->twist.twist.linear.x*1000);
		velocity.y=static_cast<int>(msg_odom->twist.twist.linear.y*1000);
		//velocity.theta=static_cast<int>(msg_odom->twist.twist.angular.z*2*16384/PI);
		velocity.theta=static_cast<int>(msg_odom->twist.twist.angular.z*2*180/PI);
                
		pos_wrt_odom.x = msg_odom->pose.pose.position.x;
		pos_wrt_odom.y = msg_odom->pose.pose.position.y;	
		double roll,pitch, yaw;      
     		tf::Quaternion q_orig(msg_odom->pose.pose.orientation.x, msg_odom->pose.pose.orientation.y, msg_odom->pose.pose.orientation.z,msg_odom->pose.pose.orientation.w);
     		tf::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);
		pos_wrt_odom.theta=yaw;
		
		 
}

//callback function for automatic_speed mode
void callback2(const nav_msgs::Odometry::ConstPtr& msg_odom){

	pos_wrt_odom.x = msg_odom->pose.pose.position.x;
	pos_wrt_odom.y = msg_odom->pose.pose.position.y;	
	double roll,pitch, yaw;      
     	tf::Quaternion q_orig(msg_odom->pose.pose.orientation.x, msg_odom->pose.pose.orientation.y, msg_odom->pose.pose.orientation.z,msg_odom->pose.pose.orientation.w);
     	tf::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);
	pos_wrt_odom.theta=yaw;
}


int getTransform(tf::StampedTransform *trans, std::string from_frame, std::string to_frame){
	
	static tf::TransformListener listener;
	tf::StampedTransform transform;
		
	try{
		listener.lookupTransform(from_frame, to_frame, 
							   ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("getTransform: %s",ex.what());
		return -1;
	}
	
	*trans = transform;
	
	return 0;
	
}

double radnorm(double radians){
	while (radians >= (M_PI)) {
		radians -= 2.0 * M_PI;
	}
	while (radians <= (-M_PI)) {
		radians += 2.0 * M_PI;
	}
	
	return radians;
}

//-------------------------------------MAIN------------------------------------

void map2odom();

// MAIN
int main(int argc, char** argv){
	
	ros::init(argc, argv, "robotnik_nav200_node");
	//ROS_INFO("robotnik_nav200 for ROS %.2f", NODE_VERSION);	
	ros::Subscriber odom_sub;
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	pn.param<std::string>("port", port, "/dev/ttyUSB0");
	pn.param<std::string>("mode", mode, "automatic_speed");
	pn.param<std::string>("nav200_laser_frame_id", nav200_laser_frame_id_, "nav200_laser_link");
	pn.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");
	pn.param<std::string>("map_frame_id", map_frame_id_, "/map");
	pn.param<std::string>("odom_frame_id", odom_frame_id_, "/odom");
	pn.param<std::string>("odom_topic", odom_topic_, "/odom");
	pn.param<double>("min_estimation_quality_", min_estimation_quality_, 70.0);

	std::cout<<"Nav 200 mode: "<<mode<<std::endl;

	static tf::TransformBroadcaster br;
	
	tf::Transform transform;
	tf::Quaternion q;
	tf::StampedTransform transfor_nav_to_baselink;
	tf::StampedTransform transform_base_to_odom;
	tf::Vector3 origin;
	tf::Quaternion rotation;
	bool transform_ok = false;
	bool nav200_ok = false;
	
	// Create node object
	rbk_nav = new robotnik_nav200(port);
	
	// Publishing robotnik_nav200_node
	//ros::Publisher pos_pub = pn.advertise<geometry_msgs::Pose2D>("pose", 50);
	ros::Publisher pos_pub = pn.advertise<nav200_laser::Pose2D_nav>("nav200_pose",10);
	ros::Publisher pose_estimated_pub = pn.advertise<geometry_msgs::PoseStamped>("pose", 10);
	
	if( rbk_nav->Open()==OK ) 
		ROS_INFO("Connected to robotnik_nav200 component");
	else
	{
		ROS_FATAL("Could not connect to robotnik_nav200 component");
		//ROS_BREAK();
		return -1;
	}
	
	
	ros::Rate r(5.0);
	
	if(mode=="automatic_speed"){
		odom_sub = n.subscribe("/odom",5,callback2);
	}else{
		odom_sub = n.subscribe("/odom",5,callback1);
	}
	
	while(pn.ok())
	{		

		// 1 State Machine
		rbk_nav->StateMachine();
	 
		if(not transform_ok){
			if(getTransform(&transfor_nav_to_baselink, nav200_laser_frame_id_, base_frame_id_) == 0){
				transform_ok = true;
				
				origin = transfor_nav_to_baselink.getOrigin();
				origin.setZ(0.0);
				rotation = transfor_nav_to_baselink.getRotation();
				transfor_nav_to_baselink.setOrigin(origin);
				//ROS_INFO("origin: x = %lf, y = %lf, z = %lf, w = %lf", origin.x(), origin.y(), origin.z(), origin.w() );
				//ROS_INFO("rotation: angle = %lf", rotation.getAngle() );
				
				//transform.setOrigin(tf::Vector3(10.0,0.0,0));
				//q.setRPY(0,0,0.0);
				//transform.setRotation(q);
				
				/*transform*=transfor_nav_to_baselink;
				origin = transform.getOrigin();
				rotation = transform.getRotation();
				ROS_INFO("-> origin: x = %lf, y = %lf, z = %lf, w = %lf", origin.x(), origin.y(), origin.z(), origin.w() );
				ROS_INFO("-> rotation: angle = %lf", rotation.getAngle() );*/
				
			}
		}else{
			// 2 Publish
			nav200_laser::Pose2D_nav pose2D_msg;				
			
			pose2D_msg.x=msg.x;
			pose2D_msg.y=msg.y;
			pose2D_msg.theta=msg.theta;
			pose2D_msg.quality=msg.quality;
			pose2D_msg.refnumb=msg.refnumb;
			pose2D_msg.header.stamp = ros::Time::now();
			pose2D_msg.header.frame_id = nav200_laser_frame_id_;
			pose2D_msg.state = rbk_nav->GetStateString();
	
			pos_pub.publish(pose2D_msg);
						
			if(rbk_nav->position_ok && msg.quality >= min_estimation_quality_){
				//map2odom();
					
				quality_msg = msg;			
				if(getTransform(&transform_base_to_odom, base_frame_id_, odom_frame_id_) == 0)
					nav200_ok = true;
			}
			
			if(nav200_ok){				
				//origin = transform_base_to_odom.getOrigin();
				//rotation = transform_base_to_odom.getRotation();
				//ROS_INFO("base to odom -> origin: x = %lf, y = %lf, z = %lf, w = %lf", origin.x(), origin.y(), origin.z(), origin.w() );
				//ROS_INFO("base to odom -> rotation: angle = %lf", rotation.getAngle() );
				
				transform.setOrigin(tf::Vector3(quality_msg.x,quality_msg.y,0));
				q.setRPY(0,0,radnorm(quality_msg.theta*PI/180));
				transform.setRotation(q);
				transform*=transfor_nav_to_baselink;
				transform*=transform_base_to_odom;
				
				//origin = transform.getOrigin();
				//rotation = transform.getRotation();
				//ROS_INFO("map to odom -> origin: x = %lf, y = %lf, z = %lf, w = %lf", origin.x(), origin.y(), origin.z(), origin.w() );
				//ROS_INFO("map to odom -> rotation: angle = %lf", rotation.getAngle() );
				br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),map_frame_id_, odom_frame_id_));
				
				/*geometry_msgs::PoseStamped ps;
				
				ps.header.stamp = ros::Time::now();
				
				ps.pose.position.x = robot_odometry.pose.pose.position.x + ts.transform.translation.x;
				ps.pose.position.y = robot_odometry.pose.pose.position.y + ts.transform.translation.y;
				ps.pose.orientation = robot_odometry.pose.pose.orientation;
				
				pose_estimated_publisher.publish(ps);*/
			}
			

		}
		ros::spinOnce();
		r.sleep();
	}
	


	rbk_nav->Close();

	return 0;
}


void map2odom(){
	
	odom_pos.theta=(msg.theta*PI/180-pos_wrt_odom.theta);
	/*std::cout<<"fi:"<<msg.theta*PI/180<<std::endl;
	std::cout<<"alfa:"<<pos_wrt_odom.theta<<std::endl;
	std::cout<<"odom_pos.theta"<<odom_pos.theta<<std::endl;*/
	
		odom_pos.x = msg.x -pos_wrt_odom.x*cos(odom_pos.theta) + pos_wrt_odom.y*sin(odom_pos.theta);
		odom_pos.y = msg.y -pos_wrt_odom.y*cos(odom_pos.theta) - pos_wrt_odom.x*sin(odom_pos.theta);			
}


// EOF

