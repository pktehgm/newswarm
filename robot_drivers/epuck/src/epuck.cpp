#include <time.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#define WHEEL_DIAMETER 2.1        // cm.
#define WHEEL_SEPARATION 8,6    // Separation between wheels (cm).

using namespace serial;
using namespace std;
using namespace ros;

int speedLeft = 0, speedRight = 0;
// port, baudrate, timeout in milliseconds
Serial epuck_ser("/dev/rfcomm1",115200, serial::Timeout::simpleTimeout(1000));

ros::Subscriber cmdVelSubscriber;
//function to write serial commands for motors
void epuckMotors(){
	    char sercmd[15];
        sprintf(sercmd,"D,%d,%d\n",speedLeft,speedRight);
        epuck_ser.write(sercmd);
	}

void epuckCmd(){
		epuck_ser.write("D,50,50\n");
		ros::Duration(2).sleep();
		epuck_ser.write("D,-50,-50\n");
		ros::Duration(2).sleep();
	}

//function to get left and right motor speeds from ROS twist msgs
void handlerVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{// Controls the velocity of each wheel based on linear and angular velocities.
    double linear = msg->linear.x;    // Divide by 3 to adapt the values received from the rviz "teleop" module that are too high.
    double angular = msg->angular.z/4;
    cout<<"linear="<<linear<<" angular="<<angular<<endl;
    // Kinematic model for differential robot.
    double wl = (linear - ((WHEEL_SEPARATION / 2.0) * angular)) / (WHEEL_DIAMETER);
    double wr = (linear + ((WHEEL_SEPARATION / 2.0) * angular)) / (WHEEL_DIAMETER);
    // At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
    if(int(wl*200)>100){speedLeft=100;}
    else if (int(wl*200)<-100){speedLeft=-100;}
    else {speedLeft = int(wl*200);}
    
	if(int(wr*200)>100){speedRight=100;}
    else if (int(wr*200)<-100){speedRight=-100;}
    else {speedRight = int(wr*200);}
   // changedActuators[MOTORS] = true;
	
    cout << "epuck - new speed: " << speedLeft << ", " << speedRight << std::endl;
    
	
}

int main(int argc, char **argv){

    cout<<"Connecting to the epuck robot..."<<endl;
	
	try {
        epuck_ser.open();
    }
    catch(std::exception e) {
        std::stringstream output;
        output<<"Failed to open serial port "<< epuck_ser.getPort() << "error: " << e.what() <<endl;
    }

    if(epuck_ser.isOpen()){

        cout<<"epuck_ser is Connected on Port: "<<epuck_ser.getPort()<<" at Baudrate: "<<epuck_ser.getBaudrate()<<endl;
    }
    else{

        ROS_ERROR("serial port not openened, verify the epuck robot selector position");
    }
	//test serial command
    epuck_ser.write("D,0,0\n");

	ros::init(argc, argv, "epuck");

    ros::NodeHandle n;

   cmdVelSubscriber = n.subscribe("/robot_motors", 10, handlerVelocity);

    ros::Rate loop_rate(10);
    while(ros::ok()){
		epuckMotors();
		//epuckCmd();
        ros::spinOnce(); //Allow ROS to check for new ROS Messages
        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate

    }

    return 0;
}
