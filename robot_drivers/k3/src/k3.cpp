#include <time.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#define WHEEL_DIAMETER 4.1        // cm.
#define WHEEL_SEPARATION 8,6    // Separation between wheels (cm).

using namespace serial;
using namespace std;
using namespace ros;

int speedLeft = 0, speedRight = 0;
// port, baudrate, timeout in milliseconds
Serial k3_ser("/dev/rfcomm0",115200, serial::Timeout::simpleTimeout(1000));

ros::Subscriber cmdVelSubscriber;
//function to write serial commands for motors
void k3Motors(){
	    char sercmd[20];
        sprintf(sercmd,"D,l%d,l%d\n",speedLeft,speedRight);
        k3_ser.write(sercmd);
	}

void k3Cmd(){
		k3_ser.write("D,l30000,l30000\n");
		ros::Duration(2).sleep();
		k3_ser.write("D,l-30000,l-30000\n");
		ros::Duration(2).sleep();
	}

//function to get left and right motor speeds from ROS twist msgs
void handlerVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{// Controls the velocity of each wheel based on linear and angular velocities.
    double linear = msg->linear.x;    // Divide by 3 to adapt the values received from the rviz "teleop" module that are too high.
    double angular = msg->angular.z/3;
    cout<<"linear="<<linear<<" angular="<<angular<<endl;
    // Kinematic model for differential robot.
    double wl = (linear - ((WHEEL_SEPARATION / 2.0) * angular)) / (WHEEL_DIAMETER);
    double wr = (linear + ((WHEEL_SEPARATION / 2.0) * angular)) / (WHEEL_DIAMETER);
    // At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
    if(int(wl*30000)>35000){speedLeft=35000;}
    else if (int(wl*30000)<-35000){speedLeft=-35000;}
    else {speedLeft = int(wl*30000);}
    
	if(int(wr*30000)>35000){speedRight=100;}
    else if (int(wr*30000)<-35000){speedRight=-35000;}
    else {speedRight = int(wr*30000);}
   // changedActuators[MOTORS] = true;
	
    cout << "k3 - new speed: " << speedLeft << ", " << speedRight << std::endl;
    
	
}

int main(int argc, char **argv){

    cout<<"Connecting to the k3 robot..."<<endl;
	
	try {
        k3_ser.open();
    }
    catch(std::exception e) {
        std::stringstream output;
        output<<"Failed to open serial port "<< k3_ser.getPort() << "error: " << e.what() <<endl;
    }

    if(k3_ser.isOpen()){

        cout<<"k3_ser is Connected on Port: "<<k3_ser.getPort()<<" at Baudrate: "<<k3_ser.getBaudrate()<<endl;
    }
    else{

        ROS_ERROR("serial port not openened, verify the k3 robot selector position");
    }
	//test serial command
    k3_ser.write("D,0,0\n");

	ros::init(argc, argv, "k3");

    ros::NodeHandle n;

   cmdVelSubscriber = n.subscribe("/robot_motors", 10, handlerVelocity);

    ros::Rate loop_rate(10);
    while(ros::ok()){
		k3Motors();
		//k3Cmd();
        ros::spinOnce(); //Allow ROS to check for new ROS Messages
        loop_rate.sleep(); //Sleep for some amount of time determined by loop_rate

    }

    return 0;
}
