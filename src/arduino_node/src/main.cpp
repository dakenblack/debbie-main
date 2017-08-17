#include <iostream>
#include "command.h"
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define TURNING_CONSTANT 50
signed short val_x, val_y;

void cb(geometry_msgs::Twist::ConstPtr msg) {
    //ROS_INFO("got msg x=%d, y=%d",(int)msg->linear.x,(int)msg->linear.y);
    val_x = (int)msg->linear.x;
    val_y = (int)msg->linear.y;
}

int map(int val, int from_neg, int from_pos, int to_neg, int to_pos) {
    float from_range = from_pos - from_neg;
    float to_range = to_pos - to_neg;
    float ratio = (val - from_neg)/from_range;
    return (int)(ratio * to_range + to_neg);
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"arduino_node");
    ros::NodeHandle n;
    ros::Rate send_rate(1);
    ros::Subscriber sub = n.subscribe("cmd_vel",1000,cb);
    val_x = val_y = 0;

    while(ros::ok()) {
        ros::spinOnce();
        //ROS_INFO("Sending...");
        signed short left, right;
        left = right = val_y;
        signed short temp = map(val_x,-100,100,-1*TURNING_CONSTANT,TURNING_CONSTANT);
        left += temp;
        right -= temp;
        ROS_INFO("sending msg x=%d, y=%d, left=%d, right=%d",(int)val_x,(int)val_y,left,right);
        sendCommand(left,right,0);
        send_rate.sleep();
    }
    return 0;
}
