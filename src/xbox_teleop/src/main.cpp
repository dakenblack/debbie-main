#include <SDL2/SDL.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define DEADZONE_X 8000
#define DEADZONE_Y 8000

#define X_AXIS 0
#define Y_AXIS 1

//#define DEBUG_OUT

int map(int val, int from_neg, int from_pos, int to_neg, int to_pos) {
    float from_range = from_pos - from_neg;
    float to_range = to_pos - to_neg;
    float ratio = (val - from_neg)/from_range;
    return (int)(ratio * to_range + to_neg);
}

int main(int argc, char ** argv) {
    if (SDL_Init( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK ))
    {
        fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
        exit(1);
    }

    ros::init(argc,argv,"xbox_teleop");
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::Rate pub_rate(15);

    if(SDL_NumJoysticks() < 1 ) {
        std::cout << "No Joystick connected, Exiting...\n";
        SDL_Quit();
        return -1;
    }

    SDL_Joystick *joy;
    joy = SDL_JoystickOpen(0);

    int exit = 0;
    while(!exit && ros::ok()) {
        int val_x, val_y;
        val_x = SDL_JoystickGetAxis(joy,X_AXIS);
        val_y = -SDL_JoystickGetAxis(joy,Y_AXIS);

        val_x = std::abs(val_x) > DEADZONE_X ? val_x : 0;
        val_y = std::abs(val_y) > DEADZONE_Y ? val_y : 0;

        val_x = map(val_x, -32768, 32767, -100, 100);
        val_y = map(val_y, -32768, 32767, -100, 100);

#ifdef DEBUG_OUT
        std::stringstream ss;
        ss << "[x=" << val_x << ", y=" << val_y << "]";
        ROS_INFO("%s",ss.str().c_str());
#endif

        geometry_msgs::Twist msg;
        msg.linear.x = val_x;
        msg.linear.y = val_y;

        vel_pub.publish(msg);

        if(SDL_JoystickGetButton(joy, SDL_CONTROLLER_BUTTON_A))
            exit = 1;

        pub_rate.sleep();
        SDL_JoystickUpdate();
    }

    SDL_Quit();
    std::cout << "Quiting!!!!!\n";
    return 0;
}
