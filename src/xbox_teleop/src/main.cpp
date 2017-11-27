#include <SDL2/SDL.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define DEADZONE_X 8000
#define DEADZONE_Y 8000

#define LJOY_X 0
#define LJOY_Y 1
#define RJOY_X 3
#define RJOY_Y 4
#define LTRIG 2
#define RTRIG 5

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

    // Declares and initiates joystick
    SDL_Joystick *joy;
    joy = SDL_JoystickOpen(0);

    int exit = 0;
    int dir_x, dir_y, power, brake;
    // length, x component and y component of dir unit vector
    double dir_l, dir_ux, dir_uy;
    while(!exit && ros::ok()) {
        // Gets the direction vectors and converts them to unit length
        dir_x = -SDL_JoystickGetAxis(joy,LJOY_X);
        dir_y = SDL_JoystickGetAxis(joy,LJOY_Y);
        dir_x = std::abs(dir_x) > DEADZONE_X ? dir_x : 0;
        dir_y = std::abs(dir_y) > DEADZONE_Y ? dir_y : 0;

        dir_l = std::sqrt(std::pow((double)dir_x,2)+std::pow((double)dir_y,2));
        dir_ux = dir_l != 0 ? (double)dir_x / dir_l : 0;
        dir_uy = dir_l != 0 ? (double)dir_y / dir_l : 0;

        // Make turn rate (differential between wheels) some function of 
        // dir_l and the total speed be some function of pow_y



        dir_x = map(dir_x, -32768, 32767, -100, 100);
        dir_y = map(dir_y, -32768, 32767, -100, 100);

#ifdef DEBUG_OUT
        std::stringstream ss;
        ss << "[x=" << dir_x << ", y=" << dir_y << "]";
        ROS_INFO("%s",ss.str().c_str());
#endif

        geometry_msgs::Twist msg;
        msg.linear.x = dir_x;
        msg.linear.y = dir_y;

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
