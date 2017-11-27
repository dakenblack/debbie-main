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

    if(SDL_NumJoysticks() < 2 ) {
        std::cout << "No Joysticks connected, Exiting...\n";
        SDL_Quit();
        return -1;
    }

    // Declares and initiates two joysticks, assumes 0 is left 1 is right
    SDL_Joystick *dir;
    SDL_Joystick *pow;
    dir = SDL_JoystickOpen(0);
    pow = SDL_JoystickOpen(1);

    int exit = 0;
    while(!exit && ros::ok()) {
        int dir_x, dir_y, pow_x, pow_y;
        // length, x component and y component of dir unit vector
        double dir_l, dir_ux, dir_uy;
        // Gets the direction vectors and converts them to unit length
        dir_x = -SDL_JoystickGetAxis(dir,X_AXIS);
        dir_y = SDL_JoystickGetAxis(dir,Y_AXIS);
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
