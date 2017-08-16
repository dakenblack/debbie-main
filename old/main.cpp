#include <iostream>
#include "command.h"
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char** argv) {
    sendCommand(0,0,10);
    sendCommand(0,50,10);
    usleep(2000000);
    sendCommand(50,0,10);
    usleep(2000000);
    sendCommand(0,0,10);
    usleep(1000000);
}
