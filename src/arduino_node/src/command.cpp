#include <unistd.h>
#include <boost/asio.hpp>
#include "command.h"
#include <iostream>
#include <unistd.h>

static int isInitialized = 0;

static boost::asio::io_service io;
static boost::asio::serial_port serial(io);

static unsigned char* serialize_int(unsigned char* buf, signed short a);

void initialize() {
    std::string num = "0";
    for(int i = 0; i < 5; i++){

        try{
            serial.open("/dev/ttyACM" + static_cast<std::string>(num));
            break;
        } catch(std::exception &e){
            std::cout << "trying " << num << " failed, continuing" << std::endl;
            num[0]++;
        }

    }
    if(num[0] == '5'){
        printf("could not open any serial ports\n");
        throw(std::exception());
    }
    std::cout << "opened port ACM" << num << std::endl;

    serial.set_option(boost::asio::serial_port_base::baud_rate(9600));
    usleep(1000000);
}

char c;
static void handler (const boost::system::error_code& error, std::size_t nbytes) {
    if(error.value()) {
        std::cout << "ERR: " << error.message() << std::endl;
    } else {
        std::cout << "N: " << nbytes << ", " << c << std::endl;
    }
}

void sendCommand(short left, short right, int timeout_ms) {
    if(!isInitialized) {
        isInitialized = 1;
        initialize();
    }

    unsigned char buffer[12];
    unsigned char* ptr = buffer;
    ptr = serialize_int(ptr,1337);
    ptr = serialize_int(ptr,left);
    ptr = serialize_int(ptr,right);
    int retval = boost::asio::write(serial,boost::asio::buffer(buffer,6));
    if(retval <= 0)
        std::cout << "serial write failed: " << retval << std::endl;


    /*boost::asio::async_read(serial, boost::asio::buffer(&c, 1), handler);*/
    /*usleep(5000000);*/
    /*boost::system::error_code ec;*/
    /*boost::asio::read(serial, boost::asio::buffer(&c,1), ec);*/
    /*if(ec.value()) {*/
    /*    std::cout << "Error:  " << ec.message() << std::endl;*/
    /*} else {*/
    /*    std::cout << "Return: " << c << std::endl;*/
    /*}*/
}

static unsigned char* serialize_int(unsigned char* buf, signed short a) {
    /**buf++ = (a >> 24) & 0x00ff;*/
    /**buf++ = (a >> 16) & 0x00ff;*/
    *buf++ = (a >> 8) & 0x00ff;
    *buf++ = a & 0x00ff;
    return buf;
}

static unsigned char* deserialize_int(unsigned char* buf, signed short *a) {
    *a = 0;
    /**a = (*buf++ << 24) | *a;*/
    /**a = (*buf++ << 16) | *a;*/
    *a = (*buf++ << 8) | *a;
    *a = (*buf++) | *a;
    return buf;
}
