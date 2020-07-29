#ifndef SERIAL_H
#define SERIAL_H

#include <math.h>
#include <stdio.h>      // standard input / output functions
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitionss
#include <iostream>


struct serial_transmit_data{
    u_char raw_data[10];
    int size;
    u_char head = 0xaa;
    u_char end = 0xbb;
    void get_xy_data(int16_t x, int16_t y, int8_t found);
};




using namespace std;

class _serial{
public:
    //"/dev/ttyTHS0"
    serial_transmit_data data;
    _serial(const char* filename = "/dev/ttyUSB0", int buadrate = 0 );
    void send_data(void);
    void read_data(void);
    int fd;

};


#endif // SERIAL_H
