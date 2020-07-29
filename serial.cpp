#include <iostream>
#include <serial.h>
#include <termio.h>
#include <unistd.h>

using namespace std;


_serial::_serial(const char* filename, int buadrate)
{

    fd = open(filename,  O_RDWR | O_NOCTTY | O_SYNC);
    if(fd == -1){ // if open is unsucessful
        printf("open_port: Unable to open /dev/port_settingsUSB0. \n");
    }
    else  {
        fcntl(fd, F_SETFL, 0);
        printf("port is open.\n");
    }
    struct termios port_settings;               // structure to store the port settings in
    if(buadrate == 0)
    {
        cfsetispeed(&port_settings, B115200);       // set baud rates

        cfsetospeed(&port_settings, B115200);
    }
    else if(buadrate == 1)
    {
        cfsetispeed(&port_settings, B921600);       // set baud rates
        cfsetospeed(&port_settings, B921600);
    }
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
    port_settings.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = 10;           // read doesn't block
    port_settings.c_cc[VTIME] = 5;          // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings);             // apply the settings to the port

/*
//    struct termios port_settings;               // structure to store the port settings in
//    cfsetispeed(&port_settings, B115200);       // set baud rates
//    cfsetospeed(&port_settings, B115200);

//    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
//    // disable IGNBRK for mismatched speed tests; otherwise receive break
//    // as \000 chars
//    port_settings.c_iflag &= ~IGNBRK;         // disable break processing
//    port_settings.c_lflag = 0;                // no signaling chars, no echo,
//                                    // no canonical processing
//    port_settings.c_oflag = 0;                // no remapping, no delays
//    port_settings.c_cc[VMIN]  = 0;            // read doesn't block
//    port_settings.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

//    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

//    port_settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
//                                    // enable reading
//    port_settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
//    port_settings.c_cflag |= 0;
//    port_settings.c_cflag &= ~CSTOPB;
//    port_settings.c_cflag &= ~CRTSCTS;

//    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port


    typedef struct termios TERMIOS;
    TERMIOS SerialPortSettings;
    tcgetattr(fd, &SerialPortSettings);

    cfsetispeed(&SerialPortSettings, B115200);
    cfsetospeed(&SerialPortSettings, B115200);

    SerialPortSettings.c_cflag&= ~ PARENB;
    SerialPortSettings.c_cflag&= ~ CSTOPB;
    SerialPortSettings.c_cflag&= ~ CSIZE;
    SerialPortSettings.c_cflag|=   CS8;
    SerialPortSettings.c_cflag&= ~ CRTSCTS;
    SerialPortSettings.c_cflag|=   CREAD | CLOCAL;
    SerialPortSettings.c_cflag&= ~ (IXON | IXOFF | IXANY);
    SerialPortSettings.c_cflag&= ~ (ICANON | ECHO | ECHOE | ISIG);
    SerialPortSettings.c_cflag&= ~ OPOST;

    tcgetattr(fd, &SerialPortSettings);
    SerialPortSettings.c_cflag |= CREAD | CLOCAL;
    SerialPortSettings.c_cc[VMIN] = 1;
    SerialPortSettings.c_cc[VTIME] = 50;

    if (tcsetattr(fd, TCSANOW, &SerialPortSettings) != 0)
        printf("\n  ERROR ! in Setting attributes\n");
    else
        printf("\n  BaudRate = 115200 \n  StopBits = 1 \n  Parity   = none \n");*/


}

void _serial::send_data(){
//    size_t size = 10;
//    u_char ddata[10]={0x55,'f','u','c','k',0x77};
//    write(fd, ddata, size);

    if (data.size != write(fd, data.raw_data, data.size)){
//        cout << "!!! send Data fail !!! \n" << endl;
    }
    else {
//        cout << "x:" << data.raw_data[1] << endl
//             << "y:" << data.raw_data[3] << endl
//             << "found:" << data.raw_data[5] << endl;
//        printf("send_yaw: %x  %x\n", data.raw_data[1], data.raw_data[2]);
//        printf("send_pitch: %x  %x\n", data.raw_data[3], data.raw_data[4]);
//        printf("send_found: %d\n", data.raw_data[5]);
    }
//    tcflush(fd, TCIOFLUSH);
}

void _serial::read_data(void){
        int size_read;
        unsigned char buffer[10], tmp;
        tcflush(fd, TCIFLUSH);
        size_read = read(fd, &tmp, 1);
        cout << "size: " << size_read << endl;
        if (data.head == tmp) {
            read(fd, buffer, 6);
            if (buffer[5] == data.end) {
                printf("read_yaw: %x  %x\n", buffer[0], buffer[1]);
                printf("read_pitch: %x  %x\n", buffer[2], buffer[3]);
                printf("read_found: %d\n", buffer[4]);
            }
        }
// read(fd, buffer, 6);
//        for (int i = 0; i < 10 ; i++)
//            buffer[i] = 0;
//        tcflush(fd, TCIOFLUSH);
//        read(fd, &tmp, 1);
////        cout << "size: " << read_size << endl;
//        if (tmp == 0x55){
//            read(fd, buffer, 5);
//            cout << "get buffer "<<endl;
//            if(buffer[4]== 0x77){
//                cout << "get data = " << char(buffer[0]) << char(buffer[1]) << char(buffer[2]) << char(buffer[3]) << endl;
//            }
//        }

}

//}

//void _serial::read_data(void){
//    int buffer_size;
//    static bool get_head = false;
//    unsigned char buffer[10],tmp;
////    ioctl(fd, FIONREAD, &buffer_size);
////    if(buffer_size > 20)
////        cout << "buffer size = "<< buffer_size <<endl;


//                read(fd, &tmp, 1);
//                if (tmp == 0x55){
//                    read(fd, buffer, 5);
//                    cout << "get buffer "<<endl;
//                    if(buffer[4]== 0x77){
//                        cout << "get data = " << char(buffer[0]) << char(buffer[1]) << char(buffer[2]) << char(buffer[3]) << endl;
//                    }

//                }




////        while(buffer_size > 0){
////            read(fd, buffer, 1);
//////            cout << "buffer = "<< int(buffer[0]) << endl;
////            if(buffer[0] == 0x11){
////                car_color = 1; //red
////            }
////            else if(buffer[0] == 0x22){
////                car_color = 0; //blue
////            }
////            ioctl(fd, FIONREAD, &buffer_size);
////        }

//}


void serial_transmit_data::get_xy_data(int16_t x, int16_t y,int8_t found)
{
    size = 7;
    raw_data[0] = head;
    raw_data[size-1] = end;
//    y = x = 999;
    raw_data[1] = y&0xff;
    raw_data[2] = (y>>8)&0xff;
    raw_data[3] = x&0xff;
    raw_data[4] =  (x>>8)&0xff;
    raw_data[5] = found;

}

