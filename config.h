#ifndef _CONFIG_H
#define _CONFIG_H

#pragma once

static const int on_off_socket = 1;
//socket send
static const int port = 50000;
static const float imageScaleFactor = 0.4;
static const char ip[] ="192.168.3.82"; // Add here your alternative ip (e.g "192.168.1.6")


//serila control
static const int on_off_serial= 1;

static const char *dev_lidar  = "/dev/ttyUSB0";
static const int baudrate_lidar = 9600;

static const char *dev_upper  = "/dev/ttyACM0";
static const int baudrate_upper = 115200;

unsigned char test[4]  = {128,6,3,119};
unsigned char hello_upper[5] = {'h','e','l','l','o'};

#endif
