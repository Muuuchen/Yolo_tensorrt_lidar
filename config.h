#ifndef _CONFIG_H
#define _CONFIG_H
#include <string.h>
#pragma once

static const int only_self = 0;

static const int use_tcp = 1;
static const int on_off_socket = 1;
//socket send
static const int port = 50000;
static const char ip[] ="192.168.186.137"; // Add here your alternative ip (e.g "192.168.1.6")
static const float imageScaleFactor = 0.4;


//serila control
static const int on_off_serial= 1;

static const int savedFlag = 0;
static const string outputVideopath = "./saved.avi";

// change /dev/udev/rules.d   bind-usb.rules

static const char *dev_lidar  = "/dev/lidarserial";
static const int baudrate_lidar = 9600;

// static const char *dev_upper  = "/dev/upperserial";
// static const int baudrate_upper = 9600;

static const int use_chasis = 1;

static const char *dev_chasis = "/dev/upperserial";
static const int bundrate_chasis = 9600;

unsigned char test[4]  = {128,6,3,119};
unsigned char hello_upper[5] = {'h','e','l','l','o'};

#endif
