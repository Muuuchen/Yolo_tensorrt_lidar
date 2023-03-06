#ifndef UDPSOCKET_H
#define UDPSOCKET_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <string.h>
#include <iostream>
#define TRUE 1
#define FALSE 0

class UDPSocket
{
private:
    int sock;
    long retval;
    sockaddr_in outaddr;
    char ip[30];
    char received[30];
public:
    UDPSocket(const char* address, const int port, bool Datagram, bool reusesock);
    ~UDPSocket();

    long receive(char* msg, int msgsize);
    char* received_from();
    long send(const unsigned char* msg, int msgsize);
    long sendTo(const unsigned char* msg, int msgsize, const char* name);
    int getAddress(const char * name, char * addr);
    const char* getAddress(const char * name);
};
#endif