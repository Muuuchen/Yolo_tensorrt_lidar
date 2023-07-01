#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H
#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netdb.h>
#include <sys/uio.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <fstream>
#define TRUE 1
#define FALSE 0

class TCPSocket
{
private:
    /* data */
    int serv_socket;
    int newSocket;
    long retval;
    char ip_address[30];
    int tcp_port;
    char received[30];
    sockaddr_in serv_addr;
    sockaddr_in newSocketAddr;
    char smsg[1500];
    char rmsg[1500];
public:
    TCPSocket(/* args */int port);
    ~TCPSocket();
    void tcpSend(std::string tcpMessage);
    std::string tcpReceive();
    
};


#endif