
#include "TCPSocket.h"

TCPSocket::TCPSocket(int port)
{
    tcp_port = port;
    bzero((char*)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(tcp_port);

    serv_socket = socket(AF_INET, SOCK_STREAM,0);
    if(serv_socket < 0)
    {
        std::cerr << "Error establishing the server socket" <<std::endl;
        exit(0);
    }
    int optval = 1;
    setsockopt(serv_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    int bindStatus = bind(serv_socket, (struct sockaddr*) &serv_addr, sizeof(serv_addr));
    if(bindStatus <0)
    {
        std::cerr<< "Error bind the udp socket"<<std::endl;
        exit(0);
    }
    std::cout<<"Waiting for a client to connect..."<<std::endl;
    listen(serv_socket, 5);
    socklen_t newSocketAddrSize = sizeof(newSocketAddr);
    newSocket  = accept(serv_socket,(sockaddr*)&newSocketAddr,&newSocketAddrSize);
    if(newSocket < 0)
    {
        std::cerr << "Error accept request from client"<<std::endl;
        exit(1);
    }
    std::cout<<"Connected with Clinet"<<std::endl;
}

TCPSocket::~TCPSocket()
{
    close(serv_socket);
    close(newSocket);
    std::cerr<<"current connection is closed"<<std::endl;
}

void TCPSocket::tcpSend(std::string tcpMessage)
{
    int bytesWritten = 0;
    memset(&smsg,0,sizeof(smsg));
    strcpy(smsg,tcpMessage.c_str());
    bytesWritten += send(newSocket, (char*)&smsg,strlen(smsg), 0);
    return;
}

std::string TCPSocket::tcpReceive()
{
    int bytesRead = 0;
    memset(&rmsg,0,sizeof(rmsg));
    bytesRead = recv(newSocket, (char*)&rmsg, sizeof(rmsg), 0);
    std::string s = rmsg;

    std::cout<<s<<std::endl;
    return s;
}
