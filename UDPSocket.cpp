#include"UDPSocket.h"

UDPSocket::UDPSocket(const char* address, const int port, bool broadcast, bool reusesock)
{
    sockaddr_in addr;
    sock = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP); //DGRAM no Stream
    if(sock < 0)
    {
        std::cerr << "Error establishing the udp socket" <<std::endl;
        exit(0);
    }
    std::cout<<"udp sock set successfully"<<std::endl;
    //AF_INET ipv4地址  DGRAM 是数据传输方式 面向无链接 UDP
    //set bind address
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET; //地址族（Address Family），也就是地址类型
    addr.sin_addr.s_addr = htonl(INADDR_ANY);//32位IP地址
    addr.sin_port = htons(port);//16位的端口号

    //set up sending address
    memset(&outaddr,0,sizeof(outaddr));
    outaddr.sin_family = AF_INET;
    outaddr.sin_addr.s_addr = inet_addr(address);
    outaddr.sin_port = htons(port); //duan口号需要用 htons() 函数转换，
    
    int Optval = 1;

    if(broadcast)
        retval  = setsockopt(sock,SOL_SOCKET,SO_BROADCAST,&Optval,sizeof(Optval));
    if(reusesock)
        retval = setsockopt(sock, SOL_SOCKET,SO_REUSEADDR,&Optval,sizeof(Optval));
    retval  = bind(sock,(struct  sockaddr *)&addr, sizeof(addr));
    if(retval <0)
    {
        std::cerr<< "Error bind the udp socket"<<std::endl;
        exit(0);
    }
    std::cout<<"udp sock bind successfully"<<std::endl;
    //服务器端要用 bind() 函数将套接字与特定的IP地址和端口绑定起来，只有这样，流经该IP地址和端口的数据才能交给套接字处理；
    //而客户端要用 connect() 函数建立连接。
}

UDPSocket::~UDPSocket()
{
    close(sock);
}

int UDPSocket::getAddress(const char * name, char * addr)
{
    struct hostent *hp;
    if ((hp = gethostbyname(name)) == NULL) return (0);
    strcpy(addr, inet_ntoa( *( struct in_addr*)( hp->h_addr)));
    return (1);
}

const char* UDPSocket::getAddress(const char * name)
{
    struct hostent *hp;
    if ((hp = gethostbyname(name)) == NULL) return (0);
    strcpy(ip, inet_ntoa( *( struct in_addr*)( hp->h_addr)));
    return ip;
}

long UDPSocket::receive(char* msg, int msgsize)
{
    struct sockaddr_in sender;
    socklen_t sendersize = sizeof(sender);
    int retval = recvfrom(sock,msg,msgsize,0, (struct sockaddr *)&sender, &sendersize);
    strcpy(received,inet_ntoa(sender.sin_addr));
    return retval;
}

char* UDPSocket::received_from()
{
    return received;
}

long UDPSocket::send(const unsigned char* msg, int msgsize)
{
    return sendto(sock, msg, msgsize, 0, (struct sockaddr *)&outaddr,sizeof(outaddr));
}

long UDPSocket::sendTo(const unsigned char* msg, int msgsize, const char* addr)
{
    outaddr.sin_addr.s_addr = inet_addr(addr);
    return sendto(sock, msg, msgsize, 0, (struct sockaddr *)&outaddr,sizeof(outaddr));
}