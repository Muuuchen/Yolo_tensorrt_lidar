#include<iostream>
#include<cstring>


int dealstoi(const char* buffer, int left ,int right)
{
    int i = left;
    int res = 0;
    while(buffer[i]=='0' && i <= right) i++;
    while(i<=right)
    {
        res*=10;
        res += buffer[i]-'0';
        i++;
    }
    return res;
}

void dealStrategy(std::string s)
{
    const char *dealBuffer = s.c_str(); 
    //x坐标是 45678 y坐标 9 10 11 12 13
    int mode = dealstoi(dealBuffer, 1,2);
    int x = dealstoi(dealBuffer, 4,8);
    int y = dealstoi(dealBuffer, 9,13);
    int index = dealstoi(dealBuffer,14,15);
    int parity = dealstoi(dealBuffer,16,17);
    std::cout<<mode<<x<<y<<index<<parity<<std::endl;
    if((mode + x+y+index)%31 == parity)
    {
        //success and send
        printf("OK");
    } 
    else
    {
        //jiaoyanshibai
        printf("error");
    }


}

int main()
{
    std::string s = "@00#10233012560019";
    dealStrategy(s);
    return 0;

}