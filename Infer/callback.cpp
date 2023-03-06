#include <iostream>
#include <opencv2/opencv.hpp>
 
//Event
 
#define CV_EVENT_MOUSEMOVE 0             //滑动
#define CV_EVENT_LBUTTONDOWN 1           //左键点击
#define CV_EVENT_RBUTTONDOWN 2           //右键点击
#define CV_EVENT_MBUTTONDOWN 3           //中键点击
#define CV_EVENT_LBUTTONUP 4             //左键放开
#define CV_EVENT_RBUTTONUP 5             //右键放开
#define CV_EVENT_MBUTTONUP 6             //中键放开
#define CV_EVENT_LBUTTONDBLCLK 7         //左键双击
#define CV_EVENT_RBUTTONDBLCLK 8         //右键双击
#define CV_EVENT_MBUTTONDBLCLK 9         //中键双击

using namespace cv;
using namespace std;
 
void on_Mouse(int event, int x, int y, int flags, void* param) {
	if (event == CV_EVENT_MOUSEMOVE)//鼠标移动将会触发此事件，CV_EVENT_MOUSEMOVE和0等效
	cout << "x:"<<x << "y："<< y << endl;//鼠标在图片每移动一次就会输出一次坐标
}
 
int main()
{
	Mat src = imread("C:\\Users\\45374\\Desktop\\一些经典算法的代码整理\\code\\img\\lena.jpg");
	
	namedWindow("src");//创建窗口
 
	setMouseCallback("src", on_Mouse);//窗口名必须和之前创建窗口同名
 
	imshow("src", src);
 
	waitKey();
}