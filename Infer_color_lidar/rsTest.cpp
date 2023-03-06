#include<iostream>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/h/rs_types.h>
#include <opencv2/opencv.hpp>

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

int select_point[2] = {320,240};
int out_flag = 0;
void on_Mouse(int event,int x,int y,int flags,void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN)//鼠标dianji将会触发此事件，CV_EVENT_MOUSEMOVE和0等效
    {
        select_point[0] = x;select_point[1] =y;out_flag=1;
	    std::cout << "x: "<<x << "y: "<< y << std::endl;//鼠标在图片每移动一次就会输出一次坐标
    }
}

void calAngle(rs2_intrinsics intrinsics , int x,int y)
{
    double cx = intrinsics.ppx;
    double cy = intrinsics.ppy;   
    double fx = intrinsics.fx;
    double fy = intrinsics.fy;  
    cv::Mat cam = (cv::Mat_<float>(3,3) << fx,0.0,cx,0.0,fy,cy,0.0,0.0,1.0);   
    //cv::Mat distort = cv::Mat_<float>(1,5) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4];   
    cv::Mat distort(1,5,CV_32FC1, intrinsics.coeffs);
    cv::Point2f pnt;
    std::vector<cv::Point2f> in;
    std::vector<cv::Point2f> out;
    in.push_back(cv::Point2f(x,y));
    cv::undistortPoints(in,out, cam, distort,cv::noArray(),cam);
    pnt = out.front();
    double rx=(x-cx)/fx;
    double ry=(y-cy)/fy;
    
    double rxNew=(pnt.x-cx)/fx;
    double ryNew=(pnt.y-cy)/fy;
    if(out_flag)
    {
        std::cout<< "x: "<<x<<" xNew:"<<pnt.x<<std::endl;
        std::cout<< "y: "<<y<<" yNew:"<<pnt.y<<std::endl;
        // 输出未去畸变时测得的角度和去畸变后测得的角度
        // 输出未去畸变时测得的角度和去畸变后测得的角度
        std::cout<< "angx: "<<atan(rx)/CV_PI*180<<" angleNew:"<<atan(rxNew)/CV_PI*180<<std::endl;
        std::cout<< "angy: "<<atan(ry)/CV_PI*180<<" angleNew:"<<atan(ryNew)/CV_PI*180<<std::endl;
        out_flag = 0;
    }
}
int main()
{

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    pipe.start(cfg);
    cv::namedWindow("src");//创建窗口
    cv::setMouseCallback("src", on_Mouse);
    
    while (1)
    {
        /* code */
    rs2::frameset pipe_frame = pipe.wait_for_frames();
    rs2::depth_frame depth = pipe_frame.get_depth_frame(); /// 获取深度图像数据
    rs2::frame color = pipe_frame.get_color_frame();       /// 获取彩色图像数据

    // 获取内参和外参
    rs2::stream_profile dprofile = depth.get_profile();
    rs2::stream_profile cprofile = color.get_profile();
    rs2::video_stream_profile cvsprofile(cprofile);
    rs2_intrinsics color_intrin = cvsprofile.get_intrinsics();
    rs2::video_stream_profile dvsprofile(dprofile);
    rs2_intrinsics depth_intrin = dvsprofile.get_intrinsics();
    
    cv::Mat img(cv::Size(640, 480), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
    calAngle(color_intrin,select_point[0],select_point[1]);
    cv::circle(img, cv::Point(select_point[0],select_point[1]), 3, cv::Scalar(255, 0, 0), -1);
    cv::imshow("src",img);
    int key = cv::waitKey(1);
    if (key == 'q') break;
    }
}