#ifndef _DEPTH_MODE_H
#define _DEPTH_MODE_H

#include<iostream>
#include<ctime>
#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
#include<librealsense2/h/rs_types.h>
#include <opencv2/opencv.hpp>
int main()
{

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    rs2::pipeline pipe;
    pipe.start(cfg);
    std::cerr << "Pipline Started!" <<std::endl;
    while(1)
    {
        auto start = std::chrono::system_clock::now();
        rs2::frameset pipe_frame = pipe.wait_for_frames();
        rs2::depth_frame depth  = pipe_frame.get_depth_frame(); ///获取深度图像数据
        rs2::frame color = pipe_frame.get_color_frame();  ///获取彩色图像数据

        rs2::stream_profile dprofile =  depth.get_profile();
        rs2::stream_profile cprofile =  color.get_profile();
        rs2::video_stream_profile cvsprofile(cprofile);
        rs2_intrinsics color_intrin =  cvsprofile.get_intrinsics();
        rs2::video_stream_profile dvsprofile(dprofile);
        rs2_intrinsics depth_intrin =  dvsprofile.get_intrinsics();

        rs2_extrinsics depth2color = dprofile.get_extrinsics_to(cprofile); //depth2color
        rs2_extrinsics color2depth = cprofile.get_extrinsics_to(dprofile); //depth2color


        if (!color || !depth) continue;            ///如果获取不到数据则退出         
        cv::Mat img(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat img_depth(cv::Size(640, 480), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
        cv::imshow("depth", img_depth);
        
        int key = cv::waitKey(1);
        if( key == 'q')
            break;
        
    }

    return ;
}

#endif