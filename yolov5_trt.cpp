#include <iostream>
#include <chrono>
#include <cmath>
#include<pthread.h>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
#include "calibrator.h"
#include <ctime>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/h/rs_types.h>
#include <opencv2/opencv.hpp>
#include "SerialInterface.h"
#include<netinet/in.h>
#include<arpa/inet.h>
#include"UDPSocket.h"
#include"config.h"
#include"TCPSocket.h"
#include"_track.h"


#define USE_FP16 // set USE_INT8 or USE_FP16 or USE_FP32
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
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

#define KEYDOWN(vk) (0x8000 & ::GetAsy)


clock_t t_start, t_end;
static Logger gLogger;

char *my_classes[] = {"cylinder"};


// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1; // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char *INPUT_BLOB_NAME = "data";
const char *OUTPUT_BLOB_NAME = "prob";
#define DEVICE 0 // GPU id

int select_point[2] = {320,240};
int send_point[2] = {320,240};
int out_flag = 0;
int send_index = 0;
int select_index=0;
unsigned char buff_lidar[512];
unsigned char buff_upper[1];
serialPort myserial_lidar;
serialPort myserial_upper;
int nread_upper,nwrite_upper;
int i,nread,nwrite;
pthread_mutex_t mutex_upper;
pthread_mutex_t mutex_lidar;
UDPSocket *s;
TCPSocket *tcpSocket;
int tcpAlive = 0;


static int get_width(int x, float gw, int divisor = 8)
{
    return int(ceil((x * gw) / divisor)) * divisor;
}

static int get_depth(int x, float gd)
{
    if (x == 1)
        return 1;
    int r = round(x * gd);
    if (x * gd - int(x * gd) == 0.5 && (int(x * gd) % 2) == 0)
    {
        --r;
    }
    return std::max<int>(r, 1);
}

void* readLidar(void* args)
{
    while (1)
    {

        myserial_lidar.readBuffer(buff_lidar, 10);
        // for (int i = 0; i < 10; i++)
        // {
        //     printf("%c", std::thread thread_lidar;buff[i]);
        // }
        // printf("\n");
    }
}
//
void* readUpper(void* args)
{
        while(1)
        {
            // pthread_mutex_lock(&mutex_upper);
            // myserial_upper.readBuffer(buff_upper,1);
            // int temp = (char)buff_upper[0] - '0';
            // if(temp <0) continue;
            
            // select_index = temp;
            // std::cerr<<"接受字符串信号成功"<<std::endl;
            // std::cerr<<"select_index: "<<select_index<<std::endl;
            // out_flag = 1;
            // memset(buff_upper, 0, sizeof(buff_upper));  
            // pthread_mutex_unlock(&mutex_upper);
            std::string s = tcpSocket->tcpReceive();
            if(s == "e")
            {
                std::cerr<<"client exit unexception"<<std::endl;
                tcpAlive = 0;
                pthread_exit(NULL);
                break;
            }
            int sint = std::stoi(s);
            select_index = sint;
            out_flag = 1;
            std::cout<<"read char: "<<s<<std::endl;
        }
}
double calAngle(rs2_intrinsics intrinsics , int x,int y)
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
        std::cout<< "x_pixel:"<<pnt.x<<std::endl;
        std::cout<< "y_pixel:"<<pnt.y<<std::endl;
        out_flag = 0;
        float angle = atan(rxNew);
        
        unsigned char send_buff[14];
        send_buff[0] = '9';
	    send_buff[1] = '$';
        send_buff[2] = (unsigned char)buff_lidar[5];
        send_buff[3] = (unsigned char)buff_lidar[7];
        send_buff[4] = (unsigned char)buff_lidar[8];
        send_buff[5] = (unsigned char)buff_lidar[9];
        send_buff[6] = '#';
        send_buff[7] =  (angle - 0.0 > 0.0) ? '+':'-';
        if(angle < 0) angle = -angle;
	    int k = 0;
        while(k < 4)
        {
            send_buff[k+8] = (int)angle + '0';
            angle -= (int)angle;
            angle *= 10;
            k++;
        }
        send_buff[12] = '!';
	    send_buff[13] = 0;
        std::cerr<<"发送的字符串为："<<(char*)send_buff<<std::endl;
        printf("\n");
        if(on_off_serial)
        {
            myserial_upper.writeBuffer(send_buff, 13);
            memset(send_buff, 0, sizeof(send_buff));
        }
    }
}
        
void doInference(IExecutionContext &context, cudaStream_t &stream, void **buffers, float *input, float *output, int batchSize)
{
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

bool tracking_cmp(TrackingBox a, TrackingBox b){
    return a.id<b.id;
}


int main(int argc, char **argv)
{

    VideoWriter outputVideo;

    //input para define
    if(savedFlag)
    {
        int fps = 30;
        outputVideo.open(outputVideopath, cv::VideoWriter::fourcc('X','V','I','D'),fps,cv::Size(640,480),true);   
        if(!outputVideo.isOpened())
        {
            cout<< "failed to oppen!" <<endl;
            return -1; 
        }
    }


    tcpSocket = new TCPSocket(9876);
    tcpAlive = 1;



    ///
    cudaSetDevice(DEVICE);

    std::string engine_name = "../weights/best.engine";
    bool is_p6 = false;
    float gd = 0.0f, gw = 0.0f;
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good())
    {
        std::cerr << "read " << engine_name << " error!" << std::endl;
        return -1;
    }
    char *trtModelStream = nullptr;
    size_t size = 0;
    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);
    trtModelStream = new char[size];
    assert(trtModelStream);
    file.read(trtModelStream, size);
    file.close();
    // prepare input data ---------------------------
    static float data[BATCH_SIZE * 3 * INPUT_H * INPUT_W];
    // for (int i = 0; i < 3 * INPUT_H * INPUT_W; i++)
    //     data[i] = 1.0;
    static float prob[BATCH_SIZE * OUTPUT_SIZE];
    IRuntime *runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    ICudaEngine *engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    IExecutionContext *context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    void *buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine->getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine->getBindingIndex(OUTPUT_BLOB_NAME);
    assert(inputIndex == 0);
    assert(outputIndex == 1);
    // Create GPU buffers on device
    CUDA_CHECK(cudaMalloc(&buffers[inputIndex], BATCH_SIZE * 3 * INPUT_H * INPUT_W * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&buffers[outputIndex], BATCH_SIZE * OUTPUT_SIZE * sizeof(float)));
    // Create stream
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));

    std::cerr << "Read Successfully!" <<std::endl;

    // VIDEO STREAM
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    pipe.start(cfg);
    std::cerr << "Pipline Started!" <<std::endl;

    cout<<"serialPort Test"<<endl;
    myserial_lidar.OpenPort(dev_lidar);
    myserial_lidar.setup(baudrate_lidar,0,8,1,'N'); 
    myserial_upper.OpenPort(dev_upper);
    myserial_upper.setup(baudrate_upper,0,8,1,'N'); 
    myserial_lidar.writeBuffer(test, 4);
    myserial_upper.writeBuffer(hello_upper,5);
    std::cout<<"ready to setup udp"<<std::endl;
    s = new UDPSocket(ip, port, TRUE, TRUE);
    printf("Socket successful!");
    //cv::namedWindow("src");//创建窗口

    pthread_t thread_lidar_tid;
    int ret = pthread_create(&thread_lidar_tid,NULL, readLidar,NULL);
    
    pthread_t thread_upper_tid;
    int ret_u = pthread_create(&thread_upper_tid,NULL, readUpper,NULL);
    

    int key;
    int fcount = 0;

    int tcnt = 0;
    Init_Tracker();
    while (1)
    {
        //恢复工作
        if(tcpAlive == 0)
        {       
            delete tcpSocket;
            tcpSocket = new TCPSocket(9876);
            pthread_create(&thread_upper_tid,NULL, readUpper,NULL);
            tcpAlive = 1;
        }
        rs2::frameset pipe_frame = pipe.wait_for_frames();
        //rs2::depth_frame depth = pipe_frame.get_depth_frame(); /// 获取深度图像数据
        rs2::frame color = pipe_frame.get_color_frame();       /// 获取彩色图像数据
        // 获取内参和外参
        //rs2::stream_profile dprofile = depth.get_profile();
        rs2::stream_profile cprofile = color.get_profile();
        rs2::video_stream_profile cvsprofile(cprofile);
        rs2_intrinsics color_intrin = cvsprofile.get_intrinsics();
        //rs2::video_stream_profile dvsprofile(dprofile);
        //rs2_intrinsics depth_intrin = dvsprofile.get_intrinsics();
        //rs2_extrinsics depth2color = dprofile.get_extrinsics_to(cprofile); // depth2color
        //rs2_extrinsics color2depth = cprofile.get_extrinsics_to(dprofile); // depth2color

        //可视化内参矩阵
        auto principal_point = std::make_pair(color_intrin.ppx, color_intrin.ppy);
        auto focal_length = std::make_pair(color_intrin.fx, color_intrin.fy);
        rs2_distortion model = color_intrin.model;

        // std::cout << "Principal Point         : " << principal_point.first << ", " << principal_point.second << std::endl;
        // std::cout << "Focal Length            : " << focal_length.first << ", " << focal_length.second << std::endl;
        // std::cout << "Distortion Model        : " << model << std::endl;
        // std::cout << "Distortion Coefficients : [" << color_intrin.coeffs[0] << "," << color_intrin.coeffs[1] << "," <<
        //     color_intrin.coeffs[2] << "," << color_intrin.coeffs[3] << "," << color_intrin.coeffs[4] << "]" << std::endl;

        //if (!color || !depth)
        if (!color)
            break; /// 如果获取不到数据则退出
        cv::Mat img(cv::Size(640, 480), CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP);
        //cv::Mat img_depth(cv::Size(640, 480), CV_16U, (void *)depth.get_data(), cv::Mat::AUTO_STEP);

        fcount++;
        for (int b = 0; b < fcount; b++)
        {
            // cv::Mat img = cv::imread(img_dir + "/" + file_names[f - fcount + 1 + b]);
            if (img.empty())
                continue;
            cv::Mat pr_img = preprocess_img(img, INPUT_W, INPUT_H); // letterbox BGR to RGB
            int i = 0;
            for (int row = 0; row < INPUT_H; ++row)
            {
                uchar *uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col)
                {
                    data[b * 3 * INPUT_H * INPUT_W + i] = (float)uc_pixel[2] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }
        }
        auto start = std::chrono::system_clock::now();
        doInference(*context, stream, buffers, data, prob, BATCH_SIZE);
        auto end = std::chrono::system_clock::now();
        // std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
        int fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::vector<std::vector<Yolo::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++)
        {
            auto &res = batch_res[b];
            nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
        }

         std::vector<TrackingBox> detData;
        
        for (int b = 0; b < fcount; b++){
            auto &res = batch_res[b];
            for (size_t j = 0; j < res.size(); j++){
                TrackingBox tb;
                tb.box = cv::Rect_<float>(res[j].bbox[0], res[j].bbox[1], res[j].bbox[2], res[j].bbox[3]);
                // tb.box = cv::Rect_<float>(cv::Point_<float>(res[j].bbox[0], res[j].bbox[1]), Point_<float>(res[j].bbox[0]+res[j].bbox[2], res[j].bbox[1]+res[j].bbox[3]));
                tb.id = tcnt++;
                detData.push_back(tb);
            }
        }

        vector<TrackingBox> frameTrackingResult;

        Do_Track(&detData, &frameTrackingResult);

        std::sort(frameTrackingResult.begin(), frameTrackingResult.end(), tracking_cmp);
        int tmp_cnt = 0;
        for(auto tb : frameTrackingResult){
            float _bbox[4] = {tb.box.x, tb.box.y,tb.box.width,tb.box.height};
            cv::Rect r = get_rect(img, _bbox);
	    if(select_index == tmp_cnt)
	   	 rectangle(img,r, cv::Scalar(255, 255, 0), 2);
	    else
		    rectangle(img,r,cv::Scalar(0,0,0),2);
	    std::string label = cv::format("%d",tmp_cnt) ;
            cv::putText(img, label, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
            tmp_cnt++;
        }
        if(select_index < frameTrackingResult.size())
        {
            send_point[0] = frameTrackingResult[select_index].box.x;
            send_point[1] = frameTrackingResult[select_index].box.y;
        }
        //std::cout<<"start touch!"<<std::endl;
        // cv::circle(img, cv::Point(send_point[0],send_point[1]), 3, cv::Scalar(255, 0, 0), -1);
        //cv::line(img,cv::Point(send_point[0], 1), cv::Point(send_point[0],639),cv::Scalar(255, 0, 0),1,8);
        float xnew = calAngle(color_intrin,send_point[0],send_point[1]);
        
        //cv::imshow("src", img);
        if(on_off_socket)
        {
            cv::resize(img, img, cv::Size(img.cols * imageScaleFactor, img.rows * imageScaleFactor));
            int pixel_number = img.rows * img.cols / 2;
            std::vector<uchar> buf(pixel_number);
            cv::imencode(".jpg", img, buf);
            int length = buf.size();
            std::string str = "11111111";
            s->send(reinterpret_cast<const unsigned char*>(str.c_str()), str.length());
            /* Transmit chunks of 1024 bytes */
            int chunkSize = 1024;
            for (size_t i = 0; i < buf.size(); i += chunkSize) {
                std::vector<uchar> tempVec = std::vector<uchar>(buf.begin() + i, buf.begin() + i + chunkSize);
                s->send(tempVec.data(), chunkSize);
            }
            /* End image transmission */
            str = "22222222";
            s->send(reinterpret_cast<const unsigned char*>(str.c_str()), str.length());
            //printf("Sending image  to %s:%d with length %d bytes \n", ip, port, length);
        }
        if(savedFlag)
        {
            outputVideo<< img;
        }

        // cv::imshow("img", img);
        key = cv::waitKey(1);
        if( key == 'q')
        {
            break;
        }
        fcount = 0;
    }
    // Release stream and buffers
    cudaStreamDestroy(stream);
//     CUDA_CHECK(cu#include "./sort-cpp-master/sort-c++/Hungarian.h"
// #include "./sort-cpp-master/sort-c++/KalmanTracker.h"daFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
    return 0;
}