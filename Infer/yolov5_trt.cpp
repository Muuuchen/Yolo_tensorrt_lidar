#include <iostream>
#include <chrono>
#include <cmath>
#include "cuda_utils.h"
#include "logging.h"
#include "common.hpp"
#include "utils.h"
#include "calibrator.h"
#include<ctime>
#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
#include<librealsense2/h/rs_types.h>
#include <opencv2/opencv.hpp>
#include "SerialInterface.h"
#include<netinet/in.h>
#include<arpa/inet.h>
#include<iostream>
#include"UDPSocket.h"
#include"config.h"

#define USE_FP16  // set USE_INT8 or USE_FP16 or USE_FP32
#define DEVICE 0  // GPU id
#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
#define BATCH_SIZE 1
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


struct Rect_data
{
    int index = 0;
    float coordinate[3] = {0.0,0.0,0.0};
    int center[2] = {640/2,480/2};
    float depth = 0.0;
};

clock_t t_start,t_end;
static Logger gLogger;
int select_x=0;
int select_y = 0;

char *my_classes[]={ "cylinder" };



// stuff we know about the network and the input/output blobs
static const int INPUT_H = Yolo::INPUT_H;
static const int INPUT_W = Yolo::INPUT_W;
static const int CLASS_NUM = Yolo::CLASS_NUM;
static const int OUTPUT_SIZE = Yolo::MAX_OUTPUT_BBOX_COUNT * sizeof(Yolo::Detection) / sizeof(float) + 1;  // we assume the yololayer outputs no more than MAX_OUTPUT_BBOX_COUNT boxes that conf >= 0.1
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";




void on_Mouse(int event,int x,int y,int flags,void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN)//鼠标移动将会触发此事件，CV_EVENT_MOUSEMOVE和0等效
	{
    cout << "click_pos: "<<"x: "<<x << "y: "<< y << endl;//鼠标在图片每移动一次就会输出一次坐标
    select_x = x ; select_y = y;    //
    }
}

static float get_pixel_dis(int x1,int y1,int x2,int y2)
{
    return float(sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)));
}

static int get_width(int x, float gw, int divisor = 8) {
    return int(ceil((x * gw) / divisor)) * divisor;
}

static int get_depth(int x, float gd) {
    if (x == 1) return 1;
    int r = round(x * gd);
    if (x * gd - int(x * gd) == 0.5 && (int(x * gd) % 2) == 0) {
        --r;
    }
    return std::max<int>(r, 1);
}

float get_depth_arg(int x,int y, cv::Mat img_depth,rs2::depth_frame depth){
    float _depth = img_depth.at<float>(x ,y) ;
    _depth = depth.get_distance(x ,y);
    return _depth;
}

void doInference(IExecutionContext& context, cudaStream_t& stream, void **buffers, float* input, float* output, int batchSize) {
    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CUDA_CHECK(cudaMemcpyAsync(buffers[0], input, batchSize * 3 * INPUT_H * INPUT_W * sizeof(float), cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchSize * OUTPUT_SIZE * sizeof(float), cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);
}

void get_column_center_position(int bbox[4], int * depth_image,
rs2_intrinsics *color_intrin, rs2_intrinsics *depth_intrin,
rs2_extrinsics *color2depth, rs2_extrinsics *depth2color,
rs2::depth_frame *depth, float coor[3]);
void get_3d_camera_coordinate(int position[2],float other_pixel[3], float depth ,
 rs2_intrinsics *color_intrin,rs2_intrinsics *depth_intrin, rs2_extrinsics *depth2color);

int main(int argc, char** argv) {
    cudaSetDevice(DEVICE);

    std::string engine_name = "../weights/best.engine";
    bool is_p6 = false;
    float gd = 0.0f, gw = 0.0f;
    //std::string img_dir = "../samples";

    //这里序列化生成engine文件
    // create a model using the API directly and serialize it to a stream
    // if (!wts_name.empty()) {
    //     IHostMemory* modelStream{ nullptr };
    //     APIToModel(BATCH_SIZE, &modelStream, is_p6, gd, gw, wts_name);
    //     assert(modelStream != nullptr);
    //     std::ofstream p(engine_name, std::ios::binary);
    //     if (!p) {
    //         std::cerr << "could not open plan output file" << std::endl;
    //         return -1;
    //     }
    //     p.write(reinterpret_cast<const char*>(modelStream->data()), modelStream->size());
    //     modelStream->destroy();
    //     return 0;
    // }
    
    
    //反序列化推理
    // deserialize the .engine and run inference
    std::ifstream file(engine_name, std::ios::binary);
    if (!file.good()) {
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
    //for (int i = 0; i < 3 * INPUT_H * INPUT_W; i++)
    //    data[i] = 1.0;
    static float prob[BATCH_SIZE * OUTPUT_SIZE];
    IRuntime* runtime = createInferRuntime(gLogger);
    assert(runtime != nullptr);
    ICudaEngine* engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    IExecutionContext* context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;
    assert(engine->getNbBindings() == 2);
    void* buffers[2];
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
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    rs2::pipeline pipe;     
    pipe.start(cfg);
    std::cerr << "Pipline Started!" <<std::endl;
    
    char buff[512];
    serialPort myserial;
    int i,nread,nwrite;
    cout<<"serialPort Test"<<endl;
    myserial.OpenPort(dev);
    myserial.setup(baudrate,0,8,1,'N'); 

    UDPSocket *s = new UDPSocket(ip, port, TRUE, TRUE);
    if(on_off_socket)
        printf("Socket successful!");
    int key;
    int fcount = 0;
    cv::namedWindow("src");//创建窗口
    cv::setMouseCallback("src", on_Mouse);
    while(1)
    { 
        auto start = std::chrono::system_clock::now();
        rs2::frameset pipe_frame = pipe.wait_for_frames();
        rs2::depth_frame depth  = pipe_frame.get_depth_frame(); ///获取深度图像数据
        rs2::frame color = pipe_frame.get_color_frame();  ///获取彩色图像数据


        // 获取内参和外参         
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

        fcount++;
        for (int b = 0; b < fcount; b++) 
        {
            //cv::Mat img = cv::imread(img_dir + "/" + file_names[f - fcount + 1 + b]);
            if (img.empty()) continue;
            cv::Mat pr_img = preprocess_img(img, INPUT_W, INPUT_H); // letterbox BGR to RGB
            int i = 0;
            for (int row = 0; row < INPUT_H; ++row) {
                uchar* uc_pixel = pr_img.data + row * pr_img.step;
                for (int col = 0; col < INPUT_W; ++col) {
                    data[b * 3 * INPUT_H * INPUT_W + i] = (float)uc_pixel[2] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + INPUT_H * INPUT_W] = (float)uc_pixel[1] / 255.0;
                    data[b * 3 * INPUT_H * INPUT_W + i + 2 * INPUT_H * INPUT_W] = (float)uc_pixel[0] / 255.0;
                    uc_pixel += 3;
                    ++i;
                }
            }
        }
        doInference(*context, stream, buffers, data, prob, BATCH_SIZE);
        std::vector<std::vector<Yolo::Detection>> batch_res(fcount);
        for (int b = 0; b < fcount; b++) {
            auto& res = batch_res[b];
            nms(res, &prob[b * OUTPUT_SIZE], CONF_THRESH, NMS_THRESH);
        }
        for (int b = 0; b < fcount; b++) {
                auto& res = batch_res[b];
                //std::cout << res.size() << std::endl;
                //cv::Mat img = cv::imread(img_dir + "/" + file_names[f - fcount + 1 + b]);
                Rect_data*  anchor= new Rect_data[res.size()] ;
                for (size_t j = 0; j < res.size(); j++) {
                    anchor[j].index = j;
                    cv::Rect r = get_rect(img, res[j].bbox);

                    if (r.x >= 640) r.x = 639;
                    if (r.y >= 480) r.y =479;
                    if (r.x + r.width >= 640) r.width = 639 - r.x;
                    if (r.y + r.height >= 480) r.height = 479 - r.y;
                    
                    if (r.x < 0) r.width = r.width + r.x , r.x = 0;
                    if (r.y < 0) r.height = r.height + r.y , r.y = 0;
                    if (r.x + r.width >= 640) r.width = 639 - r.x;
                    if (r.y + r.height >= 480) r.height = 479 - r.y;
                    

                    cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);

                    int position[2] = {r.x+r.width/2, r.y+r.height/2};
                    float coor[3] = {0,0,0};
                    //float sumx = dep_middle;
                    //int sumid = 1;
                    float minnx = 100000.0;
                    int minpx= position[0];
                    int minpy=position[1];
                    //printf("################################");
                    for (int i = 0; i <= r.width/2; i++){
                        if (position[0] - i >= 0) {
                            float nowx = get_depth_arg(position[0] - i ,position[1] ,img_depth ,depth);
                            ///printf("nowx : %.4f", nowx);
                            if (nowx < minnx && nowx > 0.001) {minnx = nowx; minpx =position[0]-i;}; 
                        }
                        if (position[0] + i < 640) {
                            float nowx = get_depth_arg(position[0] + i ,position[1] ,img_depth ,depth);
                            //printf("nowx : %.4f", nowx);
                            if (nowx < minnx && nowx > 0.001) {minnx = nowx;minpx= position[0]+i;};
                        }
                    }
                    //printf("################################");
                    float _depth = minnx;
                    int minposition[2] = {minpx,minpy};
                    get_3d_camera_coordinate(minposition,coor,_depth,&color_intrin,&depth_intrin,&depth2color);
                    anchor[j].center[0] = position[0];// minpx
                    anchor[j].center[1] = position[1];
                    anchor[j].coordinate[0] =coor[0];
                    anchor[j].coordinate[1] =coor[1];
                    anchor[j].coordinate[2] =coor[2];
                    anchor[j].depth = _depth;
                    
                    //printf("depth : minnx %.4f\n", _depth);
                    //std::string label = my_classes[(int)res[j].class_id] + _space + std::to_string(_depth);
                    //cv::putText(img, label1, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                    //cv::putText(img, label2, cv::Point(r.x, r.y + r.height + 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
                    auto end = std::chrono::system_clock::now();
                    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
                    int fps = 1000.0/std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
                    std::string jetson_fps = "Jetson NX FPS: " + std::to_string(fps);
                    cv::putText(img, jetson_fps, cv::Point(11,80), cv::FONT_HERSHEY_PLAIN, 0.3, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
                    cv::Point p; p.x = minposition[0]; p.y = minposition[1];
                    cv::circle(img, p, 3,cv::Scalar(255,0,0),-1);
            } 
                //计算当前要返回竹子的坐标  
                //判断发送哪个
                int send_index = -1;
                
                float min_pix_dis =10000.0;
                for(int i = 0;i<res.size();i++)
                {
                    float now_pix_dis  = anchor[i].depth;
                    if (now_pix_dis > 100) continue;
                    if(now_pix_dis < min_pix_dis && (now_pix_dis - 0.05) > 0)
                    {
                        min_pix_dis = now_pix_dis;
                        send_index = i;
                    }
                }
                printf("send _index %d ",send_index);
                //发送及打印
                if(on_off_socket)
                {
                    cv::resize(img, img, cv::Size(img.cols * imageScaleFactor, img.rows * imageScaleFactor));
                    int pixel_number = img.rows * img.cols / 2;
                    std::vector<uchar> buf(pixel_number);
                    cv::imencode(".jpg", img, buf);
                    int length = buf.size();
                    std::string str = "buffer_start";
                    s->send(reinterpret_cast<const unsigned char*>(str.c_str()), str.length());
                    /* Transmit chunks of 1024 bytes */
                    int chunkSize = 1024;
                    for (size_t i = 0; i < buf.size(); i += chunkSize) {
                        std::vector<uchar> tempVec = std::vector<uchar>(buf.begin() + i, buf.begin() + i + chunkSize);
                        s->send(tempVec.data(), chunkSize);
                    }
                    /* End image transmission */
                    str = "buffer_end";
                    s->send(reinterpret_cast<const unsigned char*>(str.c_str()), str.length());
                    printf("Sending image  to %s:%d with length %d bytes \n", ip, port, length);
                }



                if(send_index >= 0){
                    std::string  send_flag = "able to send";
                    cv::putText(img, "able to send" ,cv::Point(anchor[send_index].center[0],anchor[send_index].center[1]), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255), 2);
                    std::string _space = "   ";
                    std::string label1 =  _space
                    +std::to_string(anchor[send_index].coordinate[0])+_space
                    +std::to_string(anchor[send_index].coordinate[1])+_space
                    +std::to_string(anchor[send_index].coordinate[2]);
                    std::string label2 =std::to_string(anchor[send_index].depth);
                    std::string label = label1 + label2;
                    if(on_off_serial)
                    {
                        strcpy(buff, label.c_str());
                        int len = strlen(buff);
                        nwrite = myserial.writeBuffer(buff, len);
                    }
                    printf("send index : %d", send_index);
                    printf("corrdinate : %s \n depth : %s \n",label1.c_str(),label2.c_str());
                }
                else{
                    std::cout<<"error now correct object !\n";
                }
        }
        cv::imshow("src", img);
        cv::imshow("depth", img_depth);
        // ============debug=================//
        //保存img和深度
        static int save_times = 1;
        if(save_times == 1)
        {
            
        }

        key = cv::waitKey(1);
        if (key == 'q'){
            break; 
        }
        fcount = 0;
    }
   
    // Release stream and buffers
    cudaStreamDestroy(stream);
    CUDA_CHECK(cudaFree(buffers[inputIndex]));
    CUDA_CHECK(cudaFree(buffers[outputIndex]));
    // Destroy the engine
    context->destroy();
    engine->destroy();
    runtime->destroy();
    // Print histogram of the output distribution
    //std::cout << "\nOutput:\n\n";
    //for (unsigned int i = 0; i < OUTPUT_SIZE; i++)
    //{
    //    std::cout << prob[i] << ", ";
    //    if (i % 10 == 0) std::cout << std::endl;
    //}
    //std::cout << std::endl;
    return 0;
}

void get_3d_camera_coordinate(int position[2],float other_pixel[3], float depth ,
 rs2_intrinsics *color_intrin,rs2_intrinsics *depth_intrin, rs2_extrinsics *depth2color){
    float depth_pixel[2] = { position[0], position[1] };
    float depth_point[3]; float other_point[3]; 
    rs2_deproject_pixel_to_point(depth_point, depth_intrin, depth_pixel, depth);
    rs2_transform_point_to_point(other_pixel,depth2color, depth_point);
    // rs2_project_point_to_pixel(other_pixel, color_intrin, other_point);
    
    return;
}

void get_column_center_position(int bbox[4], int * depth_image,
rs2_intrinsics *color_intrin, rs2_intrinsics *depth_intrin,
rs2_extrinsics *color2depth, rs2_extrinsics *depth2color,
rs2::depth_frame *depth, float coor[3]){
    int centerx = (bbox[0] + bbox[2]/2);
    int centery = (bbox[1] + bbox[3]/2);
    float center_depth = depth->get_distance(centerx, centery);
    int finalpos[2]={centerx, centery};float finaldepth = center_depth;
    for(int deltax = -bbox[2]/4; deltax<=bbox[2]/4; deltax++)
        for(int deltay=-bbox[3]/4; deltay<=bbox[3]/4;deltay++){
            float _depth = depth->get_distance(bbox[0]+deltax, bbox[1]+deltay);
            if(_depth<=center_depth && center_depth-_depth<=0.25)
                finalpos[0]=bbox[0]+deltax, finalpos[1]=bbox[1]+deltay, finaldepth = _depth;
        }
    get_3d_camera_coordinate(finalpos,coor,finaldepth,color_intrin,depth_intrin,depth2color);
    return;
}
