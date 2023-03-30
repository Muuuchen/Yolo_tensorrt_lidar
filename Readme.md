# Yolov5 + Tensorrt
## 生成engine文件
1.  generate .wts from pytorch with .pt, or download .wts from model zoo
    ```
    git clone -b v5.0 https://github.com/ultralytics/yolov5.git
    git clone https://github.com/wang-xinyu/tensorrtx.git
    // download https://github.com/ultralytics/yolov5/releases/download/v5.0/yolov5s.pt
    cp {tensorrtx}/yolov5/gen_wts.py {ultralytics}/yolov5
    cd {ultralytics}/yolov5
    python gen_wts.py -w yolov5s.pt -o yolov5s.wts
    // a file 'yolov5s.wts' will be generated.
    ```
2. build tensorrtx/yolov5 and run
    ```
    cd {tensorrtx}/yolov5/
    // update CLASS_NUM in yololayer.h if your model is trained on custom dataset
    mkdir build
    cd build
    cp {ultralytics}/yolov5/yolov5s.wts {tensorrtx}/yolov5/build
    cmake ..
    make
    sudo ./yolov5 -s [.wts] [.engine] [s/m/l/x/s6/m6/l6/x6 or c/c6 gd gw]  // serialize model to plan file
    sudo ./yolov5 -d [.engine] [image folder]  // deserialize and run inference, the images in [image folder] will be processed.
    // For example yolov5s
    sudo ./yolov5 -s yolov5s.wts yolov5s.engine s
    sudo ./yolov5 -d yolov5s.engine ../samples
    // For example Custom model with depth_multiple=0.17, width_multiple=0.25 in yolov5.yaml
    sudo ./yolov5 -s yolov5_custom.wts yolov5.engine c 0.17 0.25
    sudo ./yolov5 -d yolov5.engine ../samples

    ```
## 模型推理
    ```
    mkdir build
    cd build
    cmake ..
    make -j6
    ```
## 注意点
    ```
    1. 设置好engine的路径 ok
    2. 图片目前采用文件读取，后续肯定要转换为实时视频流
    ```