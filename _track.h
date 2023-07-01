#ifndef TRACK_H
#define TRACK_H

#include <iostream>
#include <chrono>
#include <cmath>
#include<pthread.h>

#include <opencv2/opencv.hpp>


#include "./sort-cpp-master/sort-c++/Hungarian.h"
#include "./sort-cpp-master/sort-c++/KalmanTracker.h"

using namespace std;
using namespace cv;
typedef struct TrackingBox
{
	int frame;
	int id;
	Rect_<float> box;
}TrackingBox;

void Init_Tracker(void);
void Do_Track(std::vector<TrackingBox>* detData,  std::vector<TrackingBox>* frameTrackingResult);

#endif