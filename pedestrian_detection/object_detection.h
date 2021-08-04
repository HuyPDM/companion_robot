#ifndef OBJECT_DETECTION_H
#define OBJECT_DETECTION_H
#pragma once
#include <opencv4/opencv2/tracking/tracker.hpp>
//#include <opencv2/core/utility.hpp>
#include <vector>
#include <string>
#include "gpu.h"
#include "mat.h"
#include "net.h"
#include <iostream>
struct Object{
    cv::Rect rec;
    float prob;
};

class ObjectDetection
{
    public:
        ObjectDetection(const std::string _model_path);
        std::vector<cv::Rect> detectObject(const cv::Mat& _frame);
	~ObjectDetection();
    private:
	ncnn::Net mobilenet;
};

#endif
