#pragma once
#include "ros/ros.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <circle_msgs/circle.h>
#include <circle_msgs/circleArray.h>
#include <corner_msgs/corner.h>
#include <corner_msgs/cornerArray.h>

namespace estimator{

class Initializer{
public:
    Initializer(){};
    ~Initializer(){};
    bool initialSucc(){ return b_initialized;}
    bool judgeBufferStatus(int min_buffer_size){
        if (getCornerBufferNum() < initial_window_size || getCircleBufferNum() < initial_window_size){
            ROS_INFO("No sufficient frames");
            return false;
        }
        return true;
    }
    
    void process();

    std::vector<corner_msgs::cornerArray> corner_buffer_;
    std::vector<circle_msgs::circleArray> circle_buffer_;
    int initial_window_size;
    float square_size;
    cv::Size conv_cam_size, ev_cam_size;

private:
    int getCornerBufferNum(){ return corner_buffer_.size();}
    int getCircleBufferNum(){ return circle_buffer_.size();}

    std::vector<std::vector<cv::Point2f>> corner_image_cluster; 
    std::vector<std::vector<cv::Point3f>> corner_world_cluster;

    bool b_initialized;
};

};