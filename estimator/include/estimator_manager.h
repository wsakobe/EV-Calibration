#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <circle_msgs/circle.h>
#include <circle_msgs/circleArray.h>
#include <corner_msgs/corner.h>
#include <corner_msgs/cornerArray.h>

#include <utils/sophus_utils.hpp>
#include <utils/ceres_callbacks.h>
#include <utils/yaml_utils.h>

namespace estimator{

class EstimatorManager{
public:
    EstimatorManager(const YAML::Node& yaml_node, ros::NodeHandle& nh);
    ~EstimatorManager(){}
    
    void cornerArrayCallback(const corner_msgs::cornerArray& msg);
    void circleArrayCallback(const circle_msgs::circleArray& msg);

    void initialization();
    void optimization();

private:
    ros::NodeHandle nh_;
    ros::Subscriber corner_sub_;
    ros::Subscriber circle_sub_;

    std::vector<corner_msgs::cornerArray> corner_buffer_;
    std::vector<circle_msgs::circleArray> circle_buffer_;

    int corner_buffer_size, circle_buffer_size;
};

};
