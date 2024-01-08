#pragma once
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include <circle_msgs/circle.h>
#include <circle_msgs/circleArray.h>
#include <corner_msgs/corner.h>
#include <corner_msgs/cornerArray.h>
#include <utils/sophus_utils.hpp>
#include <spline/trajectory.h>

#define CONV_CAM 0
#define EV_CAM 1

namespace estimator{

class Initializer{
public:
    Initializer(){
        b_conv_initialized = false;
        b_ev_initialized = false;
        b_both_initialized = false;
    };
    ~Initializer(){};
    bool convInitialSucc(){ return b_conv_initialized;}
    bool evInitialSucc(){ return b_ev_initialized;}
    bool judgeBufferStatus(int cam_type){
        if (cam_type == 0){
            if (getCornerBufferNum() < initial_window_size){
                ROS_INFO("Conv cam no sufficient frames, now: %d, %d", getCornerBufferNum(), initial_window_size);
                return false;
            }
            return true;
        }
        else{
            if (getCircleBufferNum() < initial_window_size){
                ROS_INFO("EV cam no sufficient frames, now: %d", getCircleBufferNum());
                return false;
            }
            return true;
        }
    }
    
    void processConv();
    void processEv();
    void estimateInitialExtrinsic();

    std::vector<corner_msgs::cornerArray> corner_buffer_, corner_buffer_selected;
    std::vector<circle_msgs::circleArray> circle_buffer_, circle_buffer_selected;
    int initial_window_size = 10;
    float square_size;
    cv::Size conv_cam_size, ev_cam_size;

private:
    int getCornerBufferNum(){ 
        corner_buffer_selected.clear();
        legal_corner_size = 0;
        if (corner_buffer_.empty()) return 0;
        for (auto c:corner_buffer_){
            static bool fisrt_judge = true;
            if (fisrt_judge){
                fisrt_judge = false;
                last_corner = c;
                corner_buffer_selected.emplace_back(c);
            }
            else{
                float dist = 0;
                for (auto corner_last:last_corner.corners){
                    for (auto corner_now:c.corners){
                        if (corner_last.x_grid == corner_now.x_grid && corner_last.y_grid == corner_now.y_grid){
                            dist += std::sqrt((corner_last.x - corner_now.x) * (corner_last.x - corner_now.x) + (corner_last.y - corner_now.y) * (corner_last.y - corner_now.y));
                        }
                    }
                }
                dist /= c.corners.size();
                if (dist > 150){
                    legal_corner_size++;
                    last_corner = c;
                    corner_buffer_selected.emplace_back(c);
                }
            }
            
        }
        return legal_corner_size;
    }
    int getCircleBufferNum(){
        circle_buffer_selected.clear();
        legal_circle_size = 0;
        if (circle_buffer_.empty()) return 0;
        for (auto cir:circle_buffer_){
            static bool first_inside = true;
            if (first_inside){
                first_inside = false;
                last_circle = cir;
                circle_buffer_selected.emplace_back(cir);
            }
            else{
                float dist = 0;
                for (auto circle_last:last_circle.circles){
                    for (auto circle_now:cir.circles){
                        if (circle_last.x_grid == circle_now.x_grid && circle_last.y_grid == circle_now.y_grid){
                            dist += std::sqrt((circle_last.x - circle_now.x) * (circle_last.x - circle_now.x) + (circle_last.y - circle_now.y) * (circle_last.y - circle_now.y));
                        }
                    }
                }
                dist /= last_circle.circles.size();
                if (dist > 30){
                    legal_circle_size++;
                    last_circle = cir;
                    circle_buffer_selected.emplace_back(cir);
                }
            }
            
        }
        return legal_circle_size;
    }

    void solveRelativePose(const corner_msgs::cornerArray& corners, const cv::Mat& Intrinsic, const cv::Mat& distCoeffs, cv::Mat& Transformation);
    void solveRelativePose(const circle_msgs::circleArray& circles, const cv::Mat& Intrinsic, const cv::Mat& distCoeffs, cv::Mat& Transformation);

    std::vector<std::vector<cv::Point2f>> corner_image_cluster, circle_image_cluster; 
    std::vector<std::vector<cv::Point3f>> corner_world_cluster, circle_world_cluster;

    bool b_conv_initialized, b_ev_initialized, b_both_initialized;
    corner_msgs::cornerArray last_corner;
    circle_msgs::circleArray last_circle;
    int legal_corner_size, legal_circle_size;

    cv::Mat convCameraMatrix, convDistCoeffs;
    std::vector<cv::Mat> convRvecs, convTvecs;
    cv::Mat evCameraMatrix, evDistCoeffs;
    std::vector<cv::Mat> evRvecs, evTvecs;

    friend class EstimatorManager;
};

};