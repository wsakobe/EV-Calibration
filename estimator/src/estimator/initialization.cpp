#include "initialization.h"

namespace estimator{

void Initializer::processConv(){
    //conventional camera intrinsic calibration
    for (auto corners:corner_buffer_selected){
        std::vector<cv::Point2f> corner_image; 
        std::vector<cv::Point3f> corner_world;
        for (auto corner:corners.corners){
            corner_image.emplace_back(cv::Point2f(corner.x, corner.y));
            corner_world.emplace_back(cv::Point3f(corner.x_grid, corner.y_grid, 0) * square_size);
        }
        corner_image_cluster.emplace_back(corner_image);
        corner_world_cluster.emplace_back(corner_world);
    }
    
    cv::Mat convCameraMatrix, convDistCoeffs;
    std::vector<cv::Mat> convRvecs, convTvecs;
    double rms_conv = cv::calibrateCamera(corner_world_cluster, corner_image_cluster, conv_cam_size, convCameraMatrix, convDistCoeffs, convRvecs, convTvecs);
    std::cout << "Conv P: " << convCameraMatrix << std::endl << convDistCoeffs << std::endl << "ReErr: " << rms_conv;

    if (rms_conv < 1){
        ROS_INFO("Conv cam initial succ");
        b_conv_initialized = true;
    }
}

void Initializer::processEv(){
    //conventional camera intrinsic calibration
    for (auto circles:circle_buffer_selected){
        std::vector<cv::Point2f> circle_image; 
        std::vector<cv::Point3f> circle_world;
        for (auto circle:circles.circles){
            circle_image.emplace_back(cv::Point2f(circle.x, circle.y));
            circle_world.emplace_back(cv::Point3f(circle.x_grid, circle.y_grid, 0) * square_size);
        }
        circle_image_cluster.emplace_back(circle_image);
        circle_world_cluster.emplace_back(circle_world);
    }
    
    cv::Mat evCameraMatrix, evDistCoeffs;
    std::vector<cv::Mat> evRvecs, evTvecs;
    double rms_ev = cv::calibrateCamera(circle_world_cluster, circle_image_cluster, ev_cam_size, evCameraMatrix, evDistCoeffs, evRvecs, evTvecs);
    std::cout << "Event P: " << evCameraMatrix << std::endl << evDistCoeffs << std::endl << "ReErr: " << rms_ev;

    if (rms_ev < 1){
        ROS_INFO("Ev cam initial succ");
        b_ev_initialized = true;
    }else{
        ROS_INFO("Ev cam initial failed, restart again");
        circle_buffer_.clear();
    }
}

};
