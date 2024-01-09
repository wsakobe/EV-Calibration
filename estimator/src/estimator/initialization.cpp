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
    
    double rms_conv = cv::calibrateCamera(corner_world_cluster, corner_image_cluster, conv_cam_size, convCameraMatrix, convDistCoeffs, convRvecs, convTvecs);
    std::cout << "Conv P: " << convCameraMatrix << std::endl << convDistCoeffs << std::endl << "ReErr: " << rms_conv;

    if (rms_conv < 1){
        ROS_INFO("Conv cam initial succ");
        b_conv_initialized = true;
    }else{
        ROS_INFO("Conv cam initial failed, restart again");
        corner_buffer_.clear();
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

void Initializer::solveRelativePose(const corner_msgs::cornerArray& features, const cv::Mat& Intrinsic, const cv::Mat& distCoeffs, cv::Mat& Transformation){
    std::vector<cv::Point2f> feature_image; 
    std::vector<cv::Point3f> feature_world;
    for (auto corner:features.corners){
        feature_image.emplace_back(cv::Point2f(corner.x, corner.y));
        feature_world.emplace_back(cv::Point3f(corner.x_grid, corner.y_grid, 0) * square_size);
    }
    cv::Mat rvec, tvec;
    cv::solvePnP(feature_world, feature_image, Intrinsic, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
    
    cv::Mat rotationMat;
    cv::Rodrigues(rvec, rotationMat);
    cv::Mat transformationMat;
    cv::hconcat(rotationMat, tvec, Transformation);
    cv::Mat_<double> last_row = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    cv::vconcat(Transformation, last_row, Transformation);

    std::cout << Transformation << std::endl;
}

void Initializer::solveRelativePose(const circle_msgs::circleArray& features, const cv::Mat& Intrinsic, const cv::Mat& distCoeffs, cv::Mat& Transformation){
    std::vector<cv::Point2f> feature_image; 
    std::vector<cv::Point3f> feature_world;
    for (auto circle:features.circles){
        feature_image.emplace_back(cv::Point2f(circle.x, circle.y));
        feature_world.emplace_back(cv::Point3f(circle.x_grid, circle.y_grid, 0) * square_size);
    }
    cv::Mat rvec, tvec;
    cv::solvePnP(feature_world, feature_image, Intrinsic, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);
    
    cv::Mat rotationMat;
    cv::Rodrigues(rvec, rotationMat);
    cv::Mat transformationMat;
    cv::hconcat(rotationMat, tvec, Transformation);
    cv::Mat_<double> last_row = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    cv::vconcat(Transformation, last_row, Transformation);

    std::cout << Transformation << std::endl;
}

void Initializer::estimateInitialExtrinsic(){
    for (size_t i = 0; i < corner_buffer_.size(); ++i){
        ros::Time time_ = corner_buffer_[i].timestamp;
        for (size_t j = 0; j < circle_buffer_.size(); ++j){
            ros::Duration time_interval = time_ - circle_buffer_[j].header.stamp;
            std::cout << time_interval.toSec() << std::endl;
            if (time_interval.toSec() < 0){
                break;
            }
            if (time_interval.toSec() < 0.05){
                cv::Mat T_ev, T_c;
                solveRelativePose(circle_buffer_[i], evCameraMatrix, evDistCoeffs, T_ev);
                solveRelativePose(corner_buffer_[j], convCameraMatrix, convDistCoeffs, T_c);
                cv::Mat T = T_ev * T_c.inv();
                Eigen::Matrix4d eigenT;
                cv::cv2eigen(T, eigenT);
                SE3d se3(eigenT);
                //trajectory_->setExtrinsicMat(se3);
                std::cout << "Extrinsic params guess:\n" << T << std::endl;
                b_both_initialized = true;
                return;
            }
        }
    }
    return;
}

};
