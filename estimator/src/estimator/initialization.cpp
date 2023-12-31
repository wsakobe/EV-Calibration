#include "initialization.h"

namespace estimator{

void Initializer::process(){
    //conventional camera intrinsic calibration
    for (auto corners:corner_buffer_){
        std::vector<cv::Point2f> corner_image; 
        std::vector<cv::Point3f> corner_world;
        for (auto corner:corners.corners){
            corner_image.emplace_back(cv::Point2f(corner.x, corner.y));
            corner_world.emplace_back(cv::Point3f(corner.x_grid, corner.y_grid, 0) * square_size);
        }
        corner_image_cluster.emplace_back(corner_image);
        corner_world_cluster.emplace_back(corner_world);
    }

    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(corner_world_cluster, corner_image_cluster, conv_cam_size, cameraMatrix, distCoeffs, rvecs, tvecs);

}

};
