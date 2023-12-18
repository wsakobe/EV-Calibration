#pragma once

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include "string.h"
#include <sys/stat.h>
#include "corner_msgs/corner.h"
#include "corner_msgs/cornerArray.h"

#define PI 3.1415926

namespace corner_detector{
struct cornerInformation {
    cv::Point corner_position;
	cv::Point2f corner_position_subpixel;
	float response_score = -1;
    float angle_black_edge, angle_white_edge;
    cv::Point grid_pose;
};

struct Chessboard{
    int boardHeight, boardWidth;
};

class CornerTemplate {
public:
    CornerTemplate();
	CornerTemplate(int maskL);
	~CornerTemplate();

	cv::Mat tmp;
    cv::Mat hypersurface_temp;
    int maskSurface = 5;
	int maskSurfaceL = 2 * maskSurface + 1;

private:
    int maskL = 13;    
    cv::Mat hypersurface_temp_x2, hypersurface_temp_y2, hypersurface_temp_xy, hypersurface_temp_x, hypersurface_temp_y;
};

CornerTemplate::CornerTemplate() {
    //Search for the template in the dic. If not exist, generate once.
    
    std::string tmpFile = "template";
    tmpFile.append(std::to_string(maskL));
    tmpFile.append(".bmp");

    struct stat buffer;
    if (stat(tmpFile.c_str(), &buffer) != 0)
    {
        cv::Mat tmpMSAA, tmpCrop;
        tmpMSAA = cv::Mat::zeros(10 * maskL, 10 * maskL, CV_32FC1);
        tmp = cv::Mat::zeros(36 * maskL, 36 * maskL, CV_32FC1);
        for (float B = 0, angleB = 0; B < 36; angleB = angleB + 5, ++B) {
            for (float W = 0, angleW = 0; W < 36; angleW = angleW + 5, ++W) {
                float ix = 0.5 - float(tmpMSAA.cols) / 2;
                float iy;
                for (int x = 0; x < tmpMSAA.cols; ++x, ++ix) {
                    iy = float(tmpMSAA.rows) / 2 - 0.5;
                    for (int y = 0; y <= x; ++y, --iy) {
                        float temp = (atan2(ix, iy)) / CV_PI * 180 + 45;

                        if (angleB == angleW) continue;
                        if (temp > angleW && temp < angleB) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else if (angleB < angleW && (temp<angleB || temp>angleW)) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else {
                            tmpMSAA.at<float>(y, x) = 0;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 0;
                        }
                    }
                }
                tmpCrop = tmp(cv::Rect(B * maskL, W * maskL, maskL, maskL));
                resize(tmpMSAA, tmpCrop, cv::Size(maskL, maskL), 0, 0, cv::INTER_AREA);
            }
        }
        imwrite(tmpFile, 255 * tmp);
        std::cout << "Write file success!\n";
        tmpMSAA.release();
        tmpCrop.release();
    }
    else
    {
        tmp = cv::imread(tmpFile);
        cvtColor(tmp, tmp, cv::COLOR_BGR2GRAY);
        tmp.convertTo(tmp, CV_32FC1);
        tmp = tmp / 255;
    }

    //Generate the order of the hypersurface neighborhood
    hypersurface_temp_x2 = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_y2 = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_xy = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_x  = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp_y  = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    hypersurface_temp    = cv::Mat::zeros(maskSurfaceL * maskSurfaceL, 1, CV_32FC1);
    for (int i = 0; i < maskSurfaceL; i++)
        for (int j = 0; j < maskSurfaceL; j++) {
            hypersurface_temp_x2.at<float>(i * maskSurfaceL + j, 0) = i * i;
            hypersurface_temp_y2.at<float>(i * maskSurfaceL + j, 0) = j * j;
            hypersurface_temp_xy.at<float>(i * maskSurfaceL + j, 0) = i * j;
            hypersurface_temp_x.at<float>(i * maskSurfaceL + j, 0) = i;
            hypersurface_temp_y.at<float>(i * maskSurfaceL + j, 0) = j;
        }
    hconcat(hypersurface_temp_x2, hypersurface_temp_xy, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_y2, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_x, hypersurface_temp);
    hconcat(hypersurface_temp, hypersurface_temp_y, hypersurface_temp);
    hconcat(hypersurface_temp, cv::Mat::ones(maskSurfaceL * maskSurfaceL, 1, CV_32FC1), hypersurface_temp);
}

CornerTemplate::CornerTemplate(int maskL) {
    //Search for the template in the dic. If not exist, generate once.
    
    std::string tmpFile = "template";
    tmpFile.append(std::to_string(maskL));
    tmpFile.append(".bmp");

    struct stat buffer;
    if (stat(tmpFile.c_str(), &buffer) != 0)
    {
        cv::Mat tmpMSAA, tmpCrop;
        tmpMSAA = cv::Mat::zeros(10 * maskL, 10 * maskL, CV_32FC1);
        tmp = cv::Mat::zeros(36 * maskL, 36 * maskL, CV_32FC1);
        for (float B = 0, angleB = 0; B < 36; angleB = angleB + 5, ++B) {
            for (float W = 0, angleW = 0; W < 36; angleW = angleW + 5, ++W) {
                float ix = 0.5 - float(tmpMSAA.cols) / 2;
                float iy;
                for (int x = 0; x < tmpMSAA.cols; ++x, ++ix) {
                    iy = float(tmpMSAA.rows) / 2 - 0.5;
                    for (int y = 0; y <= x; ++y, --iy) {
                        float temp = (atan2(ix, iy)) / CV_PI * 180 + 45;

                        if (angleB == angleW) continue;
                        if (temp > angleW && temp < angleB) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else if (angleB < angleW && (temp<angleB || temp>angleW)) {
                            tmpMSAA.at<float>(y, x) = 1;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 1;
                        }
                        else {
                            tmpMSAA.at<float>(y, x) = 0;
                            tmpMSAA.at<float>(tmpMSAA.rows - y - 1, tmpMSAA.cols - x - 1) = 0;
                        }
                    }
                }
                tmpCrop = tmp(cv::Rect(B * maskL, W * maskL, maskL, maskL));
                resize(tmpMSAA, tmpCrop, cv::Size(maskL, maskL), 0, 0, cv::INTER_AREA);
            }
        }
        imwrite(tmpFile, 255 * tmp);
        std::cout << "Write file success!\n";
        tmpMSAA.release();
        tmpCrop.release();
    }
    else
    {
        tmp = cv::imread(tmpFile);
        cvtColor(tmp, tmp, cv::COLOR_BGR2GRAY);
        tmp.convertTo(tmp, CV_32FC1);
        tmp = tmp / 255;
    }
}

CornerTemplate::~CornerTemplate() {
    tmp.release();
}

class ImageProcessor {
public:
    ImageProcessor(ros::NodeHandle& nh);
    ~ImageProcessor(){
        image_sub_.shutdown();
    };

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    void extractCorner(const cv::Mat& input_image);
    void cornerPreFilter(const cv::Mat& image);
    void templateMatching(const cv::Mat& image);
    void organizeCorner();
    float Distance(cv::Point2f line_point1, cv::Point2f line_point2, cv::Point2f line_point_through, cornerInformation corner);

    ros::NodeHandle nh;
    image_transport::Subscriber image_sub_;
    ros::Publisher corner_pub_ = nh.advertise<corner_msgs::cornerArray>("/corner_gs/cornerArray",10);
    CornerTemplate ct;

    Chessboard cb;
    long long input_timestamp;
    
    std::vector<cornerInformation> corners;

    // corner pre filter
    cv::Mat image_blur;
    int maskR = 6;
	int kernal_size = 7;
	int sigma = 3;
	cv::Mat Gx, Gy, Gxx, Gyy, Gxy, G_score, G_score_after_NMS, score_sequence;
    float G_filtermin, G_filtermax;
	std::priority_queue <float, std::vector<float>, std::less<float> > Q;

    // corner template matching
    int maskL = 13, maskTemR = (maskL - 1) / 2;
	int edgeIdx, directIdx;
	float angle1, angle2, edge_angle, direction_angle;
	float response_score_max = -1, T_temp_max = 0.85, T_response = 0.25;
	cv::Mat hypersurface_coeffs, img_hypersurface;
	cv::Mat coeffs, roots;

    // corner organization
    float T_dis = 2.0, T_angle = 10.0;
    std::vector<cornerInformation> candidate_line1, candidate_line2, onboard_corners;
};

ImageProcessor::ImageProcessor(ros::NodeHandle& nh) {
    nh.param("BoardHeight", cb.boardHeight, 3);
    nh.param("BoardWidth", cb.boardWidth, 6);
    
    // Corner template generation
    CornerTemplate ct(maskL);

    image_transport::ImageTransport it(nh);
    image_sub_ = it.subscribe("/hik_camera/image", 1, &ImageProcessor::imageCallback, this);
}

void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
        if (image.channels() == 3)
            cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
        if (image.channels() == 1) {
            image.convertTo(image, CV_32FC1, 1/255.0);
        }
        input_timestamp = msg->header.stamp.nsec;
        extractCorner(image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void ImageProcessor::extractCorner(const cv::Mat& input_image) {
    corners.clear();

    cornerPreFilter(input_image);
    std::cout << "Pre: " << corners.size() << std::endl;
    templateMatching(input_image);
    std::cout << "Template: " << corners.size() << std::endl;
    organizeCorner();
    std::cout << "Org: " << onboard_corners.size() << std::endl;
    if (onboard_corners.size() < cb.boardHeight * cb.boardWidth - 3) return;
    std::vector<cv::Point2f> corners_refined;
    for (int i = 0; i < onboard_corners.size(); i++){
        corners_refined.emplace_back((cv::Point2f)onboard_corners[i].corner_position);
    }
    cv::cornerSubPix(input_image, corners_refined, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria());
    for (int i = 0; i < corners_refined.size(); i++){
        onboard_corners[i].corner_position_subpixel = corners_refined[i];
    }
    cv::Mat image_plot(input_image.rows, input_image.cols, CV_32FC3);
    cv::cvtColor(input_image, image_plot, cv::COLOR_GRAY2RGB);
    for (int i = 0; i < onboard_corners.size(); i++){
        cv::circle(image_plot, onboard_corners[i].corner_position, 10, cv::Scalar(120, 120, 10), 3);
    }

    cv::imshow("Detected corners", image_plot);
    cv::waitKey(1);

    //Publish corner message
    corner_msgs::cornerArray corner_array;
    corner_msgs::corner corner_msg;
    
    for (int i = 0; i < onboard_corners.size(); i++){
        corner_msg.x = onboard_corners[i].corner_position_subpixel.x;
        corner_msg.y = onboard_corners[i].corner_position_subpixel.y;
        corner_msg.x_grid = onboard_corners[i].grid_pose.x;
        corner_msg.y_grid = onboard_corners[i].grid_pose.y;
        corner_array.corners.emplace_back(corner_msg);
    }
    corner_array.timestamp = input_timestamp;
    corner_pub_.publish(corner_array);
    ROS_INFO("Publishing corner message");

    return;
}

void ImageProcessor::cornerPreFilter(const cv::Mat& image){
    Q.empty();

    cv::GaussianBlur(image, image_blur, cv::Size(kernal_size, kernal_size), sigma);
    cv::Scharr(image_blur, Gx, CV_32FC1, 1, 0);
    cv::Scharr(image_blur, Gy, CV_32FC1, 0, 1);

    cv::Scharr(Gx, Gxx, CV_32FC1, 1, 0);
    cv::Scharr(Gy, Gyy, CV_32FC1, 0, 1);
    cv::Scharr(Gx, Gxy, CV_32FC1, 0, 1);
    
    G_score = Gxy.mul(Gxy) - Gxx.mul(Gyy);

    cv::dilate(G_score, G_score_after_NMS, cv::Mat());
    for (int i = maskR; i < image_blur.rows - maskR; i++)
        for (int j = maskR; j < image_blur.cols - maskR; j++)
            if (G_score.ptr<float>(i)[j] == G_score_after_NMS.ptr<float>(i)[j]) {
                Q.push(G_score_after_NMS.ptr<float>(i)[j]);
            }
            else {
                G_score_after_NMS.ptr<float>(i)[j] = 0;
            }
                
    G_filtermax = Q.top();
    for (int i = 0; i < cb.boardHeight * cb.boardWidth * 3; i++) Q.pop();
    G_filtermin = Q.top();
    
    for (int i = maskR; i < image_blur.rows - maskR; i++)
        for (int j = maskR; j < image_blur.cols - maskR; j++) {
    		if (G_score_after_NMS.ptr<float>(i)[j] < G_filtermin)
                G_score_after_NMS.ptr<float>(i)[j] = 0;
            else {
                cornerInformation temporal_corner;
                temporal_corner.corner_position = cv::Point(j ,i);
                corners.emplace_back(temporal_corner);
            }
        }	

    return;
}

void ImageProcessor::templateMatching(const cv::Mat& image){
    for (int i = 0; i < corners.size(); i++) {
        cv::Rect rect(corners[i].corner_position.x - ct.maskSurface, corners[i].corner_position.y - ct.maskSurface, ct.maskSurfaceL, ct.maskSurfaceL);
        img_hypersurface = image(rect).clone();
        img_hypersurface = img_hypersurface.reshape(1, ct.maskSurfaceL * ct.maskSurfaceL);
        
        solve(ct.hypersurface_temp, img_hypersurface, hypersurface_coeffs, cv::DECOMP_SVD);
        
        coeffs = (cv::Mat_<float>(3, 1) << hypersurface_coeffs.at<float>(0, 0), -1 * hypersurface_coeffs.at<float>(1, 0), hypersurface_coeffs.at<float>(2, 0));
        solvePoly(coeffs, roots);
        
        angle1 = atan(roots.at<float>(0, 0)) * 180.0 / PI;
        angle2 = atan(roots.at<float>(1, 0)) * 180.0 / PI;
        
        if ((angle1 * angle2 * hypersurface_coeffs.at<float>(0, 0)) < 0) {
            corners[i].angle_white_edge = std::max(angle1, angle2);
            corners[i].angle_black_edge = std::min(angle1, angle2);
        }
        else {
            corners[i].angle_white_edge = std::min(angle1, angle2);
            corners[i].angle_black_edge = std::max(angle1, angle2);
        }
    }

    for (int i = 0; i < corners.size(); i++) {        
        edgeIdx   = round((corners[i].angle_black_edge + 135) / 5);
        directIdx = round((corners[i].angle_white_edge + 135) / 5);

        edgeIdx %= 36;
        directIdx %= 36;
        
        if (edgeIdx < 0 || edgeIdx > 35) edgeIdx = 0;
        if (directIdx < 0 || directIdx > 35) directIdx = 0;
        
        cv::Mat tmpCrop(ct.tmp, cv::Rect(edgeIdx * maskL, directIdx * maskL, maskL, maskL));
        cv::Mat crop(image, cv::Rect(corners[i].corner_position.x - maskTemR, corners[i].corner_position.y - maskTemR, maskL, maskL));

        cv::Scalar meanTmp, meanCrop, stdTmp, stdCrop;
        meanStdDev(tmpCrop, meanTmp, stdTmp);
        meanStdDev(crop, meanCrop, stdCrop);

        float covar = (tmpCrop - meanTmp).dot(crop - meanCrop) / (maskL * maskL);
        corners[i].response_score = covar / (stdTmp[0] * stdCrop[0]);
        if (corners[i].response_score > response_score_max) {
            response_score_max = corners[i].response_score;
        }
    }

    if (response_score_max < T_temp_max) corners.clear();

    for (std::vector<cornerInformation>::iterator it = corners.begin(); it != corners.end();)
    {
        if (((*it).response_score) < (response_score_max - T_response))
            it = corners.erase(it);
        else
            it++;
    }

    return;
}

float ImageProcessor::Distance(cv::Point2f line_point1, cv::Point2f line_point2, cv::Point2f line_point_through, cornerInformation corner){
    float A = line_point2.y - line_point1.y;
    float B = line_point1.x - line_point2.x;
    float C = line_point2.x * line_point_through.y -line_point1.x * line_point_through.y + line_point1.y * line_point_through.x - line_point2.y * line_point_through.x;

    float distance = std::abs(A * corner.corner_position_subpixel.x + B * corner.corner_position_subpixel.y + C) / std::sqrt(A * A + B * B);
    return distance;
}

void ImageProcessor::organizeCorner(){
    onboard_corners.clear();
    if (corners.size() < cb.boardHeight * cb.boardWidth) return;

    // sort by scores
    std::sort(corners.begin(), corners.end(), 
              [](const cornerInformation &a, const cornerInformation &b) {
                  return a.response_score > b.response_score;
              });
    
    cornerInformation template_corner = corners[0];
    onboard_corners.emplace_back(template_corner);
    for (auto it = corners.begin() + 1; it != corners.end(); it++) {
        if ((abs(it->angle_black_edge - template_corner.angle_black_edge) < T_angle && abs(it->angle_white_edge - template_corner.angle_white_edge) < T_angle) || (abs(it->angle_black_edge - template_corner.angle_white_edge) < T_angle && abs(it->angle_white_edge - template_corner.angle_black_edge) < T_angle)) {
            onboard_corners.emplace_back(*it);
        }
        else if ((abs(abs(it->angle_black_edge - template_corner.angle_black_edge) - 180) < T_angle && abs(it->angle_white_edge - template_corner.angle_white_edge) < T_angle) || (abs(it->angle_black_edge - template_corner.angle_black_edge) < T_angle && abs(abs(it->angle_white_edge - template_corner.angle_white_edge) - 180) < T_angle) || (abs(abs(it->angle_black_edge - template_corner.angle_white_edge) - 180) < T_angle && abs(it->angle_white_edge - template_corner.angle_black_edge) < T_angle) || (abs(it->angle_black_edge - template_corner.angle_white_edge) < T_angle && abs(abs(it->angle_white_edge - template_corner.angle_black_edge) - 180) < T_angle)){
            onboard_corners.emplace_back(*it);
        }
        if (onboard_corners.size() == cb.boardHeight * cb.boardWidth) break;
    }
    /*
    // sort by y-axis
    std::sort(corners.begin(), corners.end(), 
              [](const cornerInformation &a, const cornerInformation &b) {
                  return a.corner_position_subpixel.y < b.corner_position_subpixel.y;
              });
    
    corner_start = corners;
    while (corner_start.size() >= boardHeight * boardWidth){
        candidate_line1.clear();
        candidate_line2.clear();

        cornerInformation start_point = corner_start[0];
        corner_start.erase(corner_start.begin());
        corner_temp = corner_start;

        // sort by x-axis
        std::sort(corner_temp.begin(), corner_temp.end(), 
                [](const cornerInformation &a, const cornerInformation &b) {
                    return a.corner_position_subpixel.x < b.corner_position_subpixel.x;
                });
        cornerInformation candidate_left  = corner_temp[0];     // left-down point
        cornerInformation candidate_right = corner_temp.back(); // right-down point
        
        candidate_line1.emplace_back(start_point);
        candidate_line2.emplace_back(start_point);
        candidate_line1.emplace_back(corner_temp[0]);
        candidate_line2.emplace_back(corner_temp.back());
        onboard_corners.emplace_back(start_point);
        onboard_corners.emplace_back(corner_temp[0]);
        onboard_corners.emplace_back(corner_temp.back());
        corner_temp.erase(corner_temp.begin());
        corner_temp.erase(corner_temp.end());

        for (auto it = corner_temp.begin(); it != corner_temp.end();) {
            if (Distance(start_point.corner_position_subpixel, candidate_left.corner_position_subpixel, start_point.corner_position_subpixel, *it) < T_dis) {
                candidate_line1.emplace_back(*it);
                onboard_corners.emplace_back(*it);
                it = corner_temp.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = corner_temp.begin(); it != corner_temp.end();) {
            if (Distance(start_point.corner_position_subpixel, candidate_right.corner_position_subpixel, start_point.corner_position_subpixel, *it) < T_dis) {
                candidate_line2.emplace_back(*it);
                onboard_corners.emplace_back(*it);
                it = corner_temp.erase(it); 
            } else {
                ++it;
            }
        }
        if (candidate_line1.size() < std::min(boardHeight, boardWidth) || candidate_line2.size() < std::min(boardHeight, boardWidth)) continue;
        if (candidate_line1.size() * candidate_line2.size() == boardHeight * boardWidth){
            for (auto it = candidate_line2.begin() + 1; it != candidate_line2.end(); ++it) { // skip the start point
                for (auto iter = corner_temp.begin(); iter != corner_temp.end();) {
                    if (Distance(start_point.corner_position_subpixel, candidate_left.corner_position_subpixel, it->corner_position_subpixel, *iter) < T_dis) {
                        onboard_corners.emplace_back(*iter);
                        iter = corner_temp.erase(iter);
                    } else {
                        ++iter;
                    }
                }
            }
        }
        if (onboard_corners.size() < boardHeight * boardWidth) continue;
        else {

        }
    } */  

}

} // namespace corner_detector
