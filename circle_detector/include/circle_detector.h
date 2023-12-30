#ifndef circle_detector_H_
#define circle_detector_H_

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Time.h>
#include <circle_msgs/circle.h>
#include <circle_msgs/circleArray.h>

#include <vector>
#include <thread>
#include <mutex>

template<typename T>
void info(T msg) {
    std::cout << msg << std::endl;
}

namespace circle_detector{
using EventQueue = std::deque<dvs_msgs::Event>;
cv::Size sensor_size_;

struct Chessboard{
    int boardHeight, boardWidth;
}cb;

class CircleDetector{

struct pcaInfo{
    cv::Point2d comp1, comp2;
    double vertical;
    float magnitude_comp1, magnitude_comp2;
    cv::Point2d center;
};

struct circleInformation {
	cv::Point2f circle_position;
    float width, height;
    std::vector<cv::Point> area;
    cv::Point grid_pose;
    int timestamp;
    pcaInfo pca_res;
};

public:
    CircleDetector(ros::NodeHandle& nh) : nh_(nh){};
    ~CircleDetector(){
        circle_pub_.shutdown();
    };
    void eventMaptDetect(const cv::Mat& event_map_no_polarity, const cv::Mat& event_map_positive, const cv::Mat& event_map_negative);
    
private:
    void connectedComponentLabeling(const cv::Mat& src, std::vector<std::vector<cv::Point>>& quadArea);
    pcaInfo pcaAnalysis(std::vector<cv::Point> points);
    double euclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    double distanceFromTwoClusters(const circleInformation& cluster_1, const circleInformation& cluster_2);
    float distanceFromLine(cv::Point2f input_point, cv::Point2f line_point, cv::Vec2f line_direction);
    bool coEllipse(const circleInformation& a, const circleInformation& b);
    void organizeCircles(const cv::Mat& event_map);

    bool sortByX(const circleInformation& a, const circleInformation& b) {
        return a.circle_position.x < b.circle_position.x;
    }

    bool sortByY(const circleInformation& a, const circleInformation& b) {
        return a.circle_position.y < b.circle_position.y;
    }

    ros::NodeHandle nh_;
    ros::Publisher circle_pub_ = nh_.advertise<circle_msgs::circleArray>("/circle_ev/circleArray", 10);;
    circle_msgs::circle circle_msg;
    circle_msgs::circleArray circle_array;

    int count = 0;
    std::vector<std::vector<cv::Point>> quadArea_pos, quadArea_neg;
    std::deque<circleInformation> candidate_pos, candidate_neg;
    std::vector<circleInformation> candidate_circles, onboard_circles;
    std::vector<std::pair<circleInformation, circleInformation>> candidate_full;
    cv::RotatedRect m_ellipsetemp;
    circleInformation current;

    float PCA_high = 15, PCA_low = 2;

    // organization
    std::vector<circleInformation> candidate_line1, candidate_line2;
    float T_dis = 10.0, T_angle = 15.0;
    bool b_width_line_found, b_height_line_found;
    cv::Vec2f width_direction_, height_direction_;
};

double CircleDetector::euclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool CircleDetector::coEllipse(const circleInformation& a, const circleInformation& b){
    std::vector<cv::Point> ellipse_points;
    ellipse_points.insert(ellipse_points.end(), a.area.begin(), a.area.end());
    ellipse_points.insert(ellipse_points.end(), b.area.begin(), b.area.end());
    m_ellipsetemp = cv::fitEllipse(ellipse_points);
    
    // pca magnitude restriction
    std::cout << "PCA: " << a.pca_res.magnitude_comp1 << " " << b.pca_res.magnitude_comp1 << std::endl;
    if (std::abs(a.pca_res.magnitude_comp1 - b.pca_res.magnitude_comp1) / (a.pca_res.magnitude_comp1 + b.pca_res.magnitude_comp1) > 0.3){
        ROS_ERROR("PCA failed");
        return false;
    }

    // fitting error restriction
    float fit_error = 0.0;
    for (auto p : ellipse_points){
        float x_rot = cos(m_ellipsetemp.angle) * (p.x - m_ellipsetemp.center.x) + sin(m_ellipsetemp.angle) * (p.y - m_ellipsetemp.center.y);
        float y_rot = -sin(m_ellipsetemp.angle) * (p.x - m_ellipsetemp.center.x) + cos(m_ellipsetemp.angle) * (p.y - m_ellipsetemp.center.y);
        fit_error += std::abs((x_rot * x_rot) / (m_ellipsetemp.size.width * m_ellipsetemp.size.width) + (y_rot * y_rot) / (m_ellipsetemp.size.height * m_ellipsetemp.size.height) - 0.25);
    }
    fit_error /= ellipse_points.size();
    std::cout << "Fit error: " << fit_error << std::endl;
    if (fit_error > 0.2){
        ROS_ERROR("Fitting failed");
        return false;
    } 
    
    // ellipse size restriction
    if (m_ellipsetemp.size.width > sensor_size_.height / std::min(cb.boardHeight, cb.boardWidth) || m_ellipsetemp.size.height > sensor_size_.height / std::min(cb.boardHeight, cb.boardWidth)){
        ROS_ERROR("Size failed");
        return false;
    }
    
    // angle restriction
    float angle_min_x = 1000, angle_max_x = -1, angle_min_y = 1000, angle_max_y = -1, angle_now, angle_min_a, angle_max_a, angle_min_b, angle_max_b;
    for (auto point : a.area){
        angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, point.x - m_ellipsetemp.center.x);
        if (angle_now > angle_max_x) angle_max_x = angle_now;
        if (angle_now < angle_min_x) angle_min_x = angle_now;
        angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, -point.x + m_ellipsetemp.center.x);
        if (angle_now > angle_max_y) angle_max_y = angle_now;
        if (angle_now < angle_min_y) angle_min_y = angle_now;
    }
    if (angle_max_x - angle_min_x < angle_max_y - angle_min_y){
        angle_max_a = angle_max_x;
        angle_min_a = angle_min_x;
    }else{
        angle_max_a = angle_max_y;
        angle_min_a = angle_min_y;
    }

    angle_min_x = 1000, angle_max_x = -1, angle_min_y = 1000, angle_max_y = -1;
    for (auto point : b.area){
        angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, point.x - m_ellipsetemp.center.x);
        if (angle_now > angle_max_x) angle_max_x = angle_now;
        if (angle_now < angle_min_x) angle_min_x = angle_now;
        angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, -point.x + m_ellipsetemp.center.x);
        if (angle_now > angle_max_y) angle_max_y = angle_now;
        if (angle_now < angle_min_y) angle_min_y = angle_now;
    }
    if (angle_max_x - angle_min_x < angle_max_y - angle_min_y){
        angle_max_b = angle_max_x;
        angle_min_b = angle_min_x;
    }else{
        angle_max_b = angle_max_y;
        angle_min_b = angle_min_y;
    }
    
    std::cout << "Angle a: " << angle_max_a << " " << angle_min_a << " Angle b: " << angle_max_b << " " << angle_min_b << std::endl;
    std::cout << "Angle diff a: " << angle_max_a - angle_min_a << std::endl;
    std::cout << "Angle diff b: " << angle_max_b - angle_min_b << std::endl;
    if (std::abs(angle_max_a - angle_min_a - angle_max_b + angle_min_b) > 55 || angle_max_a - angle_min_a < 100 || angle_max_b - angle_min_b < 100) {
        ROS_ERROR("Angle failed");
        return false;
    };

    // distance restriction
    double dist = euclideanDistance(a.pca_res.center, b.pca_res.center);
    std::cout << "Dist: " << dist << std::endl;
    if (dist > sensor_size_.height / std::min(cb.boardHeight, cb.boardWidth) || dist < 10) {
        ROS_ERROR("Dist failed");
        return false;
    }
    
    // pca direction restriction
    //double vertical = ((a.pca_res.center.x - b.pca_res.center.x) * (a.pca_res.comp1.x + b.pca_res.comp1.x) + (a.pca_res.center.y - b.pca_res.center.y) * (a.pca_res.comp1.y + b.pca_res.comp1.y))
    // / std::sqrt((a.pca_res.center.x - b.pca_res.center.x) * (a.pca_res.center.x - b.pca_res.center.x) + (a.pca_res.center.y - b.pca_res.center.y) * (a.pca_res.center.y - b.pca_res.center.y));
    
    // if (abs(vertical) > 0.4) {
    //     ROS_ERROR("Vertical failed");
    //     return false;
    // }
    ROS_INFO("Passed");
    return true;
}

void CircleDetector::connectedComponentLabeling(const cv::Mat& src, std::vector<std::vector<cv::Point>>& quadArea){
    cv::Mat img_labeled, stats, centroids;
    std::vector<bool> illegal;
    int nccomp_area = 0;
    nccomp_area = cv::connectedComponentsWithStats(src, img_labeled, stats, centroids, 8, 4, cv::CCL_GRANA);
    quadArea.resize(nccomp_area);
    illegal.resize(nccomp_area);
    fill(illegal.begin(), illegal.end(), 0);

    for (int i = 0; i < nccomp_area; i++){
        if (stats.at<int>(i, cv::CC_STAT_AREA) < 30 || stats.at<int>(i, cv::CC_STAT_AREA) > round(0.01 * src.cols * src.rows)){
            illegal[i] = true;
        }
    }
    for (int i = 0; i < img_labeled.rows; i++){
        for (int j = 0; j < img_labeled.cols; j++){
            if (!illegal[img_labeled.at<int>(i, j)])
                quadArea[img_labeled.at<int>(i, j)].push_back(cv::Point(j, i));
        }
    } 
    int count = 0;
    for (auto iter = quadArea.begin(); iter != quadArea.end(); ){
        if (illegal[count++]){
            iter = quadArea.erase(iter);
        }
        else{
            iter++;
        }
    }
    /*
    std::vector<cv::Vec3b> colors(nccomp_area);
    colors[0] = cv::Vec3b(0,0,0); // background pixels remain black.
    for(int i = 1; i < nccomp_area; i++ ) {
        colors[i] = cv::Vec3b(rand()%256, rand()%256, rand()%256);
        //去除面积小于30的连通域
        if (stats.at<int>(i, cv::CC_STAT_AREA) < 30 || stats.at<int>(i, cv::CC_STAT_AREA) > round(0.01 * src.cols * src.rows))
            colors[i] = cv::Vec3b(0,0,0); // small regions are painted with black too.
    }
    //按照label值，对不同的连通域进行着色
    cv::Mat img_color = cv::Mat::zeros(src.size(), CV_32FC3);
    cv::cvtColor(src, img_color, cv::COLOR_GRAY2RGB);
    for( int y = 0; y < img_color.rows; y++ ){
        for( int x = 0; x < img_color.cols; x++ ){
            int label = img_labeled.at<int>(y, x);
            //img_color.at<cv::Vec3b>(y, x) = colors[label];
            if (label != 0)
                circle(img_color, cv::Point(x, y), 1, cv::Scalar(colors[label]));
        }
    }
    cv::imshow("CCL", img_color);
    cv::waitKey(1);*/
}

CircleDetector::pcaInfo CircleDetector::pcaAnalysis(std::vector<cv::Point> points){
    cv::Mat data(points.size(), 2, CV_32F);
    for (int i = 0; i < data.rows; ++i) {
        data.at<float>(i, 0) = points[i].x;
        data.at<float>(i, 1) = points[i].y;
    }

    cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

    pcaInfo pca_now;

    pca_now.comp1 = cv::Point2d(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
    pca_now.comp2 = cv::Point2d(pca.eigenvectors.at<float>(1, 0), pca.eigenvectors.at<float>(1, 1));
    pca_now.vertical = pca_now.comp1.x * pca_now.comp2.x + pca_now.comp1.y * pca_now.comp2.y;
    pca_now.magnitude_comp1 = pca.eigenvalues.at<float>(0, 0);
    pca_now.magnitude_comp2 = pca.eigenvalues.at<float>(1, 0);
    
    pca_now.center = cv::Point2d(pca.mean.at<float>(0, 0), pca.mean.at<float>(0, 1));

    //std::cout << pca.eigenvalues << std::endl << "PCA 1: " << cv::fastAtan2(pca_now.comp1.y, pca_now.comp1.x) << " PCA 2: " << cv::fastAtan2(pca_now.comp2.y, pca_now.comp2.x) << std::endl;
    return pca_now;
}

double CircleDetector::distanceFromTwoClusters(const circleInformation& cluster_1, const circleInformation& cluster_2){
    cv::Point2f center_1, center_2;
    for (auto a:cluster_1.area){
        center_1 += (cv::Point2f)a;
    }
    for (auto b:cluster_2.area){
        center_2 += (cv::Point2f)b;
    }
    center_1 = cv::Point2f(center_1.x / cluster_1.area.size(), center_1.y / cluster_1.area.size());
    center_2 = cv::Point2f(center_2.x / cluster_2.area.size(), center_2.y / cluster_2.area.size());

    return euclideanDistance(center_1, center_2);
}

void CircleDetector::eventMaptDetect(const cv::Mat& event_map_no_polarity, const cv::Mat& event_map_positive, const cv::Mat& event_map_negative){
    cv::Mat event_show = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_no_polarity, event_show, cv::COLOR_GRAY2BGR);
    cv::Mat imgMark = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_no_polarity, imgMark, cv::COLOR_GRAY2RGB);
    

    cv::medianBlur(event_map_positive, event_map_positive, 3);
    cv::medianBlur(event_map_negative, event_map_negative, 3);
    cv::Mat imgPos = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_positive, imgPos, cv::COLOR_GRAY2RGB);
    cv::Mat imgNeg = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_negative, imgNeg, cv::COLOR_GRAY2RGB);
    cv::Mat event_map_no_polarity_blurred = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC1);

    /*
    for( int y = 0; y < event_map_positive.rows; y++ ){
        for( int x = 0; x < event_map_positive.cols; x++ ){
            if (event_map_positive.at<float>(y, x) || event_map_negative.at<float>(y, x))
                event_map_no_polarity_blurred.at<float>(y, x) = 1;
        }
    }    
    cv::imshow("full_blur", event_map_no_polarity_blurred * 255);
    cv::waitKey(1);*/

    cv::Mat event_map_positive_8U, event_map_negative_8U;
    event_map_positive.convertTo(event_map_positive_8U, CV_8UC1);
    event_map_negative.convertTo(event_map_negative_8U, CV_8UC1);

    candidate_pos.clear();
    connectedComponentLabeling(event_map_positive_8U, quadArea_pos);
    for (auto quad : quadArea_pos){
        pcaInfo temp_pca = pcaAnalysis(quad);

        if (temp_pca.magnitude_comp1 / temp_pca.magnitude_comp2 > PCA_high || temp_pca.magnitude_comp1 / temp_pca.magnitude_comp2 < PCA_low){continue;}

        circleInformation temp_circ;
        temp_circ.pca_res = temp_pca;
        temp_circ.area = quad;
        candidate_pos.push_back(temp_circ);

        for (auto p : quad){
            cv::circle(imgPos, p, 1, cv::Scalar(200,  0, 0), -1);
        }        
        cv::arrowedLine(imgPos, temp_pca.center, temp_pca.center + temp_pca.comp1 * temp_pca.magnitude_comp1, cv::Scalar(0, 200, 0), 3);
        cv::arrowedLine(imgPos, temp_pca.center, temp_pca.center + temp_pca.comp2 * temp_pca.magnitude_comp2, cv::Scalar(0, 0, 200), 3);
        cv::circle(imgPos, temp_pca.center, 5, cv::Scalar(200,  100, 100), -1);
    }    
    cv::imshow("pos", imgPos);
    cv::waitKey(1);    
    
    candidate_neg.clear();
    connectedComponentLabeling(event_map_negative_8U, quadArea_neg);
    for (auto quad : quadArea_neg){
        pcaInfo temp_pca = pcaAnalysis(quad);

        if (temp_pca.magnitude_comp1 / temp_pca.magnitude_comp2 > PCA_high || temp_pca.magnitude_comp1 / temp_pca.magnitude_comp2 < PCA_low){continue;}

        circleInformation temp_circ;
        temp_circ.pca_res = temp_pca;
        temp_circ.area = quad;
        candidate_neg.push_back(temp_circ);

        for (auto p : quad){
            cv::circle(imgNeg, p, 1, cv::Scalar(200,  0, 0), -1);
        }        
        cv::arrowedLine(imgNeg, temp_pca.center, temp_pca.center + temp_pca.comp1 * temp_pca.magnitude_comp1, cv::Scalar(0, 200, 0), 3);
        cv::arrowedLine(imgNeg, temp_pca.center, temp_pca.center + temp_pca.comp2 * temp_pca.magnitude_comp2, cv::Scalar(0, 0, 200), 3);
        cv::circle(imgNeg, temp_pca.center, 5, cv::Scalar(200,  100, 100), -1);
    }
    cv::imshow("neg", imgNeg);
    cv::waitKey(1); 
    
    candidate_full.clear();
    while (!candidate_pos.empty()) {
        cv::cvtColor(event_map_no_polarity, imgMark, cv::COLOR_GRAY2RGB);
        current = candidate_pos.front();
        candidate_pos.pop_front();
        for (auto p : current.area){
            cv::circle(imgMark, p, 1, cv::Scalar(200,  0, 0), -1);
        }
        std::sort(candidate_neg.begin(), candidate_neg.end(), [this](const circleInformation& p1, const circleInformation& p2) {
            return distanceFromTwoClusters(current, p1) < distanceFromTwoClusters(current, p2);
        });

        for (auto it = candidate_neg.begin(); it != candidate_neg.end();) {
            
            for (auto p : it->area){
                cv::circle(imgMark, p, 1, cv::Scalar(0,  0, 255), -1);
            }
            if (coEllipse(current, *it)) {
                candidate_full.emplace_back(std::make_pair(current, *it));
                it = candidate_neg.erase(it);
                //cv::imshow("ellipse", imgMark);
                //cv::waitKey(0);
                break;
            } else {
                ++it;
                //cv::imshow("ellipse", imgMark);
                //cv::waitKey(0);
            }
        }
    }

    candidate_circles.clear();
    for (auto cand_pair : candidate_full){
        std::vector<cv::Point> ellipse_points;
        ellipse_points.insert(ellipse_points.end(), cand_pair.first.area.begin(), cand_pair.first.area.end());
        ellipse_points.insert(ellipse_points.end(), cand_pair.second.area.begin(), cand_pair.second.area.end());
        m_ellipsetemp = cv::fitEllipse(ellipse_points);
        
        circleInformation temp_circ;
        temp_circ.height = m_ellipsetemp.size.height;
        temp_circ.width = m_ellipsetemp.size.width;
        temp_circ.circle_position = m_ellipsetemp.center;
        temp_circ.area = ellipse_points;
        candidate_circles.push_back(temp_circ);

        ellipse(imgMark, m_ellipsetemp, cv::Scalar(0, 255, 0), 2);
        //cv::line(imgMark, cvPoint(m_ellipsetemp.center.x - 5, m_ellipsetemp.center.y), cvPoint(m_ellipsetemp.center.x + 5, m_ellipsetemp.center.y), cv::Scalar(0,120,250), 1, 8, 0);	
        //cv::line(imgMark, cvPoint(m_ellipsetemp.center.x, m_ellipsetemp.center.y - 5), cvPoint(m_ellipsetemp.center.x, m_ellipsetemp.center.y + 5), cv::Scalar(120,120,250), 1, 8, 0);    
    }
    if (candidate_circles.size() < cb.boardHeight * cb.boardWidth){
        return;
    }

    cv::imshow("ellipse", imgMark);
    cv::waitKey(1);

    organizeCircles(event_map_no_polarity);

    // Publish circle messages
    for (auto circle : onboard_circles){
        circle_msg.x = circle.circle_position.x;
        circle_msg.y = circle.circle_position.y;
        circle_msg.x_grid = circle.grid_pose.x;
        circle_msg.y_grid = circle.grid_pose.y;
        circle_msg.timestamp = 0;
        circle_array.circles.push_back(circle_msg);
    }
    circle_pub_.publish(circle_array);
    ROS_INFO("Publishing circle message");

    return;
}

float CircleDetector::distanceFromLine(cv::Point2f input_point, cv::Point2f line_point, cv::Vec2f line_direction){
    cv::Point2f vector_to_input = input_point - line_point;
    double vector_magnitude = cv::norm(vector_to_input);
    cv::Point2f unit_vector = vector_to_input / vector_magnitude;

    if (cv::norm(line_direction) != 1){
        line_direction /= cv::norm(line_direction);
    }

    double dot_product = unit_vector.dot(line_direction);
    double distance = std::abs(vector_magnitude * std::sin(std::acos(dot_product)));

    return distance;
}

void CircleDetector::organizeCircles(const cv::Mat& event_map){
    cv::Mat image_plot = cv::Mat::zeros(event_map.size(), CV_32FC3);;
    cv::cvtColor(event_map, image_plot, cv::COLOR_GRAY2BGR);
    
    onboard_circles.clear();
    if (candidate_circles.size() < cb.boardHeight * cb.boardWidth) return;
    
    circleInformation best_circle = candidate_circles[0];

    cv::circle(image_plot, best_circle.circle_position, 20, cv::Scalar(120, 120, 120), 3);
      
    std::vector<std::pair<float, circleInformation>> dist_from_bestcircle;
    for (int i = 1; i < candidate_circles.size(); i++) {
        dist_from_bestcircle.push_back(std::make_pair(euclideanDistance(best_circle.circle_position, candidate_circles[i].circle_position), candidate_circles[i]));
    }
    // sort by distance
    std::sort(dist_from_bestcircle.begin(), dist_from_bestcircle.end(), 
              [](const std::pair<float, circleInformation> &a, const std::pair<float, circleInformation> &b) {
                  return a.first < b.first;
              });
    b_width_line_found = false;
    b_height_line_found = false;
    float min_dist_ = dist_from_bestcircle[0].first;
    for (int i = 0; i < dist_from_bestcircle.size(); i++){
        if (dist_from_bestcircle[i].first > 1.2 * min_dist_) break;
        //cv::circle(image_plot, dist_from_bestcircle[i].second.circle_position, 10, cv::Scalar(255, 0, 0), 3);

        int fitted_num = 0;
        cv::Vec2f direction_(best_circle.circle_position.x - dist_from_bestcircle[i].second.circle_position.x, best_circle.circle_position.y - dist_from_bestcircle[i].second.circle_position.y);

        for (int j = i + 1; j < dist_from_bestcircle.size(); j++){
            if (distanceFromLine(dist_from_bestcircle[j].second.circle_position, best_circle.circle_position, direction_) / euclideanDistance(dist_from_bestcircle[j].second.circle_position, best_circle.circle_position) < 0.05){
                fitted_num++;
                cv::circle(image_plot, dist_from_bestcircle[j].second.circle_position, 10, cv::Scalar(0, 230, 0), 3);
            }
        }
        if (!b_width_line_found && fitted_num == cb.boardWidth - 2){
            cv::Vec2f width_line = (dist_from_bestcircle[i].second.circle_position.y - best_circle.circle_position.y, dist_from_bestcircle[i].second.circle_position.x - best_circle.circle_position.x);
            width_direction_ = direction_;
            b_width_line_found = true;
        }
        if (!b_height_line_found && fitted_num == cb.boardHeight - 2){
            cv::Vec2f height_line = (dist_from_bestcircle[i].second.circle_position.y - best_circle.circle_position.y, dist_from_bestcircle[i].second.circle_position.x - best_circle.circle_position.x);
            height_direction_ = direction_;
            b_height_line_found = true;
        }
        
        if (b_width_line_found && b_height_line_found){
            ROS_INFO("Found width and height");
            break;
        } 
    }
    std::cout << "Height direction: " << height_direction_[0] << " " << height_direction_[1] << std::endl;
    std::cout << "Width direction:  " << width_direction_[0] << " " << width_direction_[1] << std::endl;

    std::vector<std::vector<circleInformation>> circle_cluster;
    std::vector<circleInformation> height_closest_circles, width_circles;
    height_closest_circles.push_back(best_circle);
    for (int i = 0; i < dist_from_bestcircle.size(); i++){
        if (distanceFromLine(dist_from_bestcircle[i].second.circle_position, best_circle.circle_position, height_direction_) / euclideanDistance(dist_from_bestcircle[i].second.circle_position, best_circle.circle_position) < 0.05){
            height_closest_circles.push_back(dist_from_bestcircle[i].second);
        }
    }
    // for (auto p:height_closest_circles){
    //     cv::circle(image_plot, p.circle_position, 7, cv::Scalar(0, 0, 250), 3);
    // }
    for (auto height_corner : height_closest_circles){
        width_circles.clear();
        width_circles.push_back(height_corner);
        //cv::circle(image_plot, height_corner.circle_position, 15, cv::Scalar(0, 250, 250), 3);
        for (auto corner : dist_from_bestcircle){
            if (distanceFromLine(corner.second.circle_position, height_corner.circle_position, width_direction_) / euclideanDistance(corner.second.circle_position, height_corner.circle_position) < 0.1){
                width_circles.push_back(corner.second);
            }
        }
        if (width_circles.size() < cb.boardWidth) return;
        circle_cluster.push_back(width_circles);
    }
    if (circle_cluster.size() < cb.boardHeight) return;
    
    //sort twice
    for (auto& cluster : circle_cluster) {
        std::sort(cluster.begin(), cluster.end(), std::abs(width_direction_[0]) > std::abs(width_direction_[1]) ? 
              [](const circleInformation& a, const circleInformation& b) {
                  return a.circle_position.x < b.circle_position.x;
              } :
              [](const circleInformation& a, const circleInformation& b) {
                  return a.circle_position.y < b.circle_position.y;
              });
    }

    std::sort(circle_cluster.begin(), circle_cluster.end(), std::abs(width_direction_[0]) > std::abs(width_direction_[1]) ?
              [](const std::vector<circleInformation>& a, const std::vector<circleInformation>& b) {
                  return a[0].circle_position.y < b[0].circle_position.y;
              } :
              [](const std::vector<circleInformation>& a, const std::vector<circleInformation>& b) {
                  return a[0].circle_position.x > b[0].circle_position.x;
              });


    // TODO judge first circle polarity

    bool b_reverse = true;
    for (int i = 0; i < circle_cluster.size(); i++){
        for (int j = 0; j < circle_cluster[i].size(); j++){
            circleInformation temp_corner;
            temp_corner = circle_cluster[i][j];
            temp_corner.grid_pose.x = b_reverse ? j : cb.boardWidth - j - 1;
            temp_corner.grid_pose.y = b_reverse ? i : cb.boardHeight - i - 1;
            onboard_circles.push_back(temp_corner);
        }
    }
    for (auto a : onboard_circles){
        cv::Vec3b color = cv::Vec3b(rand()%256, rand()%256, rand()%256);
        cv::circle(image_plot, a.circle_position, 10, color, 3);
        cv::putText(image_plot, std::to_string(a.grid_pose.y * cb.boardWidth + a.grid_pose.x), a.circle_position + cv::Point2f(2, 2), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 255, 0), 3);
    }
    cv::imshow("org corners", image_plot);
    cv::waitKey(1);
}

class EventQueueMat 
{
    struct Job{
        size_t start_col_, end_col_;
        size_t start_row_, end_row_;
        size_t i_thread_;
        ros::Time trig_time;
        cv::Mat* event_no_polarity_;
        cv::Mat* event_positive_;
        cv::Mat* event_negative_;
    };

public:
    EventQueueMat(int width, int height, int queueLen){
        width_ = width;
        height_ = height;
        queueLen_ = queueLen;
        eqMat_ = std::vector<EventQueue>(width_ * height_, EventQueue());
    }

    void insertEvent(const dvs_msgs::Event& e){
        if(!insideImage(e.x, e.y))
            return;
        else{
            EventQueue& eq = getEventQueue(e.x, e.y);
            eq.push_back(e);
            while(eq.size() > queueLen_)
                eq.pop_front();
        }
    }

    void generateEventMap(const ros::Time& triggered_time, CircleDetector& cd){
        cv::Mat event_map_no_polarity = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);
        cv::Mat event_map_positive = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);
        cv::Mat event_map_negative = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);
        for(size_t x = 0; x < width_; ++x){
            for(size_t y = 0; y < height_; ++y)
            {
                EventQueue& eq = getEventQueue(x, y);
                if (!eq.empty() && eq.back().ts > triggered_time){
                    event_map_no_polarity.at<float>(y, x) = 1;
                    if (eq.back().polarity){
                        event_map_positive.at<float>(y, x) = 1;
                    }else{
                        event_map_negative.at<float>(y, x) = 1;
                    }
                }
            }
        }
        cd.eventMaptDetect(event_map_no_polarity, event_map_positive, event_map_negative);
        ROS_INFO("Event map generated");
    }

    void generateEventMap_hyperthread(const ros::Time& triggered_time, CircleDetector& cd){
        ROS_INFO("Event map begin");

        cv::Mat event_map_no_polarity = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);
        cv::Mat event_map_positive = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);
        cv::Mat event_map_negative = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);

        // distribute jobs
        int NUM_THREAD_TS = 10;
        std::vector<Job> jobs(NUM_THREAD_TS);
        size_t num_col_per_thread = width_ / NUM_THREAD_TS;
        size_t res_col = width_ % NUM_THREAD_TS;
        for(size_t i = 0; i < NUM_THREAD_TS; i++)
        {
            jobs[i].i_thread_ = i;
            jobs[i].start_col_ = num_col_per_thread * i;
            if(i == NUM_THREAD_TS - 1)
                jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1 + res_col;
            else
                jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1;
            jobs[i].start_row_ = 0;
            jobs[i].end_row_ = height_ - 1;
            jobs[i].trig_time = triggered_time;
            jobs[i].event_no_polarity_ = &event_map_no_polarity;
            jobs[i].event_positive_ = &event_map_positive;
            jobs[i].event_negative_ = &event_map_negative;
        }

        // hyper thread processing
        std::vector<std::thread> threads;
        threads.reserve(NUM_THREAD_TS);
        for(size_t i = 0; i < NUM_THREAD_TS; i++)
            threads.emplace_back(std::bind(&EventQueueMat::thread, this, jobs[i]));
        for(auto& thread:threads)
            if(thread.joinable())
                thread.join();
        ROS_INFO("Generated");
        cd.eventMaptDetect(event_map_no_polarity, event_map_positive, event_map_negative);        
    }

    void clear(){
      eqMat_.clear();
    }

    bool insideImage(const size_t x, const size_t y){
      return !(x < 0 || x >= width_ || y < 0 || y >= height_);
    }

    inline EventQueue& getEventQueue(const size_t x, const size_t y){
      return eqMat_[x + width_ * y];
    }

    size_t width_;
    size_t height_;
    size_t queueLen_;
    std::vector<EventQueue> eqMat_;

private:
    void thread(Job &job){
        size_t start_col = job.start_col_;
        size_t end_col = job.end_col_;
        size_t start_row = job.start_row_;
        size_t end_row = job.end_row_;
        size_t i_thread = job.i_thread_;
        ros::Time trig_time = job.trig_time;
        cv::Mat& event_map_no_polarity = *job.event_no_polarity_;
        cv::Mat& event_map_positive = *job.event_positive_;
        cv::Mat& event_map_negative = *job.event_negative_;

        for(size_t y = start_row; y <= end_row; y++){
            for(size_t x = start_col; x <= end_col; x++){
                EventQueue& eq = getEventQueue(x, y);
                if (!eq.empty() && eq.back().ts > trig_time){
                    event_map_no_polarity.at<float>(y, x) = 1;
                    if (eq.back().polarity){
                        event_map_positive.at<float>(y, x) = 1;
                    }else{
                        event_map_negative.at<float>(y, x) = 1;
                    }
                }
            }
        }
    }
};

class EventProcessor {
public:
    EventProcessor(ros::NodeHandle& nh);
    ~EventProcessor(){event_sub_.shutdown();};

private:
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    void init(int width, int height);
    void clearEventQueue();

    ros::NodeHandle nh_;
    ros::Subscriber event_sub_;
    CircleDetector cd_;

    bool bSensorInitialized_ = false;
    int max_event_queue_length_ = 1;
    int output_event_num_;
    ros::Time trig_time_ = ros::Time::now();

    // containers
    EventQueue events_;
    std::shared_ptr<EventQueueMat> pEventQueueMat_;
};  

EventProcessor::EventProcessor(ros::NodeHandle& nh) : cd_(nh){
    event_sub_ = nh.subscribe("/dvs/events", 0, &EventProcessor::eventsCallback, this);
    nh.param("BoardHeight", cb.boardHeight, 3);
    nh.param("BoardWidth", cb.boardWidth, 6);
    
    // Judge if event queue is empty
    if(pEventQueueMat_){ 
        pEventQueueMat_->clear();
    }
}

void EventProcessor::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg){
    if(!bSensorInitialized_){
        init(msg->width, msg->height);
        output_event_num_ = (int)(msg->width * msg->height * 0.2);
    }     

    for(const dvs_msgs::Event& e : msg->events){        
        events_.emplace_back(e);
        int i = events_.size() - 2;
        while(i >= 0 && events_[i].ts > e.ts)
        {
            events_[i + 1] = events_[i];
            i--;
        }
        events_[i + 1] = e;

        const dvs_msgs::Event& last_event = events_.back();
        pEventQueueMat_->insertEvent(last_event);
        
        if (events_.size() >= output_event_num_){
            std::cout << trig_time_.sec << " " << trig_time_.nsec << std::endl;
            pEventQueueMat_->generateEventMap_hyperthread(trig_time_, cd_);
            trig_time_ = last_event.ts;
            ROS_INFO("Event map message launched");
            clearEventQueue();
        }
    }
}

void EventProcessor::init(int width, int height){
    sensor_size_ = cv::Size(width, height);
    bSensorInitialized_ = true;
    pEventQueueMat_.reset(new EventQueueMat(width, height, max_event_queue_length_));
    ROS_INFO("Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);
}

void EventProcessor::clearEventQueue()
{
    /*static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 5000000;
    if (events_.size() > MAX_EVENT_QUEUE_LENGTH)
    {
      size_t remove_events = events_.size() - MAX_EVENT_QUEUE_LENGTH;
      events_.erase(events_.begin(), events_.begin() + remove_events);
    }*/
    events_.clear();
}


} // namespace circle_detector


#endif // circle_detector