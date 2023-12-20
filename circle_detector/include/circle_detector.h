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

struct Chessboard{
    int boardHeight, boardWidth;
}cb;

class CircleDetector{

struct circleInformation {
	cv::Point2f circle_position;
    float width, height;
    std::vector<cv::Point> area;
    cv::Point grid_pose;
    int timestamp;
};

public:
    CircleDetector(ros::NodeHandle& nh) : nh_(nh){};
    ~CircleDetector(){
        circle_pub_.shutdown();
    };
    void eventMaptDetect(const cv::Mat& event_map_no_polarity, const cv::Mat& event_map_positive, const cv::Mat& event_map_negative);
    
private:
    void connectedComponentLabeling(const cv::Mat& src, std::vector<std::vector<cv::Point>>& quadArea);
    double euclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2);
    bool coEllipse(const circleInformation& a, const circleInformation& b);
    void organizeCircles();

    ros::NodeHandle nh_;
    ros::Publisher circle_pub_ = nh_.advertise<circle_msgs::circleArray>("/circle_ev/circleArray", 10);;
    circle_msgs::circle circle_msg;
    circle_msgs::circleArray circle_array;

    int count = 0;
    std::vector<std::vector<cv::Point>> quadArea_pos, quadArea_neg;
    std::vector<circleInformation> candidate_pos, candidate_neg, onboard_circles;
    std::vector<std::pair<circleInformation, circleInformation>> candidate_full;
    cv::RotatedRect m_ellipsetemp; 
};

double CircleDetector::euclideanDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool CircleDetector::coEllipse(const circleInformation& a, const circleInformation& b){
    double dist = euclideanDistance(a.circle_position, b.circle_position);
    if (dist < std::min(a.height, a.width) / 2 && dist < std::min(b.height, b.width) / 2){
        return true;
    }        
    else{
        return false;
    }        
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
    cv::waitKey(1);
}

void CircleDetector::eventMaptDetect(const cv::Mat& event_map_no_polarity, const cv::Mat& event_map_positive, const cv::Mat& event_map_negative){
    cv::Mat event_show = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::Mat pos_and_neg = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols * 2, CV_32FC1);
    cv::Mat imgMark = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_no_polarity, imgMark, cv::COLOR_GRAY2RGB);

    cv::medianBlur(event_map_no_polarity, event_map_no_polarity, 3);
    cv::medianBlur(event_map_positive, event_map_positive, 3);
    cv::medianBlur(event_map_negative, event_map_negative, 3);
    cv::cvtColor(event_map_no_polarity, event_show, cv::COLOR_GRAY2BGR);

    cv::Mat event_map_no_polarity_8U, event_map_positive_8U, event_map_negative_8U;
    event_map_no_polarity.convertTo(event_map_no_polarity_8U, CV_8UC1);
    event_map_positive.convertTo(event_map_positive_8U, CV_8UC1);
    event_map_negative.convertTo(event_map_negative_8U, CV_8UC1);

    candidate_pos.clear();
    connectedComponentLabeling(event_map_positive_8U, quadArea_pos);
    for (auto quad : quadArea_pos){
        m_ellipsetemp = cv::fitEllipse(quad);
        
        // area restriction 
		if (m_ellipsetemp.size.width * m_ellipsetemp.size.height > 0.02 * event_map_no_polarity.rows * event_map_no_polarity.cols) continue;

        //if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.5 || m_ellipsetemp.size.height / m_ellipsetemp.size.width < 0.5) continue;

        // angle restriction
        float angle_min_x = 1000, angle_max_x = -1, angle_min_y = 1000, angle_max_y = -1, angle_now, angle_min, angle_max;
        float dist_min = 1000, dist_max = -1, dist_now;
        for (auto point : quad){
            dist_now = euclideanDistance(point, m_ellipsetemp.center);
            if (dist_now > dist_max) dist_max = dist_now;
            if (dist_now < dist_min) dist_min = dist_now;

            angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, point.x - m_ellipsetemp.center.x);
            if (angle_now > angle_max_x) angle_max_x = angle_now;
            if (angle_now < angle_min_x) angle_min_x = angle_now;
            angle_now = cv::fastAtan2(point.x - m_ellipsetemp.center.x, point.y - m_ellipsetemp.center.y);
            if (angle_now > angle_max_y) angle_max_y = angle_now;
            if (angle_now < angle_min_y) angle_min_y = angle_now;
        }
        
        if (angle_max_x - angle_min_x < angle_max_y - angle_min_y){
            angle_max = angle_max_x;
            angle_min = angle_min_x;
        }else{
            angle_max = angle_max_y;
            angle_min = angle_min_y;
        }

        float ratio_max = std::abs(dist_max - std::max(m_ellipsetemp.size.width / 2, m_ellipsetemp.size.height / 2)) / std::max(m_ellipsetemp.size.width / 2, m_ellipsetemp.size.height / 2);
        float ratio_min = std::abs(dist_min - std::min(m_ellipsetemp.size.width / 2, m_ellipsetemp.size.height / 2)) / std::min(m_ellipsetemp.size.width / 2, m_ellipsetemp.size.height / 2);
        std::cout << "Angle diff: " << angle_max - angle_min << "Angle max: " << angle_max << " min:" << angle_min << std::endl << "Dist max: " << dist_max << " min:" << dist_min << std::endl;
        std::cout << "Ratio max: " << ratio_max << " min: " << ratio_min << std::endl;
                
        if (std::max(ratio_min, ratio_max) > 0.2) {continue;};
        if (angle_max - angle_min > 180 || angle_max - angle_min < 100) {continue;};
        
        circleInformation temp_circ;
        temp_circ.height = m_ellipsetemp.size.height;
        temp_circ.width = m_ellipsetemp.size.width;
        temp_circ.circle_position = m_ellipsetemp.center;
        temp_circ.area = quad;
        candidate_pos.push_back(temp_circ);

        //ellipse(imgMark, m_ellipsetemp, cv::Scalar(255, 0, 0), 3);
        //cv::line(imgMark, cvPoint(m_ellipsetemp.center.x - 10, m_ellipsetemp.center.y), cvPoint(m_ellipsetemp.center.x + 10, m_ellipsetemp.center.y), cv::Scalar(0,120,250), 5, 8, 0);	
        //cv::line(imgMark, cvPoint(m_ellipsetemp.center.x, m_ellipsetemp.center.y - 10), cvPoint(m_ellipsetemp.center.x, m_ellipsetemp.center.y + 10), cv::Scalar(120,120,250), 5, 8, 0);    
    }    

    candidate_neg.clear();
    connectedComponentLabeling(event_map_negative_8U, quadArea_neg);
    for (auto quad : quadArea_neg){
        m_ellipsetemp = cv::fitEllipse(quad);
        
        // area restriction 
		if (m_ellipsetemp.size.width * m_ellipsetemp.size.height > 0.02 * event_map_no_polarity.rows * event_map_no_polarity.cols) continue;

        //if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.5 || m_ellipsetemp.size.height / m_ellipsetemp.size.width < 0.5) continue;

        // angle restriction
        float angle_min_x = 1000, angle_max_x = -1, angle_min_y = 1000, angle_max_y = -1, angle_now, angle_min, angle_max;
        float dist_min = 1000, dist_max = -1, dist_now;
        for (auto point : quad){
            dist_now = euclideanDistance(point, m_ellipsetemp.center);
            if (dist_now > dist_max) dist_max = dist_now;
            if (dist_now < dist_min) dist_min = dist_now;

            angle_now = cv::fastAtan2(point.y - m_ellipsetemp.center.y, point.x - m_ellipsetemp.center.x);
            if (angle_now > angle_max_x) angle_max_x = angle_now;
            if (angle_now < angle_min_x) angle_min_x = angle_now;
            angle_now = cv::fastAtan2(point.x - m_ellipsetemp.center.x, point.y - m_ellipsetemp.center.y);
            if (angle_now > angle_max_y) angle_max_y = angle_now;
            if (angle_now < angle_min_y) angle_min_y = angle_now;
        }
        
        if (angle_max_x - angle_min_x < angle_max_y - angle_min_y){
            angle_max = angle_max_x;
            angle_min = angle_min_x;
        }else{
            angle_max = angle_max_y;
            angle_min = angle_min_y;
        }

        float ratio_max = std::abs(dist_max - std::max(m_ellipsetemp.size.width / 2, m_ellipsetemp.size.height / 2)) / std::max(m_ellipsetemp.size.width / 2, m_ellipsetemp.size.height / 2);
        float ratio_min = std::abs(dist_min - std::min(m_ellipsetemp.size.width / 2, m_ellipsetemp.size.height / 2)) / std::min(m_ellipsetemp.size.width / 2, m_ellipsetemp.size.height / 2);
        std::cout << "Angle diff: " << angle_max - angle_min << "Angle max: " << angle_max << " min:" << angle_min << std::endl << "Dist max: " << dist_max << " min:" << dist_min << std::endl;
        std::cout << "Ratio max: " << ratio_max << " min: " << ratio_min << std::endl;
                
        if (std::max(ratio_min, ratio_max) > 0.2) {continue;};
        if (angle_max - angle_min > 180 || angle_max - angle_min < 100) {continue;};
        
        circleInformation temp_circ;
        temp_circ.height = m_ellipsetemp.size.height;
        temp_circ.width = m_ellipsetemp.size.width;
        temp_circ.circle_position = m_ellipsetemp.center;
        temp_circ.area = quad;
        candidate_neg.push_back(temp_circ);

        //ellipse(imgMark, m_ellipsetemp, cv::Scalar(0, 255, 0), 3);
        //cv::line(imgMark, cvPoint(m_ellipsetemp.center.x - 10, m_ellipsetemp.center.y), cvPoint(m_ellipsetemp.center.x + 10, m_ellipsetemp.center.y), cv::Scalar(0,120,250), 5, 8, 0);	
        //cv::line(imgMark, cvPoint(m_ellipsetemp.center.x, m_ellipsetemp.center.y - 10), cvPoint(m_ellipsetemp.center.x, m_ellipsetemp.center.y + 10), cv::Scalar(120,120,250), 5, 8, 0);    
    }

    cv::hconcat(event_map_positive, event_map_negative, pos_and_neg);
    cv::imshow("pos and neg", pos_and_neg);
    cv::waitKey(1);

    candidate_full.clear();
    for (auto cand_pos : candidate_pos){
        for (auto cand_neg : candidate_neg){
            if (coEllipse(cand_pos, cand_neg)){
                candidate_full.emplace_back(std::make_pair(cand_pos, cand_neg));
                break;
            }
        }
    }

    for (auto cand_pair : candidate_full){
        std::vector<cv::Point> ellipse_points;
        ellipse_points.insert(ellipse_points.end(), cand_pair.first.area.begin(), cand_pair.first.area.end());
        ellipse_points.insert(ellipse_points.end(), cand_pair.second.area.begin(), cand_pair.second.area.end());
        m_ellipsetemp = cv::fitEllipse(ellipse_points);
        
        // area restriction 
		if (m_ellipsetemp.size.width * m_ellipsetemp.size.height > 0.02 * event_map_no_polarity.rows * event_map_no_polarity.cols) continue;

        circleInformation temp_circ;
        temp_circ.height = m_ellipsetemp.size.height;
        temp_circ.width = m_ellipsetemp.size.width;
        temp_circ.circle_position = m_ellipsetemp.center;
        temp_circ.area = ellipse_points;
        onboard_circles.push_back(temp_circ);

        ellipse(imgMark, m_ellipsetemp, cv::Scalar(0, 255, 255), 6);
        cv::line(imgMark, cvPoint(m_ellipsetemp.center.x - 10, m_ellipsetemp.center.y), cvPoint(m_ellipsetemp.center.x + 10, m_ellipsetemp.center.y), cv::Scalar(0,120,250), 5, 8, 0);	
        cv::line(imgMark, cvPoint(m_ellipsetemp.center.x, m_ellipsetemp.center.y - 10), cvPoint(m_ellipsetemp.center.x, m_ellipsetemp.center.y + 10), cv::Scalar(120,120,250), 5, 8, 0);    
    }
    cv::imshow("ellipse", imgMark);
    cv::waitKey(1);

    onboard_circles.clear();
    if (onboard_circles.size() < cb.boardHeight * cb.boardWidth - 5){
        return;
    }
    organizeCircles();

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

void CircleDetector::organizeCircles(){
    
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

    cv::Size sensor_size_;
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
        output_event_num_ = (int)(msg->width * msg->height * 0.25);
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