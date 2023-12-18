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

template<typename T>
void info(T msg) {
    std::cout << msg << std::endl;
}

namespace circle_detector{

cv::Mat event_map_no_polarity, event_map_positive, event_map_negative;
using EventQueue = std::deque<dvs_msgs::Event>;

struct Chessboard{
    int boardHeight, boardWidth;
}cb;

class CircleDetector{

struct circleInformation {
	cv::Point2f circle_position;
    float width, height;
    cv::Point grid_pose;
    int timestamp;
};

public:
    CircleDetector(){};
    ~CircleDetector(){
        circle_pub_.shutdown();
    };
    void eventMaptDetect(const cv::Mat& event_map_no_polarity, const cv::Mat& event_map_positive, const cv::Mat& event_map_negative);
    
private:
    void clusterEvents(const cv::Mat& event_map_no_polarity);
    void findCircle(const cv::Mat& event_map_positive, const cv::Mat& event_map_negative);
    void organizeCircles();

    ros::Publisher circle_pub_;
    circle_msgs::circle circle_msg;
    circle_msgs::circleArray circle_array;

    int count = 0;

    std::vector<circleInformation> onboard_circles;
    cv::RotatedRect m_ellipsetemp; 
};

void CircleDetector::eventMaptDetect(const cv::Mat& event_map_no_polarity, const cv::Mat& event_map_positive, const cv::Mat& event_map_negative){
    cv::Mat imgContour = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
 
    //cv::medianBlur(event_map_no_polarity, event_map_no_polarity, 3);
    cv::Mat event_map_no_polarity_8U;
    event_map_no_polarity.convertTo(event_map_no_polarity_8U, CV_8UC1);
    cv::imshow("No pola", event_map_no_polarity_8U * 255);
    cv::waitKey(1);
    //std::string str = "/data/out/" + std::to_string(count++) + ".bmp";
    //cv::imwrite(str, event_map_no_polarity * 255);
    
    std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
    cv::findContours(event_map_no_polarity_8U, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    onboard_circles.clear();

    for (int i = 0; i < contours.size();i++){
    	if (contours[i].size() <= 50 || contours[i].size() >= 200) continue;
		if (contourArea(contours[i]) < 30 && contourArea(contours[i]) > 100) continue;

        cv::drawContours(imgContour, contours, i, cv::Scalar(255, 5, 55), 2, cv::LINE_8);

		m_ellipsetemp = cv::fitEllipse(contours[i]);
        
		if (m_ellipsetemp.size.width / m_ellipsetemp.size.height < 0.8 && m_ellipsetemp.size.height / m_ellipsetemp.size.width < 0.8) continue;

        circleInformation temp_circ;
        temp_circ.height = m_ellipsetemp.size.height;
        temp_circ.width = m_ellipsetemp.size.width;
        temp_circ.circle_position = m_ellipsetemp.center;
        onboard_circles.push_back(temp_circ);
	}
    cv::imshow("contour", imgContour);
    cv::waitKey(1);
       
    if (onboard_circles.size() < cb.boardHeight * cb.boardWidth - 5){
        return;
    }
    organizeCircles();
    for (auto circle : onboard_circles){
        circle_msg.x = circle.circle_position.x;
        circle_msg.y = circle.circle_position.y;
        circle_msg.x_grid = circle.grid_pose.x;
        circle_msg.y_grid = circle.grid_pose.y;
        circle_msg.timestamp = 0;
        circle_array.circles.push_back(circle_msg);
    }
    //circle_pub_.publish(circle_array);
    ROS_INFO("Publishing circle message");

    return;
}

void CircleDetector::organizeCircles(){
    cv::Mat imgMark = cv::Mat::zeros(event_map_no_polarity.rows, event_map_no_polarity.cols, CV_32FC3);
    cv::cvtColor(event_map_no_polarity, imgMark, cv::COLOR_GRAY2RGB);

    // plot ellipse
    for (auto circle : onboard_circles){
        //ellipse(imgMark, m_ellipsetemp, cv::Scalar(255, 0, 0), 5);
        cv::line(imgMark, cvPoint(circle.circle_position.x - 10, circle.circle_position.y), cvPoint(circle.circle_position.x + 10, circle.circle_position.y), cv::Scalar(0,120,250), 5, 8, 0);	
        cv::line(imgMark, cvPoint(circle.circle_position.x, circle.circle_position.y - 10), cvPoint(circle.circle_position.x, circle.circle_position.y + 10), cv::Scalar(120,120,250), 5, 8, 0);
    }
    
    cv::imshow("ellipse", imgMark);
    cv::waitKey(1);
}

class EventQueueMat 
{
    struct Job
    {
        size_t start_col_, end_col_;
        size_t start_row_, end_row_;
        size_t i_thread_;
        ros::Time trig_time;
    };

public:
    EventQueueMat(int width, int height, int queueLen)
    {
        width_ = width;
        height_ = height;
        event_map_no_polarity = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);
        event_map_positive = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);
        event_map_negative = cv::Mat::zeros(cv::Size(width_, height_), CV_32F);
        queueLen_ = queueLen;
        eqMat_ = std::vector<EventQueue>(width_ * height_, EventQueue());
    }

    void insertEvent(const dvs_msgs::Event& e)
    {
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
        for(size_t x = 0; x < width_; ++x){
            for(size_t y = 0; y < height_; ++y)
            {
                event_map_no_polarity.at<float>(y, x) = 0;
                event_map_negative.at<float>(y, x) = 0;
                event_map_positive.at<float>(y, x) = 0;
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
        }

        // hyper thread processing
        std::vector<std::thread> threads;
        threads.reserve(NUM_THREAD_TS);
        for(size_t i = 0; i < NUM_THREAD_TS; i++)
            threads.emplace_back(std::bind(&EventQueueMat::thread, this, jobs[i]));
        for(auto& thread:threads)
            if(thread.joinable())
                thread.join();

        cd.eventMaptDetect(event_map_no_polarity, event_map_positive, event_map_negative);
        ROS_INFO("Event map generated");
    }

    void clear()
    {
      eqMat_.clear();
    }

    bool insideImage(const size_t x, const size_t y)
    {
      return !(x < 0 || x >= width_ || y < 0 || y >= height_);
    }

    inline EventQueue& getEventQueue(const size_t x, const size_t y)
    {
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

        for(size_t y = start_row; y <= end_row; y++)
            for(size_t x = start_col; x <= end_col; x++){
                event_map_no_polarity.at<float>(y, x) = 0;
                event_map_negative.at<float>(y, x) = 0;
                event_map_positive.at<float>(y, x) = 0;
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
};

class EventProcessor {
public:
    EventProcessor(ros::NodeHandle& nh, CircleDetector& cd);
    ~EventProcessor(){event_sub_.shutdown();};

private:
    void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    void init(int width, int height);
    void clearEventQueue();

    ros::NodeHandle nh_;
    CircleDetector cd_;
    ros::Subscriber event_sub_;

    cv::Size sensor_size_;
    bool bSensorInitialized_ = false;
    int max_event_queue_length_ = 1;
    int output_event_num_;
    ros::Time trig_time_ = ros::Time::now();

    // containers
    EventQueue events_;
    std::shared_ptr<EventQueueMat> pEventQueueMat_;
};  

EventProcessor::EventProcessor(ros::NodeHandle& nh, CircleDetector& cd) : nh_(nh), cd_(cd) {
    event_sub_ = nh_.subscribe("/dvs/events", 0, &EventProcessor::eventsCallback, this);
    nh_.param("BoardHeight", cb.boardHeight, 3);
    nh_.param("BoardWidth", cb.boardWidth, 6);

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

    for(const dvs_msgs::Event& e : msg->events)
    {        
        events_.push_back(e);
        int i = events_.size() - 2;
        while(i >= 0 && events_[i].ts > e.ts)
        {
            events_[i + 1] = events_[i];
            i--;
        }
        events_[i + 1] = e;

        const dvs_msgs::Event& last_event = events_.back();
        pEventQueueMat_->insertEvent(last_event);

        if (events_.size() == output_event_num_){
            clearEventQueue();
            pEventQueueMat_->generateEventMap_hyperthread(trig_time_, cd_);
            trig_time_ = last_event.ts;
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