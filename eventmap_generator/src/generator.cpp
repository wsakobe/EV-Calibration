#include "generator.h"

namespace generator{

void measureTimeElapsed() {
    static std::chrono::time_point<std::chrono::steady_clock> last_time = std::chrono::steady_clock::now();
    auto current_time = std::chrono::steady_clock::now();

    // 计算时间差
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);
    
    // 输出时间差
    std::cout << "Time elapsed since last call: " << duration.count() << " milliseconds" << std::endl;

    // 更新上次调用的时间点为当前时间
    last_time = current_time;
}

EventMap::EventMap(ros::NodeHandle& nh, ros::NodeHandle& nh_private) : nh_(nh){
    event_sub_ = nh_.subscribe("/dvs/events", 10, &EventMap::eventsCallback, this);
    image_transport::ImageTransport it_(nh_);
    eventmap_pub_ = it_.advertise("/event_map/image", 1);

    // Judge if event queue is empty
    bSensorInitialized_ = false;
    if(pEventQueueMat_)
        pEventQueueMat_->clear();
    sensor_size_ = cv::Size(0,0);
}

void EventMap::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg){
    if(!bSensorInitialized_){
        init(msg->width, msg->height);
        output_event_num_ = (int)(msg->width * msg->height * 0.25);
    }     

    //measureTimeElapsed();
    for(const dvs_msgs::Event& e : msg->events){
        static bool first_input = true;
        if (first_input){
            first_input = false;
            trig_time_ = e.ts;
        }
        events_.push_back(e);
        
        ros::Duration duration = events_.back().ts - trig_time_;
        double time_diff_ms = duration.toSec() * 1000;
        if (time_diff_ms > 80){
            clearEventQueue();
            for (auto last_event:events_){
                pEventQueueMat_->insertEvent(last_event);
            }
            ROS_INFO("The timestamp is %lld.%lld:", trig_time_.sec, trig_time_.nsec);
            generateEventMap_hyperthread(trig_time_);
            trig_time_ = events_.back().ts;
        }
    }

    //std::cout << "generate cost:";
    //measureTimeElapsed();
}

void EventMap::init(int width, int height){
    sensor_size_ = cv::Size(width, height);
    bSensorInitialized_ = true;
    pEventQueueMat_.reset(new EventQueueMat(width, height, max_event_queue_length_));
    ROS_INFO("Sensor size: (%d x %d)", sensor_size_.width, sensor_size_.height);
}

void EventMap::clearEventQueue()
{
    static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 20000;
    if (events_.size() > MAX_EVENT_QUEUE_LENGTH)
    {
      size_t remove_events = events_.size() - MAX_EVENT_QUEUE_LENGTH;
      events_.erase(events_.begin(), events_.begin() + remove_events);
    }
}

void EventMap::generateEventMap(const ros::Time& triggered_time){
    cv::Mat event_map_no_polarity = cv::Mat::zeros(cv::Size(sensor_size_.width, sensor_size_.height), CV_32F);
    for(size_t x = 0; x < sensor_size_.width; ++x){
        for(size_t y = 0; y < sensor_size_.height; ++y)
        {
            EventQueue& eq = pEventQueueMat_->getEventQueue(x, y);
            if (!eq.empty() && eq.back().ts > triggered_time){
                //event_map_no_polarity.at<float>(y, x) = 1;
                if (eq.back().polarity){
                    event_map_no_polarity.at<float>(y, x) = 1;
                }else{
                    event_map_no_polarity.at<float>(y, x) = -1;
                }
            }
        }
    }
    event_map_no_polarity = (event_map_no_polarity + 1) / 2;
    ROS_INFO("Event map generated");
    
    // Publish event image
    static cv_bridge::CvImage cv_image;
    cv_image.encoding = "32FC1";
    cv_image.image = event_map_no_polarity.clone();
    cv_image.header.stamp = triggered_time;

    if(eventmap_pub_.getNumSubscribers() > 0)
    {
        eventmap_pub_.publish(cv_image.toImageMsg());
    }
}

void EventMap::generateEventMap_hyperthread(const ros::Time& triggered_time){
    ROS_INFO("Event map begin");
    cv::Mat event_map_no_polarity = cv::Mat::zeros(cv::Size(sensor_size_.width, sensor_size_.height), CV_32F);

    // distribute jobs
    int NUM_THREAD_TS = 20;
    std::vector<Job> jobs(NUM_THREAD_TS);
    size_t num_col_per_thread = sensor_size_.width / NUM_THREAD_TS;
    size_t res_col = sensor_size_.width % NUM_THREAD_TS;
    for(size_t i = 0; i < NUM_THREAD_TS; i++)
    {
        jobs[i].i_thread_ = i;
        jobs[i].start_col_ = num_col_per_thread * i;
        if(i == NUM_THREAD_TS - 1)
            jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1 + res_col;
        else
            jobs[i].end_col_ = jobs[i].start_col_ + num_col_per_thread - 1;
        jobs[i].start_row_ = 0;
        jobs[i].end_row_ = sensor_size_.height - 1;
        jobs[i].trig_time = triggered_time;
        jobs[i].event_no_polarity_ = &event_map_no_polarity;
    }

    // hyper thread processing
    std::vector<std::thread> threads;
    threads.reserve(NUM_THREAD_TS);
    for(size_t i = 0; i < NUM_THREAD_TS; i++)
        threads.emplace_back(std::bind(&EventMap::thread, this, jobs[i]));
    for(auto& thread:threads)
        if(thread.joinable())
            thread.join();

    event_map_no_polarity = (event_map_no_polarity + 1) / 2;
    ROS_INFO("Event map generated");
    
    // Publish event image
    static cv_bridge::CvImage cv_image;
    cv_image.encoding = "mono8";
    cv_image.image = event_map_no_polarity.clone();
    cv_image.header.stamp = triggered_time;

    if(eventmap_pub_.getNumSubscribers() > 0)
    {
        eventmap_pub_.publish(cv_image.toImageMsg());
    }
}

void EventMap::thread(Job &job){
    size_t start_col = job.start_col_;
    size_t end_col = job.end_col_;
    size_t start_row = job.start_row_;
    size_t end_row = job.end_row_;
    size_t i_thread = job.i_thread_;
    ros::Time trig_time = job.trig_time;
    cv::Mat& event_map_no_polarity = *job.event_no_polarity_;

    for(size_t y = start_row; y <= end_row; y++){
        for(size_t x = start_col; x <= end_col; x++){
            EventQueue& eq = pEventQueueMat_->getEventQueue(x, y);
            if (!eq.empty() && eq.back().ts > trig_time){
                if (eq.back().polarity){
                    event_map_no_polarity.at<float>(y, x) = 1;
                }else{
                    event_map_no_polarity.at<float>(y, x) = -1;
                }
            }
        }
    }
}

};