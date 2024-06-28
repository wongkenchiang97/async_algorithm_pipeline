#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <async_algorithm_pipeline/async_detector.h>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <ros/ros.h>
#include <semaphore.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>

namespace vision {
class Algorithm : public std::enable_shared_from_this<Algorithm> {
public:
    using Ptr = std::shared_ptr<Algorithm>;
    Algorithm(/* args */);
    ~Algorithm();
    void sensorCallback(sensor_msgs::PointCloud2::ConstPtr _msg);
    void createDetectorProcess();

private:
    int num_detector_;
    std::mutex mt_;
    sem_t detector_sem_;
    std::thread create_detector_thread_;
    std::condition_variable create_detector_cv_;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> msgs_;
};

} // namespace vision

#endif
