#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <semaphore.h>
#include <sensor_msgs/PointCloud2.h>

namespace vision {
class Algorithm : public std::enable_shared_from_this<Algorithm> {
public:
    using Ptr = std::shared_ptr<Algorithm>;
    Algorithm(/* args */);
    ~Algorithm();

private:
    int num_detector_;
    std::mutex mt_;
    sem_t detector_sem_;
};

} // namespace vision

#endif
