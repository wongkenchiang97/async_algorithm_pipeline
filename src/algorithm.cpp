#include <async_algorithm_pipeline/algorithm.h>

namespace vision {

Algorithm::Algorithm(/* args */)
{
    create_detector_thread_ = std::thread(&Algorithm::createDetectorProcess, this);
}

Algorithm::~Algorithm()
{
    create_detector_cv_.notify_all();
    if (create_detector_thread_.joinable())
        create_detector_thread_.join();
}

void Algorithm::sensorCallback(sensor_msgs::PointCloud2::ConstPtr _msg)
{
    bool create_detector = sem_trywait(&detector_sem_);

    if (!create_detector)
        return;

    std::lock_guard<std::mutex> lock(mt_);
    msgs_.push(_msg);
    if (msgs_.size() > num_detector_)
        msgs_.pop();

    create_detector_cv_.notify_one();
}

void Algorithm::createDetectorProcess()
{
    while (true) {
        /*程序沉睡控制*/
        std::unique_lock<std::mutex> lock(mt_);
        create_detector_cv_.wait(lock, [&]() {
            bool wait_predication = !ros::ok()
                || !msgs_.empty();
            return wait_predication;
        });

        /*程序終結控制*/
        if (!ros::ok())
            break;

        /*獲得最晚的點雲信息*/
        const auto latest_cloud = msgs_.front();
        msgs_.pop();

        /*create SIMD detector*/

        lock.unlock();
    }
}

} // namespace vision
