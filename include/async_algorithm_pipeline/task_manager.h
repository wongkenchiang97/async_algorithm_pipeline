#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace task_manager {

template <typename TaskIn, typename TaskOut, typename Worker>
class Work {
public:
    using Ptr = std::shared_ptr<Work>;
    using Output = std::shared_future<TaskOut>;
    using Task = std::packaged_task<TaskOut>;
    Work();
    virtual ~Work();
    virtual void assignTask() = 0;

    TaskIn input;
    Output output;
    Task task;
    Worker worker;
};

template <typename Work, typename TaskOut>
class Manager {
public:
    Manager(size_t num_workers = std::thread::hardware_concurrency());
    virtual ~Manager();
    void enqueue(Work _work);
    void setCallback(std::function<void(TaskOut)> _callback_f);

protected:
    std::atomic_bool stop;

private:
    void work();

    std::mutex mt_;
    std::condition_variable cv_;
    std::vector<std::thread> workers_;
    std::queue<Work> works;
    std::function<void(TaskOut)> callback_;
};

} // namespace task_manager

#endif