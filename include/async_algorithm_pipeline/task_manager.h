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

template <typename TaskIn, typename TaskOut>
class Worker;

template <typename TaskIn, typename TaskOut>
class Work {
public:
    using Ptr = std::shared_ptr<Work<TaskIn, TaskOut>>;
    using Output = std::shared_future<TaskOut>;
    using Task = std::packaged_task<TaskOut()>;

    Output output;
    Task task;
    typename Worker<TaskIn, TaskOut>::Ptr worker;
};

template <typename TaskIn, typename TaskOut>
class Worker {
public:
    using Ptr = std::shared_ptr<Worker>;

    Worker(/* args */);
    virtual ~Worker();
    virtual typename Worker<TaskIn, TaskOut>::Ptr clone() const = 0;
    virtual void setInput(const TaskIn) = 0;
    virtual typename Work<TaskIn, TaskOut>::Task assignTask(TaskIn) = 0;
};

template <typename TaskIn, typename TaskOut>
class Manager {
public:
    Manager(size_t num_workers = std::thread::hardware_concurrency());
    virtual ~Manager();
    void enqueue(typename Work<TaskIn, TaskOut>::Ptr _work);
    void setCallback(std::function<void(TaskOut)> _callback_f);

protected:
    std::atomic_bool stop;

private:
    void AssignWork();

    std::mutex mt_;
    std::condition_variable cv_;
    std::vector<std::thread> threads_;
    std::queue<typename Work<TaskIn, TaskOut>::Ptr> works_;
    std::function<void(TaskOut)> callback_;
};

} // namespace task_manager

#endif