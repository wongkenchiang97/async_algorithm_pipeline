#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace task_manager {

template <typename TaskIn, typename TaskOut>
class Worker;

template <typename TaskIn, typename TaskOut, typename WorkerT>
class Work {
public:
    using Ptr = std::shared_ptr<Work<TaskIn, TaskOut, WorkerT>>;
    using Output = std::shared_future<TaskOut>;
    using Task = std::packaged_task<TaskOut()>;
    using Worker = std::shared_ptr<WorkerT>;

    Work(const TaskIn _task_in, Worker _worker);
    virtual ~Work();
    static typename Work<TaskIn, TaskOut, WorkerT>::Ptr create(const TaskIn _task_in, Worker _worker);

    Worker worker;
    Task task;
    Output output;
};

template <typename TaskIn, typename TaskOut>
class Worker {
public:
    using Ptr = std::shared_ptr<Worker>;

    Worker(/* args */);
    virtual ~Worker();
    virtual TaskOut work(TaskIn _task_in) = 0;

private:
    virtual Worker<TaskIn, TaskOut>* clone_impl() const = 0;
};

template <typename TaskIn, typename TaskOut, typename WorkerT>
class Manager {
public:
    using Ptr = std::shared_ptr<Manager<TaskIn, TaskOut, WorkerT>>;
    using Worker = std::shared_ptr<WorkerT>;
    Manager(Worker worker_prototype, size_t num_workers = std::thread::hardware_concurrency());
    virtual ~Manager();
    static typename Manager<TaskIn, TaskOut, WorkerT>::Ptr create(Worker worker_prototype, size_t num_workers = std::thread::hardware_concurrency());
    void enqueue(TaskIn _task_in);
    void setCallback(std::function<void(TaskOut)> _callback_f);

protected:
    std::atomic_bool stop;

private:
    void manage();

    std::mutex mt_;
    std::condition_variable cv_;
    std::vector<std::thread> threads_;
    Worker worker_prototype_;
    std::queue<typename Work<TaskIn, TaskOut, WorkerT>::Ptr> works_;
    std::function<void(TaskOut)> callback_;
};

} // namespace task_manager

#include <async_algorithm_pipeline/task_manager_impl.h>

#endif