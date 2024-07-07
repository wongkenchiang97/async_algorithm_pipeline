#ifndef TASK_MANAGER_IMPL_H
#define TASK_MANAGER_IMPL_H

#include <async_algorithm_pipeline/task_manager.h>

namespace task_manager {

template <typename TaskIn, typename TaskOut, typename WorkerT>
Work<TaskIn, TaskOut, WorkerT>::Work(const TaskIn _task_in, Worker _worker)
    : worker(_worker)
    , task(std::bind(&WorkerT::work, worker, _task_in))
    , output(task.get_future().share())
{
}

template <typename TaskIn, typename TaskOut, typename WorkerT>
Work<TaskIn, TaskOut, WorkerT>::~Work()
{
}

template <typename TaskIn, typename TaskOut, typename WorkerT>
typename Work<TaskIn, TaskOut, WorkerT>::Ptr Work<TaskIn, TaskOut, WorkerT>::create(const TaskIn _task_in, Worker _worker)
{
    typename Work<TaskIn, TaskOut, WorkerT>::Ptr obj_ptr;
    obj_ptr = std::make_shared<Work<TaskIn, TaskOut, WorkerT>>(_task_in, _worker);
    return obj_ptr;
}

template <typename TaskIn, typename TaskOut>
Worker<TaskIn, TaskOut>::Worker(/* args */)
{
}

template <typename TaskIn, typename TaskOut>
Worker<TaskIn, TaskOut>::~Worker()
{
}

template <typename TaskIn, typename TaskOut, typename WorkerT>
Manager<TaskIn, TaskOut, WorkerT>::Manager(Worker worker_prototype, size_t num_workers)
    : worker_prototype_(worker_prototype)
    , stop(false)
{
    for (size_t i = 0; i < num_workers; i++) {
        threads_.emplace_back(std::thread(&Manager<TaskIn, TaskOut, WorkerT>::manage, this));
    }
}

template <typename TaskIn, typename TaskOut, typename WorkerT>
Manager<TaskIn, TaskOut, WorkerT>::~Manager()
{
    {
        std::unique_lock<std::mutex> lock(mt_);
        stop = true;
    }
    cv_.notify_all();

    for (auto& thread : threads_) {
        if (thread.joinable())
            thread.join();
    }
}

template <typename TaskIn, typename TaskOut, typename WorkerT>
typename Manager<TaskIn, TaskOut, WorkerT>::Ptr Manager<TaskIn, TaskOut, WorkerT>::create(Worker worker_prototype, size_t num_workers)
{
    typename Manager<TaskIn, TaskOut, WorkerT>::Ptr obj_ptr;
    obj_ptr = std::make_shared<Manager<TaskIn, TaskOut, WorkerT>>(worker_prototype, num_workers);
    return obj_ptr;
}

template <typename TaskIn, typename TaskOut, typename WorkerT>
void Manager<TaskIn, TaskOut, WorkerT>::setCallback(std::function<void(TaskOut)> _callback_f)
{
    std::unique_lock<std::mutex> lock(mt_);
    callback_ = _callback_f;
}

template <typename TaskIn, typename TaskOut, typename WorkerT>
void Manager<TaskIn, TaskOut, WorkerT>::enqueue(TaskIn _task_in)
{
    std::unique_lock<std::mutex> lock(mt_);
    auto work = Work<TaskIn, TaskOut, WorkerT>::create(_task_in, worker_prototype_->clone());
    works_.emplace(std::move(work));
    cv_.notify_one();
}

template <typename TaskIn, typename TaskOut, typename WorkerT>
void Manager<TaskIn, TaskOut, WorkerT>::manage()
{
    while (true) {
        /*Get Task*/
        typename Work<TaskIn, TaskOut, WorkerT>::Ptr work;
        {
            std::unique_lock<std::mutex> lock(mt_);
            cv_.wait(lock, [this]() {
                bool wait_predication = stop || !works_.empty();
                return wait_predication;
            });

            if (stop)
                break;

            work = std::move(works_.front());
            works_.pop();
        }

        /*Execute Task*/
        work->task();
        if (callback_ == nullptr)
            continue;

        /*Execute Callback*/
        callback_(work->output.get());
    }
}

} // namespace task_manager

#endif