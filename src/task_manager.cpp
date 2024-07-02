#include <async_algorithm_pipeline/task_manager.h>

namespace task_manager {

template <typename TaskIn, typename TaskOut>
Worker<TaskIn, TaskOut>::Worker(/* args */)
{
}

template <typename TaskIn, typename TaskOut>
Worker<TaskIn, TaskOut>::~Worker()
{
}

template <typename TaskIn, typename TaskOut>
Manager<TaskIn, TaskOut>::Manager(size_t num_workers)
{
    for (size_t i = 0; i < num_workers; i++) {
        threads_.emplace_back(std::thread(&Manager<TaskIn, TaskOut>::AssignWork, this));
    }
}

template <typename TaskIn, typename TaskOut>
Manager<TaskIn, TaskOut>::~Manager()
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

template <typename TaskIn, typename TaskOut>
void Manager<TaskIn, TaskOut>::setCallback(std::function<void(TaskOut)> _callback_f)
{
    std::unique_lock<std::mutex> lock(mt_);
    callback_ = _callback_f;
}

template <typename TaskIn, typename TaskOut>
void Manager<TaskIn, TaskOut>::enqueue(typename Work<TaskIn, TaskOut>::Ptr _work)
{
    std::unique_lock<std::mutex> lock(mt_);
    works_.emplace(std::move(_work.task));
    cv_.notify_one();
}

template <typename TaskIn, typename TaskOut>
void Manager<TaskIn, TaskOut>::AssignWork()
{
    while (true) {
        /*Get Task*/
        typename Work<TaskIn, TaskOut>::Ptr work;
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
        work.task();
        if (callback_ == nullptr)
            continue;

        /*Execute Callback*/
        callback_(work.output.get());
    }
}

} // namespace task_manager
