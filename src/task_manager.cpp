#include <async_algorithm_pipeline/task_manager.h>

namespace task_manager {

template <typename TaskIn, typename TaskOut, typename Worker>
Work<TaskIn, TaskOut, Worker>::Work()
{
}

template <typename TaskIn, typename TaskOut, typename Worker>
Work<TaskIn, TaskOut, Worker>::~Work()
{
}

template <typename Work, typename TaskOut>
Manager<Work, TaskOut>::Manager(size_t num_workers)
{
    for (size_t i = 0; i < num_workers; i++) {
        workers_.emplace_back(std::thread(&Manager<Work, TaskOut>::work, this));
    }
}

template <typename Work, typename TaskOut>
Manager<Work, TaskOut>::~Manager()
{
    {
        std::unique_lock<std::mutex> lock(mt_);
        stop = true;
    }
    cv_.notify_all();

    for (auto& worker : workers_) {
        if (worker.joinable())
            worker.join();
    }
}

template <typename Work, typename TaskOut>
void Manager<Work, TaskOut>::setCallback(std::function<void(TaskOut)> _callback_f)
{
    std::unique_lock<std::mutex> lock(mt_);
    callback_ = _callback_f;
}

template <typename Work, typename TaskOut>
void Manager<Work, TaskOut>::enqueue(Work _work)
{
    std::unique_lock<std::mutex> lock(mt_);
    works.emplace(std::move(_work.task));
}

template <typename Work, typename TaskOut>
void Manager<Work, TaskOut>::work()
{
    while (true) {
        /*Get Task*/
        Work work;
        {
            std::unique_lock<std::mutex> lock(mt_);
            cv_.wait(lock, [this]() {
                bool wait_predication = stop || !works.empty();
                return wait_predication;
            });

            if (stop)
                break;

            work = std::move(works.front());
            works.pop();
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
