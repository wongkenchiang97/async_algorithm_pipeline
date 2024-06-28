#ifndef ASYNC_DETECTOR_H
#define ASYNC_DETECTOR_H

#include <memory>

namespace vision {

class AsyncDetector : public std::enable_shared_from_this<AsyncDetector> {
public:
    using Ptr = std::shared_ptr<AsyncDetector>;
    AsyncDetector(/* args */);
    virtual ~AsyncDetector();

    AsyncDetector::Ptr next, prev;

protected:
    bool insertFront(AsyncDetector::Ptr);
    bool insertBack(AsyncDetector::Ptr);
    bool erase(AsyncDetector::Ptr);

private:
};

} // namespace vision

#endif