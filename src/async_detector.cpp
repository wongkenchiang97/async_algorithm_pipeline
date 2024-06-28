#include <async_algorithm_pipeline/async_detector.h>

namespace vision {

AsyncDetector::AsyncDetector(/* args */)
{
}

AsyncDetector::~AsyncDetector()
{
}

bool AsyncDetector::insertFront(AsyncDetector::Ptr)
{
    return true;
}

bool AsyncDetector::insertBack(AsyncDetector::Ptr)
{
    return true;
}

bool AsyncDetector::erase(AsyncDetector::Ptr)
{
    return true;
}

} // namespace vision
