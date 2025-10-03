#include "UnifiedRotatedRect.h"
#include <algorithm>

namespace armor_detection {

cv::RotatedRect unifyRotatedRect(const cv::RotatedRect &rect)
{
    cv::Point2f center = rect.center;
    cv::Size2f size = rect.size;
    float angle = rect.angle;

    // 确保高度大于宽度
    if (size.width > size.height)
    {
        std::swap(size.width, size.height);
        angle = 90 - angle;
    }

    return cv::RotatedRect(center, size, angle);
}

} // namespace armor_detection