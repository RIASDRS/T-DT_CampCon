#ifndef UNIFIEDROTATEDRECT_H
#define UNIFIEDROTATEDRECT_H

#include <opencv2/opencv.hpp>

namespace armor_detection {

// 对已有的RotatedRect进行统一化处理
cv::RotatedRect unifyRotatedRect(const cv::RotatedRect &rect);

} // namespace armor_detection

#endif