#ifndef IMAGEUTILS_H
#define IMAGEUTILS_H

#include <opencv2/opencv.hpp>
#include <string>

namespace armor_detection {

class ImageUtils {
public:
    // 调整图像大小
    static cv::Mat resizeImage(const cv::Mat& image, int width, int height);
    
    // 增强图像对比度
    static cv::Mat enhanceContrast(const cv::Mat& image);
    
    // 提取ROI区域
    static cv::Mat extractROI(const cv::Mat& image, const cv::Rect& roi);
    
    // 绘制旋转矩形
    static void drawRotatedRect(cv::Mat& image, const cv::RotatedRect& rect, 
                               const cv::Scalar& color, int thickness = 2);
    
    // 绘制特征点
    static void drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points,
                          const cv::Scalar& color, int radius = 3);
    
    // 保存图像
    static bool saveImage(const cv::Mat& image, const std::string& filename);
    
    // 创建调试图像
    static cv::Mat createDebugImage(const cv::Mat& original, const cv::Mat& processed);
};

} // namespace armor_detection

#endif