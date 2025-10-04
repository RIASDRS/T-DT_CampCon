#include "ImageUtils.h"
#include <iostream>
#include <filesystem>

namespace armor_detection {

cv::Mat ImageUtils::resizeImage(const cv::Mat& image, int width, int height) {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(width, height));
    return resized;
}

cv::Mat ImageUtils::enhanceContrast(const cv::Mat& image) {
    cv::Mat enhanced;
    
    if (image.channels() == 3) {
        cv::cvtColor(image, enhanced, cv::COLOR_BGR2GRAY);
    } else {
        image.copyTo(enhanced);
    }
    
    // 直方图均衡化
    cv::equalizeHist(enhanced, enhanced);
    
    return enhanced;
}

cv::Mat ImageUtils::extractROI(const cv::Mat& image, const cv::Rect& roi) {
    cv::Rect valid_roi = roi & cv::Rect(0, 0, image.cols, image.rows);
    if (valid_roi.width > 0 && valid_roi.height > 0) {
        return image(valid_roi).clone();
    }
    return cv::Mat();
}

void ImageUtils::drawRotatedRect(cv::Mat& image, const cv::RotatedRect& rect, 
                                const cv::Scalar& color, int thickness) {
    cv::Point2f vertices[4];
    rect.points(vertices);
    
    for (int i = 0; i < 4; i++) {
        cv::line(image, vertices[i], vertices[(i+1)%4], color, thickness);
    }
    
    // 绘制中心点
    cv::circle(image, rect.center, 3, color, -1);
}

void ImageUtils::drawPoints(cv::Mat& image, const std::vector<cv::Point2f>& points,
                           const cv::Scalar& color, int radius) {
    for (const auto& point : points) {
        cv::circle(image, point, radius, color, -1);
    }
}

bool ImageUtils::saveImage(const cv::Mat& image, const std::string& filename) {
    // 创建目录（如果不存在）
    std::filesystem::path path(filename);
    std::filesystem::create_directories(path.parent_path());
    
    bool success = cv::imwrite(filename, image);
    if (success) {
        std::cout << "✅ 图像保存成功: " << filename << std::endl;
    } else {
        std::cerr << "❌ 图像保存失败: " << filename << std::endl;
    }
    
    return success;
}

cv::Mat ImageUtils::createDebugImage(const cv::Mat& original, const cv::Mat& processed) {
    cv::Mat debug_image;
    
    if (original.channels() == 3 && processed.channels() == 1) {
        // 将处理后的单通道图像转换为3通道用于显示
        cv::Mat processed_color;
        cv::cvtColor(processed, processed_color, cv::COLOR_GRAY2BGR);
        
        // 水平拼接
        cv::hconcat(original, processed_color, debug_image);
    } else {
        debug_image = original.clone();
    }
    
    return debug_image;
}

} // namespace armor_detection