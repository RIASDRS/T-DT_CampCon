#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include "common.h"

namespace armor_detection {

class PnPSolver {
public:
    PnPSolver();
    
    // 设置相机参数
    void setCameraParams(const CameraParams& params);
    
    // 解算装甲板位姿
    PoseResult solveArmorPose(const std::vector<cv::Point2f>& points2d, 
                             const ArmorModel& model = ArmorModel());
    
    // 解算灯条位姿
    PoseResult solveLightBarPose(const cv::RotatedRect& light_bar);
    
    // 重投影误差计算
    double calculateReprojectionError(const std::vector<cv::Point2f>& points2d,
                                     const std::vector<Point3D>& points3d,
                                     const PoseResult& pose);
    
    // 检查相机参数是否加载
    bool isParamsLoaded() const { return params_loaded_; }

private:
    CameraParams camera_params_;
    bool params_loaded_ = false;
    
    // 生成装甲板3D模型点
    std::vector<Point3D> generateArmorModelPoints(const ArmorModel& model);
    
    // 生成灯条3D模型点
    std::vector<Point3D> generateLightBarModelPoints();
    
    // 坐标转换
    cv::Point2f reprojectPoint(const Point3D& point3d, const PoseResult& pose);
};

} // namespace armor_detection

#endif