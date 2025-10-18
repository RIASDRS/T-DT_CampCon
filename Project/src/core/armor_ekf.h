#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cmath>

// 浮点数类型定义
using float64 = double;

// 装甲板分组类型（用于辅助匹配）
enum class ArmorType {
    UNKNOWN,        // 未知类型
    FRONT_BACK,     // 前后组（位于底盘长边）
    LEFT_RIGHT      // 左右组（位于底盘宽边）
};

// 单个装甲板的观测数据结构
struct ArmorObservation {
    Eigen::Vector3d position;  // 相机坐标系下的3D坐标 (x, y, z)，单位：m
    ArmorType type = ArmorType::UNKNOWN;  // 分组类型（可选）
};

// EKF参数配置类
class ArmorEKFParams {
public:
    // 状态维度：8维（x_c, y_c, z_c, vx_c, vy_c, vz_c, omega, alpha）
    static constexpr int STATE_DIM = 8;
    // 最大观测维度（4个装甲板×3坐标）
    static constexpr int MAX_OBSERVE_DIM = 12;

    // 已知固定参数
    const float64 L;       // 底盘长度（m）
    const float64 W;       // 底盘宽度（m）
    const float64 dt;      // 采样时间（s）
    const float64 eps;     // 几何匹配容差（m）

    // 装甲板相对底盘中心的固定向量（底盘坐标系下）
    const Eigen::Vector3d d1;  // 前装甲板：[L/2, 0, 0]
    const Eigen::Vector3d d2;  // 后装甲板：[-L/2, 0, 0]
    const Eigen::Vector3d d3;  // 左装甲板：[0, W/2, 0]
    const Eigen::Vector3d d4;  // 右装甲板：[0, -W/2, 0]

    // 过程噪声协方差矩阵 Q (8x8)
    const Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;

    // 单装甲板观测噪声协方差（3x3）
    const Eigen::Matrix3d R_single;

    // 初始状态估计 (x0)
    const Eigen::Matrix<double, STATE_DIM, 1> x0;

    // 初始协方差矩阵 P0 (8x8)
    const Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0;

    // 构造函数：初始化参数
    ArmorEKFParams(float64 chassis_length,    // 底盘长度（如0.8m）
                   float64 chassis_width,     // 底盘宽度（如0.4m）
                   float64 sample_time);      // 采样时间（如1/30≈0.0333s）

private:
    // 初始化过程噪声协方差 Q
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> initQ() const;

    // 初始化单装甲板观测噪声协方差
    Eigen::Matrix3d initRSingle() const;

    // 初始化初始状态 x0
    Eigen::Matrix<double, STATE_DIM, 1> initX0() const;

    // 初始化初始协方差 P0
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> initP0() const;
};

// 扩展卡尔曼滤波器核心类（支持角速度非匀速）
class ArmorEKF {
private:
    ArmorEKFParams params;                  // 配置参数
    Eigen::Matrix<double, ArmorEKFParams::STATE_DIM, 1> x;  // 状态向量（8维）
    Eigen::Matrix<double, ArmorEKFParams::STATE_DIM, ArmorEKFParams::STATE_DIM> P;  // 协方差矩阵（8x8）
    float64 theta;  // 底盘当前旋转角度（rad，由角速度和角加速度积分得到）

    // 根据装甲板类型获取候选相对向量
    std::vector<Eigen::Vector3d> getCandidates(ArmorType type) const;

    // 为可见装甲板匹配最佳候选向量（最小化预测误差）
    std::vector<Eigen::Vector3d> matchCandidates(const std::vector<ArmorObservation>& observations) const;

public:
    // 构造函数：初始化EKF
    explicit ArmorEKF(const ArmorEKFParams& ekf_params);

    // 预测步骤：更新状态和协方差（支持角加速度驱动的角速度变化）
    void predict();

    // 更新步骤：输入可见装甲板观测（1~4个）
    void update(const std::vector<ArmorObservation>& observations);

    // 获取当前状态估计（8维：x_c, y_c, z_c, vx, vy, vz, omega, alpha）
    Eigen::Matrix<double, ArmorEKFParams::STATE_DIM, 1> getState() const;

    // 获取4个装甲板的位置估计（相机坐标系下）
    std::vector<Eigen::Vector3d> getArmorPositions() const;

    // 调试用：打印当前状态
    void printState() const;
};