#include "armor_ekf.h"
#include <iostream>
#include <algorithm>

// 初始化EKF参数配置类
ArmorEKFParams::ArmorEKFParams(float64 chassis_length,
                               float64 chassis_width,
                               float64 sample_time)
    : L(chassis_length),
      W(chassis_width),
      dt(sample_time),
      eps(0.05), // 5cm容差
      d1(L / 2, 0, 0),
      d2(-L / 2, 0, 0),
      d3(0, W / 2, 0),
      d4(0, -W / 2, 0),
      Q(initQ()),
      R_single(initRSingle()),
      x0(initX0()),
      P0(initP0())
{
}

// 初始化过程噪声协方差 Q（8x8）
Eigen::Matrix<double, ArmorEKFParams::STATE_DIM, ArmorEKFParams::STATE_DIM> ArmorEKFParams::initQ() const
{
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    // 位置噪声（x, y, z）
    Q(0, 0) = 1e-4; // x_c
    Q(1, 1) = 1e-4; // y_c
    Q(2, 2) = 2e-4; // z_c（深度方向略大）
    // 线速度噪声（vx, vy, vz）
    Q(3, 3) = 1e-3; // vx_c
    Q(4, 4) = 1e-3; // vy_c
    Q(5, 5) = 1e-3; // vz_c
    // 角速度噪声（omega）- 非匀速场景增大
    Q(6, 6) = 1e-4;
    // 角加速度噪声（alpha）
    Q(7, 7) = 1e-3; // 允许角加速度有较大波动
    return Q;
}

// 初始化单装甲板观测噪声协方差（3x3）
Eigen::Matrix3d ArmorEKFParams::initRSingle() const
{
    Eigen::Matrix3d R;
    R << 1e-3, 0, 0,
        0, 1e-3, 0,
        0, 0, 2e-3; // Z轴（深度）噪声略大
    return R;
}

// 初始化初始状态 x0（8维）
Eigen::Matrix<double, ArmorEKFParams::STATE_DIM, 1> ArmorEKFParams::initX0() const
{
    Eigen::Matrix<double, STATE_DIM, 1> x0 = Eigen::MatrixXd::Zero(STATE_DIM, 1);
    // x0(0-2)：初始位置（默认0，实际需根据首帧观测调整）
    // x0(3-5)：初始线速度（默认0）
    // x0(6)：初始角速度（默认0）
    // x0(7)：初始角加速度（默认0）
    return x0;
}

// 初始化初始协方差 P0（8x8）
Eigen::Matrix<double, ArmorEKFParams::STATE_DIM, ArmorEKFParams::STATE_DIM> ArmorEKFParams::initP0() const
{
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P0 = Eigen::MatrixXd::Zero(STATE_DIM, STATE_DIM);
    // 位置初始不确定性
    P0(0, 0) = 1e-2; // x_c
    P0(1, 1) = 1e-2; // y_c
    P0(2, 2) = 2e-2; // z_c
    // 线速度初始不确定性（较大）
    P0(3, 3) = 1.0; // vx_c
    P0(4, 4) = 1.0; // vy_c
    P0(5, 5) = 1.0; // vz_c
    // 角速度初始不确定性
    P0(6, 6) = 1.0; // omega
    // 角加速度初始不确定性（更大）
    P0(7, 7) = 2.0; // alpha
    return P0;
}

// EKF核心类实现
ArmorEKF::ArmorEKF(const ArmorEKFParams &ekf_params)
    : params(ekf_params),
      x(ekf_params.x0),
      P(ekf_params.P0),
      theta(0.0) {}

// 根据装甲板类型获取候选相对向量
std::vector<Eigen::Vector3d> ArmorEKF::getCandidates(ArmorType type) const
{
    std::vector<Eigen::Vector3d> candidates;
    if (type == ArmorType::FRONT_BACK || type == ArmorType::UNKNOWN)
    {
        candidates.push_back(params.d1); // 前装甲板
        candidates.push_back(params.d2); // 后装甲板
    }
    if (type == ArmorType::LEFT_RIGHT || type == ArmorType::UNKNOWN)
    {
        candidates.push_back(params.d3); // 左装甲板
        candidates.push_back(params.d4); // 右装甲板
    }
    return candidates;
}

// 为可见装甲板匹配最佳候选向量
std::vector<Eigen::Vector3d> ArmorEKF::matchCandidates(const std::vector<ArmorObservation> &observations) const
{
    std::vector<Eigen::Vector3d> matched_d;
    Eigen::Vector3d center(x(0), x(1), x(2)); // 当前底盘中心估计
    Eigen::Matrix3d R;                        // 当前旋转矩阵
    R << std::cos(theta), -std::sin(theta), 0,
        std::sin(theta), std::cos(theta), 0,
        0, 0, 1;

    for (const auto &obs : observations)
    {
        auto candidates = getCandidates(obs.type);
        float64 min_error = 1e9;
        Eigen::Vector3d best_d;
        for (const auto &d : candidates)
        {
            Eigen::Vector3d pred = center + R * d;        // 预测位置
            float64 error = (pred - obs.position).norm(); // 预测误差
            if (error < min_error)
            {
                min_error = error;
                best_d = d;
            }
        }
        matched_d.push_back(best_d);
    }
    return matched_d;
}

// 预测步骤
void ArmorEKF::predict()
{
    // 1. 更新状态向量（考虑角加速度）
    // 位置 = 位置 + 速度 * dt
    x(0) += x(3) * params.dt; // x_c
    x(1) += x(4) * params.dt; // y_c
    x(2) += x(5) * params.dt; // z_c
    // 角速度 = 角速度 + 角加速度 * dt（新增角加速度影响）
    x(6) += x(7) * params.dt; // omega
    // 角加速度暂视为常值（扰动由Q体现）

    // 2. 更新旋转角度 theta（匀变速旋转公式）
    theta += x(6) * params.dt + 0.5 * x(7) * params.dt * params.dt;

    // 3. 构造状态转移矩阵 F（8x8）
    Eigen::Matrix<double, ArmorEKFParams::STATE_DIM, ArmorEKFParams::STATE_DIM> F =
        Eigen::MatrixXd::Identity(ArmorEKFParams::STATE_DIM, ArmorEKFParams::STATE_DIM);
    // 位置对速度的偏导
    F(0, 3) = params.dt; // x_c 对 vx_c
    F(1, 4) = params.dt; // y_c 对 vy_c
    F(2, 5) = params.dt; // z_c 对 vz_c
    // 角速度对加速度的偏导（新增）
    F(6, 7) = params.dt; // omega 对 alpha

    // 4. 协方差预测：P = F * P * F^T + Q
    P = F * P * F.transpose() + params.Q;
}

// 更新步骤
void ArmorEKF::update(const std::vector<ArmorObservation> &observations)
{
    if (observations.empty())
        return; // 无观测则不更新

    int m = observations.size(); // 可见装甲板数量
    int obs_dim = 3 * m;         // 观测维度

    // 1. 匹配每个观测装甲板的相对向量
    auto matched_d = matchCandidates(observations);

    // 2. 计算旋转矩阵 R(theta)
    float64 cos_theta = std::cos(theta);
    float64 sin_theta = std::sin(theta);
    Eigen::Matrix3d R;
    R << cos_theta, -sin_theta, 0,
        sin_theta, cos_theta, 0,
        0, 0, 1;

    // 3. 构建预测观测值 h(x) 和观测矩阵 H
    Eigen::VectorXd h(obs_dim);                            // 预测观测（3m x 1）
    Eigen::MatrixXd H(obs_dim, ArmorEKFParams::STATE_DIM); // 观测矩阵（3m x 8）
    H.setZero();

    Eigen::Vector3d center(x(0), x(1), x(2)); // 底盘中心位置

    for (int i = 0; i < m; ++i)
    {
        const auto &d = matched_d[i]; // 匹配的相对向量
        const auto &obs_pos = observations[i].position;

        // 预测观测值（第i个装甲板的x,y,z）
        Eigen::Vector3d pred = center + R * d;
        h.segment<3>(3 * i) = pred;

        // 构建观测矩阵 H 的第i组（3行）
        // 位置对底盘中心坐标的偏导（单位矩阵）
        H.block<3, 3>(3 * i, 0) = Eigen::Matrix3d::Identity();

        // 位置对 omega（索引6）和 alpha（索引7）的偏导
        Eigen::Matrix3d dR_dt; // R对theta的导数
        dR_dt << -sin_theta, -cos_theta, 0,
            cos_theta, -sin_theta, 0,
            0, 0, 0;
        Eigen::Vector3d deriv_omega = dR_dt * d * params.dt;                   // 对omega的偏导
        Eigen::Vector3d deriv_alpha = dR_dt * d * params.dt * params.dt / 2.0; // 对alpha的偏导

        H.block<3, 1>(3 * i, 6) = deriv_omega; // 第6列（omega）
        H.block<3, 1>(3 * i, 7) = deriv_alpha; // 第7列（alpha，新增）
    }

    // 4. 构建观测噪声协方差 R（3m x 3m）
    Eigen::MatrixXd R_obs(obs_dim, obs_dim);
    R_obs.setZero();
    for (int i = 0; i < m; ++i)
    {
        R_obs.block<3, 3>(3 * i, 3 * i) = params.R_single;
    }

    // 5. 计算观测残差 y = z - h
    Eigen::VectorXd z(obs_dim);
    for (int i = 0; i < m; ++i)
    {
        z.segment<3>(3 * i) = observations[i].position;
    }
    Eigen::VectorXd y = z - h;

    // 6. 计算卡尔曼增益 K = P * H^T * (H*P*H^T + R)^-1
    Eigen::MatrixXd S = H * P * H.transpose() + R_obs;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();

    // 7. 更新状态估计和协方差
    x += K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(ArmorEKFParams::STATE_DIM, ArmorEKFParams::STATE_DIM);
    P = (I - K * H) * P;
}

// 获取当前状态估计
Eigen::Matrix<double, ArmorEKFParams::STATE_DIM, 1> ArmorEKF::getState() const
{
    return x;
}

// 获取4个装甲板的位置估计
std::vector<Eigen::Vector3d> ArmorEKF::getArmorPositions() const
{
    std::vector<Eigen::Vector3d> armors(4);
    Eigen::Vector3d center(x(0), x(1), x(2));
    Eigen::Matrix3d R;
    R << std::cos(theta), -std::sin(theta), 0,
        std::sin(theta), std::cos(theta), 0,
        0, 0, 1;

    armors[0] = center + R * params.d1; // 前装甲板
    armors[1] = center + R * params.d2; // 后装甲板
    armors[2] = center + R * params.d3; // 左装甲板
    armors[3] = center + R * params.d4; // 右装甲板
    return armors;
}

// 打印当前状态（调试用）
void ArmorEKF::printState() const
{
    std::cout << "当前状态估计：" << std::endl;
    std::cout << "底盘中心位置：(x=" << x(0) << ", y=" << x(1) << ", z=" << x(2) << ") m" << std::endl;
    std::cout << "线速度：(vx=" << x(3) << ", vy=" << x(4) << ", vz=" << x(5) << ") m/s" << std::endl;
    std::cout << "角速度：" << x(6) << " rad/s" << std::endl;
    std::cout << "角加速度：" << x(7) << " rad/s²" << std::endl;
    std::cout << "旋转角度：" << theta << " rad" << std::endl;
}