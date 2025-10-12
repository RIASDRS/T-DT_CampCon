#ifndef TRAJECTORY_PREDICTOR_H
#define TRAJECTORY_PREDICTOR_H

#include <vector>
#include <opencv2/opencv.hpp>

/**
 * @brief 基于多项式拟合的三维轨迹预测器
 * 
 * 该类使用多项式回归来拟合三维点序列，并预测未来的轨迹点。
 * 支持可配置的多项式阶数，适用于各种复杂度的轨迹模式。
 */
class TrajectoryPredictor {
private:
    std::vector<cv::Point3f> points;        ///< 存储已知的三维点序列
    int polynomial_degree;                  ///< 多项式拟合的阶数
    
public:
    /**
     * @brief 构造函数
     * @param degree 多项式阶数，默认为3
     */
    explicit TrajectoryPredictor(int degree = 3);
    
    /**
     * @brief 析构函数
     */
    virtual ~TrajectoryPredictor() = default;
    
    /**
     * @brief 添加三维点坐标
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     */
    void addPoint(float x, float y, float z);
    
    /**
     * @brief 添加三维点坐标
     * @param point cv::Point3f类型的点
     */
    void addPoint(const cv::Point3f& point);
    
    /**
     * @brief 添加三维点坐标
     * @param point cv::Point3d类型的点
     */
    void addPoint(float x, float y, float z, int size);


    size_t getPointCount() const;
    
    /**
     * @brief 获取指定索引的点
     * @param index 点索引
     * @return 三维点坐标
     * @throws std::out_of_range 如果索引超出范围
     */
    cv::Point3f getPoint(size_t index) const;
    
    /**
     * @brief 清除所有存储的点
     */
    void clearPoints();
    
    /**
     * @brief 设置多项式阶数
     * @param degree 新的多项式阶数
     */
    void setPolynomialDegree(int degree);
    
    /**
     * @brief 获取当前多项式阶数
     * @return 多项式阶数
     */
    int getPolynomialDegree() const;
    
    /**
     * @brief 拟合轨迹模型
     * 
     * 使用当前存储的点序列进行多项式拟合。
     * 如果点数不足，会输出错误信息。
     */
    void fit();
    
    /**
     * @brief 预测后续轨迹点
     * @param num_points 要预测的点数量
     * @return 预测的三维点序列，如果预测失败返回空向量
     */
    std::vector<cv::Point3f> predict(int num_points);
    
    /**
     * @brief 评估拟合质量（计算R平方值）
     * @return R平方值，越接近1表示拟合越好
     */
    double evaluateFit() const;
    
private:
    /**
     * @brief 多项式拟合函数
     * @param x 自变量序列
     * @param y 因变量序列
     * @param degree 多项式阶数
     * @return 多项式系数向量（从常数项开始）
     */
    std::vector<double> polynomialFit(const std::vector<double>& x, 
                                     const std::vector<double>& y, 
                                     int degree) const;
    
    /**
     * @brief 计算多项式值
     * @param coeffs 多项式系数
     * @param x 自变量值
     * @return 多项式函数值
     */
    double polynomialValue(const std::vector<double>& coeffs, double x) const;
};

/**
 * @brief 线性轨迹预测器
 * 
 * 基于最后两个点的方向进行简单的线性外推预测。
 * 适用于直线运动或变化缓慢的轨迹。
 */
class LinearPredictor {
private:
    std::vector<cv::Point3f> points;        ///< 存储已知的三维点序列
    
public:
    /**
     * @brief 默认构造函数
     */
    LinearPredictor() = default;
    
    /**
     * @brief 析构函数
     */
    virtual ~LinearPredictor() = default;
    
    

    /**
     * @brief 添加三维点坐标
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     */
    void addPoint(float x, float y, float z);
    
    /**
     * @brief 添加三维点坐标
     * @param point cv::Point3f类型的点
     */
    void addPoint(const cv::Point3f& point);
    
    /**
     * @brief 获取当前存储的点数量
     * @return 点的数量
     */
    size_t getPointCount() const;
    
    /**
     * @brief 获取指定索引的点
     * @param index 点索引
     * @return 三维点坐标
     * @throws std::out_of_range 如果索引超出范围
     */
    cv::Point3f getPoint(size_t index) const;
    
    /**
     * @brief 清除所有存储的点
     */
    void clearPoints();
    
    /**
     * @brief 预测后续轨迹点
     * @param num_points 要预测的点数量
     * @return 预测的三维点序列，如果预测失败返回空向量
     */
    std::vector<cv::Point3f> predict(int num_points);
    
    /**
     * @brief 计算最后两个点确定的方向向量
     * @return 方向向量，如果点数不足返回零向量
     */
    cv::Point3f getDirection() const;
};

#endif // TRAJECTORY_PREDICTOR_H