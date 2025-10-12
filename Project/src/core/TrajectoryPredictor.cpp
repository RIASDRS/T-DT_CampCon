#include "TrajectoryPredictor.h"
#include <iostream>
#include <cmath>
#include <stdexcept>

// TrajectoryPredictor 实现
TrajectoryPredictor::TrajectoryPredictor(int degree) 
    : polynomial_degree(degree) {
}

void TrajectoryPredictor::addPoint(float x, float y, float z, int size) {
    points.push_back(cv::Point3f(x, y, z));
    if (points.size()> size ){
        points.erase(points.begin());
    }

}


void TrajectoryPredictor::addPoint(float x, float y, float z) {
    points.push_back(cv::Point3f(x, y, z));
    
}

void TrajectoryPredictor::addPoint(const cv::Point3f& point) {
    points.push_back(point);
    
}

size_t TrajectoryPredictor::getPointCount() const {
    return points.size();
}

cv::Point3f TrajectoryPredictor::getPoint(size_t index) const {
    if (index >= points.size()) {
        throw std::out_of_range("Index out of range");
    }
    return points[index];
}

void TrajectoryPredictor::clearPoints() {
    points.clear();
}

void TrajectoryPredictor::setPolynomialDegree(int degree) {
    polynomial_degree = degree;
}

int TrajectoryPredictor::getPolynomialDegree() const {
    return polynomial_degree;
}

void TrajectoryPredictor::fit() {
    if (points.size() < polynomial_degree + 1) {
        std::cerr << "需要至少 " << polynomial_degree + 1 << " 个点来进行拟合" << std::endl;
        return;
    }
    
    std::cout << "使用 " << points.size() << " 个点进行轨迹拟合，多项式阶数: " << polynomial_degree << std::endl;
}

std::vector<cv::Point3f> TrajectoryPredictor::predict(int num_points) {
    if (points.size() < 2) {
        std::cerr << "需要至少2个点来进行预测" << std::endl;
        return {};
    }
    
    std::vector<cv::Point3f> predictions;
    
    // 分别对x, y, z坐标进行多项式拟合和预测
    for (int coord = 0; coord < 3; coord++) {
        std::vector<double> time_series;
        std::vector<double> values;
        
        // 创建时间序列和对应的坐标值
        for (size_t i = 0; i < points.size(); i++) {
            time_series.push_back(static_cast<double>(i));
            float coord_value;
            switch (coord) {
                case 0: coord_value = points[i].x; break;
                case 1: coord_value = points[i].y; break;
                case 2: coord_value = points[i].z; break;
            }
            values.push_back(coord_value);
        }
        
        // 使用多项式拟合
        auto coeffs = polynomialFit(time_series, values, polynomial_degree);
        
        // 预测未来点
        for (int i = 1; i <= num_points; i++) {
            double t = time_series.back() + i;
            double predicted_value = polynomialValue(coeffs, t);
            
            if (coord == 0) {
                predictions.push_back(cv::Point3f(0, 0, 0));
            }
            
            switch (coord) {
                case 0: predictions[i-1].x = predicted_value; break;
                case 1: predictions[i-1].y = predicted_value; break;
                case 2: predictions[i-1].z = predicted_value; break;
            }
        }
    }
    
    return predictions;
}

double TrajectoryPredictor::evaluateFit() const {
    if (points.size() < polynomial_degree + 1) {
        return 0.0;
    }
    
    double total_variance = 0.0;
    double residual_sum_squares = 0.0;
    
    // 分别计算每个坐标的R平方
    for (int coord = 0; coord < 3; coord++) {
        std::vector<double> time_series;
        std::vector<double> values;
        
        for (size_t i = 0; i < points.size(); i++) {
            time_series.push_back(static_cast<double>(i));
            float coord_value;
            switch (coord) {
                case 0: coord_value = points[i].x; break;
                case 1: coord_value = points[i].y; break;
                case 2: coord_value = points[i].z; break;
            }
            values.push_back(coord_value);
        }
        
        auto coeffs = polynomialFit(time_series, values, polynomial_degree);
        
        // 计算均值
        double mean = 0.0;
        for (double val : values) {
            mean += val;
        }
        mean /= values.size();
        
        // 计算总平方和和残差平方和
        double tss = 0.0; // Total Sum of Squares
        double rss = 0.0; // Residual Sum of Squares
        
        for (size_t i = 0; i < values.size(); i++) {
            double predicted = polynomialValue(coeffs, time_series[i]);
            tss += (values[i] - mean) * (values[i] - mean);
            rss += (values[i] - predicted) * (values[i] - predicted);
        }
        
        total_variance += tss;
        residual_sum_squares += rss;
    }
    
    if (total_variance == 0.0) return 1.0;
    
    return 1.0 - (residual_sum_squares / total_variance);
}

std::vector<double> TrajectoryPredictor::polynomialFit(const std::vector<double>& x, 
                                     const std::vector<double>& y, 
                                     int degree) const {
    int n = x.size();
    cv::Mat A(n, degree + 1, CV_64F);
    cv::Mat b(n, 1, CV_64F);
    
    // 构建矩阵A和向量b
    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= degree; j++) {
            A.at<double>(i, j) = std::pow(x[i], j);
        }
        b.at<double>(i, 0) = y[i];
    }
    
    // 使用OpenCV的SVD求解最小二乘问题
    cv::Mat coeffs;
    cv::solve(A, b, coeffs, cv::DECOMP_SVD);
    
    std::vector<double> result(degree + 1);
    for (int i = 0; i <= degree; i++) {
        result[i] = coeffs.at<double>(i, 0);
    }
    
    return result;
}

double TrajectoryPredictor::polynomialValue(const std::vector<double>& coeffs, double x) const {
    double result = 0.0;
    for (size_t i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * std::pow(x, i);
    }
    return result;
}

// LinearPredictor 实现
void LinearPredictor::addPoint(float x, float y, float z) {
    points.push_back(cv::Point3f(x, y, z));
}

void LinearPredictor::addPoint(const cv::Point3f& point) {
    points.push_back(point);
}

size_t LinearPredictor::getPointCount() const {
    return points.size();
}

cv::Point3f LinearPredictor::getPoint(size_t index) const {
    if (index >= points.size()) {
        throw std::out_of_range("Index out of range");
    }
    return points[index];
}

void LinearPredictor::clearPoints() {
    points.clear();
}

std::vector<cv::Point3f> LinearPredictor::predict(int num_points) {
    if (points.size() < 2) {
        std::cerr << "需要至少2个点来进行线性预测" << std::endl;
        return {};
    }
    
    std::vector<cv::Point3f> predictions;
    
    // 计算最后两个点的向量作为方向
    cv::Point3f last_point = points.back();
    cv::Point3f second_last_point = points[points.size() - 2];
    cv::Point3f direction = last_point - second_last_point;
    
    // 基于方向预测后续点
    for (int i = 1; i <= num_points; i++) {
        cv::Point3f new_point = last_point + direction * i;
        predictions.push_back(new_point);
    }
    
    return predictions;
}

cv::Point3f LinearPredictor::getDirection() const {
    if (points.size() < 2) {
        return cv::Point3f(0, 0, 0);
    }
    
    cv::Point3f last_point = points.back();
    cv::Point3f second_last_point = points[points.size() - 2];
    return last_point - second_last_point;
}