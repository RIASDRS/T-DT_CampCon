#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <vector>
#include <string>
#include <iostream>

using namespace cv;
using namespace cv::ml;
using namespace std;

int main() {
    vector<Mat> train_features; // 改为更明确的名称
    vector<int> train_labels;

    // 先检查HOG特征维度
    size_t feature_size = 0;
    bool first_feature = true;

    // 只循环1-5
    for (int digit = 1; digit <= 5; digit++) {
        vector<String> filenames;
        string pattern = "/home/hz/T-DT_CampCon/data/train/" + to_string(digit) + "/*.png";
        glob(pattern, filenames);

        cout << "处理数字 " << digit << ", 找到 " << filenames.size() << " 个文件" << endl;

        for (const auto& filename : filenames) {
            Mat img = imread(filename, IMREAD_GRAYSCALE);
            if (img.empty()) {
                cerr << "无法读取图片: " << filename << endl;
                continue;
            }

            // 预处理：统一尺寸和二值化
            Mat img_processed;
            resize(img, img_processed, Size(32, 32));
            threshold(img_processed, img_processed, 128, 255, THRESH_BINARY | THRESH_OTSU);

            // 使用HOG特征提取
            Ptr<HOGDescriptor> hog = new HOGDescriptor(
                Size(32, 32), // 窗口尺寸
                Size(16, 16), // 块尺寸
                Size(8, 8),   // 块步长
                Size(8, 8),   // 胞元尺寸
                9             // 方向bin数
            );
            
            vector<float> descriptors;
            hog->compute(img_processed, descriptors);
            
            // 关键修改：确保数据格式正确
            Mat feature = Mat(descriptors).t(); // 转置为行向量
            feature.convertTo(feature, CV_32F); // 转换为32位浮点
            
            // 检查特征维度一致性
            if (first_feature) {
                feature_size = feature.cols;
                first_feature = false;
                cout << "特征维度: " << feature_size << endl;
            } else if (feature.cols != feature_size) {
                cerr << "特征维度不一致! 期望: " << feature_size << ", 实际: " << feature.cols << endl;
                continue;
            }
            
            train_features.push_back(feature);
            train_labels.push_back(digit);
        }
    }

    // 检查是否有训练数据
    if (train_features.empty()) {
        cerr << "错误: 没有找到有效的训练数据!" << endl;
        return -1;
    }

    cout << "总共收集到 " << train_features.size() << " 个训练样本" << endl;

    // 构建训练数据矩阵 - 使用更安全的方式
    Mat train_data;
    try {
        // 方法1: 使用vconcat
        vconcat(train_features, train_data);
        
        // 方法2: 或者手动构建矩阵
        // train_data = Mat(train_features.size(), feature_size, CV_32F);
        // for (size_t i = 0; i < train_features.size(); i++) {
        //     train_features[i].copyTo(train_data.row(i));
        // }
    } catch (const Exception& e) {
        cerr << "构建训练数据矩阵失败: " << e.what() << endl;
        return -1;
    }

    // 构建标签矩阵
    Mat train_labels_mat(train_labels);
    train_labels_mat = train_labels_mat.reshape(1, train_labels_mat.total());
    train_labels_mat.convertTo(train_labels_mat, CV_32S); // SVM需要32位整数标签

    cout << "训练数据维度: " << train_data.rows << " x " << train_data.cols << endl;
    cout << "标签数据维度: " << train_labels_mat.rows << " x " << train_labels_mat.cols << endl;

    // 创建并训练SVM
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::RBF);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 1000, 1e-6));
    
    cout << "开始训练1-5数字SVM模型..." << endl;
    
    try {
        // 先尝试普通训练
        svm->train(train_data, ROW_SAMPLE, train_labels_mat);
        
        // 如果普通训练成功，再尝试自动训练（可选）
        // svm->trainAuto(train_data, ROW_SAMPLE, train_labels_mat);
        
        cout << "训练完成！" << endl;
    } catch (const Exception& e) {
        cerr << "训练失败: " << e.what() << endl;
        return -1;
    }

    // 保存模型
    svm->save("armor_digit_1to5_svm.yml");
    cout << "模型已保存为: armor_digit_1to5_svm.yml" << endl;

    // 测试模型准确率
    Mat predictions;
    svm->predict(train_data, predictions);
    
    int correct = 0;
    for (int i = 0; i < predictions.rows; i++) {
        if (predictions.at<float>(i) == train_labels[i]) {
            correct++;
        }
    }
    double accuracy = (double)correct / predictions.rows;
    cout << "训练集准确率: " << accuracy * 100 << "%" << endl;

    return 0;
}