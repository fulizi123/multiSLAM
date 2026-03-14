#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iomanip>

using namespace std;

// --- 1. 读取 Bin 文件的辅助函数 (沿用之前的) ---
bool getXFeat(std::string frameBinDir, std::vector<cv::KeyPoint> &vKeyPoints, cv::Mat &vDescriptors) {
    std::ifstream infile(frameBinDir.c_str(), std::ifstream::binary);
    if (!infile.is_open()) {
        std::cerr << "Failed to open bin file: " << frameBinDir << std::endl;
        return false;
    }
    infile.seekg(0, std::ios::end);
    std::streampos fileSize = infile.tellg();
    infile.seekg(0, std::ios::beg);
    if (fileSize == 0) return false;
    std::vector<float> fDataBuff(fileSize / sizeof(float));
    infile.read((char*)&fDataBuff.front(), fileSize);
    infile.close();
    const int descDim = 64;                 
    const int floatPerPoint = 2 + descDim;
    int N = fDataBuff.size() / floatPerPoint;
    if (N == 0) return false;
    vKeyPoints.resize(N);
    vDescriptors = cv::Mat(N, descDim, CV_32F);
    for (int i = 0; i < N; i++) {
        int baseIdx = i * floatPerPoint;
        vKeyPoints[i].pt.x = fDataBuff[baseIdx];
        vKeyPoints[i].pt.y = fDataBuff[baseIdx + 1];
        for (int j = 0; j < descDim; j++) {
            vDescriptors.at<float>(i, j) = fDataBuff[baseIdx + 2 + j];
        }
    }
    return true;
}

// --- 主函数 ---
int main(int argc, char** argv) {
    // 需要传入4个路径：图1 Bin1 图2 Bin2
    if(argc != 5) {
        cout << "Usage: ./VisMatches <img1_path> <bin1_path> <img2_path> <bin2_path>" << endl;
        cout << "Example (Stereo): ./VisMatches left.png left.bin right.png right.bin" << endl;
        return -1;
    }

    string imgPath1 = argv[1]; string binPath1 = argv[2];
    string imgPath2 = argv[3]; string binPath2 = argv[4];

    // 1. 读取数据
    cout << "Loading images and features..." << endl;
    cv::Mat img1 = cv::imread(imgPath1);
    cv::Mat img2 = cv::imread(imgPath2);
    if (img1.empty() || img2.empty()) { cerr << "Failed to load images." << endl; return -1; }

    vector<cv::KeyPoint> kps1, kps2;
    cv::Mat desc1, desc2;
    if (!getXFeat(binPath1, kps1, desc1) || !getXFeat(binPath2, kps2, desc2)) {
        cerr << "Failed to load bin files." << endl; return -1;
    }
    cout << "Loaded -> Img1: " << kps1.size() << " kpts, Img2: " << kps2.size() << " kpts." << endl;

    // 2. 定义要测试的倍率列表
    vector<float> multipliers = {20.0f, 30.0f, 50.0f, 100.0f};
    // ORB-SLAM 的固定阈值
    const int TH_LOW = 50;

    cout << "\nStarting matching visualization..." << endl;

    // 3. 循环测试不同倍率
    for (float mult : multipliers) {
        cout << "Testing multiplier: *" << mult << endl;
        vector<cv::DMatch> good_matches;
        
        // 暴力匹配 (Brute Force Matching)
        // 模拟 SLAM 中的搜索过程
        for (int i = 0; i < desc1.rows; ++i) {
            float min_l2 = 1000.0f;
            int best_idx2 = -1;

            // 在图2中找最近邻
            for (int j = 0; j < desc2.rows; ++j) {
                float l2 = cv::norm(desc1.row(i), desc2.row(j), cv::NORM_L2);
                if (l2 < min_l2) {
                    min_l2 = l2;
                    best_idx2 = j;
                }
            }

            // 核心逻辑：应用当前的倍率和阈值
            if (best_idx2 != -1) {
                // 这就是你在 FeatureMatcher.cc 里修改的那行代码的模拟
                int dist_score = (int)(min_l2 * mult);

                // 模拟 ORB-SLAM 的筛选门槛
                if (dist_score < TH_LOW) {
                    // 如果通过了阈值，加入匹配列表
                    good_matches.emplace_back(i, best_idx2, (float)dist_score);
                }
            }
        }

        cout << "  -> Found " << good_matches.size() << " matches passing TH_LOW=50." << endl;

        // 4. 绘制并保存结果
        cv::Mat img_matches;
        // 使用绿色绘制匹配线，红色绘制未匹配的点
        cv::drawMatches(img1, kps1, img2, kps2, good_matches, img_matches,
                        cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), vector<char>(), 
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // 在图片上写上当前的倍率信息
        string label = "Multiplier: *" + to_string((int)mult) + " | Matches: " + to_string(good_matches.size());
        cv::putText(img_matches, label, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

        // 保存文件
        string savePath = "result_mult_" + to_string((int)mult) + ".jpg";
        cv::imwrite(savePath, img_matches);
        cout << "  -> Saved visualization to: " << savePath << endl;
    }

    cout << "\nDone! Please download the 'result_mult_XX.jpg' files to analyze." << endl;
    return 0;
}