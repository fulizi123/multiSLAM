//
//
//#include <iostream>
//#include <fstream>
//#include <vector>//
//#include<Eigen/Dense>
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//using namespace std;
//
//std::vector<float> getSignal(std::string frameBinDir){
//    std::vector<float> fDataBuff(2000000); // 申请空间，应足够大
//    std::ifstream infile(frameBinDir.c_str(), std::ifstream::binary);
//    infile.read((char*)&fDataBuff.front(), fDataBuff.size()*sizeof(float));
//    int num_tx_samps = infile.gcount()/sizeof(float);
//    fDataBuff.erase(fDataBuff.begin()+num_tx_samps, fDataBuff.end()); // 清除多余空间
//    return fDataBuff;
//}
//
//
//int main() {
//
//    std::vector<float> a = getSignal("/home/intelnuc/CLionProjects/LL_SLAM_MultiCamera/000000.bin");
//    for (int i = 0; i < a.size(); i+= 258) {
//        cout << " x " << a[i] << " y " << a[i+1] << endl;
//        for (int j = 0; j < 256; j+= 1) {
//            cout <<  a[i + 2 + j] << " ";
//        }
//        cout << endl;
//    }
//
//    std::vector<std::vector<float>> desc ;
//    std::vector<pair<int, int>> pts ;
//    for (int i = 0; i < a.size(); i+= 258) {
//        std::vector<float>temp(256, 0.0);
//        for (int j = 0; j < 256; j+= 1) {
//            temp[j] = a[i + 2 + j];
//        }
//        desc.push_back(temp);
//        pts.push_back({a[i], a[i+1]});
//    }
//
//    auto desc0 = desc[0];
//    auto desc1 = desc[1];
//    float score_float = 0;
//    float score_int = 0;
//    float score_eigen = 0;
//    float score_cv = 0;
//    for (int j = 0; j < 256; j+= 1) {
//        score_float += desc0[j] * desc1[j];
//        score_int += float(int(256 * desc0[j]) * int(256 * desc1[j])) / (256.0*256.0);
//    }
//    cout << " score_float " << score_float << endl;
//    cout << " score_int " << score_int << endl;
//
//    Eigen::Matrix<float, 256, 1> desc0_eigen;
//    Eigen::Matrix<float, 256, 1> desc1_eigen;
//    cv::Mat desc0_cv(256, 1, CV_32FC1);
//    cv::Mat desc1_cv(256, 1, CV_32FC1);
//    for (int j = 0; j < 256; j+= 1) {
//        desc0_eigen(j, 0) = desc0[j];
//        desc1_eigen(j, 0) = desc1[j];
//        desc0_cv.at<float>(j, 0) = desc0[j];
//        desc1_cv.at<float>(j, 0) = desc1[j];
////        desc1_cv(j, 0) = desc1[j];
//    }
//
//    {
//        //0.001ms
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        /////////////////
//        for (int j = 0; j < 256; j+= 1) {
//            score_float += desc0[j] * desc1[j];
//        }
//        /////////////////
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//        cout << "vector use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;
//    }
//    {
//        //0.06ms score_eigen += desc0_eigen(j, 0) * desc1_eigen(j, 0);
//        //0.0026ms score_eigen = desc0_eigen.dot(desc1_eigen);
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        /////////////////
//        for (int j = 0; j < 256; j+= 1) {
//            score_eigen += desc0_eigen(j, 0) * desc1_eigen(j, 0);
//        }
////        score_eigen = desc0_eigen.dot(desc1_eigen);
//        /////////////////
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//        cout << "eigen use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;
//    }
//    {
//        ////////////////////////////////////////////////////
//        //0.0014ms score_eigen += desc0_cv.at<float>(j, 0) * desc1_cv.at<float>(j, 0);
//        ////////////////////////////////////////////////////
//        //0.0025ms score_eigen += desc0_cv.at<float>(j) * desc1_cv.at<float>(j);
//        //0.06ms score_cv = desc0_cv.dot(desc1_cv);
//        //0.07ms score_cv = cv::norm(desc0_cv,desc1_cv,cv::NORM_L1);
//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//        /////////////////
//        for (int j = 0; j < 256; j+= 1) {
//            score_eigen += desc0_cv.at<float>(j, 0) * desc1_cv.at<float>(j, 0);
////            score_eigen += desc0_cv.at<float>(j) * desc1_cv.at<float>(j);
//        }
////        score_cv = desc0_cv.dot(desc1_cv);
////        score_cv = cv::norm(desc0_cv,desc1_cv,cv::NORM_L1);
//        /////////////////
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//        cout << "cv use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;
//    }
//
//    cv::Mat img = cv::imread("/home/intelnuc/CLionProjects/LL_SLAM_MultiCamera/000000.png", -1);
//    cv::Mat img_100 = img.clone();
//    for (int i = 0; i < 100; i++) { cv::circle(img_100, cv::Point2f(pts[i].first, pts[i].second),2,cv::Scalar(0, 255, 0),-1); }
//    cv::imshow("img_100", img_100);
//
//    cv::Mat img_500 = img.clone();
//    for (int i = 0; i < 500; i++) { cv::circle(img_500, cv::Point2f(pts[i].first, pts[i].second),2,cv::Scalar(0, 255, 0),-1); }
//    cv::imshow("img_500", img_500);
//
//    cv::Mat img_1000 = img.clone();
//    for (int i = 0; i < 1000; i++) { cv::circle(img_1000, cv::Point2f(pts[i].first, pts[i].second),2,cv::Scalar(0, 255, 0),-1); }
//    cv::imshow("img_1000", img_1000);
//
//    cv::Mat img_2000 = img.clone();
//    for (int i = 0; i < 2000; i++) { cv::circle(img_2000, cv::Point2f(pts[i].first, pts[i].second),2,cv::Scalar(0, 255, 0),-1); }
//    cv::imshow("img_2000", img_2000);
//
//    cv::waitKey(0);
//
//    return 0;
//}


//
//#include <System.h>
//
//using namespace std;
//
//int main() {
//    std::cout << "Hello, World!" << std::endl;
//    std::cout << "Hello, World!" << std::endl;
//    cout << endl;
//    LL_SLAM::System("aaa");
//    string a = "asdad";
//    cout << a << endl;
//    return 0;
//}
//



#include <mutex>
#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <stdio.h>

#include <condition_variable>



#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<Eigen/Dense>
#include<opencv2/core/core.hpp>

#include<System.h>
//#include<Converter.h>
#include "Thirdparty/Sophus/sophus/geometry.hpp"

using namespace std;

bool b_continue_session;



void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;

}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageCam00,  vector<string> &vstrImageCam01,  vector<string> &vstrImageCam02,
                vector<string> &vstrImageCam03,  vector<string> &vstrImageCam04,  vector<string> &vstrImageCam05,
                vector<string> &vstrImageCam06,  vector<string> &vstrImageCam07,  vector<string> &vstrImageCam08,
                vector<string> &vstrImageCam09,  vector<string> &vstrImageCam10,  vector<string> &vstrImageCam11,
                vector<double> &vTimestamps,
                float SpeedUp = 1.0);


void LoadSuperPoint(const string &strPathToSequence,
                vector<string> &vstrSuperPointCam00,  vector<string> &vstrSuperPointCam01,  vector<string> &vstrSuperPointCam02,
                vector<string> &vstrSuperPointCam03,  vector<string> &vstrSuperPointCam04,  vector<string> &vstrSuperPointCam05,
                vector<string> &vstrSuperPointCam06,  vector<string> &vstrSuperPointCam07,  vector<string> &vstrSuperPointCam08,
                vector<string> &vstrSuperPointCam09,  vector<string> &vstrSuperPointCam10,  vector<string> &vstrSuperPointCam11
                );

std::vector<float> getSuperPoint(std::string frameBinDir, std::vector<std::vector<int>> &vKeyPoints, std::vector<std::vector<float>> &vDescriptors){
    std::vector<float> fDataBuff(2000000); // 申请空间，应足够大
    std::ifstream infile(frameBinDir.c_str(), std::ifstream::binary);
    infile.read((char*)&fDataBuff.front(), fDataBuff.size()*sizeof(float));
    int num_tx_samps = infile.gcount()/sizeof(float);
    fDataBuff.erase(fDataBuff.begin()+num_tx_samps, fDataBuff.end()); // 清除多余空间

    int N = fDataBuff.size() / 258;
    if (fDataBuff.size() != N * 258) {
        cout << "Desc ERROR!!!" << endl;
    }
    vKeyPoints.resize(N, std::vector<int>(2, -1));
    vDescriptors.resize(N, std::vector<float>(256, 0.0));

    for (int i = 0; i < N; i += 1) {
        int BeginIndex = i * 258;

        //pt : x ,y
        vKeyPoints[i][0] = fDataBuff[BeginIndex];
        vKeyPoints[i][1] = fDataBuff[BeginIndex + 1];

        //Desc
        for (int j = 0; j < 256; j += 1) {
            vDescriptors[i][j] = fDataBuff[BeginIndex + 2 + j];
        }
    }

    return fDataBuff;
}


int main(int argc, char **argv)
{

    if(argc != 2)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_settings" << endl;
        cerr << endl << "Adjust Settings" << endl;
        return 1;
    }

    string SettingPath = string(argv[1]);

    cv::FileStorage fsSettings(SettingPath.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << SettingPath << endl;
        exit(-1);
    }


    string YOLOModelPath = fsSettings["YOLOModelPath"];
    string VocabularyPath = fsSettings["VocabularyPath"];
    string SequencePath = fsSettings["SequencePath"];

    int SequenceBegin = fsSettings["SequenceBegin"];
    int SequenceEnd = fsSettings["SequenceEnd"];
    float SpeedUp = fsSettings["SpeedUp"];

    cout<<" SequenceBegin "<<SequenceBegin<<" SequenceEnd "<<SequenceEnd<<" SpeedUp "<<SpeedUp<<endl;


    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;


    float imageScale = fsSettings["Camera.imageScale"];

    //multi


    Eigen::Matrix<float,4,4> Tbc_cam00 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam00"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam01 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam01"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam02 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam02"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam03 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam03"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam04 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam04"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam05 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam05"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam06 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam06"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam07 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam07"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam08 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam08"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam09 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam09"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam10 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam10"].mat());
    Eigen::Matrix<float,4,4> Tbc_cam11 = LL_SLAM::CommonTools::toMatrix4f( fsSettings["Tbc_cam11"].mat());

    Eigen::Matrix3f K;
    K << 369.504172281, 0, 640.0, 0.0, 369.504172281, 360.0, 0.0, 0.0, 1.0;
    if (imageScale != 1.f) {K = K * imageScale;}
    Eigen::Matrix3f K_cam00 = K;
    Eigen::Matrix3f K_cam01 = K;
    Eigen::Matrix3f K_cam02 = K;
    Eigen::Matrix3f K_cam03 = K;
    Eigen::Matrix3f K_cam04 = K;
    Eigen::Matrix3f K_cam05 = K;
    Eigen::Matrix3f K_cam06 = K;
    Eigen::Matrix3f K_cam07 = K;
    Eigen::Matrix3f K_cam08 = K;
    Eigen::Matrix3f K_cam09 = K;
    Eigen::Matrix3f K_cam10 = K;
    Eigen::Matrix3f K_cam11 = K;

    int NumCam = 12;
    vector<Eigen::Matrix4f> Tbc_cams = {
        Tbc_cam00, Tbc_cam01, Tbc_cam02,
        Tbc_cam03, Tbc_cam04, Tbc_cam05,
        Tbc_cam06, Tbc_cam07, Tbc_cam08,
        Tbc_cam09, Tbc_cam10, Tbc_cam11};
    vector<Eigen::Matrix3f> K_cams = {
        K_cam00, K_cam01, K_cam02,
        K_cam03, K_cam04,  K_cam05,
        K_cam06, K_cam07, K_cam08,
        K_cam09, K_cam10, K_cam11};
//    vector<Eigen::Matrix4f> Tbc_cams = {};
//    vector<Eigen::Matrix3f> K_cams = {};




    // Retrieve paths to images

    vector <string> vstrImageCam00;
    vector <string> vstrImageCam01;
    vector <string> vstrImageCam02;
    vector <string> vstrImageCam03;
    vector <string> vstrImageCam04;
    vector <string> vstrImageCam05;
    vector <string> vstrImageCam06;
    vector <string> vstrImageCam07;
    vector <string> vstrImageCam08;
    vector <string> vstrImageCam09;
    vector <string> vstrImageCam10;
    vector <string> vstrImageCam11;
    vector<double> vTimestamps;

    LoadImages(SequencePath,
                vstrImageCam00,  vstrImageCam01,  vstrImageCam02,  vstrImageCam03,  vstrImageCam04,  vstrImageCam05,
                 vstrImageCam06,  vstrImageCam07,  vstrImageCam08,  vstrImageCam09,  vstrImageCam10,  vstrImageCam11,
                vTimestamps, SpeedUp);

    // Retrieve paths to SuperPoint

    vector <string> vstrSuperPointCam00;
    vector <string> vstrSuperPointCam01;
    vector <string> vstrSuperPointCam02;
    vector <string> vstrSuperPointCam03;
    vector <string> vstrSuperPointCam04;
    vector <string> vstrSuperPointCam05;
    vector <string> vstrSuperPointCam06;
    vector <string> vstrSuperPointCam07;
    vector <string> vstrSuperPointCam08;
    vector <string> vstrSuperPointCam09;
    vector <string> vstrSuperPointCam10;
    vector <string> vstrSuperPointCam11;

    LoadSuperPoint(SequencePath,
               vstrSuperPointCam00,  vstrSuperPointCam01,  vstrSuperPointCam02,  vstrSuperPointCam03,  vstrSuperPointCam04,  vstrSuperPointCam05,
               vstrSuperPointCam06,  vstrSuperPointCam07,  vstrSuperPointCam08,  vstrSuperPointCam09,  vstrSuperPointCam10,  vstrSuperPointCam11);

    const int nImages = vTimestamps.size();

    LL_SLAM::System SLAM(fsSettings, NumCam, Tbc_cams, K_cams);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

//    float imageScale = SLAM.GetImageScale();
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    double t_track = 0.f;
    double t_resize = 0.f;

    // Main loop
    cv::Mat
        imCam00, imCam01, imCam02,
        imCam03, imCam04, imCam05,
        imCam06, imCam07, imCam08,
        imCam09, imCam10, imCam11;
    for(int ni=max(0,SequenceBegin); ni<min(nImages,SequenceEnd); ni++)
    {


        // Read left and right images from file
        imCam00 = cv::imread(vstrImageCam00[ni],cv::IMREAD_UNCHANGED);
        imCam01 = cv::imread(vstrImageCam01[ni],cv::IMREAD_UNCHANGED);
        imCam02 = cv::imread(vstrImageCam02[ni],cv::IMREAD_UNCHANGED);
        imCam03 = cv::imread(vstrImageCam03[ni],cv::IMREAD_UNCHANGED);
        imCam04 = cv::imread(vstrImageCam04[ni],cv::IMREAD_UNCHANGED);
        imCam05 = cv::imread(vstrImageCam05[ni],cv::IMREAD_UNCHANGED);
        imCam06 = cv::imread(vstrImageCam06[ni],cv::IMREAD_UNCHANGED);
        imCam07 = cv::imread(vstrImageCam07[ni],cv::IMREAD_UNCHANGED);
        imCam08 = cv::imread(vstrImageCam08[ni],cv::IMREAD_UNCHANGED);
        imCam09 = cv::imread(vstrImageCam09[ni],cv::IMREAD_UNCHANGED);
        imCam10 = cv::imread(vstrImageCam10[ni],cv::IMREAD_UNCHANGED);
        imCam11 = cv::imread(vstrImageCam11[ni],cv::IMREAD_UNCHANGED);
//        imCam00 = cv::imread(vstrImageCam00[ni],cv::IMREAD_UNCHANGED);
//        imCam01 = imCam00;
//        imCam02 = imCam00;
//        imCam03 = imCam00;
//        imCam04 = imCam00;
//        imCam05 = imCam00;
//        imCam06 = imCam00;
//        imCam07 = imCam00;
//        imCam08 = imCam00;
//        imCam09 = imCam00;
//        imCam10 = imCam00;
//        imCam11 = imCam00;


        vector<vector<int>>
                vKeyPointCam00, vKeyPointCam01, vKeyPointCam02,
                vKeyPointCam03, vKeyPointCam04, vKeyPointCam05,
                vKeyPointCam06, vKeyPointCam07, vKeyPointCam08,
                vKeyPointCam09, vKeyPointCam10, vKeyPointCam11;
        vector<vector<float>>
                vDescriptorCam00, vDescriptorCam01, vDescriptorCam02,
                vDescriptorCam03, vDescriptorCam04, vDescriptorCam05,
                vDescriptorCam06, vDescriptorCam07, vDescriptorCam08,
                vDescriptorCam09, vDescriptorCam10, vDescriptorCam11;

        getSuperPoint(vstrSuperPointCam00[ni], vKeyPointCam00, vDescriptorCam00);
        getSuperPoint(vstrSuperPointCam01[ni], vKeyPointCam01, vDescriptorCam01);
        getSuperPoint(vstrSuperPointCam02[ni], vKeyPointCam02, vDescriptorCam02);
        getSuperPoint(vstrSuperPointCam03[ni], vKeyPointCam03, vDescriptorCam03);
        getSuperPoint(vstrSuperPointCam04[ni], vKeyPointCam04, vDescriptorCam04);
        getSuperPoint(vstrSuperPointCam05[ni], vKeyPointCam05, vDescriptorCam05);
        getSuperPoint(vstrSuperPointCam06[ni], vKeyPointCam06, vDescriptorCam06);
        getSuperPoint(vstrSuperPointCam07[ni], vKeyPointCam07, vDescriptorCam07);
        getSuperPoint(vstrSuperPointCam08[ni], vKeyPointCam08, vDescriptorCam08);
        getSuperPoint(vstrSuperPointCam09[ni], vKeyPointCam09, vDescriptorCam09);
        getSuperPoint(vstrSuperPointCam10[ni], vKeyPointCam10, vDescriptorCam10);
        getSuperPoint(vstrSuperPointCam11[ni], vKeyPointCam11, vDescriptorCam11);



        double tframe = vTimestamps[ni];

        vector<cv::Mat> vImCams = {
                imCam00, imCam01, imCam02,
                imCam03, imCam04, imCam05,
                imCam06, imCam07, imCam08,
                imCam09, imCam10, imCam11};

        vector<vector<vector<int>>> *pvKeyPoints = new vector<vector<vector<int>>>({
            vKeyPointCam00, vKeyPointCam01, vKeyPointCam02,
                    vKeyPointCam03, vKeyPointCam04, vKeyPointCam05,
                    vKeyPointCam06, vKeyPointCam07, vKeyPointCam08,
                    vKeyPointCam09, vKeyPointCam10, vKeyPointCam11});

        vector<vector<vector<float>>> *pvDescriptor =  new vector<vector<vector<float>>>({
            vDescriptorCam00, vDescriptorCam01, vDescriptorCam02,
                    vDescriptorCam03, vDescriptorCam04, vDescriptorCam05,
                    vDescriptorCam06, vDescriptorCam07, vDescriptorCam08,
                    vDescriptorCam09, vDescriptorCam10, vDescriptorCam11});

//        vector<vector<vector<int>>> vKeyPoints = {
//            vKeyPointCam00, vKeyPointCam01, vKeyPointCam02,
//                    vKeyPointCam03, vKeyPointCam04, vKeyPointCam05,
//                    vKeyPointCam06, vKeyPointCam07, vKeyPointCam08,
//                    vKeyPointCam09, vKeyPointCam10, vKeyPointCam11};
//
//        vector<vector<vector<float>>> vDescriptor = {
//            vDescriptorCam00, vDescriptorCam01, vDescriptorCam02,
//                    vDescriptorCam03, vDescriptorCam04, vDescriptorCam05,
//                    vDescriptorCam06, vDescriptorCam07, vDescriptorCam08,
//                    vDescriptorCam09, vDescriptorCam10, vDescriptorCam11};




        if(imCam00.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageCam00[ni]) << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {

            int width = imCam00.cols * imageScale;
            int height = imCam00.rows * imageScale;
            for (int imCami = 0; imCami < vImCams.size(); imCami++) {
                cv::resize(vImCams[imCami], vImCams[imCami], cv::Size(width, height));
            }

        }


        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // Pass the images to the SLAM system
        //SLAM.TrackStereo(imLeft,imRight,tframe);
        cv::Mat Tcw;
        Sophus::SE3f Tcw_Sophus;//Tcw_Sophus =
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //SLAM.TrackMultiCamera(vImCams, tframe, {}, {});
        Eigen::Matrix4f Twb = SLAM.TrackMultiCamera(vImCams, tframe, pvKeyPoints, pvDescriptor);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        cout << "SLAM use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;


        //release the memory
        for (int imCami = 0; imCami < vImCams.size(); imCami++) {
            vImCams[imCami].release();
        }
        vector<vector<vector<int>>>().swap(* pvKeyPoints);
        vector<vector<vector<float>>>().swap(* pvDescriptor);





        cout << "Number of Images : " <<  ni << endl;
        cout << "twb : " <<  LL_SLAM::CommonTools::T2t(Twb).transpose() << endl;
        cout << endl;
        cout << endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    //SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

//    // Save camera trajectory
//    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");
//    SLAM.SaveTrajectoryTUM("CameraTrajectoryTUM.txt");

    return 0;
}

void LoadImages(const string &strPathToSequence,
                vector<string> &vstrImageCam00,  vector<string> &vstrImageCam01,  vector<string> &vstrImageCam02,
                vector<string> &vstrImageCam03,  vector<string> &vstrImageCam04,  vector<string> &vstrImageCam05,
                vector<string> &vstrImageCam06,  vector<string> &vstrImageCam07,  vector<string> &vstrImageCam08,
                vector<string> &vstrImageCam09,  vector<string> &vstrImageCam10,  vector<string> &vstrImageCam11,
                vector<double> &vTimestamps,
                float SpeedUp )
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            t = t/SpeedUp;
            vTimestamps.push_back(t);
        }
    }

    const int nTimes = vTimestamps.size();

    vstrImageCam00.resize(nTimes);  vstrImageCam01.resize(nTimes);  vstrImageCam02.resize(nTimes);
    vstrImageCam03.resize(nTimes);  vstrImageCam04.resize(nTimes);  vstrImageCam05.resize(nTimes);
    vstrImageCam06.resize(nTimes);  vstrImageCam07.resize(nTimes);  vstrImageCam08.resize(nTimes);
    vstrImageCam09.resize(nTimes);  vstrImageCam10.resize(nTimes);  vstrImageCam11.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;

        vstrImageCam00[i] = strPathToSequence + "/Cam00/" + ss.str() + ".png";
        vstrImageCam01[i] = strPathToSequence + "/Cam01/" + ss.str() + ".png";
        vstrImageCam02[i] = strPathToSequence + "/Cam02/" + ss.str() + ".png";
        vstrImageCam03[i] = strPathToSequence + "/Cam03/" + ss.str() + ".png";
        vstrImageCam04[i] = strPathToSequence + "/Cam04/" + ss.str() + ".png";
        vstrImageCam05[i] = strPathToSequence + "/Cam05/" + ss.str() + ".png";

        vstrImageCam06[i] = strPathToSequence + "/Cam06/" + ss.str() + ".png";
        vstrImageCam07[i] = strPathToSequence + "/Cam07/" + ss.str() + ".png";
        vstrImageCam08[i] = strPathToSequence + "/Cam08/" + ss.str() + ".png";
        vstrImageCam09[i] = strPathToSequence + "/Cam09/" + ss.str() + ".png";
        vstrImageCam10[i] = strPathToSequence + "/Cam10/" + ss.str() + ".png";
        vstrImageCam11[i] = strPathToSequence + "/Cam11/" + ss.str() + ".png";

    }
}




void LoadSuperPoint(const string &strPathToSequence,
                vector<string> &vstrSuperPointCam00,  vector<string> &vstrSuperPointCam01,  vector<string> &vstrSuperPointCam02,
                vector<string> &vstrSuperPointCam03,  vector<string> &vstrSuperPointCam04,  vector<string> &vstrSuperPointCam05,
                vector<string> &vstrSuperPointCam06,  vector<string> &vstrSuperPointCam07,  vector<string> &vstrSuperPointCam08,
                vector<string> &vstrSuperPointCam09,  vector<string> &vstrSuperPointCam10,  vector<string> &vstrSuperPointCam11
                )
{
    vector<double> vTimestamps;
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            t = t;
            vTimestamps.push_back(t);
        }
    }

    const int nTimes = vTimestamps.size();

    vstrSuperPointCam00.resize(nTimes);  vstrSuperPointCam01.resize(nTimes);  vstrSuperPointCam02.resize(nTimes);
    vstrSuperPointCam03.resize(nTimes);  vstrSuperPointCam04.resize(nTimes);  vstrSuperPointCam05.resize(nTimes);
    vstrSuperPointCam06.resize(nTimes);  vstrSuperPointCam07.resize(nTimes);  vstrSuperPointCam08.resize(nTimes);
    vstrSuperPointCam09.resize(nTimes);  vstrSuperPointCam10.resize(nTimes);  vstrSuperPointCam11.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;

        vstrSuperPointCam00[i] = strPathToSequence + "/SuperPointCam00/" + ss.str() + ".bin";
        vstrSuperPointCam01[i] = strPathToSequence + "/SuperPointCam01/" + ss.str() + ".bin";
        vstrSuperPointCam02[i] = strPathToSequence + "/SuperPointCam02/" + ss.str() + ".bin";
        vstrSuperPointCam03[i] = strPathToSequence + "/SuperPointCam03/" + ss.str() + ".bin";
        vstrSuperPointCam04[i] = strPathToSequence + "/SuperPointCam04/" + ss.str() + ".bin";
        vstrSuperPointCam05[i] = strPathToSequence + "/SuperPointCam05/" + ss.str() + ".bin";

        vstrSuperPointCam06[i] = strPathToSequence + "/SuperPointCam06/" + ss.str() + ".bin";
        vstrSuperPointCam07[i] = strPathToSequence + "/SuperPointCam07/" + ss.str() + ".bin";
        vstrSuperPointCam08[i] = strPathToSequence + "/SuperPointCam08/" + ss.str() + ".bin";
        vstrSuperPointCam09[i] = strPathToSequence + "/SuperPointCam09/" + ss.str() + ".bin";
        vstrSuperPointCam10[i] = strPathToSequence + "/SuperPointCam10/" + ss.str() + ".bin";
        vstrSuperPointCam11[i] = strPathToSequence + "/SuperPointCam11/" + ss.str() + ".bin";

    }
}

