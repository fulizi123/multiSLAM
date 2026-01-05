
#include "Viewer.h"


namespace LL_SLAM
{
    Viewer::Viewer(System *pSystem) {
        mpSystem = pSystem;
    }

    void Viewer::Run() {
        while (1) {

            if (mvvImCams.empty()) {
                usleep(2000);
            } else {
                vector<cv::Mat> vImCams;
                Frame *pCurrentFrame;
                {
                    unique_lock<mutex> lock(mMutexMsg);
                    vImCams = mvvImCams.front();
                    pCurrentFrame = mvpFrame.front();
                    mvvImCams.clear();
                    mvpFrame.clear();
                }

                Visualization(vImCams, pCurrentFrame);

//                VisualizationYZ(vImCams, pCurrentFrame);
//
//                VisualizationXY(vImCams, pCurrentFrame);
            }
        }
    }


    void Viewer::InsertFrame(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame)
    {
//        cout << "InsertFrame " << endl;
        unique_lock<mutex> lock(mMutexMsg);
        mvvImCams.push_back(vImCams);
        mvpFrame.push_back(pCurrentFrame);
//
//        vector<vector<cv::Mat>> mvvImCams;
//        vector<Frame *> mvpFrame;
    }





    void Viewer::Visualization(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) {

        cv::Mat imShow = vImCams[0].clone();
        vector<cv::KeyPoint> vKeys = pCurrentFrame->mvCamKeysUn[0];
        for (int i = 0; i < vKeys.size(); i++) { if (vKeys[i].octave == 0) {cv::circle(imShow, cv::Point2f(vKeys[i].pt),2,cv::Scalar(0, 255, 0),-1);} }
//        cv::imshow("imShow", imShow);
//        cv::resize(imShow, imShow, cv::Size(320, 180));
        cv::resize(imShow, imShow, cv::Size(480, 270));
//        cv::imshow("imShow", imShow);

//        Eigen::Vector3f t_w_viewer = mpReferenceKF->Gettwb();
        Eigen::Vector3f t_w_viewer = pCurrentFrame->Gettwb();
        Twbs.push_back(pCurrentFrame->GetTwb());


        ///////visual
//        int w = 1000;
        int w = 1000;
        int h = 720;
        float fbl = 0.08;
//        float fbl = 0.3;
        cv::Mat img_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
//      img_map.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);

        //connection
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);

                cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(64, 64, 64), 1);
            }
        }

        //connection in viewer frame
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);

                if (Pj_u >= 0 && Pj_u < w && Pj_v >= 0 && Pj_v < h) {
                    cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 128, 128), 1);
                }

            }
        }


        //ALL MapPoint
        for (int MP_i = 0; MP_i < mpSystem->mpMap->mvpLocalMP.size(); MP_i++) {
            MapPoint *pMP = mpSystem->mpMap->mvpLocalMP[MP_i];
            if (pMP == NULL) {
                continue;
            }
            Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
            int Pi_u = int( Pw.x() / fbl + w / 2.0);
            int Pi_v = int(-Pw.z() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(128, 128, 0),-1);
            //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,pMP->mColor,-1);

        }
        //Tracked MapPoint
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {
            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
                int Pi_u = int( Pw.x() / fbl + w / 2.0);
                int Pi_v = int(-Pw.z() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 0),-1);
                //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,pMP->mColor,-1);

            }
        }

        //history

        for (int i = 0; i < Twbs.size(); i++) {
            Eigen::Matrix4f Twb_i = Twbs[i];
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;

            int Pi_u = int( twb_i.x() / fbl + w / 2.0);
            int Pi_v = int(-twb_i.z() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(30, 128, 30),-1);
        }
        {
//            cout << "pCurrentFrame->Gettwb() " << pCurrentFrame->Gettwb() << endl;
            Eigen::Matrix4f Twb_i = pCurrentFrame->PredictPose().inverse();
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;
//            cout << "pCurrentFrame->PredictPose() " << CommonTools::T2t(Twb_i) << endl;

            int Pi_u = int( twb_i.x() / fbl + w / 2.0);
            int Pi_v = int(-twb_i.z() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(255, 128, 30),-1);
        }

        //camera
        {
            float camera_size = 0.3;
            vector<vector<float>> camera_vertex = {
                    {0,                    0,            0},
                    {1.73f * camera_size,  camera_size,  camera_size},
                    {1.73f * camera_size,  -camera_size, camera_size},
                    {-1.73f * camera_size, camera_size,  camera_size},
                    {-1.73f * camera_size, -camera_size, camera_size}};
            for (int cam_i = 0; cam_i < pCurrentFrame->mNumCam; cam_i++) {

                Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
                Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
                Eigen::Matrix4f Twb = Tbw.inverse();
                Eigen::Matrix4f Twc = Twb * Tbc;
                for (int i = 0; i < camera_vertex.size(); i++) {
                    for (int j = i + 1; j < camera_vertex.size(); j++) {
                        Eigen::Vector3f Pc_cameravertex_i(camera_vertex[i][0], camera_vertex[i][1],
                                                          camera_vertex[i][2]);
                        Eigen::Vector3f Pc_cameravertex_j(camera_vertex[j][0], camera_vertex[j][1],
                                                          camera_vertex[j][2]);
                        Eigen::Vector3f Pw_cameravertex_i =
                                CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;
                        Eigen::Vector3f Pw_cameravertex_j =
                                CommonTools::T2R(Twc) * Pc_cameravertex_j + CommonTools::T2t(Twc) - t_w_viewer;

                        int Pi_u = int(Pw_cameravertex_i.x() / fbl + w / 2.0);
                        int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
                        int Pj_u = int(Pw_cameravertex_j.x() / fbl + w / 2.0);
                        int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);

                        cv::line(img_map,
                                 cv::Point(Pi_u, Pi_v),
                                 cv::Point(Pj_u, Pj_v),
                                 cv::Scalar(0, 255, 0), 2);
                    }
                }

                Eigen::Vector3f Pc_cameravertex_i(camera_vertex[0][0], camera_vertex[0][1], camera_vertex[0][2]);
                Eigen::Vector3f Pw_cameravertex_i =
                        CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;

                int Pi_u = int(Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v), 4, cv::Scalar(128, 255, 128), -1);

            }
        }

        {
            //axis_z
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_z_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_z_j(0, 0, 0.75);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_z_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_z_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
            int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
            int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
            int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 0, 255), 4);
        }
        {
            //axis_x
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_x_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_x_j(0.75, 0, 0);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_x_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_x_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
            int Pi_v = int(-Pw_cameravertex_i.z() / fbl + h / 2.0);
            int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
            int Pj_v = int(-Pw_cameravertex_j.z() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(255, 0, 0), 4);
        }


        {
            stringstream s;
            Eigen::Matrix4f Twb = pCurrentFrame->GetTwb();
            Eigen::Vector3f twb = CommonTools::T2t(Twb);
            s << " twc : " << twb.x() << " " << twb.y() << " " << twb.z();
            cv::putText(img_map, s.str(), cv::Point(5, img_map.rows * 0.95), cv::FONT_HERSHEY_PLAIN, 1,
                        cv::Scalar(255, 255, 255), 1, 8);
        }


        //////////////////////////////////////////////////////////////////////////
//        //debug
//
//
//        for (int MP_i = 0; MP_i < mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual.size(); MP_i++) {
//            Eigen::Vector3f Pw = mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual[MP_i] - t_w_viewer;
//
//            int Pi_u = int( Pw.x() / fbl + w / 2.0);
//            int Pi_v = int(-Pw.z() / fbl + h / 2.0);
//            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 255),-1);
//
//        }
//
//
//



        //////////////////////////////////////////////////////////////////////////


        imShow.copyTo(img_map(cv::Rect(max(int(0), 0),
                                       max(int(0), 0),
                                      (min(int(imShow.cols), img_map.cols) - max(0, 0)),
                                      (min(int(imShow.rows), img_map.rows) - max(0, 0)))));

        // 将所有像素设置为白色（最高亮度值）
        cv::imshow("img_map", img_map);
//


        cv::waitKey(2);
    }


    void Viewer::VisualizationYZ(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) {

        cv::Mat imShow = vImCams[0].clone();
        vector<cv::KeyPoint> vKeys = pCurrentFrame->mvCamKeysUn[0];
        for (int i = 0; i < vKeys.size(); i++) { if (vKeys[i].octave == 0) {cv::circle(imShow, cv::Point2f(vKeys[i].pt),2,cv::Scalar(0, 255, 0),-1);} }
//        cv::imshow("imShow", imShow);
//        cv::resize(imShow, imShow, cv::Size(320, 180));
        cv::resize(imShow, imShow, cv::Size(480, 270));
//        cv::imshow("imShow", imShow);

//        Eigen::Vector3f t_w_viewer = mpReferenceKF->Gettwb();
        Eigen::Vector3f t_w_viewer = pCurrentFrame->Gettwb();
        Twbs.push_back(pCurrentFrame->GetTwb());


        ///////visual
//        int w = 1000;
        int w = 2500;
        int h = 1400;
//        float fbl = 0.03;
        float fbl = 0.3;
        cv::Mat img_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
//      img_map.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);

        //connection
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                int Pj_u = int(-Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(64, 64, 64), 1);
            }
        }

        //connection in viewer frame
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                int Pj_u = int(-Pw_cameravertex_j.z() / fbl + w / 2.0);
                int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                if (Pj_u >= 0 && Pj_u < w && Pj_v >= 0 && Pj_v < h) {
                    cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 128, 128), 1);
                }

            }
        }


        //ALL MapPoint
        for (int MP_i = 0; MP_i < mpSystem->mpMap->mvpLocalMP.size(); MP_i++) {
            MapPoint *pMP = mpSystem->mpMap->mvpLocalMP[MP_i];
            if (pMP == NULL) {
                continue;
            }
            Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
            int Pi_u = int(-Pw.z() / fbl + w / 2.0);
            int Pi_v = int( Pw.y() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(128, 128, 0),-1);
            //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,pMP->mColor,-1);

        }
        //Tracked MapPoint
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {
            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
                int Pi_u = int(-Pw.z() / fbl + w / 2.0);
                int Pi_v = int( Pw.y() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 0),-1);
                //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,pMP->mColor,-1);

            }
        }

        //history

        for (int i = 0; i < Twbs.size(); i++) {
            Eigen::Matrix4f Twb_i = Twbs[i];
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;

            int Pi_u = int(-twb_i.z() / fbl + w / 2.0);
            int Pi_v = int( twb_i.y() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(30, 128, 30),-1);
        }
        {
//            cout << "pCurrentFrame->Gettwb() " << pCurrentFrame->Gettwb() << endl;
            Eigen::Matrix4f Twb_i = pCurrentFrame->PredictPose().inverse();
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;
//            cout << "pCurrentFrame->PredictPose() " << CommonTools::T2t(Twb_i) << endl;

            int Pi_u = int(-twb_i.z() / fbl + w / 2.0);
            int Pi_v = int( twb_i.y() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(255, 128, 30),-1);
        }

        //camera
        {
            float camera_size = 0.3;
            vector<vector<float>> camera_vertex = {
                    {0,                    0,            0},
                    {1.73f * camera_size,  camera_size,  camera_size},
                    {1.73f * camera_size,  -camera_size, camera_size},
                    {-1.73f * camera_size, camera_size,  camera_size},
                    {-1.73f * camera_size, -camera_size, camera_size}};
            for (int cam_i = 0; cam_i < pCurrentFrame->mNumCam; cam_i++) {

                Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
                Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
                Eigen::Matrix4f Twb = Tbw.inverse();
                Eigen::Matrix4f Twc = Twb * Tbc;
                for (int i = 0; i < camera_vertex.size(); i++) {
                    for (int j = i + 1; j < camera_vertex.size(); j++) {
                        Eigen::Vector3f Pc_cameravertex_i(camera_vertex[i][0], camera_vertex[i][1],
                                                          camera_vertex[i][2]);
                        Eigen::Vector3f Pc_cameravertex_j(camera_vertex[j][0], camera_vertex[j][1],
                                                          camera_vertex[j][2]);
                        Eigen::Vector3f Pw_cameravertex_i =
                                CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;
                        Eigen::Vector3f Pw_cameravertex_j =
                                CommonTools::T2R(Twc) * Pc_cameravertex_j + CommonTools::T2t(Twc) - t_w_viewer;

                        int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                        int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                        int Pj_u = int(-Pw_cameravertex_j.z() / fbl + w / 2.0);
                        int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                        cv::line(img_map,
                                 cv::Point(Pi_u, Pi_v),
                                 cv::Point(Pj_u, Pj_v),
                                 cv::Scalar(0, 255, 0), 2);
                    }
                }

                Eigen::Vector3f Pc_cameravertex_i(camera_vertex[0][0], camera_vertex[0][1], camera_vertex[0][2]);
                Eigen::Vector3f Pw_cameravertex_i =
                        CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;

                int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v), 4, cv::Scalar(128, 255, 128), -1);

            }
        }

        {
            //axis_z
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_z_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_z_j(0, 0, 0.75);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_z_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_z_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
            int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
            int Pj_u = int(-Pw_cameravertex_j.z() / fbl + w / 2.0);
            int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 0, 255), 4);
        }
        {
            //axis_x
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_x_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_x_j(0.75, 0, 0);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_x_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_x_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
            int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
            int Pj_u = int(-Pw_cameravertex_j.z() / fbl + w / 2.0);
            int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(255, 0, 0), 4);
        }


        {
            stringstream s;
            Eigen::Matrix4f Twb = pCurrentFrame->GetTwb();
            Eigen::Vector3f twb = CommonTools::T2t(Twb);
            s << " twc : " << twb.x() << " " << twb.y() << " " << twb.z();
            cv::putText(img_map, s.str(), cv::Point(5, img_map.rows * 0.95), cv::FONT_HERSHEY_PLAIN, 1,
                        cv::Scalar(255, 255, 255), 1, 8);
        }


        //////////////////////////////////////////////////////////////////////////
        //debug


        for (int MP_i = 0; MP_i < mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual.size(); MP_i++) {
            Eigen::Vector3f Pw = mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual[MP_i] - t_w_viewer;

            int Pi_u = int(-Pw.z() / fbl + w / 2.0);
            int Pi_v = int( Pw.y() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 255),-1);

        }






        //////////////////////////////////////////////////////////////////////////


        imShow.copyTo(img_map(cv::Rect(max(int(0), 0),
                                       max(int(0), 0),
                                       (min(int(imShow.cols), img_map.cols) - max(0, 0)),
                                       (min(int(imShow.rows), img_map.rows) - max(0, 0)))));

        // 将所有像素设置为白色（最高亮度值）
        cv::imshow("img_map_YZ", img_map);
//


        cv::waitKey(2);
    }


    void Viewer::VisualizationXY(const vector<cv::Mat> &vImCams, Frame *pCurrentFrame) {

        cv::Mat imShow = vImCams[0].clone();
        vector<cv::KeyPoint> vKeys = pCurrentFrame->mvCamKeysUn[0];
        for (int i = 0; i < vKeys.size(); i++) { if (vKeys[i].octave == 0) {cv::circle(imShow, cv::Point2f(vKeys[i].pt),2,cv::Scalar(0, 255, 0),-1);} }
//        cv::imshow("imShow", imShow);
//        cv::resize(imShow, imShow, cv::Size(320, 180));
        cv::resize(imShow, imShow, cv::Size(480, 270));
//        cv::imshow("imShow", imShow);

//        Eigen::Vector3f t_w_viewer = mpReferenceKF->Gettwb();
        Eigen::Vector3f t_w_viewer = pCurrentFrame->Gettwb();
        Twbs.push_back(pCurrentFrame->GetTwb());


        ///////visual
//        int w = 1000;
        int w = 2500;
        int h = 1400;
        float fbl = 0.03;
//        float fbl = 0.3;
        cv::Mat img_map = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
//      img_map.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);

        //connection
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(64, 64, 64), 1);
            }
        }

        //connection in viewer frame
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Matrix4f Twc = Twb * Tbc;
            int cols = pCurrentFrame->mvWidthHeight[cam_i].first;
            int rows = pCurrentFrame->mvWidthHeight[cam_i].second;

            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos();


                Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2t(Twc) - t_w_viewer;
                Eigen::Vector3f Pw_cameravertex_j = Pw - t_w_viewer;

                int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                if (Pj_u >= 0 && Pj_u < w && Pj_v >= 0 && Pj_v < h) {
                    cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 128, 128), 1);
                }

            }
        }


        //ALL MapPoint
        for (int MP_i = 0; MP_i < mpSystem->mpMap->mvpLocalMP.size(); MP_i++) {
            MapPoint *pMP = mpSystem->mpMap->mvpLocalMP[MP_i];
            if (pMP == NULL) {
                continue;
            }
            Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
            int Pi_u = int( Pw.x() / fbl + w / 2.0);
            int Pi_v = int( Pw.y() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(128, 128, 0),-1);
            //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,pMP->mColor,-1);

        }
        //Tracked MapPoint
        for (int cam_i = 0; cam_i < pCurrentFrame->mvMapPoints.size(); cam_i++) {
            for (int kpi = 0; kpi < pCurrentFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pCurrentFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                Eigen::Vector3f Pw = pMP->GetWorldPos() - t_w_viewer;
                int Pi_u = int( Pw.x() / fbl + w / 2.0);
                int Pi_v = int( Pw.y() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 0),-1);
                //cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,pMP->mColor,-1);

            }
        }

        //history

        for (int i = 0; i < Twbs.size(); i++) {
            Eigen::Matrix4f Twb_i = Twbs[i];
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;

            int Pi_u = int( twb_i.x() / fbl + w / 2.0);
            int Pi_v = int( twb_i.y() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(30, 128, 30),-1);
        }
        {
//            cout << "pCurrentFrame->Gettwb() " << pCurrentFrame->Gettwb() << endl;
            Eigen::Matrix4f Twb_i = pCurrentFrame->PredictPose().inverse();
            Eigen::Vector3f twb_i = CommonTools::T2t(Twb_i) - t_w_viewer;
//            cout << "pCurrentFrame->PredictPose() " << CommonTools::T2t(Twb_i) << endl;

            int Pi_u = int( twb_i.x() / fbl + w / 2.0);
            int Pi_v = int( twb_i.y() / fbl + h / 2.0);

            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),5,cv::Scalar(255, 128, 30),-1);
        }

        //camera
        {
            float camera_size = 0.3;
            vector<vector<float>> camera_vertex = {
                    {0,                    0,            0},
                    {1.73f * camera_size,  camera_size,  camera_size},
                    {1.73f * camera_size,  -camera_size, camera_size},
                    {-1.73f * camera_size, camera_size,  camera_size},
                    {-1.73f * camera_size, -camera_size, camera_size}};
            for (int cam_i = 0; cam_i < pCurrentFrame->mNumCam; cam_i++) {

                Eigen::Matrix4f Tbc = pCurrentFrame->mvTbc_cams[cam_i];
                Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
                Eigen::Matrix4f Twb = Tbw.inverse();
                Eigen::Matrix4f Twc = Twb * Tbc;
                for (int i = 0; i < camera_vertex.size(); i++) {
                    for (int j = i + 1; j < camera_vertex.size(); j++) {
                        Eigen::Vector3f Pc_cameravertex_i(camera_vertex[i][0], camera_vertex[i][1],
                                                          camera_vertex[i][2]);
                        Eigen::Vector3f Pc_cameravertex_j(camera_vertex[j][0], camera_vertex[j][1],
                                                          camera_vertex[j][2]);
                        Eigen::Vector3f Pw_cameravertex_i =
                                CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;
                        Eigen::Vector3f Pw_cameravertex_j =
                                CommonTools::T2R(Twc) * Pc_cameravertex_j + CommonTools::T2t(Twc) - t_w_viewer;

                        int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
                        int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                        int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
                        int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);

                        cv::line(img_map,
                                 cv::Point(Pi_u, Pi_v),
                                 cv::Point(Pj_u, Pj_v),
                                 cv::Scalar(0, 255, 0), 2);
                    }
                }

                Eigen::Vector3f Pc_cameravertex_i(camera_vertex[0][0], camera_vertex[0][1], camera_vertex[0][2]);
                Eigen::Vector3f Pw_cameravertex_i =
                        CommonTools::T2R(Twc) * Pc_cameravertex_i + CommonTools::T2t(Twc) - t_w_viewer;

                int Pi_u = int(-Pw_cameravertex_i.z() / fbl + w / 2.0);
                int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
                cv::circle(img_map, cv::Point2f(Pi_u, Pi_v), 4, cv::Scalar(128, 255, 128), -1);

            }
        }

        {
            //axis_z
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_z_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_z_j(0, 0, 0.75);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_z_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_z_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
            int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
            int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
            int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(0, 0, 255), 4);
        }
        {
            //axis_x
            Eigen::Matrix4f Tbw = pCurrentFrame->GetPose();
            Eigen::Matrix4f Twb = Tbw.inverse();
            Eigen::Vector3f Pc_axis_x_i(0, 0, 0);
            Eigen::Vector3f Pc_axis_x_j(0.75, 0, 0);
            Eigen::Vector3f Pw_cameravertex_i = CommonTools::T2R(Twb) * Pc_axis_x_i + CommonTools::T2t(Twb) - t_w_viewer;
            Eigen::Vector3f Pw_cameravertex_j = CommonTools::T2R(Twb) * Pc_axis_x_j + CommonTools::T2t(Twb) - t_w_viewer;
            int Pi_u = int( Pw_cameravertex_i.x() / fbl + w / 2.0);
            int Pi_v = int( Pw_cameravertex_i.y() / fbl + h / 2.0);
            int Pj_u = int( Pw_cameravertex_j.x() / fbl + w / 2.0);
            int Pj_v = int( Pw_cameravertex_j.y() / fbl + h / 2.0);
            cv::line(img_map, cv::Point(Pi_u, Pi_v), cv::Point(Pj_u, Pj_v), cv::Scalar(255, 0, 0), 4);
        }


        {
            stringstream s;
            Eigen::Matrix4f Twb = pCurrentFrame->GetTwb();
            Eigen::Vector3f twb = CommonTools::T2t(Twb);
            s << " twc : " << twb.x() << " " << twb.y() << " " << twb.z();
            cv::putText(img_map, s.str(), cv::Point(5, img_map.rows * 0.95), cv::FONT_HERSHEY_PLAIN, 1,
                        cv::Scalar(255, 255, 255), 1, 8);
        }


        //////////////////////////////////////////////////////////////////////////
        //debug


        for (int MP_i = 0; MP_i < mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual.size(); MP_i++) {
            Eigen::Vector3f Pw = mpSystem->mpTracker->mpReferenceKF->mvMapPointPositionVisual[MP_i] - t_w_viewer;

            int Pi_u = int( Pw.x() / fbl + w / 2.0);
            int Pi_v = int( Pw.y() / fbl + h / 2.0);
            cv::circle(img_map, cv::Point2f(Pi_u, Pi_v),2,cv::Scalar(255, 255, 255),-1);

        }






        //////////////////////////////////////////////////////////////////////////


        imShow.copyTo(img_map(cv::Rect(max(int(0), 0),
                                       max(int(0), 0),
                                       (min(int(imShow.cols), img_map.cols) - max(0, 0)),
                                       (min(int(imShow.rows), img_map.rows) - max(0, 0)))));

        // 将所有像素设置为白色（最高亮度值）
        cv::imshow("img_map_XY", img_map);
//


        cv::waitKey(2);
    }


















} //namespace ORB_SLAM
