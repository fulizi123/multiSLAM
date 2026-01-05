
#include "Frame.h"


namespace LL_SLAM
{

    long unsigned int Frame::nNextId=0;
//    int Frame::FRAME_GRID_ROWS = 48;
//    int Frame::FRAME_GRID_COLS = 64;
    int Frame::FRAME_GRID_ROWS = 72;
    int Frame::FRAME_GRID_COLS = 128;

    Frame::Frame(const vector<cv::Mat> &vImColorCams, const vector<cv::Mat> &vImCams, vector<vector<vector<int>>> * pvKeyPoints, vector<vector<vector<float>>> * pvDescriptor, const double &timeStamp,
            vector<FeatureExtractor*> vextractors, System *pSystem)
    {
        mTimeStamp = timeStamp;

        //cout << "Debug Frame 1" << endl;
        mpSystem = pSystem;
        mNumCam = pSystem->mNumCam;
        mvTbc_cams = pSystem->mvTbc_cams;
        mvK_cams = pSystem->mvK_cams;
        mSettings = pSystem->mSettings;
        pPreFrame = pSystem->mpTracker->pPreFrame;

        //cout << "Debug Frame 2" << endl;

        mvpFeatureExtractors.resize(mNumCam, NULL);
        for (int imCami = 0; imCami < mNumCam; imCami++) {
            mvpFeatureExtractors[imCami] = vextractors[imCami];
        }
        mvScaleFactors.resize(mNumCam);
        for (int imCami = 0; imCami < mNumCam; imCami++) {
            mvScaleFactors[imCami] = mvpFeatureExtractors[imCami]->GetScaleFactors();
        }

        mvNCams.resize(mNumCam, 0);
        mvCamKeys.resize(mNumCam);
        mvCamKeysUn.resize(mNumCam);
        mvCamDescriptors.resize(mNumCam);
        mvWidthHeight.resize(mNumCam);


        // Frame ID
        mnId=nNextId++;
        //DistCoef
        mDistCoef = cv::Mat::zeros(4,1,CV_32F);
        mDistCoef.at<float>(0) = mSettings["Camera.k1"].real();
        mDistCoef.at<float>(1) = mSettings["Camera.k2"].real();
        mDistCoef.at<float>(2) = mSettings["Camera.p1"].real();
        mDistCoef.at<float>(3) = mSettings["Camera.p2"].real();

        for (int imCami = 0; imCami < mNumCam; imCami++) {
            mvWidthHeight[imCami] = {vImCams[imCami].cols, vImCams[imCami].rows};
        }

        if ((*pvKeyPoints).size() == mNumCam) {
            //Use SuperPoint
            mbUseDesc = true;
            //todo take 11 ms
//            mvSuperPointKeyPoints = vKeyPoints;
//            mvSuperPointDescriptor = vDescriptor;
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            /////////////////
//            for (int imCami = 0; imCami < mNumCam; imCami++) {
//                Frame::ExtractORBMultiCameraSuperPoint(imCami,vImCams[imCami],mvpFeatureExtractors[imCami],vKeyPoints[imCami],vDescriptor[imCami]);
//            }
//            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
//            /////////////////
            vector<thread> vthreads;
            vthreads.reserve(mNumCam);
            for (int imCami = 0; imCami < mNumCam; imCami++) {
                vthreads.emplace_back(&Frame::ExtractORBMultiCameraSuperPoint,this,imCami,vImCams[imCami],mvpFeatureExtractors[imCami], pvKeyPoints, pvDescriptor);
            }
//            /////////////////
//            std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
//            cout << "vthreads.emplace_back use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count() * 1000.0 << " ms." << endl;

            for (int imCami = 0; imCami < mNumCam; imCami++) {
                vthreads[imCami].join();
            }
            /////////////////
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            cout << "ExtractORBMultiCameraSuperPoint use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;

        } else {
            //Use ORB
            cout << "ExtractORB : " << endl;
            vector<thread> vthreads;
            vthreads.reserve(mNumCam);
            for (int imCami = 0; imCami < mNumCam; imCami++) {
                vthreads.emplace_back(&Frame::ExtractORBMultiCamera,this,imCami,vImCams[imCami],mvpFeatureExtractors[imCami]);
            }

            //cout << "Debug Frame 6" << endl;
            for (int imCami = 0; imCami < mNumCam; imCami++) {
                vthreads[imCami].join();
            }

        }


        for (int imCami = 0; imCami < mNumCam; imCami++) {
            mvNCams[imCami] = mvCamKeys[imCami].size();
        }

        cout << "mvCamKeys[imCami].size() " ;
        for (int imCami = 0; imCami < mNumCam; imCami++) {
//            cout << " imCami " << imCami
//            << " mvCamKeys[imCami].size() " << mvCamKeys[imCami].size()
//            << "  mvCamDescriptors[imCami].shape " << mvCamDescriptors[imCami].rows << " "  << mvCamDescriptors[imCami].cols << endl;
            cout << " " << mvCamKeys[imCami].size();
        }
        cout << endl;

        mvMapPoints.resize(mNumCam);
        for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
            mvMapPoints[cam_i].resize(mvNCams[cam_i], NULL);
        }


        mvColor.resize(mNumCam);
        for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
            mvColor[cam_i].resize(mvNCams[cam_i], cv::Vec3b(0, 0, 0));
            for (int kpi = 0; kpi < mvNCams[cam_i]; kpi++) {
                int u = mvCamKeys[cam_i][kpi].pt.x;
                int v = mvCamKeys[cam_i][kpi].pt.y;
                mvColor[cam_i][kpi] = vImColorCams[cam_i].at<cv::Vec3b>(v, u) ;
            }
        }




//        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        /////////////////
//cout << " UndistortKeyPointsMultiCamera 1" ;
//usleep(1000*1000);
        UndistortKeyPointsMultiCamera();
//cout << " UndistortKeyPointsMultiCamera 2" ;
//usleep(1000*1000);
        /////////////////
//        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//        if (mpSystem->mbUseTime) {cout << "UndistortKeyPointsMultiCamera use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;}


//        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        /////////////////
//cout << " ComputeMultiCameraMatches 1" ;
//usleep(1000*1000);
//        ComputeMultiCameraMatches();
//cout << " ComputeMultiCameraMatches 2" ;
//usleep(1000*1000);
        /////////////////
//        std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
//        if (mpSystem->mbUseTime) {cout << "ComputeMultiCameraMatches use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count() * 1000.0 << " ms." << endl;}

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        /////////////////
        mvGridMultiCamera = vector<vector<vector<vector<size_t>>>>(mNumCam,
                vector<vector<vector<size_t>>>(FRAME_GRID_COLS,
                vector<vector<size_t>>(FRAME_GRID_ROWS)));
        AssignFeaturesToGridMultiCamera(vImCams);
        /////////////////
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        cout << "AssignFeaturesToGridMultiCamera use time : " << std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count() * 1000.0 << " ms." << endl;


    }

    void Frame::ExtractORBMultiCamera(int Camid, const cv::Mat &im, FeatureExtractor* extractor)
    {
        vector<int> vLapping = {0,0};
        (*extractor)(im,cv::Mat(),mvCamKeys[Camid],mvCamDescriptors[Camid], vLapping);
    }

    void Frame::ExtractORBMultiCameraSuperPoint(int Camid, const cv::Mat &im, FeatureExtractor* extractor,
                                                vector<vector<vector<int>>> * vKeyPoints, vector<vector<vector<float>>> * vDescriptor)
    {
        vector<int> vLapping = {0,0};
        (*extractor)(im,cv::Mat(),mvCamKeys[Camid],mvCamDescriptors[Camid], vLapping, (*vKeyPoints)[Camid], (*vDescriptor)[Camid]);

    }


    void Frame::UndistortKeyPointsMultiCamera() {
//        cout << " UndistortKeyPointsMultiCamera 1.1" ;
//        usleep(1000*1000);
        for (int cam_i = 0; cam_i < mNumCam; cam_i++) {

            if (mDistCoef.at<float>(0) == 0.0) {
//                mvCamKeysUn[cam_i] = mvCamKeys[cam_i];

                mvCamKeysUn[cam_i].resize(mvNCams[cam_i]);
                for (int i = 0; i < mvNCams[cam_i]; i++) {
                    cv::KeyPoint kp = mvCamKeys[cam_i][i];
                    mvCamKeysUn[cam_i][i] = kp;
                }
                continue;
            }
            // Fill matrix with points
            cv::Mat mat(mvNCams[cam_i], 2, CV_32F);

            for (int i = 0; i < mvNCams[cam_i]; i++) {
                mat.at<float>(i, 0) = mvCamKeys[cam_i][i].pt.x;
                mat.at<float>(i, 1) = mvCamKeys[cam_i][i].pt.y;
            }

            // Undistort points
            mat = mat.reshape(2);
            //cv::undistortPoints(mat, mat, mvK_cams[cam_i], mDistCoef, cv::Mat(), mvK_cams[cam_i]);
            cv::undistortPoints(mat, mat, CommonTools::toCvMat(mvK_cams[cam_i]), mDistCoef);
            mat = mat.reshape(1);


            // Fill undistorted keypoint vector
            mvCamKeysUn[cam_i].resize(mvNCams[cam_i]);
            for (int i = 0; i < mvNCams[cam_i]; i++) {
                cv::KeyPoint kp = mvCamKeys[cam_i][i];
                kp.pt.x = mat.at<float>(i, 0);
                kp.pt.y = mat.at<float>(i, 1);
                mvCamKeysUn[cam_i][i] = kp;
            }


        }
    }





    void Frame::AssignFeaturesToGridMultiCamera(const vector<cv::Mat> &vImCams){

        for (int imCami = 0; imCami < mNumCam; imCami++) {
            int mnMinX = 0;
            int mnMinY = 0;
            int mnMaxX = vImCams[imCami].cols;
            int mnMaxY = vImCams[imCami].rows;
            float mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
            float mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
            // Fill matrix with points mvNCams[0]
            const int nCells = FRAME_GRID_COLS*FRAME_GRID_ROWS;

            int nReserve = 0.5f*mvNCams[imCami]/(nCells);

            for(unsigned int i=0; i<FRAME_GRID_COLS;i++){
                for (unsigned int j=0; j<FRAME_GRID_ROWS;j++){
                    mvGridMultiCamera[imCami][i][j].reserve(nReserve);

                }
            }


            for(int i=0;i<mvNCams[imCami];i++)
            {
                const cv::KeyPoint &kp = mvCamKeys[imCami][i];

                int nGridPosX, nGridPosY;
                nGridPosX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
                nGridPosY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);
                
                if(nGridPosX >=0 && nGridPosX < FRAME_GRID_COLS && nGridPosY >= 0 && nGridPosY < FRAME_GRID_ROWS){
                    mvGridMultiCamera[imCami][nGridPosX][nGridPosY].push_back(i);
                }
            }


        }

    }

    void Frame::ComputeMultiCameraMatches()
    {

        vector<vector<int>> MatchResult(mNumCam);
        for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
            MatchResult[cam_i].resize(mvNCams[cam_i], -1);
        }

        vector<vector<float>> MatchPosition;
        MatchPosition.reserve(20000);
        int num_matches = 0;

        for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
            // cout << "Stereo " << "cam_i" << " " << cam_i << endl;

            Eigen::Matrix4f Tbc_cami = mvTbc_cams[cam_i];
            Eigen::Vector3f tbc_cami = CommonTools::T2t(Tbc_cami);
            Eigen::Matrix<float,3,3> Rbc_cami = CommonTools::T2R(Tbc_cami);
            Eigen::Matrix3f K_camsi = mvK_cams[cam_i];
            vector<cv::KeyPoint> &vCamKeysCami = mvCamKeys[cam_i];
            cv::Mat &CamDescriptorsCami = mvCamDescriptors[cam_i];

            for (int cam_j = 0; cam_j < mNumCam; cam_j++) {

                // cout << "Stereo " << "cam_j" << " " << cam_j << endl;

                Eigen::Matrix4f Tbc_camj = mvTbc_cams[cam_j];
                Eigen::Matrix3f K_camsj = mvK_cams[cam_j];
                vector<cv::KeyPoint> &vCamKeysCamj = mvCamKeys[cam_j];
                cv::Mat &CamDescriptorsCamj = mvCamDescriptors[cam_j];

                Eigen::Matrix4f T_cami_camj = Tbc_cami.inverse() * Tbc_camj;
                Eigen::Vector3f t_cami_camj = CommonTools::T2t(T_cami_camj);
                Eigen::Matrix<float,3,3> R_cami_camj = CommonTools::T2R(T_cami_camj);

                //baseline is big enough
                if (t_cami_camj.x() <= 0.04) {
                    continue;
                }
                //stereo
                if (t_cami_camj.y() >= 0.01 || t_cami_camj.z() >= 0.01) {
                    continue;
                }
                Eigen::Vector3f n_camj(0.0, 0.0, 1.0);
                if (acos((n_camj.transpose() * (R_cami_camj * n_camj))(0, 0)) >= 2.0 / 180.0 * 3.14) {
                    continue;
                }
//
//                cout << "Stereo " << cam_i << " " << cam_j << endl;
//                cout << "T_cami_camj " << endl;
//                cout << T_cami_camj << endl;
//                cout << endl;
                //continue;



                const int thOrbDist = (FeatureMatcher::TH_HIGH+FeatureMatcher::TH_LOW)/2;
                const int nRows = mvpFeatureExtractors[cam_i]->mvImagePyramid[0].rows;

                vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());
                for(int i=0; i<nRows; i++)
                    vRowIndices[i].reserve(200);

                const int Nl = vCamKeysCami.size();
                const int Nr = vCamKeysCamj.size();

                for(int iR=0; iR<Nr; iR++)
                {
                    const cv::KeyPoint &kp = vCamKeysCamj[iR];
                    const float &kpY = kp.pt.y;
                    const float r = 2.0f*mvScaleFactors[cam_j][vCamKeysCamj[iR].octave];
                    const int maxr = min(static_cast<int>(ceil(kpY+r)), nRows - 1);
                    const int minr = max(static_cast<int>(floor(kpY-r)), 0);

                    for(int yi=minr;yi<=maxr;yi++)
                        vRowIndices[yi].push_back(iR);
                }

                const float minZ = t_cami_camj.x();
//                const float minD = 0;
                const float minD = t_cami_camj.x() * K_camsi(0, 0) /(80 * minZ);
                const float maxD = t_cami_camj.x() * K_camsi(0, 0) /minZ;

                //cout << "Debug Frame For each left keypoint " << endl;
                // For each left keypoint search a match in the right image
                vector<pair<int, int> > vDistIdx;
                vDistIdx.reserve(Nl);

                for(int iL=0; iL<Nl; iL++)
                {
                    const cv::KeyPoint &kpL = vCamKeysCami[iL];
                    const int &levelL = kpL.octave;
                    const float &vL = kpL.pt.y;
                    const float &uL = kpL.pt.x;

                    const vector<size_t> &vCandidates = vRowIndices[vL];

                    if(vCandidates.empty())
                        continue;

                    const float minU = uL-maxD;
                    const float maxU = uL-minD;

                    if(maxU<0)
                        continue;

                    int bestDist = FeatureMatcher::TH_HIGH;
                    size_t bestIdxR = 0;

                    const cv::Mat &dL = CamDescriptorsCami.row(iL);

                    //cout << "Debug Frame Compare descriptor to right keypoints" << endl;
                    // Compare descriptor to right keypoints
                    for(size_t iC=0; iC<vCandidates.size(); iC++)
                    {
                        const size_t iR = vCandidates[iC];
                        const cv::KeyPoint &kpR = vCamKeysCamj[iR];

                        if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                            continue;

                        const float &uR = kpR.pt.x;

                        if(uR>=minU && uR<=maxU)
                        {
                            const cv::Mat &dR = CamDescriptorsCamj.row(iR);
                            const int dist = FeatureMatcher::DescriptorDistance(dL,dR);

                            if(dist<bestDist)
                            {
                                bestDist = dist;
                                bestIdxR = iR;
                            }
                        }
                    }

                    // cout << "Debug Frame ComputeStereoMatches Subpixel" << endl;
                    // Subpixel match by correlation
                    if(bestDist<thOrbDist)
                    {
                        int iR = bestIdxR;
                        const float uR0 = vCamKeysCamj[bestIdxR].pt.x;

                        float disparity = (uL-uR0);

                        if(disparity>=minD && disparity<maxD) {
                            if (disparity <= 0) {
                                disparity = 0.01;
                            }
                            float depth = t_cami_camj.x() * K_camsi(0, 0) / disparity;
                            Eigen::Vector3f Pb = Rbc_cami * (depth * K_camsi.inverse() * Eigen::Vector3f(uL, vL, 1))
                                                 + tbc_cami;

                            if (MatchResult[cam_i][iL] >= 0) {
                                //left matches
                                MatchResult[cam_j][iR] = MatchResult[cam_i][iL];
                            } else if (MatchResult[cam_j][iR] >= 0) {
                                //right matches
                                MatchResult[cam_i][iL] = MatchResult[cam_j][iR];
                            } else {
                                //nomatches

                                MatchResult[cam_j][iR] = num_matches;
                                MatchResult[cam_i][iL] = num_matches;
                                MatchPosition.push_back({Pb.x(), Pb.y(), Pb.z()});
                                num_matches++;
                            }
                        }


                    }
                }



            }

        }

        mvMapPointPosition.resize(num_matches);
        for (int i = 0; i < num_matches; i++) {
            mvMapPointPosition[i] = Eigen::Vector3f(MatchPosition[i][0],
                                                    MatchPosition[i][1],
                                                    MatchPosition[i][2]);
        }




        cout << "Debug Frame num_matches " << num_matches << endl;
        mvMatchResult = MatchResult;



    }

//
//    void Frame::ComputeMultiCameraMatches()
//    {
//
//        vector<vector<int>> MatchResult(mNumCam);
//        for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
//            MatchResult[cam_i].resize(mvNCams[cam_i], -1);
//        }
//
//        vector<vector<float>> MatchPosition;
//        MatchPosition.reserve(20000);
//        int num_matches = 0;
//
//        for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
//            // cout << "Stereo " << "cam_i" << " " << cam_i << endl;
//
//            Eigen::Matrix4f Tbc_cami = mvTbc_cams[cam_i];
//            Eigen::Vector3f tbc_cami = CommonTools::T2t(Tbc_cami);
//            Eigen::Matrix<float,3,3> Rbc_cami = CommonTools::T2R(Tbc_cami);
//            Eigen::Matrix3f K_camsi = mvK_cams[cam_i];
//            vector<cv::KeyPoint> vCamKeysCami = mvCamKeys[cam_i];
//            cv::Mat CamDescriptorsCami = mvCamDescriptors[cam_i];
//
//            for (int cam_j = 0; cam_j < mNumCam; cam_j++) {
//
//                // cout << "Stereo " << "cam_j" << " " << cam_j << endl;
//
//                Eigen::Matrix4f Tbc_camj = mvTbc_cams[cam_j];
//                Eigen::Matrix3f K_camsj = mvK_cams[cam_j];
//                vector<cv::KeyPoint> vCamKeysCamj = mvCamKeys[cam_j];
//                cv::Mat CamDescriptorsCamj = mvCamDescriptors[cam_j];
//
//                Eigen::Matrix4f T_cami_camj = Tbc_cami.inverse() * Tbc_camj;
//                Eigen::Vector3f t_cami_camj = CommonTools::T2t(T_cami_camj);
//                Eigen::Matrix<float,3,3> R_cami_camj = CommonTools::T2R(T_cami_camj);
//
//                //baseline is big enough
//                if (t_cami_camj.x() <= 0.04) {
//                    continue;
//                }
//                //stereo
//                if (t_cami_camj.y() >= 0.01 || t_cami_camj.z() >= 0.01) {
//                    continue;
//                }
//                Eigen::Vector3f n_camj(0.0, 0.0, 1.0);
//                if (acos((n_camj.transpose() * (R_cami_camj * n_camj))(0, 0)) >= 2.0 / 180.0 * 3.14) {
//                    continue;
//                }
//    //
//    //                cout << "Stereo " << cam_i << " " << cam_j << endl;
//    //                cout << "T_cami_camj " << endl;
//    //                cout << T_cami_camj << endl;
//    //                cout << endl;
//                //continue;
//
//
//
//                const int thOrbDist = (FeatureMatcher::TH_HIGH+FeatureMatcher::TH_LOW)/2;
//                const int nRows = mvpFeatureExtractors[cam_i]->mvImagePyramid[0].rows;
//
//                vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());
//                for(int i=0; i<nRows; i++)
//                    vRowIndices[i].reserve(200);
//
//                const int Nl = vCamKeysCami.size();
//                const int Nr = vCamKeysCamj.size();
//
//                for(int iR=0; iR<Nr; iR++)
//                {
//                    const cv::KeyPoint &kp = vCamKeysCamj[iR];
//                    const float &kpY = kp.pt.y;
//                    const float r = 2.0f*mvScaleFactors[cam_j][vCamKeysCamj[iR].octave];
//                    const int maxr = min(static_cast<int>(ceil(kpY+r)), nRows - 1);
//                    const int minr = max(static_cast<int>(floor(kpY-r)), 0);
//
//                    for(int yi=minr;yi<=maxr;yi++)
//                        vRowIndices[yi].push_back(iR);
//                }
//
//                const float minZ = t_cami_camj.x();
//    //                const float minD = 0;
//                const float minD = t_cami_camj.x() * K_camsi(0, 0) /(80 * minZ);
//                const float maxD = t_cami_camj.x() * K_camsi(0, 0) /minZ;
//
//                //cout << "Debug Frame For each left keypoint " << endl;
//                // For each left keypoint search a match in the right image
//                vector<pair<int, int> > vDistIdx;
//                vDistIdx.reserve(Nl);
//
//                for(int iL=0; iL<Nl; iL++)
//                {
//                    const cv::KeyPoint &kpL = vCamKeysCami[iL];
//                    const int &levelL = kpL.octave;
//                    const float &vL = kpL.pt.y;
//                    const float &uL = kpL.pt.x;
//
//                    const vector<size_t> &vCandidates = vRowIndices[vL];
//
//                    if(vCandidates.empty())
//                        continue;
//
//                    const float minU = uL-maxD;
//                    const float maxU = uL-minD;
//
//                    if(maxU<0)
//                        continue;
//
//                    int bestDist = FeatureMatcher::TH_HIGH;
//                    size_t bestIdxR = 0;
//
//                    const cv::Mat &dL = CamDescriptorsCami.row(iL);
//
//                    //cout << "Debug Frame Compare descriptor to right keypoints" << endl;
//                    // Compare descriptor to right keypoints
//                    for(size_t iC=0; iC<vCandidates.size(); iC++)
//                    {
//                        const size_t iR = vCandidates[iC];
//                        const cv::KeyPoint &kpR = vCamKeysCamj[iR];
//
//                        if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
//                            continue;
//
//                        const float &uR = kpR.pt.x;
//
//                        if(uR>=minU && uR<=maxU)
//                        {
//                            const cv::Mat &dR = CamDescriptorsCamj.row(iR);
//                            const int dist = FeatureMatcher::DescriptorDistance(dL,dR);
//
//                            if(dist<bestDist)
//                            {
//                                bestDist = dist;
//                                bestIdxR = iR;
//                            }
//                        }
//                    }
//
//                    // cout << "Debug Frame ComputeStereoMatches Subpixel" << endl;
//                    // Subpixel match by correlation
//                    if(bestDist<thOrbDist)
//                    {
//                        // coordinates in image pyramid at keypoint scale
//                        const float uR0 = vCamKeysCamj[bestIdxR].pt.x;
//                        const float scaleFactor = 1.0 / mvScaleFactors[cam_i][kpL.octave];
//                        const float scaleduL = round(kpL.pt.x*scaleFactor);
//                        const float scaledvL = round(kpL.pt.y*scaleFactor);
//                        const float scaleduR0 = round(uR0*scaleFactor);
//
//                        // sliding window search
//                        const int w = 5;
//                        //FeatureExtractor
//                        const int PATCH_SIZE = 31;
//                        const int HALF_PATCH_SIZE = 15;
//                        const int EDGE_THRESHOLD = 19;
//                        cv::Mat IL = mvpFeatureExtractors[cam_i]->mvImagePyramidWholeSize[kpL.octave].rowRange(scaledvL-w+EDGE_THRESHOLD,scaledvL+w+1+EDGE_THRESHOLD).colRange(scaleduL-w+EDGE_THRESHOLD,scaleduL+w+1+EDGE_THRESHOLD);
//                        //cv::Mat IL = mpFeatureExtractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
//
//                        int bestDist = INT_MAX;
//                        int bestincR = 0;
//                        const int L = 5;
//                        vector<float> vDists;
//                        vDists.resize(2*L+1);
//
//                        const float iniu = scaleduR0+L-w;
//                        const float endu = scaleduR0+L+w+1;
//                        if(iniu<0 || endu >= mvpFeatureExtractors[cam_j]->mvImagePyramid[kpL.octave].cols)
//                            continue;
//
//                        for(int incR=-L; incR<=+L; incR++)
//                        {
//                            cv::Mat IR = mvpFeatureExtractors[cam_j]->mvImagePyramidWholeSize[kpL.octave].rowRange(scaledvL-w+EDGE_THRESHOLD,scaledvL+w+1+EDGE_THRESHOLD).colRange(scaleduR0+incR-w+EDGE_THRESHOLD,scaleduR0+incR+w+1+EDGE_THRESHOLD);
//                            //cv::Mat IR = mpFeatureExtractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
//
//                            float dist = cv::norm(IL,IR,cv::NORM_L1);
//                            if(dist<bestDist)
//                            {
//                                bestDist =  dist;
//                                bestincR = incR;
//                            }
//
//                            vDists[L+incR] = dist;
//                        }
//
//                        if(bestincR==-L || bestincR==L)
//                            continue;
//
//                        // Sub-pixel match (Parabola fitting)
//                        const float dist1 = vDists[L+bestincR-1];
//                        const float dist2 = vDists[L+bestincR];
//                        const float dist3 = vDists[L+bestincR+1];
//
//                        const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
//
//                        if(deltaR<-1 || deltaR>1)
//                            continue;
//
//                        //cout << "Debug Frame Re-scaled coordinate" << endl;
//                        // Re-scaled coordinate
//                        float bestuR = mvScaleFactors[cam_j][kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
//
//                        float disparity = (uL-bestuR);
//
//                        if(disparity>=minD && disparity<maxD)
//                        {
//                            if(disparity<=0)
//                            {
//                                disparity=0.01;
//                                bestuR = uL-0.01;
//                            }
//                            float depth = t_cami_camj.x() * K_camsi(0, 0)/disparity;
//                            Eigen::Vector3f Pb = Rbc_cami * (depth * K_camsi.inverse() * Eigen::Vector3f(uL ,vL , 1))
//                                                 + tbc_cami;
//
//
//
//                            int iR = bestIdxR;
//
//                            if (MatchResult[cam_i][iL] >=0 ) {
//                                //left matches
//                                MatchResult[cam_j][iR] = MatchResult[cam_i][iL];
//                            } else if (MatchResult[cam_j][iR] >=0 ) {
//                                //right matches
//                                MatchResult[cam_i][iL] = MatchResult[cam_j][iR];
//                            } else {
//                                //nomatches
//
//                                MatchResult[cam_j][iR] = num_matches;
//                                MatchResult[cam_i][iL] = num_matches;
//                                MatchPosition.push_back({Pb.x(), Pb.y(), Pb.z()});
//                                num_matches++;
//                            }
//
//
//
//
//                        }
//                        //cout << "Debug Frame Re-scaled coordinate end" << endl;
//                    }
//                }
//
//
//
//            }
//
//        }
//
//        mvMapPointPosition.resize(num_matches);
//        for (int i = 0; i < num_matches; i++) {
//            mvMapPointPosition[i] = Eigen::Vector3f(MatchPosition[i][0],
//                                                    MatchPosition[i][1],
//                                                    MatchPosition[i][2]);
//        }
//
//
//
//    //
//    //
//    //        ///////visual
//    //        // int w = 200;
//    //        // int h = 200;
//    //        // float fbl = 1.0;
//    //        int w = 1000;
//    //        int h = 1000;
//    //        float fbl = 0.1;
//    //        cv::Mat img_stereo(cv::Size(w, h), CV_8UC1);
//    //        for (int i = 0; i < h; i++) {
//    //            for (int j = 0; j < w; j++) {
//    //                img_stereo.at<uchar>(j, i) = 0;
//    //            }
//    //        }
//    //        cout << "Debug Frame num_matches " << num_matches << endl;
//    //        for (int i = 0; i < num_matches; i++) {
//    //            Eigen::Vector3f Pb(MatchPosition[i][0], MatchPosition[i][1], MatchPosition[i][2]);
//    //            int Pb_u = int(Pb.x() / fbl + w / 2.0);
//    //            int Pb_v = int(Pb.z() / fbl + h / 2.0);
//    //            if (Pb_u < 0 || Pb_u >= w || Pb_v < 0 || Pb_v >=h) {
//    //                continue;
//    //            }
//    ////            if (Pb.y() >= -2.0) {
//    ////                continue;
//    ////            }
//    //            int color_temp = max(min(int((Pb.y() + 10) * 25), 255), 0);
//    //            cv::circle(img_stereo, cv::Point2f(Pb_u, Pb_v),2,cv::Scalar(color_temp, color_temp, color_temp),-1);
//    //            //img_stereo.at<uchar>(Pb_v,Pb_u )+=20;
//    //        }
//    //
//    //        // 将所有像素设置为白色（最高亮度值）
//    //        cv::imshow("img_stereo", img_stereo);
//    //        cv::imwrite("img_stereo.png", img_stereo);
//    //        cv::waitKey(0);
//    ////
//
//        cout << "Debug Frame num_matches " << num_matches << endl;
//        mvMatchResult = MatchResult;
//
//
//
//    }

    vector<int> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int  &imCami, const int minLevel, const int maxLevel) const
    {
        int mnMinX = 0;
        int mnMinY = 0;
        int mnMaxX = mvWidthHeight[imCami].first;
        int mnMaxY = mvWidthHeight[imCami].second;
        float mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        float mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        vector<int> vIndices;
        //vIndices.reserve(mvNCams[imCami]);
        vIndices.reserve(20);

        float factorX = r;
        float factorY = r;

        const int nMinCellX = max(0,(int)floor((x-mnMinX-factorX)*mfGridElementWidthInv));
        if(nMinCellX>=FRAME_GRID_COLS)
        {
            return vIndices;
        }

        const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+factorX)*mfGridElementWidthInv));
        if(nMaxCellX<0)
        {
            return vIndices;
        }

        const int nMinCellY = max(0,(int)floor((y-mnMinY-factorY)*mfGridElementHeightInv));
        if(nMinCellY>=FRAME_GRID_ROWS)
        {
            return vIndices;
        }

        const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+factorY)*mfGridElementHeightInv));
        if(nMaxCellY<0)
        {
            return vIndices;
        }

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mvGridMultiCamera[imCami][ix][iy];
                if(vCell.empty())
                    continue;

                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvCamKeysUn[imCami][vCell[j]];
                    if(bCheckLevels)
                    {
                        if(kpUn.octave<minLevel)
                            continue;
                        if(maxLevel>=0)
                            if(kpUn.octave>maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<factorX && fabs(disty)<factorY)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }




    void Frame::SetPose(const Eigen::Matrix4f &Tbw) {
//        cout << "Debug Frame::SetPose" << endl;
//        usleep(1000*1000);
//        mTbw = Tbw;
        mTbw << Tbw(0,0), Tbw(0,1), Tbw(0,2), Tbw(0,3),
                Tbw(1,0), Tbw(1,1), Tbw(1,2), Tbw(1,3),
                Tbw(2,0), Tbw(2,1), Tbw(2,2), Tbw(2,3),
                Tbw(3,0), Tbw(3,1), Tbw(3,2), Tbw(3,3) ;
//        cout << "Debug Frame::SetPose1" << endl;
//        usleep(1000*1000);
        Eigen::Matrix4f Twb = Tbw.inverse();
//        mTwb = mTbw.inverse();
        mTwb << Twb(0,0), Twb(0,1), Twb(0,2), Twb(0,3),
                Twb(1,0), Twb(1,1), Twb(1,2), Twb(1,3),
                Twb(2,0), Twb(2,1), Twb(2,2), Twb(2,3),
                Twb(3,0), Twb(3,1), Twb(3,2), Twb(3,3) ;
//        cout << "Debug Frame::SetPose end" << endl;
//        usleep(1000*1000);
        return ;
    }

    Eigen::Matrix4f Frame::GetPose() const {
        return mTbw;
    }
    Eigen::Matrix4f Frame::GetTwb() const {
        return mTwb;
    }
    Eigen::Matrix3f Frame::GetRwb() const {
        return CommonTools::T2R(mTwb);
    }
    Eigen::Vector3f Frame::Gettwb() const {
        return CommonTools::T2t(mTwb);
    }
    void Frame::SetVelocity(const Eigen::Matrix4f &VelocityTwb) {
//        mVelocityTwb = VelocityTwb;

        mVelocityTwb << VelocityTwb(0,0), VelocityTwb(0,1), VelocityTwb(0,2), VelocityTwb(0,3),
                        VelocityTwb(1,0), VelocityTwb(1,1), VelocityTwb(1,2), VelocityTwb(1,3),
                        VelocityTwb(2,0), VelocityTwb(2,1), VelocityTwb(2,2), VelocityTwb(2,3),
                        VelocityTwb(3,0), VelocityTwb(3,1), VelocityTwb(3,2), VelocityTwb(3,3) ;
        return ;
    }

    Eigen::Matrix4f Frame::PredictPose() const {
        //the first
        if (pPreFrame == NULL) {
            return Eigen::Matrix4f::Identity();
        } else {
        //the last * vel
            Eigen::Matrix4f Tbw_last = pPreFrame->GetPose()  ;
            Eigen::Matrix4f Twb_last = Tbw_last.inverse()  ;
            Eigen::Matrix4f Twbvelocity = pPreFrame->mVelocityTwb;
            Eigen::Matrix4f Twb = Twb_last * Twbvelocity ;
            Eigen::Matrix4f Tbw = Twb.inverse()  ;
            return Tbw;
        }

        return Eigen::Matrix4f::Identity();
    }


} //namespace ORB_SLAM





























//
//void Frame::ComputeMultiCameraMatches()
//{
//
//    vector<vector<int>> MatchResult(mNumCam);
//    for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
//        MatchResult[cam_i].resize(mvNCams[cam_i], -1);
//    }
//
//    vector<vector<float>> MatchPosition;
//    MatchPosition.reserve(20000);
//    int num_matches = 0;
//
//    for (int cam_i = 0; cam_i < mNumCam; cam_i++) {
//        // cout << "Stereo " << "cam_i" << " " << cam_i << endl;
//
//        Eigen::Matrix4f Tbc_cami = mvTbc_cams[cam_i];
//        Eigen::Vector3f tbc_cami = CommonTools::T2t(Tbc_cami);
//        Eigen::Matrix<float,3,3> Rbc_cami = CommonTools::T2R(Tbc_cami);
//        Eigen::Matrix3f K_camsi = mvK_cams[cam_i];
//        vector<cv::KeyPoint> vCamKeysCami = mvCamKeys[cam_i];
//        cv::Mat CamDescriptorsCami = mvCamDescriptors[cam_i];
//
//        for (int cam_j = 0; cam_j < mNumCam; cam_j++) {
//
//            // cout << "Stereo " << "cam_j" << " " << cam_j << endl;
//
//            Eigen::Matrix4f Tbc_camj = mvTbc_cams[cam_j];
//            Eigen::Matrix3f K_camsj = mvK_cams[cam_j];
//            vector<cv::KeyPoint> vCamKeysCamj = mvCamKeys[cam_j];
//            cv::Mat CamDescriptorsCamj = mvCamDescriptors[cam_j];
//
//            Eigen::Matrix4f T_cami_camj = Tbc_cami.inverse() * Tbc_camj;
//            Eigen::Vector3f t_cami_camj = CommonTools::T2t(T_cami_camj);
//            Eigen::Matrix<float,3,3> R_cami_camj = CommonTools::T2R(T_cami_camj);
//
//            //baseline is big enough
//            if (t_cami_camj.x() <= 0.04) {
//                continue;
//            }
//            //stereo
//            if (t_cami_camj.y() >= 0.01 || t_cami_camj.z() >= 0.01) {
//                continue;
//            }
//            Eigen::Vector3f n_camj(0.0, 0.0, 1.0);
//            if (acos((n_camj.transpose() * (R_cami_camj * n_camj))(0, 0)) >= 2.0 / 180.0 * 3.14) {
//                continue;
//            }
////
////                cout << "Stereo " << cam_i << " " << cam_j << endl;
////                cout << "T_cami_camj " << endl;
////                cout << T_cami_camj << endl;
////                cout << endl;
//            //continue;
//
//
//
//            const int thOrbDist = (FeatureMatcher::TH_HIGH+FeatureMatcher::TH_LOW)/2;
//            const int nRows = mvpFeatureExtractors[cam_i]->mvImagePyramid[0].rows;
//
//            vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());
//            for(int i=0; i<nRows; i++)
//                vRowIndices[i].reserve(200);
//
//            const int Nl = vCamKeysCami.size();
//            const int Nr = vCamKeysCamj.size();
//
//            for(int iR=0; iR<Nr; iR++)
//            {
//                const cv::KeyPoint &kp = vCamKeysCamj[iR];
//                const float &kpY = kp.pt.y;
//                const float r = 2.0f*mvScaleFactors[cam_j][vCamKeysCamj[iR].octave];
//                const int maxr = min(static_cast<int>(ceil(kpY+r)), nRows - 1);
//                const int minr = max(static_cast<int>(floor(kpY-r)), 0);
//
//                for(int yi=minr;yi<=maxr;yi++)
//                    vRowIndices[yi].push_back(iR);
//            }
//
//            const float minZ = t_cami_camj.x();
////                const float minD = 0;
//            const float minD = t_cami_camj.x() * K_camsi(0, 0) /(80 * minZ);
//            const float maxD = t_cami_camj.x() * K_camsi(0, 0) /minZ;
//
//            //cout << "Debug Frame For each left keypoint " << endl;
//            // For each left keypoint search a match in the right image
//            vector<pair<int, int> > vDistIdx;
//            vDistIdx.reserve(Nl);
//
//            for(int iL=0; iL<Nl; iL++)
//            {
//                const cv::KeyPoint &kpL = vCamKeysCami[iL];
//                const int &levelL = kpL.octave;
//                const float &vL = kpL.pt.y;
//                const float &uL = kpL.pt.x;
//
//                const vector<size_t> &vCandidates = vRowIndices[vL];
//
//                if(vCandidates.empty())
//                    continue;
//
//                const float minU = uL-maxD;
//                const float maxU = uL-minD;
//
//                if(maxU<0)
//                    continue;
//
//                int bestDist = FeatureMatcher::TH_HIGH;
//                size_t bestIdxR = 0;
//
//                const cv::Mat &dL = CamDescriptorsCami.row(iL);
//
//                //cout << "Debug Frame Compare descriptor to right keypoints" << endl;
//                // Compare descriptor to right keypoints
//                for(size_t iC=0; iC<vCandidates.size(); iC++)
//                {
//                    const size_t iR = vCandidates[iC];
//                    const cv::KeyPoint &kpR = vCamKeysCamj[iR];
//
//                    if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
//                        continue;
//
//                    const float &uR = kpR.pt.x;
//
//                    if(uR>=minU && uR<=maxU)
//                    {
//                        const cv::Mat &dR = CamDescriptorsCamj.row(iR);
//                        const int dist = FeatureMatcher::DescriptorDistance(dL,dR);
//
//                        if(dist<bestDist)
//                        {
//                            bestDist = dist;
//                            bestIdxR = iR;
//                        }
//                    }
//                }
//
//                // cout << "Debug Frame ComputeStereoMatches Subpixel" << endl;
//                // Subpixel match by correlation
//                if(bestDist<thOrbDist)
//                {
//                    // coordinates in image pyramid at keypoint scale
//                    const float uR0 = vCamKeysCamj[bestIdxR].pt.x;
//                    const float scaleFactor = 1.0 / mvScaleFactors[cam_i][kpL.octave];
//                    const float scaleduL = round(kpL.pt.x*scaleFactor);
//                    const float scaledvL = round(kpL.pt.y*scaleFactor);
//                    const float scaleduR0 = round(uR0*scaleFactor);
//
//                    // sliding window search
//                    const int w = 5;
//                    //FeatureExtractor
//                    const int PATCH_SIZE = 31;
//                    const int HALF_PATCH_SIZE = 15;
//                    const int EDGE_THRESHOLD = 19;
//                    cv::Mat IL = mvpFeatureExtractors[cam_i]->mvImagePyramidWholeSize[kpL.octave].rowRange(scaledvL-w+EDGE_THRESHOLD,scaledvL+w+1+EDGE_THRESHOLD).colRange(scaleduL-w+EDGE_THRESHOLD,scaleduL+w+1+EDGE_THRESHOLD);
//                    //cv::Mat IL = mpFeatureExtractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
//
//                    int bestDist = INT_MAX;
//                    int bestincR = 0;
//                    const int L = 5;
//                    vector<float> vDists;
//                    vDists.resize(2*L+1);
//
//                    const float iniu = scaleduR0+L-w;
//                    const float endu = scaleduR0+L+w+1;
//                    if(iniu<0 || endu >= mvpFeatureExtractors[cam_j]->mvImagePyramid[kpL.octave].cols)
//                        continue;
//
//                    for(int incR=-L; incR<=+L; incR++)
//                    {
//                        cv::Mat IR = mvpFeatureExtractors[cam_j]->mvImagePyramidWholeSize[kpL.octave].rowRange(scaledvL-w+EDGE_THRESHOLD,scaledvL+w+1+EDGE_THRESHOLD).colRange(scaleduR0+incR-w+EDGE_THRESHOLD,scaleduR0+incR+w+1+EDGE_THRESHOLD);
//                        //cv::Mat IR = mpFeatureExtractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
//
//                        float dist = cv::norm(IL,IR,cv::NORM_L1);
//                        if(dist<bestDist)
//                        {
//                            bestDist =  dist;
//                            bestincR = incR;
//                        }
//
//                        vDists[L+incR] = dist;
//                    }
//
//                    if(bestincR==-L || bestincR==L)
//                        continue;
//
//                    // Sub-pixel match (Parabola fitting)
//                    const float dist1 = vDists[L+bestincR-1];
//                    const float dist2 = vDists[L+bestincR];
//                    const float dist3 = vDists[L+bestincR+1];
//
//                    const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));
//
//                    if(deltaR<-1 || deltaR>1)
//                        continue;
//
//                    //cout << "Debug Frame Re-scaled coordinate" << endl;
//                    // Re-scaled coordinate
//                    float bestuR = mvScaleFactors[cam_j][kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);
//
//                    float disparity = (uL-bestuR);
//
//                    if(disparity>=minD && disparity<maxD)
//                    {
//                        if(disparity<=0)
//                        {
//                            disparity=0.01;
//                            bestuR = uL-0.01;
//                        }
//                        float depth = t_cami_camj.x() * K_camsi(0, 0)/disparity;
//                        Eigen::Vector3f Pb = Rbc_cami * (depth * K_camsi.inverse() * Eigen::Vector3f(uL ,vL , 1))
//                                             + tbc_cami;
//
//
//
//                        int iR = bestIdxR;
//
//                        if (MatchResult[cam_i][iL] >=0 ) {
//                            //left matches
//                            MatchResult[cam_j][iR] = MatchResult[cam_i][iL];
//                        } else if (MatchResult[cam_j][iR] >=0 ) {
//                            //right matches
//                            MatchResult[cam_i][iL] = MatchResult[cam_j][iR];
//                        } else {
//                            //nomatches
//
//                            MatchResult[cam_j][iR] = num_matches;
//                            MatchResult[cam_i][iL] = num_matches;
//                            MatchPosition.push_back({Pb.x(), Pb.y(), Pb.z()});
//                            num_matches++;
//                        }
//
//
//
//
//                    }
//                    //cout << "Debug Frame Re-scaled coordinate end" << endl;
//                }
//            }
//
//
//
//        }
//
//    }
//
//    mvMapPointPosition.resize(num_matches);
//    for (int i = 0; i < num_matches; i++) {
//        mvMapPointPosition[i] = Eigen::Vector3f(MatchPosition[i][0],
//                                                MatchPosition[i][1],
//                                                MatchPosition[i][2]);
//    }
//
//
//
////
////
////        ///////visual
////        // int w = 200;
////        // int h = 200;
////        // float fbl = 1.0;
////        int w = 1000;
////        int h = 1000;
////        float fbl = 0.1;
////        cv::Mat img_stereo(cv::Size(w, h), CV_8UC1);
////        for (int i = 0; i < h; i++) {
////            for (int j = 0; j < w; j++) {
////                img_stereo.at<uchar>(j, i) = 0;
////            }
////        }
////        cout << "Debug Frame num_matches " << num_matches << endl;
////        for (int i = 0; i < num_matches; i++) {
////            Eigen::Vector3f Pb(MatchPosition[i][0], MatchPosition[i][1], MatchPosition[i][2]);
////            int Pb_u = int(Pb.x() / fbl + w / 2.0);
////            int Pb_v = int(Pb.z() / fbl + h / 2.0);
////            if (Pb_u < 0 || Pb_u >= w || Pb_v < 0 || Pb_v >=h) {
////                continue;
////            }
//////            if (Pb.y() >= -2.0) {
//////                continue;
//////            }
////            int color_temp = max(min(int((Pb.y() + 10) * 25), 255), 0);
////            cv::circle(img_stereo, cv::Point2f(Pb_u, Pb_v),2,cv::Scalar(color_temp, color_temp, color_temp),-1);
////            //img_stereo.at<uchar>(Pb_v,Pb_u )+=20;
////        }
////
////        // 将所有像素设置为白色（最高亮度值）
////        cv::imshow("img_stereo", img_stereo);
////        cv::imwrite("img_stereo.png", img_stereo);
////        cv::waitKey(0);
//////
//
//    cout << "Debug Frame num_matches " << num_matches << endl;
//    mvMatchResult = MatchResult;
//
//
//
//}