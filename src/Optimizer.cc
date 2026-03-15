
#include "Optimizer.h"

#include <unordered_set>

#include "MapObject.h"


namespace LL_SLAM
{

    namespace
    {
        double WrapAngle(double angle)
        {
            constexpr double kPi = 3.14159265358979323846;
            while (angle > kPi) {
                angle -= 2.0 * kPi;
            }
            while (angle < -kPi) {
                angle += 2.0 * kPi;
            }
            return angle;
        }

        double YawFromRotation(const Eigen::Matrix3d &R)
        {
            // ObjectTrack/GT boxes use local x as the vehicle forward axis.
            const Eigen::Vector3d forward = R.col(0);
            return std::atan2(forward.x(), forward.z());
        }

        double YawFromQuaternion(const Eigen::Quaternionf &q)
        {
            Eigen::Quaterniond qd(q.w(), q.x(), q.y(), q.z());
            qd.normalize();
            return YawFromRotation(qd.toRotationMatrix());
        }

        g2o::VertexSE3ExpmapMultiCamera* CreatePoseVertex(int id, const Eigen::Matrix4f &TbwInput, bool fixed)
        {
            g2o::VertexSE3ExpmapMultiCamera *vSE3 = new g2o::VertexSE3ExpmapMultiCamera();

            Eigen::Matrix4d TbwEigen = TbwInput.cast<double>();
            Eigen::Quaterniond rotation(TbwEigen.block<3, 3>(0, 0));
            rotation.normalize();
            TbwEigen.block<3, 3>(0, 0) = rotation.toRotationMatrix();

            Sophus::SE3<double> Tbw(TbwEigen);
            vSE3->setEstimate(g2o::SE3Quat(Tbw.unit_quaternion(), Tbw.translation()));
            vSE3->setId(id);
            vSE3->setFixed(fixed);

            return vSE3;
        }

        double GetObservationInvSigma2(KeyFrame *pKF, int cam_i, const cv::KeyPoint &kp)
        {
            if (pKF == nullptr) {
                return 1.0;
            }
            double scaleFactor = 1.0;
            cv::FileNode node = pKF->mSettings["FeatureExtractor.scaleFactor"];
            if (!node.empty()) {
                scaleFactor = (double)node.real();
            }
            if (scaleFactor <= 1.0 || kp.octave <= 0) {
                return 1.0;
            }
            return 1.0 / std::pow(scaleFactor, 2.0 * kp.octave);
        }

        void ConfigureMonoEdge(g2o::EdgeSE3ProjectXYZMultiCamera *e, KeyFrame *pKF, int cam_i, const cv::KeyPoint &kpUn)
        {
            Eigen::Matrix3f KCam = pKF->mvK_cams[cam_i];
            e->setK(KCam(0, 0), KCam(1, 1), KCam(0, 2), KCam(1, 2));
            e->setTcb(pKF->mvTbc_cams[cam_i].inverse().cast<double>());
            e->setInformation(Eigen::Matrix2d::Identity() * GetObservationInvSigma2(pKF, cam_i, kpUn));

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            rk->setDelta(std::sqrt(5.991));
            e->setRobustKernel(rk);
        }
    }

//    int Optimizer::PoseOptimization(Frame *pFrame){
//
//        //esay to debug
//        int nMatches = 1;
//        for (int cam_i = 0; cam_i < pFrame->mvMapPoints.size(); cam_i++) {
//            for (int kpi = 0; kpi < pFrame->mvMapPoints[cam_i].size(); kpi++) {
//                MapPoint *pMP = pFrame->mvMapPoints[cam_i][kpi];
//                if (pMP == NULL) {
//                    continue;
//                }
//                nMatches++;
//            }
//        }
//
//        return nMatches;
//    }


    int Optimizer::PoseOptimization(Frame *pFrame)
    {
//        return 0;
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        //solver->setUserLambdaInit(1e5);

        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);





        /////Frame
        g2o::VertexSE3ExpmapMultiCamera * vSE3 = new g2o::VertexSE3ExpmapMultiCamera();

        //nomalization
//        cout << pFrame->GetPose() << endl;
        Eigen::Matrix4d Tbw_eigen = pFrame->GetPose().cast<double>();
        Eigen::Quaterniond rotation(Tbw_eigen.block<3, 3>(0, 0));
        rotation.normalize();
        Tbw_eigen.block<3, 3>(0, 0) = rotation.toRotationMatrix();

        //eigen 2 sophus
        Sophus::SE3<double> Tbw(Tbw_eigen);
        vSE3->setEstimate(g2o::SE3Quat(Tbw.unit_quaternion(), Tbw.translation()));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        /////MapPoint
        unordered_map<MapPoint*, int> DictMP2Index;
        int nVertexMPIndex = 1;
        for (int cam_i = 0; cam_i < pFrame->mvMapPoints.size(); cam_i++) {
            for (int kpi = 0; kpi < pFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }
                if (DictMP2Index.find(pMP) != DictMP2Index.end()) {
                    continue;
                }


                g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
                vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
                vPoint->setId(nVertexMPIndex);
                vPoint->setMarginalized(true);
                vPoint->setFixed(true);
                optimizer.addVertex(vPoint);
                DictMP2Index[pMP] = nVertexMPIndex;
                nVertexMPIndex++;
            }
        }

        int N = 0;
        for (auto it : pFrame->mvNCams) {N += it;}




        vector<g2o::EdgeSE3ProjectXYZMultiCamera*> vpEdgesMono;
        vector<pair<int, int>> vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        const float deltaMono = sqrt(5.991);

        for (int cam_i = 0; cam_i < pFrame->mvMapPoints.size(); cam_i++) {

            Eigen::Matrix3f K_camsi = pFrame->mvK_cams[cam_i];
            double fx = K_camsi(0, 0);
            double cx = K_camsi(0, 2);
            double fy = K_camsi(1, 1);
            double cy = K_camsi(1, 2);
            Eigen::Matrix4d Tcb;
            Eigen::Matrix3d Rcb;
            Eigen::Vector3d tcb;
            Tcb = pFrame->mvTbc_cams[cam_i].inverse().cast<double>();
            Rcb = CommonTools::T2R(pFrame->mvTbc_cams[cam_i].inverse()).cast<double>();
            tcb = CommonTools::T2t(pFrame->mvTbc_cams[cam_i].inverse()).cast<double>();

            for (int kpi = 0; kpi < pFrame->mvMapPoints[cam_i].size(); kpi++) {
                MapPoint *pMP = pFrame->mvMapPoints[cam_i][kpi];
                if (pMP == NULL) {
                    continue;
                }

//                if (DictMP2Index.find(pMP) != DictMP2Index.end()) {
//                    //continue;
//                } else {
//
//                    g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
//                    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
//                    vPoint->setId(nVertexMPIndex);
//                    vPoint->setMarginalized(true);
//                    vPoint->setFixed(true);
//                    optimizer.addVertex(vPoint);
//                    DictMP2Index[pMP] = nVertexMPIndex;
//                    nVertexMPIndex++;
//                }



                Eigen::Matrix<double,2,1> obs;
                const cv::KeyPoint &kpUn = pFrame->mvCamKeysUn[cam_i][kpi];
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZMultiCamera* e = new g2o::EdgeSE3ProjectXYZMultiCamera();

                int nVertexMPIndexTemp = DictMP2Index[pMP];
//                cout << "nVertexMPIndexTemp " << nVertexMPIndexTemp << endl;
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(nVertexMPIndexTemp)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
                //e->setVertex(1, (g2o::OptimizableGraph::Vertex*)(optimizer.vertex(0)));
                e->setMeasurement(obs);
                const float invSigma2 = pFrame->mvScaleFactors[cam_i][kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(deltaMono);

                e->setK(fx, fy, cx, cy);
                e->setTcb(Tcb);

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vnIndexEdgeMono.push_back({cam_i, kpi});

////                ///////////debug
////
//                Eigen::Matrix3f Rbw = CommonTools::T2R(((g2o::VertexSE3ExpmapMultiCamera*)(optimizer.vertex(0)))->estimate().to_homogeneous_matrix().matrix().cast<float>());
//                Eigen::Vector3f tbw = CommonTools::T2t(((g2o::VertexSE3ExpmapMultiCamera*)(optimizer.vertex(0)))->estimate().to_homogeneous_matrix().matrix().cast<float>());
//                //Eigen::Vector3f Pw = pMP->GetWorldPos();
//                Eigen::Vector3f Pw = ((g2o::VertexSBAPointXYZ*)(optimizer.vertex(nVertexMPIndexTemp)))->estimate().cast<float>();
//                Eigen::Vector3f Pb = Rbw.cast<float>() * Pw.cast<float>() + tbw.cast<float>();
//                Eigen::Vector3f Pc = Rcb.cast<float>() * Pb + tcb.cast<float>();
//                Eigen::Vector3f p_2D =  1.0 / Pc.z() * Pc;
//                Eigen::Vector3f pixel_2D = pFrame->mvK_cams[cam_i] * p_2D;
////                cout << "e " << (pixel_2D.block<2, 1>(0,0) - obs.cast<float>()).norm() << endl;
////                std::cout << "optimizer.vertex(0) " << optimizer.vertex(0) << std::endl;
////                std::cout << "Tbw " << std::endl << ((g2o::VertexSE3ExpmapMultiCamera*)(optimizer.vertex(0)))->estimate().to_homogeneous_matrix().matrix() << std::endl;
////                std::cout << "((g2o::VertexSE3ExpmapMultiCamera*)(optimizer.vertex(0)))->estimate() " << std::endl << ((g2o::VertexSE3ExpmapMultiCamera*)(optimizer.vertex(0)))->estimate() << std::endl;
////                std::cout << "Pw " << Pw.transpose() << std::endl;
////                std::cout << "Pb " << Pb.transpose() << std::endl;
////                std::cout << "Pc " << Pc.transpose() << std::endl;
////                std::cout << "proj " << pixel_2D.transpose() << std::endl;
////                std::cout << "obs " << obs.transpose() << std::endl;
////                cout << "//////////////////////////////////////" << endl;
////                e->computeError();
////                /////////////////////////////////////////

            }
        }


        int nBad=0;
        vector<bool> vbOutlier(vpEdgesMono.size(), false);
        for(size_t it=0; it<4; it++)
        {
//            cout << "it " << it << endl;
//            //nomalization
            Eigen::Matrix4d Tbw_eigen = pFrame->GetPose().cast<double>();
            Eigen::Quaterniond rotation(Tbw_eigen.block<3, 3>(0, 0));
            rotation.normalize();
            Tbw_eigen.block<3, 3>(0, 0) = rotation.toRotationMatrix();

            //eigen 2 sophus
            Tbw = Sophus::SE3<double>(Tbw_eigen);
            vSE3->setEstimate(g2o::SE3Quat(Tbw.unit_quaternion().cast<double>(), Tbw.translation().cast<double>()));

//
//            Tbw = Sophus::SE3<double>(pFrame->GetPose());
//            vSE3->setEstimate(g2o::SE3Quat(Tbw.unit_quaternion().cast<double>(),Tbw.translation().cast<double>()));

            optimizer.initializeOptimization(0);
            optimizer.optimize(10);

            nBad=0;
            for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
            {
                g2o::EdgeSE3ProjectXYZMultiCamera* e = vpEdgesMono[i];

                if(vbOutlier[i])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if(chi2>deltaMono)
                {
                    vbOutlier[i]=true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    vbOutlier[i]=false;
                    e->setLevel(0);
                }

                if(it==2)
                    e->setRobustKernel(0);
            }
//
//            cout << "optimizer.edges().size() " << optimizer.edges().size() << endl;
            if(optimizer.edges().size()<10)
                break;
        }


        // Recover optimized pose and return number of inliers
        g2o::VertexSE3ExpmapMultiCamera* vSE3_recov = static_cast<g2o::VertexSE3ExpmapMultiCamera*>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
                                SE3quat_recov.translation().cast<float>());
        pFrame->SetPose(pose.matrix());
//        cout << "pose.matrix()" << endl;
//        cout << pose.matrix() << endl;
//        cout << "twb : " << CommonTools::T2t(pFrame->GetPose().inverse()).transpose() << endl;







        return optimizer.edges().size();
    }

    int Optimizer::LocalBundleAdjustment(KeyFrame *pKF, Map *pMap)
    {
        if (pKF == nullptr || pMap == nullptr || pKF->isBad()) {
            return 0;
        }

        const int kMaxOptimizableKFs = 4;
        const int kMaxFixedKFs = 10;
        unordered_set<KeyFrame*> usActiveLocalKFs;
        vector<KeyFrame*> vpOptimizableKFs;
        vpOptimizableKFs.reserve(kMaxOptimizableKFs);

        vector<KeyFrame*> vpCovisibleKFs = pKF->GetBestCovisibilityKeyFrames(kMaxOptimizableKFs);
        for (KeyFrame *pLocalKF : vpCovisibleKFs) {
            if (pLocalKF == nullptr || pLocalKF->isBad() || usActiveLocalKFs.count(pLocalKF) > 0) {
                continue;
            }
            usActiveLocalKFs.insert(pLocalKF);
            vpOptimizableKFs.push_back(pLocalKF);
        }

        if (vpOptimizableKFs.size() < 2) {
            vector<KeyFrame*> vpFallbackKFs = pMap->GetLocalKeyFrame();
            for (KeyFrame *pLocalKF : vpFallbackKFs) {
                if (int(vpOptimizableKFs.size()) >= kMaxOptimizableKFs) {
                    break;
                }
                if (pLocalKF == nullptr || pLocalKF->isBad() || pLocalKF == pKF || usActiveLocalKFs.count(pLocalKF) > 0) {
                    continue;
                }
                usActiveLocalKFs.insert(pLocalKF);
                vpOptimizableKFs.push_back(pLocalKF);
            }
        }
        usActiveLocalKFs.insert(pKF);

        unordered_set<MapPoint*> usLocalMPs;
        vector<MapPoint*> vpOptimizableMPs;
        vector<KeyFrame*> vpPointSourceKFs;
        vpPointSourceKFs.reserve(vpOptimizableKFs.size() + 1);
        vpPointSourceKFs.push_back(pKF);
        vpPointSourceKFs.insert(vpPointSourceKFs.end(), vpOptimizableKFs.begin(), vpOptimizableKFs.end());

        for (KeyFrame *pLocalKF : vpPointSourceKFs) {
            for (int cam_i = 0; cam_i < int(pLocalKF->mvMapPoints.size()); cam_i++) {
                for (int kpi = 0; kpi < int(pLocalKF->mvMapPoints[cam_i].size()); kpi++) {
                    MapPoint *pMP = pLocalKF->mvMapPoints[cam_i][kpi];
                    if (pMP == nullptr || pMP->isBad() || pMP->KeyFrameObservations() <= 0) {
                        continue;
                    }
                    if (!usLocalMPs.insert(pMP).second) {
                        continue;
                    }
                    vpOptimizableMPs.push_back(pMP);
                }
            }
        }
        if (vpOptimizableMPs.empty()) {
            return 0;
        }

        map<KeyFrame*, int> fixedKFcounter;
        for (MapPoint *pMP : vpOptimizableMPs) {
            vector<pair<KeyFrame*,std::pair<int,int>>> vObservations = pMP->GetObservations();
            for (const auto &obs : vObservations) {
                KeyFrame *pObsKF = obs.first;
                if (pObsKF == nullptr || pObsKF->isBad() || usActiveLocalKFs.count(pObsKF) > 0) {
                    continue;
                }
                fixedKFcounter[pObsKF]++;
            }
        }

        vector<pair<int, KeyFrame*>> vFixedKFsSorted;
        vFixedKFsSorted.reserve(fixedKFcounter.size());
        for (auto &it : fixedKFcounter) {
            vFixedKFsSorted.push_back({it.second, it.first});
        }
        sort(vFixedKFsSorted.begin(), vFixedKFsSorted.end(),
             [](const pair<int, KeyFrame*> &lhs, const pair<int, KeyFrame*> &rhs) {
                 if (lhs.first != rhs.first) {
                     return lhs.first > rhs.first;
                 }
                 return lhs.second->mnId > rhs.second->mnId;
             });

        vector<KeyFrame*> vpFixedKFs;
        vpFixedKFs.reserve(min(int(vFixedKFsSorted.size()), kMaxFixedKFs));
        for (const auto &it : vFixedKFsSorted) {
            if (int(vpFixedKFs.size()) >= kMaxFixedKFs) {
                break;
            }
            vpFixedKFs.push_back(it.second);
        }

        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
        g2o::BlockSolver_6_3 *solverPtr = new g2o::BlockSolver_6_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solverPtr);
        optimizer.setAlgorithm(solver);
        optimizer.setVerbose(false);

        unordered_map<KeyFrame*, int> dictKF2Index;
        unordered_map<MapPoint*, int> dictMP2Index;
        int nextVertexId = 0;

        g2o::VertexSE3ExpmapMultiCamera *vCurrentKF = CreatePoseVertex(nextVertexId, pKF->GetPose(), true);
        optimizer.addVertex(vCurrentKF);
        dictKF2Index[pKF] = nextVertexId++;

        for (KeyFrame *pLocalKF : vpOptimizableKFs) {
            g2o::VertexSE3ExpmapMultiCamera *vSE3 = CreatePoseVertex(nextVertexId, pLocalKF->GetPose(), false);
            optimizer.addVertex(vSE3);
            dictKF2Index[pLocalKF] = nextVertexId++;
        }

        for (KeyFrame *pFixedKF : vpFixedKFs) {
            if (pFixedKF == nullptr || pFixedKF->isBad() || dictKF2Index.find(pFixedKF) != dictKF2Index.end()) {
                continue;
            }
            g2o::VertexSE3ExpmapMultiCamera *vSE3 = CreatePoseVertex(nextVertexId, pFixedKF->GetPose(), true);
            optimizer.addVertex(vSE3);
            dictKF2Index[pFixedKF] = nextVertexId++;
        }

        for (MapPoint *pMP : vpOptimizableMPs) {
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
            vPoint->setId(nextVertexId);
            vPoint->setMarginalized(true);
            vPoint->setFixed(false);
            optimizer.addVertex(vPoint);
            dictMP2Index[pMP] = nextVertexId++;
        }

        int nEdges = 0;
        int nPointEdges = 0;
        for (MapPoint *pMP : vpOptimizableMPs) {
            auto itPoint = dictMP2Index.find(pMP);
            if (itPoint == dictMP2Index.end()) {
                continue;
            }

            vector<pair<KeyFrame*,std::pair<int,int>>> vObservations = pMP->GetObservations();
            for (const auto &obsInfo : vObservations) {
                KeyFrame *pObsKF = obsInfo.first;
                auto itKF = dictKF2Index.find(pObsKF);
                if (pObsKF == nullptr || pObsKF->isBad() || itKF == dictKF2Index.end()) {
                    continue;
                }

                const int cam_i = obsInfo.second.first;
                const int kpi = obsInfo.second.second;
                if (cam_i < 0 || cam_i >= int(pObsKF->mvCamKeysUn.size())) {
                    continue;
                }
                if (kpi < 0 || kpi >= int(pObsKF->mvCamKeysUn[cam_i].size())) {
                    continue;
                }

                const cv::KeyPoint &kpUn = pObsKF->mvCamKeysUn[cam_i][kpi];
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZMultiCamera *e = new g2o::EdgeSE3ProjectXYZMultiCamera();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(itPoint->second)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(itKF->second)));
                e->setMeasurement(obs);
                ConfigureMonoEdge(e, pObsKF, cam_i, kpUn);
                optimizer.addEdge(e);
                nEdges++;
                nPointEdges++;
            }
        }
        cout << "LocalBundleAdjustment point edges : " << nPointEdges << endl;

        if (nEdges < 10) {
            return nEdges;
        }

        optimizer.initializeOptimization();
        optimizer.optimize(10);

        for (KeyFrame *pLocalKF : vpOptimizableKFs) {
            auto itKF = dictKF2Index.find(pLocalKF);
            if (itKF == dictKF2Index.end()) {
                continue;
            }
            auto *vSE3 = static_cast<g2o::VertexSE3ExpmapMultiCamera*>(optimizer.vertex(itKF->second));
            g2o::SE3Quat se3QuatRecov = vSE3->estimate();
            Sophus::SE3<float> pose(se3QuatRecov.rotation().cast<float>(),
                                    se3QuatRecov.translation().cast<float>());
            pLocalKF->SetPose(pose.matrix());
            if (pLocalKF->mpFrame != nullptr) {
                pLocalKF->mpFrame->SetPose(pose.matrix());
            }
        }

        for (MapPoint *pMP : vpOptimizableMPs) {
            auto itPoint = dictMP2Index.find(pMP);
            if (itPoint == dictMP2Index.end()) {
                continue;
            }
            auto *vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(itPoint->second));
            pMP->SetWorldPos(vPoint->estimate().cast<float>());
        }

        return nEdges;
    }

    int Optimizer::OptimizeLocalObjects(KeyFrame *pKF, Map *pMap)
    {
        if (pKF == nullptr || pMap == nullptr || pKF->isBad()) {
            return 0;
        }

        vector<MapObject*> vpLocalObjects = pMap->GetLocalMapObject();
        int nOptimizedObjects = 0;
        const int kMinStaticObservations = 3;
        const double kMaxAcceptedJumpMeters = 2.0;
        const double kMaxAcceptedYawJumpRad = 35.0 * M_PI / 180.0;
        const float kObjectPositionBlend = 0.25f;
        const float kObjectYawBlend = 0.25f;

        for (MapObject *pObj : vpLocalObjects) {
            if (pObj == nullptr || pObj->isBad() || !pObj->IsStatic()) {
                continue;
            }

            vector<pair<KeyFrame*, int>> vObservations = pObj->GetObservations();
            if (vObservations.size() < kMinStaticObservations) {
                continue;
            }

            double sumX = 0.0;
            double sumY = 0.0;
            double sumZ = 0.0;
            double sumSinYaw = 0.0;
            double sumCosYaw = 0.0;
            int nValidObs = 0;

            for (const auto &obsInfo : vObservations) {
                KeyFrame *pObsKF = obsInfo.first;
                const int objectIdx = obsInfo.second;
                if (pObsKF == nullptr || pObsKF->isBad()) {
                    continue;
                }
                if (objectIdx < 0 || objectIdx >= int(pObsKF->mvObjectObservations.size())) {
                    continue;
                }

                const ObjectObservation &obs = pObsKF->mvObjectObservations[objectIdx];
                const Eigen::Matrix4f Twb = pObsKF->GetTwb();
                const Eigen::Matrix3f Rwb = CommonTools::T2R(Twb);
                const Eigen::Vector3f twb = CommonTools::T2t(Twb);
                const Eigen::Vector3f worldPos = Rwb * obs.t_ref + twb;

                Eigen::Quaternionf qwb(Rwb);
                qwb.normalize();
                Eigen::Quaternionf qwo = qwb * obs.q_ref;
                qwo.normalize();
                const double worldYaw = YawFromQuaternion(qwo);

                sumX += static_cast<double>(worldPos.x());
                sumY += static_cast<double>(worldPos.y());
                sumZ += static_cast<double>(worldPos.z());
                sumSinYaw += std::sin(worldYaw);
                sumCosYaw += std::cos(worldYaw);
                nValidObs++;
            }

            if (nValidObs < kMinStaticObservations) {
                continue;
            }

            const Eigen::Vector3f currentWorldPos = pObj->GetWorldPos();
            const Eigen::Quaternionf currentRotation = pObj->GetWorldRotation();
            const double currentYaw = YawFromQuaternion(currentRotation);

            const Eigen::Vector3f averagedWorldPos(static_cast<float>(sumX / nValidObs),
                                                   static_cast<float>(sumY / nValidObs),
                                                   static_cast<float>(sumZ / nValidObs));
            const double averagedYaw = std::atan2(sumSinYaw, sumCosYaw);

            const double positionJump = (averagedWorldPos - currentWorldPos).norm();
            const double yawJump = std::abs(WrapAngle(averagedYaw - currentYaw));
            if (positionJump > kMaxAcceptedJumpMeters || yawJump > kMaxAcceptedYawJumpRad) {
                continue;
            }

            const Eigen::Vector3f optimizedWorldPos =
                currentWorldPos + kObjectPositionBlend * (averagedWorldPos - currentWorldPos);
            const float blendedYawDelta =
                static_cast<float>(kObjectYawBlend * WrapAngle(averagedYaw - currentYaw));
            Eigen::Quaternionf optimizedRotation =
                Eigen::AngleAxisf(blendedYawDelta, Eigen::Vector3f::UnitY()) * currentRotation.normalized();
            optimizedRotation.normalize();
            pObj->SetWorldPose(optimizedWorldPos, optimizedRotation);
            nOptimizedObjects++;
        }

        return nOptimizedObjects;
    }


} //namespace ORB_SLAM
