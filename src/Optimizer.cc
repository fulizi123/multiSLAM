
#include "Optimizer.h"

#include <algorithm>
#include <limits>
#include <unordered_set>

#include "MapObject.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"


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

        double Clamp(double value, double lower, double upper)
        {
            return std::max(lower, std::min(upper, value));
        }

        double ComputeMedian(std::vector<double> values)
        {
            if (values.empty()) {
                return 0.0;
            }
            const size_t mid = values.size() / 2;
            std::nth_element(values.begin(), values.begin() + mid, values.end());
            double median = values[mid];
            if ((values.size() % 2) == 0) {
                std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
                median = 0.5 * (median + values[mid - 1]);
            }
            return median;
        }

        double ComputeMedianAbsDeviation(const std::vector<double> &values, double median)
        {
            std::vector<double> deviations;
            deviations.reserve(values.size());
            for (double value : values) {
                deviations.push_back(std::abs(value - median));
            }
            return ComputeMedian(deviations);
        }

        Eigen::Vector3d ComputeComponentMedian(const std::vector<Eigen::Vector3d> &values)
        {
            if (values.empty()) {
                return Eigen::Vector3d::Zero();
            }
            std::vector<double> xs;
            std::vector<double> ys;
            std::vector<double> zs;
            xs.reserve(values.size());
            ys.reserve(values.size());
            zs.reserve(values.size());
            for (const Eigen::Vector3d &value : values) {
                xs.push_back(value.x());
                ys.push_back(value.y());
                zs.push_back(value.z());
            }
            return Eigen::Vector3d(ComputeMedian(xs), ComputeMedian(ys), ComputeMedian(zs));
        }

        double ComputeRobustYawReference(const std::vector<double> &yaws)
        {
            if (yaws.empty()) {
                return 0.0;
            }
            double bestYaw = yaws.front();
            double bestCost = std::numeric_limits<double>::max();
            for (double candidateYaw : yaws) {
                double cost = 0.0;
                for (double yaw : yaws) {
                    cost += std::abs(WrapAngle(yaw - candidateYaw));
                }
                if (cost < bestCost) {
                    bestCost = cost;
                    bestYaw = candidateYaw;
                }
            }
            return bestYaw;
        }

        double ComputeObjectObservationWeight(const MapObject *pObj,
                                              const ObjectObservation &obs)
        {
            const double score = obs.score > 0.0f ? Clamp(obs.score, 0.20, 1.0) : 1.0;
            const int supportPoints = std::max(0, obs.num_lidar_pts + obs.num_radar_pts);
            const double supportWeight = 1.0 + std::min(10, supportPoints) / 40.0;
            const double distanceWeight = 1.0 / std::sqrt(1.0 + obs.t_ref.norm() / 30.0);
            double stabilityWeight = 1.0;
            if (pObj != nullptr) {
                stabilityWeight = Clamp(0.85 + 0.015 * std::min(pObj->GetSeenCount(), 10), 0.85, 1.0);
            }

            return Clamp(score * supportWeight * distanceWeight * stabilityWeight, 0.20, 2.5);
        }

        struct ObjectObservationEntry
        {
            KeyFrame *pKF = nullptr;
            int objectIdx = -1;
            Eigen::Vector3d worldPos = Eigen::Vector3d::Zero();
            double worldYaw = 0.0;
            double weight = 1.0;
        };

        class VertexObjectPose final : public g2o::BaseVertex<4, Eigen::Vector4d>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            void setToOriginImpl() override
            {
                _estimate.setZero();
            }

            void oplusImpl(const double *update_) override
            {
                Eigen::Map<const Eigen::Vector4d> update(update_);
                _estimate.head<3>() += update.head<3>();
                _estimate[3] = WrapAngle(_estimate[3] + update[3]);
            }
        };

        class EdgeObjectPositionUnary final
            : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexObjectPose>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            void computeError() override
            {
                const VertexObjectPose *vObj = static_cast<const VertexObjectPose*>(_vertices[0]);
                _error = vObj->estimate().head<3>() - _measurement;
            }

            void linearizeOplus() override
            {
                _jacobianOplusXi.setZero();
                _jacobianOplusXi.block<3, 3>(0, 0).setIdentity();
            }
        };

        class EdgeObjectYawUnary final
            : public g2o::BaseUnaryEdge<1, Eigen::Matrix<double, 1, 1>, VertexObjectPose>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            bool read(std::istream &) override { return false; }
            bool write(std::ostream &) const override { return false; }

            void computeError() override
            {
                const VertexObjectPose *vObj = static_cast<const VertexObjectPose*>(_vertices[0]);
                _error[0] = WrapAngle(vObj->estimate()[3] - _measurement[0]);
            }

            void linearizeOplus() override
            {
                _jacobianOplusXi.setZero();
                _jacobianOplusXi(0, 3) = 1.0;
            }
        };

        struct ObjectOptimizationEstimate
        {
            Eigen::Vector3d position = Eigen::Vector3d::Zero();
            double yaw = 0.0;
            double averageWeight = 1.0;
        };

        ObjectOptimizationEstimate OptimizeObjectWithObservations(
            const Eigen::Vector3d &initialPos,
            double initialYaw,
            const std::vector<ObjectObservationEntry> &observations,
            const MapObject *pObj)
        {
            ObjectOptimizationEstimate result;
            result.position = initialPos;
            result.yaw = initialYaw;
            if (observations.empty()) {
                return result;
            }

            double averageWeight = 0.0;
            for (const ObjectObservationEntry &obs : observations) {
                averageWeight += obs.weight;
            }
            averageWeight /= observations.size();
            result.averageWeight = averageWeight;

            g2o::SparseOptimizer optimizer;
            g2o::BlockSolverX::LinearSolverType *linearSolver =
                new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
            g2o::BlockSolverX *solverPtr = new g2o::BlockSolverX(linearSolver);
            g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solverPtr);
            optimizer.setAlgorithm(solver);
            optimizer.setVerbose(false);

            VertexObjectPose *vObj = new VertexObjectPose();
            Eigen::Vector4d estimate;
            estimate << initialPos.x(), initialPos.y(), initialPos.z(), initialYaw;
            vObj->setEstimate(estimate);
            vObj->setId(0);
            vObj->setFixed(false);
            optimizer.addVertex(vObj);

            const double positionHuberDelta = std::sqrt(7.815);
            const double yawHuberDelta = std::sqrt(3.841);
            for (const ObjectObservationEntry &obs : observations) {
                EdgeObjectPositionUnary *ePos = new EdgeObjectPositionUnary();
                ePos->setVertex(0, vObj);
                ePos->setMeasurement(obs.worldPos);
                ePos->setInformation(Eigen::Matrix3d::Identity() * obs.weight);
                g2o::RobustKernelHuber *rkPos = new g2o::RobustKernelHuber;
                rkPos->setDelta(positionHuberDelta);
                ePos->setRobustKernel(rkPos);
                optimizer.addEdge(ePos);

                EdgeObjectYawUnary *eYaw = new EdgeObjectYawUnary();
                eYaw->setVertex(0, vObj);
                Eigen::Matrix<double, 1, 1> yawMeasurement;
                yawMeasurement[0] = obs.worldYaw;
                eYaw->setMeasurement(yawMeasurement);
                eYaw->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * (0.5 * obs.weight));
                g2o::RobustKernelHuber *rkYaw = new g2o::RobustKernelHuber;
                rkYaw->setDelta(yawHuberDelta);
                eYaw->setRobustKernel(rkYaw);
                optimizer.addEdge(eYaw);
            }

            const double observationCountScale =
                std::sqrt(std::max<size_t>(size_t(1), observations.size()));
            const double temporalPositionWeight = 0.30 * averageWeight / observationCountScale;
            const double temporalYawWeight = 0.20 * averageWeight / observationCountScale;

            EdgeObjectPositionUnary *eTemporalPos = new EdgeObjectPositionUnary();
            eTemporalPos->setVertex(0, vObj);
            eTemporalPos->setMeasurement(initialPos);
            eTemporalPos->setInformation(Eigen::Matrix3d::Identity() * temporalPositionWeight);
            optimizer.addEdge(eTemporalPos);

            EdgeObjectYawUnary *eTemporalYaw = new EdgeObjectYawUnary();
            eTemporalYaw->setVertex(0, vObj);
            Eigen::Matrix<double, 1, 1> initialYawMeasurement;
            initialYawMeasurement[0] = initialYaw;
            eTemporalYaw->setMeasurement(initialYawMeasurement);
            eTemporalYaw->setInformation(Eigen::Matrix<double, 1, 1>::Identity() * temporalYawWeight);
            optimizer.addEdge(eTemporalYaw);

            optimizer.initializeOptimization();
            optimizer.optimize(10);

            const Eigen::Vector4d optimizedEstimate = vObj->estimate();
            result.position = optimizedEstimate.head<3>();
            result.yaw = WrapAngle(optimizedEstimate[3]);
            return result;
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
        const double kMinObservationWeight = 0.20;
        const double kMinYawGateRad = 15.0 * M_PI / 180.0;
        const double kMinPositionGateMeters = 1.25;
        const double kMaxAcceptedJumpMeters = 2.0;
        const double kMaxAcceptedYawJumpRad = 35.0 * M_PI / 180.0;

        for (MapObject *pObj : vpLocalObjects) {
            if (pObj == nullptr || pObj->isBad() || !pObj->IsStatic()) {
                continue;
            }

            vector<pair<KeyFrame*, int>> vObservations = pObj->GetObservations();
            if (vObservations.size() < kMinStaticObservations) {
                continue;
            }

            vector<ObjectObservationEntry> vObservationEntries;
            vObservationEntries.reserve(vObservations.size());

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

                const double observationWeight = ComputeObjectObservationWeight(pObj, obs);
                if (observationWeight < kMinObservationWeight) {
                    continue;
                }

                ObjectObservationEntry entry;
                entry.pKF = pObsKF;
                entry.objectIdx = objectIdx;
                entry.worldPos = worldPos.cast<double>();
                entry.worldYaw = worldYaw;
                entry.weight = observationWeight;
                vObservationEntries.push_back(entry);
            }

            if (vObservationEntries.size() < kMinStaticObservations) {
                continue;
            }

            const Eigen::Vector3f currentWorldPos = pObj->GetWorldPos();
            const Eigen::Quaternionf currentRotation = pObj->GetWorldRotation();
            const double currentYaw = YawFromQuaternion(currentRotation);

            std::vector<Eigen::Vector3d> vObservationPositions;
            std::vector<double> vObservationYaws;
            vObservationPositions.reserve(vObservationEntries.size());
            vObservationYaws.reserve(vObservationEntries.size());
            for (const ObjectObservationEntry &entry : vObservationEntries) {
                vObservationPositions.push_back(entry.worldPos);
                vObservationYaws.push_back(entry.worldYaw);
            }

            const Eigen::Vector3d robustCenter = ComputeComponentMedian(vObservationPositions);
            const double robustYaw = ComputeRobustYawReference(vObservationYaws);

            std::vector<double> vPositionResiduals;
            std::vector<double> vYawResiduals;
            vPositionResiduals.reserve(vObservationEntries.size());
            vYawResiduals.reserve(vObservationEntries.size());
            for (const ObjectObservationEntry &entry : vObservationEntries) {
                vPositionResiduals.push_back((entry.worldPos - robustCenter).norm());
                vYawResiduals.push_back(std::abs(WrapAngle(entry.worldYaw - robustYaw)));
            }

            const double medianPosResidual = ComputeMedian(vPositionResiduals);
            const double madPosResidual = ComputeMedianAbsDeviation(vPositionResiduals, medianPosResidual);
            const double medianYawResidual = ComputeMedian(vYawResiduals);
            const double madYawResidual = ComputeMedianAbsDeviation(vYawResiduals, medianYawResidual);
            const double positionGate =
                std::max(kMinPositionGateMeters, medianPosResidual + 2.5 * std::max(madPosResidual, 0.10));
            const double yawGate =
                std::max(kMinYawGateRad, medianYawResidual + 2.5 * std::max(madYawResidual, 3.0 * M_PI / 180.0));

            vector<ObjectObservationEntry> vInlierObservations;
            vInlierObservations.reserve(vObservationEntries.size());
            for (size_t i = 0; i < vObservationEntries.size(); i++) {
                if (vPositionResiduals[i] > positionGate) {
                    continue;
                }
                if (vYawResiduals[i] > yawGate) {
                    continue;
                }
                vInlierObservations.push_back(vObservationEntries[i]);
            }

            if (vInlierObservations.size() < kMinStaticObservations) {
                continue;
            }

            ObjectOptimizationEstimate optimizedEstimate =
                OptimizeObjectWithObservations(currentWorldPos.cast<double>(), currentYaw, vInlierObservations, pObj);

            vector<ObjectObservationEntry> vRefinedInliers;
            vRefinedInliers.reserve(vInlierObservations.size());
            const double refinedPositionGate = std::max(0.75, 0.9 * positionGate);
            const double refinedYawGate = std::max(kMinYawGateRad, 1.05 * yawGate);
            for (const ObjectObservationEntry &entry : vInlierObservations) {
                const double positionResidual =
                    (optimizedEstimate.position - entry.worldPos).norm();
                const double yawResidual =
                    std::abs(WrapAngle(optimizedEstimate.yaw - entry.worldYaw));
                if (positionResidual > refinedPositionGate) {
                    continue;
                }
                if (yawResidual > refinedYawGate) {
                    continue;
                }
                vRefinedInliers.push_back(entry);
            }

            if (vRefinedInliers.size() >= kMinStaticObservations &&
                vRefinedInliers.size() < vInlierObservations.size()) {
                optimizedEstimate =
                    OptimizeObjectWithObservations(currentWorldPos.cast<double>(), currentYaw, vRefinedInliers, pObj);
            }

            const Eigen::Vector3f optimizedWorldPos = optimizedEstimate.position.cast<float>();
            const double optimizedYaw = optimizedEstimate.yaw;

            const double positionJump = (optimizedWorldPos - currentWorldPos).norm();
            const double yawJump = std::abs(WrapAngle(optimizedYaw - currentYaw));
            const double maturityScale = Clamp(
                1.0 - 0.05 * std::min(pObj->GetSeenCount(), 10) - 0.30 * Clamp(pObj->GetConfidence(), 0.0, 1.0),
                0.35,
                1.0);
            const double lostRelaxation = 1.0 + 0.10 * std::min(pObj->GetLostCount(), 5);
            const double acceptedJumpMeters =
                Clamp(kMaxAcceptedJumpMeters * maturityScale * lostRelaxation, 0.75, kMaxAcceptedJumpMeters);
            const double acceptedYawJumpRad =
                Clamp(kMaxAcceptedYawJumpRad * maturityScale * lostRelaxation,
                      12.0 * M_PI / 180.0,
                      kMaxAcceptedYawJumpRad);

            if (positionJump > acceptedJumpMeters || yawJump > acceptedYawJumpRad) {
                continue;
            }

            const Eigen::Quaternionf qYawCurrent(Eigen::AngleAxisf(static_cast<float>(currentYaw), Eigen::Vector3f::UnitY()));
            Eigen::Quaternionf qBase = qYawCurrent.conjugate() * currentRotation.normalized();
            qBase.normalize();
            Eigen::Quaternionf optimizedRotation =
                Eigen::Quaternionf(Eigen::AngleAxisf(static_cast<float>(optimizedYaw), Eigen::Vector3f::UnitY())) * qBase;
            optimizedRotation.normalize();
            pObj->SetWorldPose(optimizedWorldPos, optimizedRotation);
            nOptimizedObjects++;
        }

        return nOptimizedObjects;
    }


} //namespace ORB_SLAM
