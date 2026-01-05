
#include "Optimizer.h"


namespace LL_SLAM
{

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


} //namespace ORB_SLAM
