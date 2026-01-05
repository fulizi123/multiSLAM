
#include "CommonTools.h"


namespace LL_SLAM
{
    Eigen::Matrix<float,4,4> CommonTools::toMatrix4f(const cv::Mat &cvMat4)
    {
        Eigen::Matrix<float,4,4> M;

        M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
                cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
                cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
                cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);
        return M;
    }


    cv::Mat CommonTools::toCvMat(const Eigen::Matrix<float,4,4> &m)
    {
        cv::Mat cvMat(4,4,CV_32F);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<float>(i,j)=m(i,j);

        return cvMat.clone();
    }

    cv::Mat CommonTools::toCvMat(const Eigen::Matrix<float,3,3> &m)
    {
        cv::Mat cvMat(3,3,CV_32F);
        for(int i=0;i<3;i++)
            for(int j=0; j<3; j++)
                cvMat.at<float>(i,j)=m(i,j);

        return cvMat.clone();
    }

    cv::Mat CommonTools::toCvMat(const Eigen::Matrix<float,3,1> &m)
    {
        cv::Mat cvMat(3,1,CV_32F);
        for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

        return cvMat.clone();
    }

    Eigen::Matrix<float,3,3> CommonTools::toSkewMatrix(const Eigen::Vector3f &t) {

        Eigen::Matrix<float,3,3> t_hat;
        t_hat<<     0,          -t(2),      t(1),
                t(2),       0,          -t(0),
                -t(1),      t(0),       0;
        return t_hat;
    }

    Eigen::Matrix<float,4,4> CommonTools::Rt2T(const Eigen::Matrix<float, 3, 3> R, const Eigen::Matrix<float, 3, 1> t){
        Eigen::Matrix<float,4,4> T;

        Eigen::Matrix<float,1,4> ZeroOne;
        ZeroOne<<0.0,0.0,0.0,1.0;
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        T.block<1,4>(3,0) = ZeroOne;

        return T;
    }

    Eigen::Matrix<float,4,4> CommonTools::P2T(const Eigen::Matrix<float, 3, 4> P){
        Eigen::Matrix<float,4,4> T;

        Eigen::Matrix<float,1,4> ZeroOne;
        ZeroOne<<0.0,0.0,0.0,1.0;
        T.block<3,4>(0,0) = P;
        T.block<1,4>(3,0) = ZeroOne;

        return T;

    }

    Eigen::Matrix<float,3,3> CommonTools::T2R(const Eigen::Matrix<float, 4, 4> T){
        Eigen::Matrix<float,3,3> R;

        R = T.block<3,3>(0,0) ;

        return R;
    }

    Eigen::Matrix<float,3,1> CommonTools::T2t(const Eigen::Matrix<float, 4, 4> T){
        Eigen::Matrix<float,3,1> t;

        t = T.block<3,1>(0,3) ;

        return t;
    }

    Eigen::Matrix<float,3,4> CommonTools::T2P(const Eigen::Matrix<float, 4, 4> T){
        Eigen::Matrix<float, 3, 4> P;

        P = T.block<3,4>(0,0) ;

        return P;
    }

    bool CommonTools::Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,4,4> &Tc1w ,Eigen::Matrix<float,4,4> &Tc2w , Eigen::Vector3f &x3D)
    {
        Eigen::Matrix4f A;
        A.block<1,4>(0,0) = x_c1(0) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(0,0);
        A.block<1,4>(1,0) = x_c1(1) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(1,0);
        A.block<1,4>(2,0) = x_c2(0) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(0,0);
        A.block<1,4>(3,0) = x_c2(1) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(1,0);

        Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);

        Eigen::Vector4f x3Dh = svd.matrixV().col(3);

        if(x3Dh(3)==0)
            return false;

        // Euclidean coordinates
        x3D = x3Dh.head(3)/x3Dh(3);

        return true;
    }

} //namespace ORB_SLAM
