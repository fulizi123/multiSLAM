
#include "types_object.h"

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o {

    using namespace std;

    Eigen::Matrix<double,4,4> Rt2T(const Eigen::Matrix<double, 3, 3> R, const Eigen::Matrix<double, 3, 1> t){
        Eigen::Matrix<double,4,4> T;

        Eigen::Matrix<double,1,4> ZeroOne;
        ZeroOne<<0.0,0.0,0.0,1.0;
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        T.block<1,4>(3,0) = ZeroOne;

        return T;
    }



    VertexObject::VertexObject() : BaseVertex<9, pair<SE3Quat, Vector3d>>() {
    }

    bool VertexObject::read(std::istream& is) {
        Vector7d est;
        for (int i=0; i<7; i++)
            is  >> est[i];
        Vector3d S;
        for (int i=0; i<3; i++)
            is  >> S[i];

        pair<SE3Quat, Vector3d> new_estimate = _estimate;
        SE3Quat cam2world;
        cam2world.fromVector(est);
        new_estimate.first = cam2world.inverse();
        new_estimate.second = S;

        setEstimate(new_estimate);

        return true;
    }

    bool VertexObject::write(std::ostream& os) const {
        SE3Quat cam2world(estimate().first.inverse());
        for (int i=0; i<7; i++)
            os << cam2world[i] << " ";

        Vector3d S = estimate().second;
        for (int i=0; i<3; i++)
            os << S[i] << " ";

        return os.good();
    }


    VertexSE3Right::VertexSE3Right() : BaseVertex<6, SE3Quat>() {
    }

    bool VertexSE3Right::read(std::istream& is) {
        Vector7d est;
        for (int i=0; i<7; i++)
            is  >> est[i];
        SE3Quat cam2world;
        cam2world.fromVector(est);
        setEstimate(cam2world.inverse());
        cout << "VertexSE3Right::read" << endl;
        return true;
    }

    bool VertexSE3Right::write(std::ostream& os) const {
        SE3Quat cam2world(estimate().inverse());
        for (int i=0; i<7; i++)
            os << cam2world[i] << " ";
        cout << "VertexSE3Right::write" << endl;
        return os.good();
    }


    Vector1d EdgeSE3ProjectObject::myComputeError(const VertexSE3Right *v1, const VertexObject *v2, const Vector4d obs) {
        cout << "EdgeSE3ProjectObject::myComputeError begin" << endl;
        SE3Quat Tcw(v1->estimate());
        Eigen::Matrix3d Rcw = Tcw.rotation().toRotationMatrix();
        Eigen::Vector3d tcw = Tcw.translation();

        SE3Quat Two(v2->estimate().first);
        Eigen::Matrix3d Rwo = Two.rotation().toRotationMatrix();
        Eigen::Vector3d two = Two.translation();
        Eigen::Vector3d S = v2->estimate().second;

        Eigen::Vector3d Line = obs.block<3, 1>(0, 0);

        Eigen::Matrix3d Q2D =
                Rcw * Rwo * S.asDiagonal().toDenseMatrix() * S.asDiagonal().toDenseMatrix() * Rwo.transpose() *
                Rcw.transpose() -
                Rcw * two * two.transpose() * Rcw.transpose() -
                Rcw * two * tcw.transpose() -
                tcw * two.transpose() * Rcw.transpose() -
                tcw * tcw.transpose();

        Eigen::Matrix<double, 1, 1> residualsMatrix = Line.transpose() * Q2D * Line;

        double f = fx;
        Eigen::Vector3d ScalerVector = Q2D.block<3, 1>(0, 2);
        double Scaler = f * 1.0 / (2.0 * (ScalerVector.transpose() * Line)(0, 0));

        Eigen::Matrix<double, 1, 1> e = Scaler * residualsMatrix;
        cout << "EdgeSE3ProjectObject::myComputeError end" << endl;
        return e;
    }

    Vector2d EdgeSE3ProjectObject::cam_project(const VertexSE3Right *v1, const VertexObject *v2) const{
        cout << "EdgeSE3ProjectObject::cam_project begin" << endl;

        SE3Quat Tcw(v1->estimate());
        Eigen::Matrix3d Rcw = Tcw.rotation().toRotationMatrix();
        Eigen::Vector3d tcw = Tcw.translation();

        SE3Quat Two(v2->estimate().first);
        Eigen::Matrix3d Rwo = Two.rotation().toRotationMatrix();
        Eigen::Vector3d two = Two.translation();
        Eigen::Vector3d S = v2->estimate().second;

        cout << "EdgeSE3ProjectObject::cam_project end" << endl;
        return (v1->estimate().map(v2->estimate().first.translation())).block<2, 1>(0, 0);
    }

    EdgeSE3ProjectObject::EdgeSE3ProjectObject() : BaseBinaryEdge<1, Vector4d, VertexObject, VertexSE3Right>() {
    }

    bool EdgeSE3ProjectObject::read(std::istream& is){
        for (int i=0; i<=4; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<=1; i++)
            for (int j=i; j<=1; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectObject::write(std::ostream& os) const {

        for (int i=0; i<=4; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<=1; i++)
            for (int j=i; j<=1; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }

    void EdgeSE3ProjectObject::linearizeOplus() {;
        cout << "EdgeSE3ProjectObject::linearizeOplus begin" << endl;
        //VertexSE3Right * vj = static_cast<VertexSE3Right *>(_vertices[1]);
        //VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        VertexSE3Right *vj = static_cast<VertexSE3Right *>(_vertices[1]);
        SE3Quat Tcw_SE3Quat(vj->estimate());
        Eigen::Matrix3d Rcw = Tcw_SE3Quat.rotation().toRotationMatrix();
        Eigen::Vector3d tcw = Tcw_SE3Quat.translation();
        Eigen::Matrix<double,4,4> Tcw = Rt2T(Rcw, tcw);

        VertexObject *vi = static_cast<VertexObject *>(_vertices[0]);
        SE3Quat Two_SE3Quat(vi->estimate().first);
        Eigen::Matrix3d Rwo = Two_SE3Quat.rotation().toRotationMatrix();
        Eigen::Vector3d two = Two_SE3Quat.translation();
        Eigen::Vector3d S = vi->estimate().second;
        Eigen::Matrix<double,4,4> Two = Rt2T(Rwo, two);

        Eigen::Vector4d Line = measurement();


        Eigen::Matrix<double, 1, 9> Jf;
        Eigen::Matrix<double, 1, 9> Jg;
        Eigen::Matrix<double, 1, 6> JfTcw;
        Eigen::Matrix<double, 1, 6> JgTcw;
        Eigen::Matrix<double, 4, 6> JLsv;
        Eigen::Matrix<double, 4, 6> JLo;

        Eigen::Matrix<double, 1, 9> JQ;
        Eigen::Matrix<double, 1, 6> JTcw;
        /////////////////////////////Q9

        Eigen::Matrix<double, 4, 4> Qo = Eigen::Vector4d(S(0) * S(0),
                                                        S(1) * S(1),
                                                        S(2) * S(2),
                                                        -1).asDiagonal().toDenseMatrix();
        Eigen::Matrix<double, 4, 4> Q = Two * Qo * Two.transpose();

        Eigen::Vector4d ScalerVector;
        ScalerVector << 0, 0, 1, 0;


        Eigen::Matrix<double, 1, 1> fzeta = Line.transpose() * Tcw * Q * Tcw.transpose() * Line;
        Eigen::Matrix<double, 1, 1> gzeta = ScalerVector.transpose() * Tcw * Q * Tcw.transpose() * Line;

        double InsWeight = 1.0;
        Eigen::Matrix<double, 1, 1> e;
        e << InsWeight * fx * fzeta(0, 0) / (2.0 * gzeta(0, 0));


        JQ.setZero();
        //fzeta
        Eigen::Vector4d Lo = Two.transpose() * Tcw.transpose() * Line;
        JLo << 0, 0, 0, 0, -Lo(2), Lo(1),
                0, 0, 0, Lo(2), 0, -Lo(0),
                0, 0, 0, -Lo(1), Lo(0), 0,
                Lo(0), Lo(1), Lo(2), 0, 0, 0;
        Jf.block<1, 6>(0, 0) = 2 * (Qo * Lo).transpose() * JLo;
        //s
        Jf.block<1, 3>(0, 6) = Eigen::Vector3d(2 * S(0) * Lo(0) * Lo(0),
                                               2 * S(1) * Lo(1) * Lo(1),
                                               2 * S(2) * Lo(2) * Lo(2)).transpose();

        //gzeta
        Eigen::Vector4d Lsv = Two.transpose() * Tcw.transpose() * ScalerVector;
        JLsv << 0, 0, 0, 0, -Lsv(2), Lsv(1),
                0, 0, 0, Lsv(2), 0, -Lsv(0),
                0, 0, 0, -Lsv(1), Lsv(0), 0,
                Lsv(0), Lsv(1), Lsv(2), 0, 0, 0;
        Jg.block<1, 6>(0, 0) = (Qo * Lsv).transpose() * JLo + (Qo * Lo).transpose() * JLsv;

        Jg.block<1, 3>(0, 6) = Eigen::Vector3d(2 * S(0) * Lsv(0) * Lo(0),
                                               2 * S(1) * Lsv(1) * Lo(1),
                                               2 * S(2) * Lsv(2) * Lo(2)).transpose();

        JQ = InsWeight * fx * (gzeta(0, 0) * Jf - fzeta(0, 0) * Jg) / (2.0 * gzeta(0, 0) * gzeta(0, 0));

        ///////////////////////////////Tcw
        JTcw.setZero();

        Lo = Tcw.transpose() * Line;
        JLo << 0, 0, 0, 0, -Lo(2), Lo(1),
                0, 0, 0, Lo(2), 0, -Lo(0),
                0, 0, 0, -Lo(1), Lo(0), 0,
                Lo(0), Lo(1), Lo(2), 0, 0, 0;
        JfTcw.block<1, 6>(0, 0) = 2 * (Two * Qo * Two.transpose() * Lo).transpose() * JLo;


        Lsv = Tcw.transpose() * ScalerVector;
        JLsv << 0, 0, 0, 0, -Lsv(2), Lsv(1),
                0, 0, 0, Lsv(2), 0, -Lsv(0),
                0, 0, 0, -Lsv(1), Lsv(0), 0,
                Lsv(0), Lsv(1), Lsv(2), 0, 0, 0;
        JgTcw.block<1, 6>(0, 0) = (Two * Qo * Two.transpose() * Lsv).transpose() * JLo + (Two * Qo * Two.transpose() * Lo).transpose() * JLsv;

        JTcw = InsWeight * fx * (gzeta(0, 0) * JfTcw - fzeta(0, 0) * JgTcw) / (2.0 * gzeta(0, 0) * gzeta(0, 0));

        Eigen::Matrix<double, 1, 9> JQ_g2o;
        Eigen::Matrix<double, 1, 6> JTcw_g2o;
        JQ_g2o.block<1, 3>(0, 0) = JQ.block<1,3>(0, 3);
        JQ_g2o.block<1, 3>(0, 3) = JQ.block<1,3>(0, 0);
        JQ_g2o.block<1, 3>(0, 6) = JQ.block<1,3>(0, 6);
        JTcw_g2o.block<1, 3>(0, 0) = JTcw.block<1,3>(0, 3);
        JTcw_g2o.block<1, 3>(0, 3) = JTcw.block<1,3>(0, 0);

        _jacobianOplusXi = JQ_g2o;
        _jacobianOplusXj = JTcw_g2o;
        cout << "EdgeSE3ProjectObject::linearizeOplus end" << endl;

    }






} // end namespace
