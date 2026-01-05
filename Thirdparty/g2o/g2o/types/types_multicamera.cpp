
//#include "types_six_dof_expmap.h"
#include "types_multicamera.h"

#include "../core/factory.h"
#include "../stuff/macros.h"

namespace g2o {

    using namespace std;

    VertexSE3ExpmapMultiCamera::VertexSE3ExpmapMultiCamera() : BaseVertex<6, SE3Quat>() {
    }

    bool VertexSE3ExpmapMultiCamera::read(std::istream& is) {
        Vector7d est;
        for (int i=0; i<7; i++)
            is  >> est[i];
        SE3Quat cam2world;
        cam2world.fromVector(est);
        setEstimate(cam2world.inverse());
        return true;
    }

    bool VertexSE3ExpmapMultiCamera::write(std::ostream& os) const {
        SE3Quat cam2world(estimate().inverse());
        for (int i=0; i<7; i++)
            os << cam2world[i] << " ";
        return os.good();
    }

    Vector2d EdgeSE3ProjectXYZMultiCamera::cam_project(const Vector3d & trans_xyz) const{

        Vector2d proj;
        proj(0) = trans_xyz(0)/trans_xyz(2);
        proj(1) = trans_xyz(1)/trans_xyz(2);
        Vector2d res;
        res[0] = proj[0]*fx + cx;
        res[1] = proj[1]*fy + cy;
        return res;
    }

    void EdgeSE3ProjectXYZMultiCamera::setTcb(const Eigen::Matrix4d & Tcb) {

        Rcb <<  Tcb(0,0), Tcb(0,1), Tcb(0,2),
                Tcb(1,0), Tcb(1,1), Tcb(1,2),
                Tcb(2,0), Tcb(2,1), Tcb(2,2) ;
        tcb << Tcb(0,3), Tcb(1,3), Tcb(2,3);
    }

    void EdgeSE3ProjectXYZMultiCamera::setK(double fx_input,double  fy_input,double  cx_input,double  cy_input){
        fx = fx_input;
        fy = fy_input;
        cx = cx_input;
        cy = cy_input;
    }

    void EdgeSE3ProjectXYZMultiCamera::computeError()  {
//        const VertexSE3ExpmapMultiCamera* v1 = static_cast<const VertexSE3ExpmapMultiCamera*>(_vertices[1]);
//        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const VertexSE3ExpmapMultiCamera* v1 = static_cast<const VertexSE3ExpmapMultiCamera*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

        Vector2d obs(_measurement);
        //_error = obs-cam_project(v1->estimate().map(v2->estimate()));
        _error = obs - EdgeSE3ProjectXYZMultiCamera::cam_project(Rcb * (v1->estimate().map(v2->estimate())) + tcb);

//        std::cout << "_error " << _error.norm() << std::endl;
//        Eigen::Vector3d Pw = v2->estimate();
//        Eigen::Vector3d Pb = v1->estimate().map(v2->estimate());
//        Eigen::Vector3d Pc = Rcb * Pb + tcb;
//        Eigen::Vector3d p_2D =  1.0 / Pc.z() * Pc;
//        Vector2d proj = EdgeSE3ProjectXYZMultiCamera::cam_project(Rcb * (v1->estimate().map(v2->estimate())) + tcb);
//        _error = obs - proj;
//        std::cout << "v1 " << v1 << std::endl;
//        std::cout << "Tbw " << std::endl << v1->estimate().to_homogeneous_matrix().matrix() << std::endl;
//        std::cout << "v1->estimate() " << std::endl << v1->estimate() << std::endl;
//        std::cout << "tcb " << std::endl << tcb.transpose() << std::endl;
//        std::cout << "Rcb " << std::endl << Rcb << std::endl;
//        std::cout << "tcb " << std::endl << tcb.transpose() << std::endl;
//        std::cout << "Pw " << Pw.transpose() << std::endl;
//        std::cout << "Pb " << Pb.transpose() << std::endl;
//        std::cout << "Pc " << Pc.transpose() << std::endl;
//        std::cout << "K " << " fx " << fx << " fy " << fy << " cx " << cx << " cy " << cy << std::endl;
//        std::cout << "proj " << proj.transpose() << std::endl;
//        std::cout << "obs " << obs.transpose() << std::endl;
//        std::cout << "_error " << _error.transpose() << std::endl;
//        std::cout << std::endl;
    }

    bool EdgeSE3ProjectXYZMultiCamera::isDepthPositive() {
        const VertexSE3ExpmapMultiCamera* v1 = static_cast<const VertexSE3ExpmapMultiCamera*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        //return (v1->estimate().map(v2->estimate()))(2)>0.0;
        return (Rcb * (v1->estimate().map(v2->estimate())) + tcb)(2)>0.0;
    }


    EdgeSE3ProjectXYZMultiCamera::EdgeSE3ProjectXYZMultiCamera() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3ExpmapMultiCamera>() {
    }

    bool EdgeSE3ProjectXYZMultiCamera::read(std::istream& is){
        for (int i=0; i<2; i++){
            is >> _measurement[i];
        }
        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++) {
                is >> information()(i,j);
                if (i!=j)
                    information()(j,i)=information()(i,j);
            }
        return true;
    }

    bool EdgeSE3ProjectXYZMultiCamera::write(std::ostream& os) const {

        for (int i=0; i<2; i++){
            os << measurement()[i] << " ";
        }

        for (int i=0; i<2; i++)
            for (int j=i; j<2; j++){
                os << " " <<  information()(i,j);
            }
        return os.good();
    }


    void EdgeSE3ProjectXYZMultiCamera::linearizeOplus() {

        VertexSE3ExpmapMultiCamera * vj = static_cast<VertexSE3ExpmapMultiCamera *>(_vertices[1]);
        SE3Quat Tbw(vj->estimate());
        VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
        Vector3d Pw = vi->estimate();
        Vector3d Pb = Tbw.map(Pw);
        Vector3d Pc = Rcb * Pb + tcb;

        ////////////////////////////////////
        double x = Pc[0];
        double y = Pc[1];
        double z = Pc[2];
        double z_inv = 1. / z;
        Matrix<double,2,3> tmp;
        tmp <<  -fx*z_inv,  0,          x*fx*z_inv*z_inv,
                0,          -fy*z_inv,  y*fy*z_inv*z_inv;

        _jacobianOplusXi =  tmp * Rcb * Tbw.rotation().toRotationMatrix();

        ////////////////////////////////////

        Eigen::Matrix<double, 3, 6> temp2;
        temp2 <<0, Pb(2), -Pb(1), 1.0, 0, 0,
                -Pb(2), 0, Pb(0), 0, 1.0, 0,
                Pb(1), -Pb(0), 0, 0, 0, 1.0 ;

        _jacobianOplusXj =  tmp * Rcb * temp2;
        ////////////////////////////////////


    }





} // end namespace
