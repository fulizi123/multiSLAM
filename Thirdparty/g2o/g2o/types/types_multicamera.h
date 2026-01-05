// g2o - General Graph Optimization
// Copyright (C) 2011 H. Strasdat
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Modified by Raúl Mur Artal (2014)
// Added EdgeSE3ProjectXYZ (project using focal_length in x,y directions)
// Modified by Raúl Mur Artal (2016)
// Added EdgeStereoSE3ProjectXYZ (project using focal_length in x,y directions)
// Added EdgeSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)
// Added EdgeStereoSE3ProjectXYZOnlyPose (unary edge to optimize only the camera pose)

#ifndef G2O_MULTICAMERA
#define G2O_MULTICAMERA

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "se3_ops.h"
#include "se3quat.h"
#include "types_sba.h"
#include <Eigen/Geometry>


namespace g2o {

    namespace types_multi_camera {
        void init();

        Vector2d project2d(const Vector3d& v)  ;

        Vector3d unproject2d(const Vector2d& v)  ;
    }

/**
 * \brief SE3 Vertex parameterized internally with a transformation matrix
 and externally with its exponential map
 */
    class  VertexSE3ExpmapMultiCamera : public BaseVertex<6, SE3Quat>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexSE3ExpmapMultiCamera();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        virtual void setToOriginImpl() {
            _estimate = SE3Quat();
        }

        virtual void oplusImpl(const double* update_)  {
            Eigen::Map<const Vector6d> update(update_);
            setEstimate(SE3Quat::exp(update)*estimate());
        }
    };

    class  EdgeSE3ProjectXYZMultiCamera: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3ExpmapMultiCamera>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectXYZMultiCamera();

        bool read(std::istream& is);

        bool write(std::ostream& os) const;

        void computeError() ;

        bool isDepthPositive() ;

        virtual void linearizeOplus();

        Vector2d cam_project(const Vector3d & trans_xyz) const;

        void setTcb(const Eigen::Matrix4d & Tcb);

        void setK(double fx_input,double  fy_input,double  cx_input,double  cy_input);

        double fx, fy, cx, cy;
        Eigen::Matrix3d Rcb;
        Eigen::Vector3d tcb;
    };


} // end namespace

#endif
