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

#ifndef G2O_OBJECT
#define G2O_OBJECT

#include "../core/base_vertex.h"
#include "../core/base_binary_edge.h"
#include "../core/base_unary_edge.h"
#include "se3_ops.h"
#include "se3quat.h"
#include "types_sba.h"
#include <Eigen/Geometry>

namespace g2o {
    namespace types_object {
        void init();
    }

    using namespace Eigen;
    using namespace std;

    typedef Matrix<double, 6, 6> Matrix6d;
    typedef Matrix<double, 1, 1> Vector1d;


    class VertexObject : public BaseVertex<9, pair<SE3Quat, Vector3d>> {
        // tx, ty, tz, thetax, thetay, thetaz, sx, sy, sz
        // thetax, thetay, thetaz, tx, ty, tz, sx, sy, sz ???
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexObject();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        virtual void setToOriginImpl() {
            cout << "VertexObject::setToOriginImpl begin" << endl;
            _estimate.first = SE3Quat();
            _estimate.second.fill(0.);
            cout << "VertexObject::setToOriginImpl end" << endl;
        }

        virtual void oplusImpl(const double *update_) {
            cout << "VertexObject::oplusImpl begin" << endl;
            pair<SE3Quat, Vector3d> new_estimate = _estimate;

            Eigen::Map<const Vector6d> update_pose(update_);
            //new_estimate.first = SE3Quat::exp(update_pose)*new_estimate.first;
            new_estimate.first = new_estimate.first * SE3Quat::exp(update_pose);

            Eigen::Map<const Vector3d> update_s(update_ + 6);
            new_estimate.second += update_s;

            setEstimate(new_estimate);
            cout << "VertexObject::oplusImpl end" << endl;
        }
    };


    /* SE3Quat : Tcw
     * dx : thetax, thetay, thetaz, tx, ty, tz???
     *
     *
     *
     */

    class VertexSE3Right : public BaseVertex<6, SE3Quat> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        VertexSE3Right();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;

        virtual void setToOriginImpl() {
            cout << "VertexSE3Right::setToOriginImpl begin" << endl;
            _estimate = SE3Quat();
            cout << "VertexSE3Right::setToOriginImpl end" << endl;
        }

        virtual void oplusImpl(const double *update_) {
            cout << "VertexSE3Right::oplusImpl begin" << endl;
            Eigen::Map<const Vector6d> update(update_);
            //setEstimate(SE3Quat::exp(update) * estimate());
            setEstimate(estimate() * SE3Quat::exp(update));
            cout << "VertexSE3Right::oplusImpl begin" << endl;
        }
    };


    class EdgeSE3ProjectObject : public BaseBinaryEdge<1, Vector4d, VertexObject, VertexSE3Right> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        EdgeSE3ProjectObject();

        bool read(std::istream &is);

        bool write(std::ostream &os) const;
        Vector1d myComputeError(const VertexSE3Right *v1, const VertexObject *v2, const Vector4d obs) ;


        void computeError() {
            cout << "EdgeSE3ProjectObject::computeError begin" << endl;
            //cout << "EdgeSE3ProjectObject::computeError 1" << endl;
            const VertexSE3Right *v1 = static_cast<const VertexSE3Right *>(_vertices[1]);
            //cout << "EdgeSE3ProjectObject::computeError 2" << endl;
            const VertexObject *v2 = static_cast<const VertexObject *>(_vertices[0]);
            //cout << "EdgeSE3ProjectObject::computeError 3" << endl;
            const Vector4d obs = _measurement;
            //todo
            //cout << "EdgeSE3ProjectObject::computeError 4" << endl;
            _error = myComputeError(v1, v2, obs);
            //_error = Vector1d(1.0);
            cout << "EdgeSE3ProjectObject::computeError end" << endl;
        }

        bool isDepthPositive() {
            cout << "EdgeSE3ProjectObject::isDepthPositive begin" << endl;
            const VertexSE3Right *v1 = static_cast<const VertexSE3Right *>(_vertices[1]);
            const VertexObject *v2 = static_cast<const VertexObject *>(_vertices[0]);
            return (v1->estimate().map(v2->estimate().first.translation()))(2) > 0.0;
            cout << "EdgeSE3ProjectObject::isDepthPositive end" << endl;
        }


        virtual void linearizeOplus();

        Vector2d cam_project(const VertexSE3Right *v1, const VertexObject *v2) const;

//        Vector1d ComputeError(const VertexSE3Right *v1,
//                              const VertexObject *v2,
//                              const Vector4f Line) const;

        double fx, fy, cx, cy;
    };


} // end namespace

#endif
