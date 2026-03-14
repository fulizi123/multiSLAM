
#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "CommonTools.h"

#include <math.h>
//
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_multicamera.h"


#include "System.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "LocalMapping.h"
#include "Tracking.h"

namespace LL_SLAM
{
    class System;
    class Frame;
    class KeyFrame;
    class Map;
    class LocalMapping;
    class Tracking;

    class Optimizer {
    public:
        int static PoseOptimization(Frame* pFrame);
        int static LocalBundleAdjustment(KeyFrame* pKF, Map* pMap);

    };


}
#endif // OPTIMIZER_H
