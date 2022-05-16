#pragma once 
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

namespace basalt {

class GraphOpt {
public:
    GraphOpt() {      
    }
    ~GraphOpt() {}

    void save(std::string binaryPath);
    void load(std::string binaryPath);

    void optimize(int maxIterations = 1);

    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::Values    estimate;
    gtsam::Marginals marginals;
};

}