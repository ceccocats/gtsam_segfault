#include <GraphOpt.h>
//#include <gtsam/sam/BearingFactor.h>
//#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
//#include <gtsam/navigation/GPSFactor.h>
//#include <gtsam/navigation/ImuFactor.h>
//#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>

#include <fstream>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>


// Define the Boost export macros:
#include <boost/serialization/export.hpp> // BOOST_CLASS_EXPORT_GUID
#include <boost/serialization/serialization.hpp>
#include <gtsam/base/GenericValue.h> // GTSAM_VALUE_EXPORT

namespace boost {
namespace serialization {
template <class Archive, typename Derived>
void serialize(Archive &ar, Eigen::EigenBase<Derived> &g,
               const unsigned int version) {
  ar &boost::serialization::make_array(g.derived().data(), g.size());
}
} // namespace serialization
} // namespace boost

/* Create GUIDs for Noisemodels */
/* ************************************************************************* */
// clang-format off
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsam_noiseModel_Constrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsam_noiseModel_Diagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsam_noiseModel_Gaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsam_noiseModel_Unit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic,"gtsam_noiseModel_Isotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsam_noiseModel_Robust");

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base, "gtsam_noiseModel_mEstimator_Base");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null,"gtsam_noiseModel_mEstimator_Null");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair, "gtsam_noiseModel_mEstimator_Fair");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber,"gtsam_noiseModel_mEstimator_Huber");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Cauchy,"gtsam_noiseModel_mEstimator_Cauchy");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsam_noiseModel_mEstimator_Tukey");

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsam_SharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsam_SharedDiagonal");
// clang-format on

/* Create GUIDs for geometry */
/* ************************************************************************* */
GTSAM_VALUE_EXPORT(gtsam::Point2);
GTSAM_VALUE_EXPORT(gtsam::Point3);
GTSAM_VALUE_EXPORT(gtsam::Rot2);
GTSAM_VALUE_EXPORT(gtsam::Rot3);
GTSAM_VALUE_EXPORT(gtsam::Pose2);
GTSAM_VALUE_EXPORT(gtsam::Pose3);
GTSAM_VALUE_EXPORT(gtsam::Cal3Fisheye);

/* Create GUIDs for factors */
/* ************************************************************************* */
// clang-format off
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsam::JacobianFactor");

BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Point3>, "gtsam::ExpressionFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::ExpressionFactor<gtsam::Rot3>, "gtsam::ExpressionFactor<gtsam::Rot3>");

// Add your custom factors, if any.

BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Point3>, "gtsam::BetweenFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::BetweenFactor<gtsam::Rot3>, "gtsam::BetweenFactor<gtsam::Rot3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Point3>, "gtsam::PriorFactor<gtsam::Point3>");
BOOST_CLASS_EXPORT_GUID(gtsam::PriorFactor<gtsam::Rot3>, "gtsam::PriorFactor<gtsam::Rot3>");

typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3Fisheye> GenericProjectionFactorCal3Fisheye;
BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3Fisheye, "GenericProjectionFactorCal3Fisheye");


namespace basalt {

void GraphOpt::save(std::string binaryPath) {
    std::cout<<"save graph: "<<binaryPath<<"\n";    
    std::ofstream stream(binaryPath);
    boost::archive::binary_oarchive archive(stream);
    archive << gtSAMgraph;
    archive << initialEstimate;
    stream.close();
    std::cout<<"graph size: "<<gtSAMgraph.size()<<" estimate size: "<<initialEstimate.size()<<"\n";
}

void GraphOpt::load(std::string binaryPath) {
    std::cout<<"load graph: "<<binaryPath<<"\n";
    static int n = 0;
    std::ifstream stream(binaryPath);
    boost::archive::binary_iarchive archive(stream);
    archive >> gtSAMgraph;
    archive >> initialEstimate;
    stream.close();
    std::cout<<"graph size: "<<gtSAMgraph.size()<<" estimate size: "<<initialEstimate.size()<<"\n";
}

void GraphOpt::optimize(int maxIterations) {
    //gtSAMgraph.print("graph:\n");

    gtsam::LevenbergMarquardtParams parameters;
    //parameters.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::ERROR;
    //parameters.absoluteErrorTol = 1e-10;
    //parameters.relativeErrorTol = 1e-10;
    parameters.maxIterations = maxIterations;
    parameters.lambdaUpperBound = 1e20;
    //parameters.linearSolverType = gtsam::NonlinearOptimizerParams::LinearSolverType::SEQUENTIAL_QR;
    parameters.orderingType = gtsam::Ordering::OrderingType::METIS;
    //parameters.print();
    gtsam::LevenbergMarquardtOptimizer optimizer(gtSAMgraph, initialEstimate, parameters);

    std::cout<<"optimize\n";
    estimate = optimizer.optimize();

    //estimate.print("Final Result:\n");
    //std::cout<<"iterations: "<<optimizer.iterations()<<"\n";
    //std::cout<<"Initial error: "<<gtSAMgraph.error(initialEstimate)<<"\n";
    //std::cout<<"Final error: "<<gtSAMgraph.error(estimate)<<"\n";
}


}