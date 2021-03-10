#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

using namespace std; 
using namespace gtsam;


int main(int argc, char **argv)
{
	//Defining the factor graph
	NonlinearFactorGraph graph;

	Pose2 priorMean(0.0, 0.0, 0.0);
	noiseModel::Diagonal::shared_ptr priorNoise =
					noiseModel::Diagonal::Sigmas(Vector3 (0.3, 0.3, 0.1));
	graph.add(PriorFactor<Pose2>(1, priorMean, priorNoise));

	Pose2 odometry(2.0, 0.0, 0.0);
	noiseModel::Diagonal::shared_ptr odometryNoise = 
				noiseModel::Diagonal::Sigmas (Vector3 (0.2, 0.2, 0.1));
	graph.add(BetweenFactor<Pose2>(1,2, odometry, odometryNoise));
	graph.add(BetweenFactor<Pose2>(2,3, odometry, odometryNoise));

	//Defining the inaccurate states
	Values initial;
	initial.insert(1, Pose2(0.5, 0.0, 0.2));
	initial.insert(2, Pose2(2.3, 0.1, -0.2));
	initial.insert(3, Pose2(4.1, 0.1, 0.1));

	Values result = LevenbergMarquardtOptimizer(graph,initial).optimize();
	result.print("result from optimization:\n");

	return 0;

}