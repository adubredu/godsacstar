#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

using namespace std; 
using namespace gtsam;

int main(int argc, char** argv)
{
	NonlinearFactorGraph graph;
	
	noiseModel::Diagonal::shared_ptr priorNoise =
					noiseModel::Diagonal::Sigmas(Vector3 (0.3, 0.3, 0.1));
	
	noiseModel::Diagonal::shared_ptr odometryNoise = 
				noiseModel::Diagonal::Sigmas (Vector3 (0.2, 0.2, 0.1));

	graph.add(PriorFactor<Pose2>(1, Pose2(0,0,0), priorNoise));
	graph.add(BetweenFactor<Pose2>(1,2, Pose2(2,0,0), odometryNoise));
	graph.add(BetweenFactor<Pose2>(2,3, Pose2(2,0,M_PI_2), odometryNoise));
	graph.add(BetweenFactor<Pose2>(3,4, Pose2(2,0,M_PI_2), odometryNoise));
	graph.add(BetweenFactor<Pose2>(4,5, Pose2(2,0,M_PI_2), odometryNoise));
	graph.add(BetweenFactor<Pose2>(5,2, Pose2(2,0,M_PI_2), odometryNoise));

	Values initial;
	initial.insert(1, Pose2(0,0,0));
	initial.insert(2, Pose2(2,0,0));
	initial.insert(3, Pose2(4,0,0));
	initial.insert(4, Pose2(4,2,0));
	initial.insert(5, Pose2(2,2,0));

	Values result;

	result = LevenbergMarquardtOptimizer(graph, initial).optimize();

	result.print("\n Means of final estimate:\n");

	cout.precision(2);
	Marginals marginals(graph, result);
	cout << "\n x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
	cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
	cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
	cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;
	cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;
	cout << endl;
}