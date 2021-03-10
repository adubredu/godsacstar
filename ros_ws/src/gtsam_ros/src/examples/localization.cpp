/*
In the creating_fg.cpp file, we use only odometry readings
to track the robot's position. This leads to compunding
uncertainty and drift.

A better way to localize the robot is to employ external
measurements like GPS, correlation of a laser range-finder
with prior map, etc. This two factor approach helps in 
constraining the pose uncertainty and ensuring better
pose estimates. 

To add external measurements to the factor graph, we would
have to create a custom UnaryFactor class. Unary because it 
depends only on the current pose.

There are different kinds of built-in factor types like
PriorFactor (for a prior in the factor graph) and 
BetweenFactor (for a factor that depends on two variables).
It seems that there is no built-in factor that depends on a 
single variable. That is why we create this custom UnaryFactor
to handle such situations.

It seems to me that we would have to do same for factors that
depend on more than two variables too.

I demonstrate this in this file.

*/

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

using namespace std; 
using namespace gtsam;


class UnaryFactor: public NoiseModelFactor1<Pose2>
{
	double mx_, my_; 

public:
	UnaryFactor (Key j, double x, double y, const SharedNoiseModel& model):
	NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

	Vector evaluateError (const Pose2& q, boost::optional<Matrix&> H = boost::none) const
	{
		if (H) (*H) = (Matrix(2,3)<< 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
		return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
	}
};



int main(int argc, char** argv)
{
	NonlinearFactorGraph graph;

	Pose2 odometry(2.0, 0.0, 0.0);
	noiseModel::Diagonal::shared_ptr odometryNoise = 
		noiseModel::Diagonal::Sigmas(Vector3 (0.2, 0.2, 0.1));

	graph.add(BetweenFactor<Pose2>(1, 2, odometry, odometryNoise));
	graph.add(BetweenFactor<Pose2>(2, 3, odometry, odometryNoise));

	noiseModel::Diagonal::shared_ptr unaryNoise = 
		noiseModel::Diagonal::Sigmas(Vector2 (0.1, 0.1));

	graph.add(boost::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
	graph.add(boost::make_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise));
	graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));

	Values initial;
	initial.insert(1, Pose2(0.5,0.0, 0.2));
	initial.insert(2, Pose2(2.3,0.1,-0.2));
	initial.insert(3, Pose2(4.1, 0.1, 0.1));

	Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();

	//Print the mean of the pose estimates
	result.print("\n means of final pose estimates: \n");

	//Print the covariance of the pose estimates
	cout.precision(2);
	Marginals marginals(graph, result);
	cout << "\n x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
	cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
	cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
	cout << endl;





	return 0;
}