#ifndef GTSAM_SOLVER_H
#define GTSAM_SOLVER_H

#include <vector>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h> 
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

using namespace std; 
using namespace gtsam;

class PoseNetiSam
{
public:
	PoseNetiSam();
	void initialize(vector<float> priorMean,vector<float> priorCov);
	void step(vector<float> odometry, vector<float> odometryNoise);
	void addObs(vector<float> measurement, vector<float> measurementNoise);
	vector<float> update(int updateNum);
	vector<float> getEstimate(int id);
	void printGraph();
	void printResult();

private:
	vector<float>  motion_model(vector<float> odometry);
	NonlinearFactorGraph graph;
	ISAM2 isam;
	ISAM2Params parameters;
	Values initialValues;
	int currentKey;
	Values currentEst;
	vector<float> currentPose;

};
#endif