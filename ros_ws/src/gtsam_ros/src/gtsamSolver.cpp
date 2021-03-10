#include "gtsamSolver.h"


PoseNetiSam::PoseNetiSam()
{
	float relinearizeThreshold = 0.01;
	float relinearizeSkip = 1;

	parameters.relinearizeThreshold = relinearizeThreshold ;
	parameters.relinearizeSkip = relinearizeSkip;

	ISAM2 iss(parameters);
	isam = iss;

	currentKey = 1; 
	currentPose.push_back(0); 
	currentPose.push_back(0); 
	currentPose.push_back(0);
	
}

vector<float>  PoseNetiSam::motion_model(vector<float> odometry)
{
	vector<float> predPose;
	predPose.push_back(currentPose[0]+odometry[0]);
	predPose.push_back(currentPose[1]+odometry[1]);
	predPose.push_back(currentPose[2]+odometry[2]);

	return predPose;
}


void PoseNetiSam::initialize(vector<float> pm,vector<float> pc)
{
	Pose2 priorMean(pm[0], pm[1], pm[2]);

	noiseModel::Diagonal::shared_ptr priorCov = 
		noiseModel::Diagonal::Sigmas(Vector3 (pc[0],pc[1],pc[2]));

	graph.add(PriorFactor<Pose2>(Symbol('x', currentKey), 
		priorMean, priorCov));

	initialValues.insert(Symbol('x', currentKey), priorMean); 

}


void PoseNetiSam::step(vector<float> odometry, vector<float> noise)
{
	Pose2 odometryGT(odometry[0], odometry[1], odometry[2]);

	noiseModel::Diagonal::shared_ptr odometryNoise = 
		noiseModel::Diagonal::Variances(Vector3 (noise[0],noise[1],noise[2]));
	
	graph.add(BetweenFactor<Pose2>(Symbol('x', currentKey), 
		Symbol('x', currentKey+1), odometryGT, odometryNoise));


	//adding the initial values
	vector<float> predMean = motion_model(odometry);
	Pose2 initialVal(predMean[0], predMean[1], predMean[2]);
	initialValues.insert(Symbol('x', currentKey+1), initialVal);

	currentKey++;
	currentPose = predMean;
}
 

void PoseNetiSam::addObs(vector<float> mm, vector<float> noise)
{

	Pose2 measurement(mm[0], mm[1], mm[2]);
	noiseModel::Diagonal::shared_ptr measurementNoise = 
		noiseModel::Diagonal::Variances(Vector3 (noise[0],noise[1],noise[2]));

	graph.add(PriorFactor<Pose2>(Symbol('x', currentKey), 
		measurement, measurementNoise));

}


vector<float> PoseNetiSam::update(int updateNum)
{
	isam.update(graph, initialValues);
	updateNum--;

	while (updateNum > 0)
	{
		isam.update();
		updateNum--;
	}

	graph.resize(0);
	initialValues.clear();
	currentEst = isam.calculateEstimate();

	currentPose[0] = currentEst.at<gtsam::Pose2>(Symbol('x', currentKey)).x();
	currentPose[1] = currentEst.at<gtsam::Pose2>(Symbol('x', currentKey)).y();
	currentPose[2] = currentEst.at<gtsam::Pose2>(Symbol('x', currentKey)).theta(); 

	return currentPose;
}


vector<float> PoseNetiSam::getEstimate(int id)
{
	vector<float> estimate;
	estimate.push_back(currentEst.at<gtsam::Pose2>(Symbol('x', currentKey)).x());
	estimate.push_back(currentEst.at<gtsam::Pose2>(Symbol('x', currentKey)).y());
	estimate.push_back(currentEst.at<gtsam::Pose2>(Symbol('x', currentKey)).theta());

	return estimate;
}


void PoseNetiSam::printGraph()
{
	graph.print();
}


void PoseNetiSam::printResult()
{
	cout << "Current estimate is x: "<< currentPose[0]<<" | y: "<<
	currentPose[1]<<" | theta: "<<currentPose[2]<<endl;
}




int main (int argc, char ** argv)
{
	PoseNetiSam isam();
	return 0;
}