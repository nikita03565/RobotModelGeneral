#include "newRM.h"
#include <opencv2/core.hpp>
#include <iostream>
#define PI 3.14159265

struct RoboModel::DhParameters
{
	double _dParam;
	double _qParam;
	double _aParam;
	double _alphaParam;
	DhParameters(const double d, const double q, const double a, const double alpha): _dParam(d), _qParam(q), _aParam(a), _alphaParam(alpha)
	{
	}
};


RoboModel::RoboModel()
{
	_kinematicChain.reserve(6);
	_kinematicChain.push_back(DhParameters(0, 0, 150, PI / 2));
	_kinematicChain.push_back(DhParameters(0, 0, 790, 0));
	_kinematicChain.push_back(DhParameters(0, 0, 250, PI / 2));
	_kinematicChain.push_back(DhParameters(835, 0, 0, -PI / 2));
	_kinematicChain.push_back(DhParameters(0, 0, 0, PI / 2));
	_kinematicChain.push_back(DhParameters(100, 0, 0, 0));
}

RoboModel::~RoboModel()
{
	
}

std::array<double, 6> RoboModel::forwardTask(std::vector<double> inputq)
{	
	_kinematicChain[0]._qParam = inputq[0];
	cv::Mat transformMatrix = prevMatTransform(0);
	std::cout << transformMatrix << std::endl;
	for (int i = 1; i < inputq.size(); ++i)
	{
		_kinematicChain[i]._qParam = inputq[i];
		transformMatrix = transformMatrix * prevMatTransform(i);
		std::cout << transformMatrix << std::endl;
	}
	
	
	std::array<double, 3> wprAngles = angles(transformMatrix);

	std::array<double, 6> res;
	res[0] = transformMatrix.at<double>(0, 3);
	res[1] = transformMatrix.at<double>(1, 3);
	res[2] = transformMatrix.at<double>(2, 3);
	res[3] = wprAngles.at(0);
	res[4] = wprAngles.at(1);
	res[5] = wprAngles.at(2);

	return res;
}

cv::Mat RoboModel::prevMatTransform(const int i)
{
	cv::Mat result(4, 4, CV_64F);
	result.at<double>(0, 0) = cos(_kinematicChain[i]._qParam);
	result.at<double>(0, 1) = -cos(_kinematicChain[i]._alphaParam) * sin(_kinematicChain[i]._qParam);
	result.at<double>(0, 2) = sin(_kinematicChain[i]._alphaParam) * sin(_kinematicChain[i]._qParam);
	result.at<double>(0, 3) = _kinematicChain[i]._aParam * cos(_kinematicChain[i]._qParam);

	result.at<double>(1, 0) = sin(_kinematicChain[i]._qParam);
	result.at<double>(1, 1) = cos(_kinematicChain[i]._alphaParam) * cos(_kinematicChain[i]._qParam);
	result.at<double>(1, 2) = -sin(_kinematicChain[i]._alphaParam) * cos(_kinematicChain[i]._qParam);
	result.at<double>(1, 3) = _kinematicChain[i]._aParam * sin(_kinematicChain[i]._qParam);

	result.at<double>(2, 0) = 0;
	result.at<double>(2, 1) = sin(_kinematicChain[i]._alphaParam);
	result.at<double>(2, 2) = cos(_kinematicChain[i]._alphaParam);
	result.at<double>(2, 3) = _kinematicChain[i]._dParam;

	result.at<double>(3, 0) = result.at<double>(3, 1) = result.at<double>(3, 2) = 0;
	result.at<double>(3, 3) = 1;

	return result;
}

std::array<double, 3> RoboModel::angles(const cv::Mat p6) const
{
	std::array<double, 3> angleVector;
	angleVector.at(0) = atan2(p6.at<double>(2, 1), p6.at<double>(2, 2));
	angleVector.at(1) = atan2(-p6.at<double>(2, 0),
		sqrt(p6.at<double>(2, 1) * p6.at<double>(2, 1) + p6.at<double>(2, 2) * p6.at<double>(2, 2)));
	angleVector.at(2) = atan2(p6.at<double>(1, 0), p6.at<double>(0, 0));
	return angleVector;
}
