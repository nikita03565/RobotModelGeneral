#pragma once
#include <vector>
#include <opencv2/core/mat.hpp>
#include <array>
/**
 * \brief class for matematics of robot manipulator based on Denavit-Hartenberg parameters
 */
class RoboModel
{
private:

	/**
	 * \brief struct of D-H parameters
	 * 
	 * d: offset along previous z to the common normal;
	 * q: angle about previous  z, from old  x to new  x;
	 * a: offset along x in current frame;
	 * alpha: angle about x in current frame;
	 */
	struct DhParameters;
	std::vector<DhParameters> _kinematicChain;
	cv::Mat prevMatTransform(const int i);
	std::array<double, 3> angles(const cv::Mat p6) const;
public:
	RoboModel();
	~RoboModel();
	std::array<double, 6> forwardTask(std::vector<double> inputq);
};
