#ifndef FANUC_MODEL
#define FANUC_MODEL
#include "newRM.h"
#define PI 3.14159265

/**
 * \brief class for matematics of robot manipulator Fanuc M20ia based on Denavit-Hartenberg parameters
 */
class FanucModel : RoboModel
{
	/**
	 * \brief function for conversion joints angles to Denavit-Hartenberg generalized angles
	 * \param[in] j joints angles
	 * \return Denavit-Hartenberg generalized angles
	 */
	std::vector<double> jointsToQ(std::array<double, 6> j);
public:
	/**
	 * \brief default constructor with Fanuc M20ia parameters
	 */
	FanucModel();
	
	/**
	 * \brief function for solving forward kinematic task for Fanuc M20ia
	 * \param[in] inputjoints joints angles
	 * \return coordinates of end-effector in world frame: x, y, z in mm and w, p, r in radians
	 */
	std::array<double, 6> fanucForwardTask(std::array<double, 6> inputjoints);
};

#endif
