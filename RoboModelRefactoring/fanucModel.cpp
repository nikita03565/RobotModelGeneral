#include "fanucModel.h"
#define PI 3.14159265

FanucModel::FanucModel(): 
	RoboModel(std::vector<std::array<double, 4>>{
	{0, 0, 150, PI / 2},
	{0, 0, 790, 0},
	{0, 0, 250, PI / 2},
	{835, 0, 0, -PI / 2},
	{0, 0, 0, PI / 2},
	{100, 0, 0, 0}})
{
}

std::vector<double> FanucModel::jointsToQ(std::array<double, 6> j)
{
	//degrees to radians
	for (int i = 0; i < 6; ++i)
	{
		j[i] *= PI / 180.0;
	}
	std::vector<double> q;
	q.resize(6);
	q[0] = j[0];
	q[1] = -j[1] + PI / 2;
	q[2] = j[2] + j[1];
	q[3] = -j[3];
	q[4] = j[4];
	q[5] = -j[5];
	return q;
}

std::array<double, 6> FanucModel::fanucForwardTask(std::array<double, 6> inputjoints)
{
	std::vector<double> q = jointsToQ(inputjoints);
	return forwardTask(q);
}
