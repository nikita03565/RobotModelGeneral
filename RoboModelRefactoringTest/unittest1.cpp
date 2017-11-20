#include "unittest1.h"
#include <array>
#include <opencv2/core/mat.hpp>

std::vector<double> roboModelRefactoringTest::UnitTest1::jointsToQ(std::array<double, 6> j)
{
	//перевод в радианы
	for (int i = 0; i < 6; ++i)
	{
		j[i] *= PI / 180.0;
	}
	std::vector<double> q;
	q.resize(6);
	q[0] = j[0];
	q[1] = -j[1] + PI / 2;//j1 и j2 в разные стороны поворачиваются.
	q[2] = j[2] + j[1];
	q[3] = -j[3];
	q[4] = j[4];
	q[5] = -j[5];
	return q;
}

void roboModelRefactoringTest::UnitTest1::testMethod1()
{
	std::array<double, 6> joints{ { -0.077, 0.203, -0.172, 0.181, 0.264, -0.188 } };
	std::array<double, 6> resExpected {{ 1088.549, -1.450, 1037.656, -3.8, -89.907, -176.2 }};
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI, _angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod2()
{
	std::array<double, 6> joints{ { -0.077, 0.203, -0.172, 0.181, -21.464, -0.188 } };
	std::array<double, 6> resExpected{ { 1081.51, -1.554, 1000.61, 179.985, -68.364, -0.131 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod3()
{
	std::array<double, 6> joints{ { -0.077, 0.203, - 0.172, 25.263, -21.464, -0.188 } };
	std::array<double, 6> resExpected{ { 1081.51, -17.054, 1004.091, 128.205, -57.364, 46.858 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod4()
{
	std::array<double, 6> joints{ { -0.077, 0.203, 18.857, 25.263, -21.464, -0.188 } };
	std::array<double, 6> resExpected{ { 960.91, - 16.854, 1295.216, 91.765, -66.364, 79.321 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod5()
{
	std::array<double, 6> joints{ { -0.077, 0.203, - 21.141, 25.263, -21.464, -0.188 } };
	std::array<double, 6> resExpected{ { 1096.632, - 17.068, 657.586, 148.433, -40.87, 31.340 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod6()
{
	std::array<double, 6> joints{ { -0.077, 0.203, - 21.141, 25.263, -21.464, -50.062 } };
	std::array<double, 6> resExpected{ { 1096.632, -17.068, 657.586, -159.164, -46.417, -39.570 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod7()
{
	std::array<double, 6> joints{ { -0.077, 0.203, -21.141, 25.263, -21.464, 68.520 } };
	std::array<double, 6> resExpected{ { 1096.632, -17.068, 657.586, 130.538, 7.517, 84.546 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod8()
{
	std::array<double, 6> joints{ { -0.077, 18.242, - 21.141, 25.263, -21.464, 68.520 } };
	std::array<double, 6> resExpected{ { 1341.082, -17.381, 617.766, 130.538, 7.517, 84.546 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod9()
{
	std::array<double, 6> joints{ { -0.077, -17.582, -21.141, 25.263, -21.464, 68.520 } };
	std::array<double, 6> resExpected{ { 855.082, -16.381, 620.536, 130.538, 7.517, 84.546 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod10()
{
	std::array<double, 6> joints{ { -40.600, - 17.582, - 21.141, 25.263, -21.464, 68.520 } };
	std::array<double, 6> resExpected{ { 639.132, -568.311, 620.536, 130.538, 7.517, 44.024 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod11()
{
	std::array<double, 6> joints{ { 22.887, -17.582, 14.331, 25.263, -21.464, 68.520 } };
	std::array<double, 6> resExpected{ { 703.294, 279.941, 1192.951, 95.187, 3.030, 104.141 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod12()
{
	std::array<double, 6> joints{ { -24.807, 42.352, 14.331, 25.263, -21.464, 68.520 } };
	std::array<double, 6> resExpected{ { 1380.254, -655.200, 1023.551, 95.187, 3.030, 56.141 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI,_angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod13()
{
	std::array<double, 6> joints{ { -24.807, 42.352, 14.331, - 58.635, - 21.464, 68.520 } };
	std::array<double, 6> resExpected{ { 1396.844, - 611.244, 1037.061, 78.187, - 77.130, 94.953 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI, _angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod14()
{
	std::array<double, 6> joints{ { -24.807, 42.352, 14.331, - 58.635, - 103.672, 68.520 } };
	std::array<double, 6> resExpected{ { 1323.135, - 520.184, 977.434, - 160.835, - 54.500, 49.245 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI, _angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod15()
{
	std::array<double, 6> joints{ { -24.807, - 15.602, 10.431, 130.980, - 103.672, 68.520 } };
	std::array<double, 6> resExpected{ { 585.265, - 351.331, 1216.345, 44.682, 34.795, - 80.168 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI, _angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod16()
{
	std::array<double, 6> joints{ { -24.807, - 15.602, 10.431, 130.980, - 103.672, - 18.550 } };
	std::array<double, 6> resExpected{ { 585.265, - 351.331, 1216.345, 45.752, - 33.197, - 22.100 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI, _angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod17()
{
	std::array<double, 6> joints{ { -24.807, - 15.602, 10.431, 130.980, 70.376, - 18.550 } };
	std::array<double, 6> resExpected{ { 717.592, - 253.331, 1103.269, 123.446, 7.519, 128.264 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI, _angleTolerance);
	}
}

void roboModelRefactoringTest::UnitTest1::testMethod18()
{
	std::array<double, 6> joints{ { -24.807, - 15.602, 10.431, 130.980, 70.376, 52.550 } };
	std::array<double, 6> resExpected{ { 717.592, - 253.331, 1103.269, 165.203, 55.565, - 164.439 } };
	std::array<double, 6> resActual = _rob.forwardTask(jointsToQ(joints));
	for (int i = 0; i < 3; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i], _posTolerance);
	}
	for (int i = 3; i < 6; ++i)
	{
		Assert::AreEqual(resExpected[i], resActual[i] * 180. / PI, _angleTolerance);
	}
}


