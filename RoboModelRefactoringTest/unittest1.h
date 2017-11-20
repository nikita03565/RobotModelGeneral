#pragma once

#include "CppUnitTest.h"
#include "../RoboModelRefactoring/newRM.h"
#include "../RoboModelRefactoring/newRM.cpp"
#include "../RoboModelRefactoring/poly34.h"
#include "../RoboModelRefactoring/poly34.cpp"
using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace roboModelRefactoringTest
{
	TEST_CLASS(UnitTest1)
	{
		RoboModel _rob;
		double _angleTolerance = 0.6, _posTolerance = 0.5;
		std::vector<double> jointsToQ(std::array<double, 6> joints);
	public:

		TEST_METHOD(testMethod1);
		TEST_METHOD(testMethod2);
		TEST_METHOD(testMethod3);
		TEST_METHOD(testMethod4);
		TEST_METHOD(testMethod5);
		TEST_METHOD(testMethod6);
		TEST_METHOD(testMethod7);
		TEST_METHOD(testMethod8);
		TEST_METHOD(testMethod9);
		TEST_METHOD(testMethod10);
		TEST_METHOD(testMethod11);
		TEST_METHOD(testMethod12);
		TEST_METHOD(testMethod13);	
		TEST_METHOD(testMethod14);
		TEST_METHOD(testMethod15);
		TEST_METHOD(testMethod16);
		TEST_METHOD(testMethod17);
		TEST_METHOD(testMethod18);
	};
}