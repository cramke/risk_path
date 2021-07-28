#include "pch.h"
#include "CppUnitTest.h"
#include "../risk_path/Population.cpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace UnitTestriskpath
{
	TEST_CLASS(UnitTest_CoordinateTransformations)
	{
	public:
		PopulationMap map = PopulationMap();
		double delta = 0.001;
		TEST_METHOD(TestMethod_PopulationMap)
		{
			Assert::IsTrue(map.has_file_loaded);
		}
		TEST_METHOD(TestMethod_DoesTransformFunctionApply)
		{
			// assert that transform[2]&[4] == 0 otherwise logic error and transform wont work. regardles of the file/map used
			Assert::AreEqual(0.0, map.transform[2]);
			Assert::AreEqual(0.0, map.transform[4]);
		}
		TEST_METHOD(TestMethod_TransformMatrix)
		{
			Assert::AreEqual(0.00027777777777799998, map.transform[1]);
			Assert::AreEqual(-0.00027777777777799998, map.transform[5]);

			double delta = 0.001;
			Assert::AreEqual(5.8676388888888891, map.transform[0], delta);
			Assert::AreEqual(55.056805555555556, map.transform[3], delta);
		}
		TEST_METHOD(TestMethod_TopLeftCorner)
		{
			Coordinates point = Coordinates(0, 0, map.transform);
			double expected_lat = 55.056805555555556;
			double expected_lon = 5.8676388888888891;
			double actual_lat = point.lat;
			Assert::AreEqual(expected_lat, point.lat, delta);
			Assert::AreEqual(expected_lon, point.lon, delta);
		}
		TEST_METHOD(TestMethod_TopLeftCornerReverse)
		{
			double lat = 55.056805555555556;
			double lon = 5.8676388888888891;
			Coordinates point = Coordinates(lat, lon, map.transform);
			int expected_x = 0;
			int expected_y = 0;
			Assert::AreEqual(expected_x, point.x);
			Assert::AreEqual(expected_y, point.y);
		}
		TEST_METHOD(TestMethod_RandomPoint)
		{
			Coordinates point = Coordinates(27046, 9315, map.transform);
			double expected_lat = 52.4693;
			double expected_lon = 13.3804;
			Assert::AreEqual(expected_lat, point.lat, delta);
			Assert::AreEqual(expected_lon, point.lon, delta);
		}
		TEST_METHOD(TestMethod3_RandomPointReverse)
		{
			Coordinates point = Coordinates(52.4693, 13.3804, map.transform);
			int expected_x = 27046;
			int expected_y = 9315;
			Assert::AreEqual(expected_x, point.x);
			Assert::AreEqual(expected_y, point.y);
		}
	};
}
