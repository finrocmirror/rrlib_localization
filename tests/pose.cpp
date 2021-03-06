//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//----------------------------------------------------------------------
/*!\file    rrlib/localization/tests/pose.cpp
 *
 * \author  Tobias Föhst
 *
 * \date    2014-06-05
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include "rrlib/util/tUnitTestSuite.h"

#include <cstring>

#include "rrlib/localization/tPose.h"
#include "rrlib/localization/tUncertainPose.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Namespace declaration
//----------------------------------------------------------------------
namespace rrlib
{
namespace localization
{

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

class TestPose : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(TestPose);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors2D);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors3D);
  RRLIB_UNIT_TESTS_ADD_TEST(AccessOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ComparisonOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(AssignmentOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ArithmeticOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ReferenceTransformations);
  RRLIB_UNIT_TESTS_ADD_TEST(Streaming);
  RRLIB_UNIT_TESTS_ADD_TEST(UnitChanges);
  RRLIB_UNIT_TESTS_ADD_TEST(Uncertainty);
  RRLIB_UNIT_TESTS_ADD_TEST(MultiplyVelocityWithFactor);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  void Constructors2D()
  {
    RRLIB_UNIT_TESTS_EQUALITY(3 * sizeof(double), sizeof(tPose2D<double>));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(2), tPose2D<double>::cDIMENSION);

    double raw[3] = { 0.0, 0.0, 0.0 };

    tPose2D<double> zero;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero, &raw, sizeof(raw)) == 0);

    raw[0] = 2.0, raw[1] = 3.0;
    raw[2] = -M_PI_2;
    tPose2D<double> pose1(tPosition2D<>(2.0, 3.0), tOrientation2D<>(math::tAngle<double, math::angle::Degree>(-90)));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose1, &raw, sizeof(raw)) == 0);

    raw[0] = 1.0, raw[1] = 2.0;
    raw[2] = M_PI_2;
    tPose2D<double> pose2(1.0, 2.0, math::tAngle<double, math::angle::Degree>(90));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose2, &raw, sizeof(raw)) == 0);

    raw[0] = 2.0, raw[1] = 3.0;
    raw[2] = -M_PI_2;
    tPose2D<double> pose3(math::tMatrix<3, 3, double>(0, 1, 2, -1, 0, 3, 0, 0, 1));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose3, &raw, sizeof(raw)) == 0);

    tPose2D<double> copy(pose2);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &pose2, sizeof(pose2)) == 0);

    tPose2D<float> converted(pose2);
    float converted_raw[3] = { 1.0, 2.0, M_PI_2 };
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted, &converted_raw, sizeof(converted_raw)) == 0);
  }

  void Constructors3D()
  {
    RRLIB_UNIT_TESTS_EQUALITY(6 * sizeof(double), sizeof(tPose3D<double>));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(3), tPose3D<double>::cDIMENSION);

    double raw[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    tPose3D<double> zero;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero, &raw, sizeof(raw)) == 0);

    raw[0] = 2.0, raw[1] = 3.0;
    raw[2] = 4.0, raw[3] = 0.0, raw[4] = M_PI_2;
    raw[5] = -M_PI_2;
    tPose3D<double> pose1(tPosition3D<>(2.0, 3.0, 4.0), tOrientation3D<>(math::tAngle<double, math::angle::Degree>(0), math::tAngle<double, math::angle::Degree>(90), math::tAngle<double, math::angle::Degree>(-90)));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose1, &raw, sizeof(raw)) == 0);

    raw[0] = 3.0, raw[1] = 4.0;
    raw[2] = 5.0, raw[3] = M_PI_2, raw[4] = -M_PI_2;
    raw[5] = 0.0;
    tPose3D<double> pose2(3.0, 4.0, 5.0, math::tAngle<double, math::angle::Degree>(90), math::tAngle<double, math::angle::Degree>(-90), math::tAngle<double, math::angle::Degree>(0));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose2, &raw, sizeof(raw)) == 0);

    raw[0] = 2.0, raw[1] = 3.0;
    raw[2] = 4.0;
    raw[3] = 0.0, raw[4] = 0.0;
    raw[5] = -M_PI_2;
    tPose3D<double> pose3(math::tMatrix<4, 4, double>(0, 1, 0, 2, -1, 0, 0, 3, 0, 0, 1, 4, 0, 0, 0, 1));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&pose3, &raw, sizeof(raw)) == 0);

    tPose3D<double> copy(pose2);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &pose2, sizeof(pose2)) == 0);

    tPose3D<float> converted(pose2);
    float converted_raw[6] = { 3.0, 4.0, 5.0, M_PI_2, -M_PI_2, 0.0 };
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted, &converted_raw, sizeof(converted_raw)) == 0);
  }

  void AccessOperators()
  {
    typedef tPose2D<double>::tOrientationComponent<double, math::angle::Degree> tDegree2D;
    tPose2D<double> pose_2d(1, 2, tDegree2D(90));
    RRLIB_UNIT_TESTS_EQUALITY(1.0, pose_2d.X().Value());
    RRLIB_UNIT_TESTS_EQUALITY(2.0, pose_2d.Y().Value());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, pose_2d.Yaw().Value().Value());
    pose_2d.X() = 3;
    pose_2d.Y() = 4;
    pose_2d.Yaw() = tDegree2D(-90);
    RRLIB_UNIT_TESTS_EQUALITY(3.0, pose_2d.X().Value());
    RRLIB_UNIT_TESTS_EQUALITY(4.0, pose_2d.Y().Value());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, pose_2d.Yaw().Value().Value());

    typedef tPose3D<double>::tOrientationComponent<double, math::angle::Degree> tDegree3D;
    tPose3D<double> pose_3d(1, 2, 3, tDegree3D(90), tDegree3D(-90), tDegree3D(0.0));
    RRLIB_UNIT_TESTS_EQUALITY(1.0, pose_3d.X().Value());
    RRLIB_UNIT_TESTS_EQUALITY(2.0, pose_3d.Y().Value());
    RRLIB_UNIT_TESTS_EQUALITY(3.0, pose_3d.Z().Value());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, pose_3d.Roll().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, pose_3d.Pitch().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(0.0, pose_3d.Yaw().Value().Value());
    pose_3d.X() = 10;
    pose_3d.Y() = 20;
    pose_3d.Z() = 30;
    pose_3d.Roll() = tDegree3D(0);
    pose_3d.Pitch() = tDegree3D(90);
    pose_3d.Yaw() = -tDegree3D(90);
    RRLIB_UNIT_TESTS_EQUALITY(10.0, pose_3d.X().Value());
    RRLIB_UNIT_TESTS_EQUALITY(20.0, pose_3d.Y().Value());
    RRLIB_UNIT_TESTS_EQUALITY(30.0, pose_3d.Z().Value());
    RRLIB_UNIT_TESTS_EQUALITY(0.0, pose_3d.Roll().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, pose_3d.Pitch().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, pose_3d.Yaw().Value().Value());
  }

  void ComparisonOperators()
  {
    typedef localization::tPose2D<float> tPose2D;
    typedef tPose2D::tOrientationComponent<> tAngle2D;
    RRLIB_UNIT_TESTS_ASSERT(tPose2D(1, 2, tAngle2D(3)) == tPose2D(1, 2, tAngle2D(3)));
    RRLIB_UNIT_TESTS_ASSERT(tPose2D(1, 2, tAngle2D(3)) != tPose2D(2, 3, tAngle2D(4)));

    typedef localization::tPose3D<double> tPose3D;
    typedef tPose3D::tOrientationComponent<> tAngle3D;
    RRLIB_UNIT_TESTS_ASSERT(tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)) == tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)));
    RRLIB_UNIT_TESTS_ASSERT(tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)) != tPose3D(2, 3, 4, tAngle3D(5), tAngle3D(6), tAngle3D(7)));
  }

  void AssignmentOperators()
  {
    typedef tPose2D<>::tOrientationComponent<> tAngle2D;
    tPose2D<double> pose_2d;
    pose_2d = tPose2D<double>(1, 2, tAngle2D(2));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<double>(1, 2, tAngle2D(2)), pose_2d);
    pose_2d = tPose2D<float>(2, 3, tAngle2D(4));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tPose2D<double>(2, 3, tAngle2D(4)), pose_2d, 0));
    pose_2d.Set(3, 4, tAngle2D(5));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<double>(3, 4, tAngle2D(5)), pose_2d);

    typedef tPose3D<>::tOrientationComponent<> tAngle3D;
    tPose3D<double> pose_3d;
    pose_3d = tPose3D<double>(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<double>(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)), pose_3d);
    pose_3d = tPose3D<float>(2, 3, 4, tAngle3D(5), tAngle3D(6), tAngle3D(7));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tPose3D<double>(2, 3, 4, tAngle3D(5), tAngle3D(6), tAngle3D(7)), pose_3d, 0));
    pose_3d.Set(3, 4, 5, tAngle3D(6), tAngle3D(7), tAngle3D(8));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<double>(3, 4, 5, tAngle3D(6), tAngle3D(7), tAngle3D(8)), pose_3d);
  }

  void ArithmeticOperators()
  {
    typedef localization::tPose2D<double> tPose2D;
    typedef tPose2D::tOrientationComponent<> tAngle2D;
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(-1, -2, -tAngle2D(3)), -tPose2D(1, 2, tAngle2D(3)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(1 + 2, 2 + 3, tAngle2D(3) + tAngle2D(4)), tPose2D(1, 2, tAngle2D(3)) + tPose2D(2, 3, tAngle2D(4)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(1 - 2, 2 - 3, tAngle2D(3) - tAngle2D(4)), tPose2D(1, 2, tAngle2D(3)) - tPose2D(2, 3, tAngle2D(4)));

    typedef localization::tPose3D<double> tPose3D;
    typedef tPose3D::tOrientationComponent<> tAngle3D;
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(-1, -2, -3, -tAngle3D(4), -tAngle3D(5), -tAngle3D(6)), -tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(1 + 2, 2 + 3, 3 + 4, tAngle3D(4 + 5), tAngle3D(5 + 6), tAngle3D(6 + 7)), tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)) + tPose3D(2, 3, 4, tAngle3D(5), tAngle3D(6), tAngle3D(7)));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(1 - 2, 2 - 3, 3 - 4, tAngle3D(4 - 5), tAngle3D(5 - 6), tAngle3D(6 - 7)), tPose3D(1, 2, 3, tAngle3D(4), tAngle3D(5), tAngle3D(6)) - tPose3D(2, 3, 4, tAngle3D(5), tAngle3D(6), tAngle3D(7)));
  }

  void ReferenceTransformations()
  {
    typedef localization::tPose2D<double> tPose2D;
    typedef tPose2D::tOrientationComponent<> tAngle2D;
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tPose2D(0, 4, rrlib::math::cPI_2), tPose2D(1, 2, tAngle2D()).GetPoseInParentFrame(tPose2D(2, 3, rrlib::math::cPI_2))));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D(1, 2, tAngle2D()), tPose2D(0, 4, rrlib::math::cPI_2).GetPoseInLocalFrame(tPose2D(2, 3, rrlib::math::cPI_2)));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tPose2D(-5.724523, -1.108977, rrlib::math::tAngleDeg(-59.999999)), tPose2D(4, 0, rrlib::math::tAngleDeg(50)).GetPoseInLocalFrame(tPose2D(1, 5, rrlib::math::tAngleDeg(110)))));

    typedef localization::tPose3D<double> tPose3D;
    typedef tPose3D::tOrientationComponent<> tAngle3D;
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tPose3D(tPose2D(0, 4, rrlib::math::cPI_2)), tPose3D(tPose2D(1, 2, tAngle2D())).GetPoseInParentFrame(tPose3D(tPose2D(2, 3, rrlib::math::cPI_2)))));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(5, 4, 6, tAngle3D(rrlib::math::cPI_2), tAngle3D(), tAngle3D(rrlib::math::cPI_2)), tPose3D(1, 2, 3, tAngle3D(), tAngle3D(), tAngle3D()).GetPoseInParentFrame(tPose3D(2, 3, 4, tAngle3D(rrlib::math::cPI_2), tAngle3D(), tAngle3D(rrlib::math::cPI_2))));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(tPose2D(1, 2, tAngle2D())), tPose3D(tPose2D(0, 4, rrlib::math::cPI_2)).GetPoseInLocalFrame(tPose3D(tPose2D(2, 3, rrlib::math::cPI_2))));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D(1, 2, 3, tAngle3D(), tAngle3D(), tAngle3D()), tPose3D(5, 4, 6, tAngle3D(rrlib::math::cPI_2), tAngle3D(), tAngle3D(rrlib::math::cPI_2)).GetPoseInLocalFrame(tPose3D(2, 3, 4, tAngle3D(rrlib::math::cPI_2), tAngle3D(), tAngle3D(rrlib::math::cPI_2))));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tPose3D(7.12685279298953, 1.66258966535201, -0.83292548989809, rrlib::math::tAngleDeg(146.09730144611353), rrlib::math::tAngleDeg(-58.43157586071569), rrlib::math::tAngleDeg(101.53514216427203)), tPose3D(4, 0, 0.5, rrlib::math::tAngleDeg(100), rrlib::math::tAngleDeg(-150), rrlib::math::tAngleDeg(50)).GetPoseInLocalFrame(tPose3D(1, 5, 5, rrlib::math::tAngleDeg(-100), rrlib::math::tAngleDeg(130), rrlib::math::tAngleDeg(110))), 1E-14));
  }

  void Streaming()
  {
    std::stringstream actual;
    std::stringstream expected;

    actual << tPose2D<double>(1, 2, math::tAngle<double, math::angle::Degree>(90));
    expected << "(1, 2, 90°)";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    actual << tPose3D<double>(1, 2, 3, math::tAngle<double, math::angle::Degree>(4), math::tAngle<double, math::angle::Degree>(5), math::tAngle<double, math::angle::Degree>(6));
    expected << "(1, 2, 3, 4°, 5°, 6°)";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    tPose2D<double> pose_2d;
    tPose3D<double> pose_3d;
    std::stringstream source;
    source.exceptions(std::istream::failbit);
    RRLIB_UNIT_TESTS_EXCEPTION(source >> pose_2d, std::ios_base::failure);
    source.clear();
    RRLIB_UNIT_TESTS_EXCEPTION(source >> pose_3d, std::ios_base::failure);
    source.clear();
    source << "(3, 4, 120°)";
    source >> pose_2d;
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<double>(3, 4, math::tAngle<double, math::angle::Degree>(120)), pose_2d);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> pose_2d, std::ios_base::failure);
    source.clear();
    source.str("");
    source << "(4, 5, 6, 10°, 20°, 30°)";
    source >> pose_3d;
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<double>(4, 5, 6, math::tAngle<double, math::angle::Degree>(10), math::tAngle<double, math::angle::Degree>(20), math::tAngle<double, math::angle::Degree>(30)), pose_3d);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> pose_3d, std::ios_base::failure);

    source.clear();
    source.str("");
    source << "(1, 2, 3°), (2, 3, 4°), ende";
    for (size_t i = 0; i < 2; ++i)
    {
      source >> pose_2d;
      math::tAngle<double, math::angle::Degree> yaw(i + 3.0);
      RRLIB_UNIT_TESTS_EQUALITY(tPose2D<double>(i + 1.0, i + 2.0, yaw), pose_2d);
      char delim;
      source >> delim;
    }

    source.clear();
    source.str("");
    source << "(1, 2, 3, 1°, 2°, 3°), (2, 3, 4, 2°, 3°, 4°), ende";
    for (size_t i = 0; i < 2; ++i)
    {
      source >> pose_3d;
      math::tAngle<double, math::angle::Degree> roll(i + 1.0);
      math::tAngle<double, math::angle::Degree> pitch(i + 2.0);
      math::tAngle<double, math::angle::Degree> yaw(i + 3.0);
      RRLIB_UNIT_TESTS_EQUALITY(tPose3D<double>(i + 1.0, i + 2.0, i + 3.0, roll, pitch, yaw), pose_3d);
      char delim;
      source >> delim;
    }

    serialization::tMemoryBuffer memory_buffer;
    serialization::tOutputStream output_stream(memory_buffer);
    serialization::tInputStream input_stream(memory_buffer);

    output_stream << tPose2D<double>(10, 20, math::tAngle<double, math::angle::Radian>(5 * M_PI_2));
    output_stream << tPose2D<float>(3, 2, math::tAngle<double, math::angle::Radian>(0));
    output_stream << tPose3D<double>(10, 20, 30, math::tAngle<double, math::angle::Radian>(1), math::tAngle<double, math::angle::Radian>(2), math::tAngle<double, math::angle::Radian>(3));
    output_stream << tPose3D<float>(2, 3, 4, math::tAngle<double, math::angle::Radian>(1.5), math::tAngle<double, math::angle::Radian>(2.5), math::tAngle<double, math::angle::Radian>(3.5));
    output_stream.Flush();

    tPose2D<float> pose_2d_float;
    tPose3D<float> pose_3d_float;

    input_stream >> pose_2d;
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<double>(10, 20, math::tAngle<double, math::angle::Radian>(M_PI_2)), pose_2d);
    input_stream >> pose_2d_float;
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<float>(3, 2, math::tAngle<double, math::angle::Radian>(0)), pose_2d_float);
    input_stream >> pose_3d;
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<double>(10, 20, 30, math::tAngle<double, math::angle::Radian>(1), math::tAngle<double, math::angle::Radian>(2), math::tAngle<double, math::angle::Radian>(3)), pose_3d);
    input_stream >> pose_3d_float;
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tPose3D<float>(2, 3, 4, math::tAngle<double, math::angle::Radian>(1.5), math::tAngle<double, math::angle::Radian>(2.5), math::tAngle<double, math::angle::Radian>(3.5)), pose_3d_float, 1E-6));
  }

  void UnitChanges()
  {
    tPose2D<> pose_2d(10, 20, tPose2D<>::tOrientationComponent<>(2));
    tPoseChange2D<> pose_change_2d(1, 2, tTwist2D<>::tOrientationComponent<>(0.2));
    typedef tPose < 2, double, si_units::tSIUnit < 1, 0, -2, 0, 0, 0, 0 > , si_units::tSIUnit < 0, 0, -2, 0, 0, 0, 0 > , math::angle::NoWrap > tAcceleration2D;
    tAcceleration2D acceleration_2d(0.1, 0.2, tAcceleration2D::tOrientationComponent<>(0.02));

    RRLIB_UNIT_TESTS_EQUALITY(pose_2d, tPose2D<>(pose_change_2d * si_units::tTime<float>(10)));
    RRLIB_UNIT_TESTS_EQUALITY(pose_2d, tPose2D<>(si_units::tTime<float>(10) * pose_change_2d));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(pose_change_2d, pose_2d / si_units::tTime<float>(10), 0));

    RRLIB_UNIT_TESTS_EQUALITY(pose_change_2d, acceleration_2d * si_units::tTime<float>(10));
    RRLIB_UNIT_TESTS_EQUALITY(pose_change_2d, si_units::tTime<float>(10) * acceleration_2d);
    RRLIB_UNIT_TESTS_EQUALITY(acceleration_2d, pose_change_2d / si_units::tTime<float>(10));

    tPose3D<> pose_3d(10, 20, 30, tPose3D<>::tOrientationComponent<>(2), tPose3D<>::tOrientationComponent<>(1), tPose3D<>::tOrientationComponent<>(1.6));
    tPoseChange3D<> pose_change_3d(1, 2, 3, tTwist3D<>::tOrientationComponent<>(0.2), tTwist3D<>::tOrientationComponent<>(0.1), tTwist3D<>::tOrientationComponent<>(0.16));
    typedef tPose < 3, double, si_units::tSIUnit < 1, 0, -2, 0, 0, 0, 0 > , si_units::tSIUnit < 0, 0, -2, 0, 0, 0, 0 > , math::angle::NoWrap > tAcceleration3D;
    tAcceleration3D acceleration_3d(0.1, 0.2, 0.3, tAcceleration3D::tOrientationComponent<>(0.02), tAcceleration3D::tOrientationComponent<>(0.01), tAcceleration3D::tOrientationComponent<>(0.016));

    RRLIB_UNIT_TESTS_EQUALITY(pose_3d, tPose3D<>(pose_change_3d * si_units::tTime<float>(10)));
    RRLIB_UNIT_TESTS_EQUALITY(pose_3d, tPose3D<>(si_units::tTime<float>(10) * pose_change_3d));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(pose_change_3d, pose_3d / si_units::tTime<float>(10), 0));

    RRLIB_UNIT_TESTS_EQUALITY(pose_change_3d, acceleration_3d * si_units::tTime<float>(10));
    RRLIB_UNIT_TESTS_EQUALITY(pose_change_3d, si_units::tTime<float>(10) * acceleration_3d);
    RRLIB_UNIT_TESTS_EQUALITY(acceleration_3d, pose_change_3d / si_units::tTime<float>(10));
  }

  void MultiplyVelocityWithFactor()
  {
    tPoseChange2D<> pose_change_2d(1, 2, tTwist2D<>::tOrientationComponent<>(0.2));
    RRLIB_UNIT_TESTS_EQUALITY(tPoseChange2D<>(1 * 3.3, 2 * 3.3, tTwist2D<>::tOrientationComponent<>(0.2 * 3.3)), pose_change_2d * 3.3);
    RRLIB_UNIT_TESTS_EQUALITY(tPoseChange2D<>(1 * 3.3, 2 * 3.3, tTwist2D<>::tOrientationComponent<>(0.2 * 3.3)), 3.3 * pose_change_2d);

    tPoseChange3D<> pose_change_3d(1, 2, 3, tTwist3D<>::tOrientationComponent<>(0.2), tTwist3D<>::tOrientationComponent<>(0.4), tTwist3D<>::tOrientationComponent<>(0.6));
    RRLIB_UNIT_TESTS_EQUALITY(tPoseChange3D<>(1 * 3.3, 2 * 3.3, 3 * 3.3, tTwist3D<>::tOrientationComponent<>(0.2 * 3.3), tTwist3D<>::tOrientationComponent<>(0.4 * 3.3), tTwist3D<>::tOrientationComponent<>(0.6 * 3.3)), pose_change_3d * 3.3);
    RRLIB_UNIT_TESTS_EQUALITY(tPoseChange3D<>(1 * 3.3, 2 * 3.3, 3 * 3.3, tTwist3D<>::tOrientationComponent<>(0.2 * 3.3), tTwist3D<>::tOrientationComponent<>(0.4 * 3.3), tTwist3D<>::tOrientationComponent<>(0.6 * 3.3)), 3.3 * pose_change_3d);

  }

  void Uncertainty()
  {
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<>(), static_cast<const tPose2D<> &>(tUncertainPose2D<>()));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<>(1, 2, math::tAngleDeg(3)), static_cast<const tPose2D<> &>(tUncertainPose2D<>(1, 2, math::tAngleDeg(3))));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<>(2, 3, math::tAngleDeg(4)), static_cast<const tPose2D<> &>(tUncertainPose2D<>(tPosition2D<>(2, 3), tOrientation2D<>(math::tAngleDeg(4)))));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<>(3, 4, math::tAngleDeg(5)), static_cast<const tPose2D<> &>(tUncertainPose2D<>(tPose2D<>(3, 4, math::tAngleDeg(5)))));

    tUncertainPose2D<> pose_1(tPose2D<>(1, 2, math::tAngleDeg(3)), tUncertainPose2D<>::tCovarianceMatrix<>(1, 2, 3, 2, 4, 5, 3, 5, 6));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<>(1, 2, math::tAngleDeg(3)), static_cast<const tPose2D<> &>(pose_1));
    RRLIB_UNIT_TESTS_EQUALITY((math::tMatrix<3, 3, double>(1, 2, 3, 2, 4, 5, 3, 5, 6)), pose_1.Covariance());

    tUncertainPose2D<> pose_2(2, 3, math::tMatrix<3, 3, double>(1, 2, 3, 2, 4, 5, 3, 5, 6));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<>(2, 3), static_cast<const tPose2D<> &>(pose_2));
    RRLIB_UNIT_TESTS_EQUALITY((math::tMatrix<3, 3, double>(1, 2, 3, 2, 4, 5, 3, 5, 6)), pose_2.Covariance());

    tUncertainPose2D<> pose_3(3, 4, math::tAngleDeg(5), math::tMatrix<3, 3, double>(1, 2, 3, 2, 4, 5, 3, 5, 6));
    RRLIB_UNIT_TESTS_EQUALITY(tPose2D<>(3, 4, math::tAngleDeg(5)), static_cast<const tPose2D<> &>(pose_3));
    RRLIB_UNIT_TESTS_EQUALITY((math::tMatrix<3, 3, double>(1, 2, 3, 2, 4, 5, 3, 5, 6)), pose_3.Covariance());

    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<>(), static_cast<const tPose3D<> &>(tUncertainPose3D<>()));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<>(1, 2, 3, math::tAngleDeg(4), math::tAngleDeg(5), math::tAngleDeg(6)), static_cast<const tPose3D<> &>(tUncertainPose3D<>(1, 2, 3, math::tAngleDeg(4), math::tAngleDeg(5), math::tAngleDeg(6))));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<>(2, 3, 4, math::tAngleDeg(5), math::tAngleDeg(6), math::tAngleDeg(7)), static_cast<const tPose3D<> &>(tUncertainPose3D<>(tPosition3D<>(2, 3, 4), tOrientation3D<>(math::tAngleDeg(5), math::tAngleDeg(6), math::tAngleDeg(7)))));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<>(3, 4, 5, math::tAngleDeg(6), math::tAngleDeg(7), math::tAngleDeg(8)), static_cast<const tPose3D<> &>(tUncertainPose3D<>(tPose3D<>(3, 4, 5, math::tAngleDeg(6), math::tAngleDeg(7), math::tAngleDeg(8)))));

    tUncertainPose3D<> pose_4(tPose3D<>(1, 2, 3, math::tAngleDeg(4), math::tAngleDeg(5), math::tAngleDeg(6)), tUncertainPose3D<>::tCovarianceMatrix<>(1, 2, 3, 4, 5, 6, 2, 4, 5, 6, 7, 8, 3, 5, 7, 8, 9, 10, 4, 6, 8, 10, 11, 12, 5, 7, 9, 11, 13, 14, 6, 8, 10, 12, 14, 16));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<>(1, 2, 3, math::tAngleDeg(4), math::tAngleDeg(5), math::tAngleDeg(6)), static_cast<const tPose3D<> &>(pose_4));
    RRLIB_UNIT_TESTS_EQUALITY((math::tMatrix<6, 6, double>(1, 2, 3, 4, 5, 6, 2, 4, 5, 6, 7, 8, 3, 5, 7, 8, 9, 10, 4, 6, 8, 10, 11, 12, 5, 7, 9, 11, 13, 14, 6, 8, 10, 12, 14, 16)), pose_4.Covariance());

    tUncertainPose3D<> pose_5(2, 3, 4, math::tMatrix<6, 6, double>(1, 2, 3, 4, 5, 6, 2, 4, 5, 6, 7, 8, 3, 5, 7, 8, 9, 10, 4, 6, 8, 10, 11, 12, 5, 7, 9, 11, 13, 14, 6, 8, 10, 12, 14, 16));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<>(2, 3, 4), static_cast<const tPose3D<> &>(pose_5));
    RRLIB_UNIT_TESTS_EQUALITY((math::tMatrix<6, 6, double>(1, 2, 3, 4, 5, 6, 2, 4, 5, 6, 7, 8, 3, 5, 7, 8, 9, 10, 4, 6, 8, 10, 11, 12, 5, 7, 9, 11, 13, 14, 6, 8, 10, 12, 14, 16)), pose_5.Covariance());

    tUncertainPose3D<> pose_6(3, 4, 5, math::tAngleDeg(6), math::tAngleDeg(7), math::tAngleDeg(8), math::tMatrix<6, 6, double>(1, 2, 3, 4, 5, 6, 2, 4, 5, 6, 7, 8, 3, 5, 7, 8, 9, 10, 4, 6, 8, 10, 11, 12, 5, 7, 9, 11, 13, 14, 6, 8, 10, 12, 14, 16));
    RRLIB_UNIT_TESTS_EQUALITY(tPose3D<>(3, 4, 5, math::tAngleDeg(6), math::tAngleDeg(7), math::tAngleDeg(8)), static_cast<const tPose3D<> &>(pose_6));
    RRLIB_UNIT_TESTS_EQUALITY((math::tMatrix<6, 6, double>(1, 2, 3, 4, 5, 6, 2, 4, 5, 6, 7, 8, 3, 5, 7, 8, 9, 10, 4, 6, 8, 10, 11, 12, 5, 7, 9, 11, 13, 14, 6, 8, 10, 12, 14, 16)), pose_6.Covariance());
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(TestPose);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
