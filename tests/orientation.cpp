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
/*!\file    rrlib/localization/tests/orientation.cpp
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

#include "rrlib/localization/tOrientation.h"

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

class TestOrientation : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(TestOrientation);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors2D);
  RRLIB_UNIT_TESTS_ADD_TEST(Constructors3D);
  RRLIB_UNIT_TESTS_ADD_TEST(AccessOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ComparisonOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(AssignmentOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(ArithmeticOperators);
  RRLIB_UNIT_TESTS_ADD_TEST(Streaming);
  RRLIB_UNIT_TESTS_ADD_TEST(UnitChanges);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  void Constructors2D()
  {
    RRLIB_UNIT_TESTS_EQUALITY(sizeof(double), sizeof(tOrientation2D<double>));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<size_t>(1), tOrientation2D<double>::cSIZE);
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(2), tOrientation2D<double>::cDIMENSION);

    double raw = 0.0;

    tOrientation2D<double> zero;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero, &raw, sizeof(raw)) == 0);

    raw = M_PI_2;
    tOrientation2D<double> ninety_degrees(tOrientation2D<double>::tComponent<>(math::tAngle<double, math::angle::Degree>(90)));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&ninety_degrees, &raw, sizeof(raw)) == 0);

    raw = 0.5;
    tOrientation2D<double> from_angle(math::tAngle<double, math::angle::Radian>(0.5));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&from_angle, &raw, sizeof(raw)) == 0);

    raw = -M_PI_2;
    tOrientation2D<double> minus_ninety_degrees(math::tMatrix<2, 2, double>(0, 1, -1, 0));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&minus_ninety_degrees, &raw, sizeof(raw)) == 0);

    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, minus_ninety_degrees.Yaw().Value().Value());

    tOrientation2D<double> copy(ninety_degrees);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &ninety_degrees, sizeof(ninety_degrees)) == 0);

    tOrientation2D<float> converted(ninety_degrees);
    float converted_raw = M_PI_2;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted, &converted_raw, sizeof(converted_raw)) == 0);
  }

  void Constructors3D()
  {
    RRLIB_UNIT_TESTS_EQUALITY(3 * sizeof(double), sizeof(tOrientation3D<double>));
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<size_t>(3), tOrientation3D<double>::cSIZE);
    RRLIB_UNIT_TESTS_EQUALITY(static_cast<unsigned int>(3), tOrientation3D<double>::cDIMENSION);

    double raw[3] = { 0.0, 0.0, 0.0 };

    tOrientation3D<double> zero;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero, &raw, sizeof(raw)) == 0);

    raw[0] = 0;
    raw[1] = 0;
    raw[2] = M_PI_2;
    tOrientation3D<double> ninety_degrees(tOrientation3D<double>::tComponent<>(math::tAngle<double, math::angle::Degree>(0)),
                                          tOrientation3D<double>::tComponent<>(math::tAngle<double, math::angle::Degree>(0)),
                                          tOrientation3D<double>::tComponent<>(math::tAngle<double, math::angle::Degree>(90)));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&ninety_degrees, &raw, sizeof(raw)) == 0);

    raw[0] = 0.5;
    raw[1] = 1.0;
    raw[2] = 1.5;
    tOrientation3D<double> from_angle(math::tAngle<double, math::angle::Radian>(0.5), math::tAngle<double, math::angle::Radian>(1.0), math::tAngle<double, math::angle::Radian>(1.5));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&from_angle, &raw, sizeof(raw)) == 0);

    raw[0] = 0;
    raw[1] = 0;
    raw[2] = -M_PI_2;
    tOrientation3D<double> minus_ninety_degrees(math::tMatrix<3, 3, double>(0, 1, 0, -1, 0, 0, 0, 0, 1));
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&minus_ninety_degrees, &raw, sizeof(raw)) == 0);

    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, minus_ninety_degrees.Yaw().Value().Value());

    tOrientation3D<double> copy(ninety_degrees);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &ninety_degrees, sizeof(ninety_degrees)) == 0);

    tOrientation3D<float> converted(ninety_degrees);
    float converted_raw[3] = { 0.0, 0.0, M_PI_2 };
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted, &converted_raw, sizeof(converted_raw)) == 0);
  }

  void AccessOperators()
  {
    typedef tOrientation2D<double>::tComponent<double, math::angle::Degree> tDegree2D;
    tOrientation2D<double> orientation_2d(tDegree2D(90));
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, orientation_2d.Yaw().Value().Value());
    orientation_2d.Yaw() = tDegree2D(-90);
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, orientation_2d.Yaw().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, orientation_2d[0].Value().Value());
    RRLIB_UNIT_TESTS_EXCEPTION(orientation_2d[1], std::logic_error);

    typedef tOrientation3D<double>::tComponent<double, math::angle::Degree> tDegree3D;
    tOrientation3D<double> orientation_3d(tDegree3D(0), tDegree3D(-90), tDegree3D(90));
    RRLIB_UNIT_TESTS_EQUALITY(0.0, orientation_3d.Roll().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, orientation_3d.Pitch().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, orientation_3d.Yaw().Value().Value());
    orientation_3d.Roll() = tDegree3D(-90);
    orientation_3d.Pitch() = tDegree3D(90);
    orientation_3d.Yaw() = tDegree3D();
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, orientation_3d.Roll().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, orientation_3d.Pitch().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(0.0, orientation_3d.Yaw().Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(-M_PI_2, orientation_3d[0].Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(M_PI_2, orientation_3d[1].Value().Value());
    RRLIB_UNIT_TESTS_EQUALITY(0.0, orientation_3d[2].Value().Value());
    RRLIB_UNIT_TESTS_EXCEPTION(orientation_3d[3], std::logic_error);
  }

  void ComparisonOperators()
  {
    typedef localization::tOrientation2D<double> tOrientation2D;
    typedef tOrientation2D::tComponent<> tAngle2D;
    RRLIB_UNIT_TESTS_ASSERT(tOrientation2D(tAngle2D(90)) == tOrientation2D(tAngle2D(90)));
    RRLIB_UNIT_TESTS_ASSERT(tOrientation2D(tAngle2D(90)) != tOrientation2D(tAngle2D(45)));

    typedef localization::tOrientation3D<double> tOrientation3D;
    typedef tOrientation3D::tComponent<> tAngle3D;
    RRLIB_UNIT_TESTS_ASSERT(tOrientation3D(tAngle3D(1), tAngle3D(2), tAngle3D(3)) == tOrientation3D(tAngle3D(1), tAngle3D(2), tAngle3D(3)));
    RRLIB_UNIT_TESTS_ASSERT(tOrientation3D(tAngle3D(1), tAngle3D(2), tAngle3D(3)) != tOrientation3D(tAngle3D(2), tAngle3D(3), tAngle3D(4)));
  }

  void AssignmentOperators()
  {
    tOrientation2D<double> orientation_2d;
    orientation_2d = tOrientation2D<double>(tOrientation2D<double>::tComponent<>(1));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D<double>(tOrientation2D<double>::tComponent<>(1)), orientation_2d);
    orientation_2d = tOrientation2D<float>(tOrientation2D<double>::tComponent<>(2));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D<double>(tOrientation2D<double>::tComponent<>(2)), orientation_2d);
    orientation_2d.Set(tOrientation2D<double>::tComponent<>(3));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D<double>(tOrientation2D<double>::tComponent<>(3)), orientation_2d);
    orientation_2d.Set(math::tAngle<float, math::angle::Degree>(4));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D<double>(math::tAngle<double, math::angle::Degree>(4)), orientation_2d);

    tOrientation3D<double> orientation_3d;
    orientation_3d = tOrientation3D<double>(tOrientation3D<double>::tComponent<>(1), tOrientation3D<double>::tComponent<>(2), tOrientation3D<double>::tComponent<>(3));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D<double>(tOrientation3D<double>::tComponent<>(1), tOrientation3D<double>::tComponent<>(2), tOrientation3D<double>::tComponent<>(3)), orientation_3d);
    orientation_3d = tOrientation3D<float>(tOrientation3D<double>::tComponent<>(2), tOrientation3D<double>::tComponent<>(3), tOrientation3D<double>::tComponent<>(4));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tOrientation3D<double>(tOrientation3D<double>::tComponent<>(2), tOrientation3D<double>::tComponent<>(3), tOrientation3D<double>::tComponent<>(4)), orientation_3d, 0));
    orientation_3d.Set(tOrientation3D<double>::tComponent<>(3), tOrientation3D<double>::tComponent<>(4), tOrientation3D<double>::tComponent<>(5));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D<double>(tOrientation3D<double>::tComponent<>(3), tOrientation3D<double>::tComponent<>(4), tOrientation3D<double>::tComponent<>(5)), orientation_3d);
    orientation_3d.Set(math::tAngle<float, math::angle::Degree>(4), math::tAngle<float, math::angle::Degree>(5), math::tAngle<float, math::angle::Degree>(6));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D<double>(math::tAngle<float, math::angle::Degree>(4), math::tAngle<float, math::angle::Degree>(5), math::tAngle<float, math::angle::Degree>(6)), orientation_3d);
  }

  void ArithmeticOperators()
  {
    typedef localization::tOrientation2D<double> tOrientation2D;
    typedef tOrientation2D::tComponent<> tAngle2D;
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D(tAngle2D(-1)), -tOrientation2D(tAngle2D(1)));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D(tAngle2D(1 + 2)), tOrientation2D(tAngle2D(1)) + tOrientation2D(tAngle2D(2)));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D(tAngle2D(1 - 2)), tOrientation2D(tAngle2D(1)) - tOrientation2D(tAngle2D(2)));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D(tAngle2D(2 * 3)), tOrientation2D(tAngle2D(2)) * 3);
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D(tAngle2D(2)) * 3, 3 * tOrientation2D(tAngle2D(2)));

    typedef localization::tOrientation3D<double> tOrientation3D;
    typedef tOrientation3D::tComponent<> tAngle3D;
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D(tAngle3D(-1), tAngle3D(-2), tAngle3D(-3)), -tOrientation3D(tAngle3D(1), tAngle3D(2), tAngle3D(3)));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D(tAngle3D(1 + 2), tAngle3D(2 + 3), tAngle3D(3 + 4)), tOrientation3D(tAngle3D(1), tAngle3D(2), tAngle3D(3)) + tOrientation3D(tAngle3D(2), tAngle3D(3), tAngle3D(4)));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D(tAngle3D(1 - 2), tAngle3D(2 - 3), tAngle3D(3 - 4)), tOrientation3D(tAngle3D(1), tAngle3D(2), tAngle3D(3)) - tOrientation3D(tAngle3D(2), tAngle3D(3), tAngle3D(4)));
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D(tAngle2D(2 * 3), tAngle3D(3 * 3), tAngle3D(4 * 3)), tOrientation3D(tAngle3D(2), tAngle3D(3), tAngle3D(4)) * 3);
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D(tAngle3D(2), tAngle3D(3), tAngle3D(4)) * 3, 3 * tOrientation3D(tAngle3D(2), tAngle3D(3), tAngle3D(4)));
  }

  void Streaming()
  {
    std::stringstream actual;
    std::stringstream expected;

    actual << tOrientation2D<double>(math::tAngle<double, math::angle::Radian>(5 * M_PI_2));
    expected << "90°";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    actual << tOrientation3D<double>(math::tAngle<double, math::angle::Degree>(1), math::tAngle<double, math::angle::Degree>(2), math::tAngle<double, math::angle::Degree>(3));
    expected << "(1°, 2°, 3°)";
    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());

    tOrientation2D<double> orientation_2d;
    tOrientation3D<double> orientation_3d;
    std::stringstream source;
    source.exceptions(std::istream::failbit);
    RRLIB_UNIT_TESTS_EXCEPTION(source >> orientation_2d, std::ios_base::failure);
    source.clear();
    RRLIB_UNIT_TESTS_EXCEPTION(source >> orientation_3d, std::ios_base::failure);
    source.clear();
    source << "120°";
    source >> orientation_2d;
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D<double>(math::tAngle<double, math::angle::Degree>(120)), orientation_2d);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> orientation_2d, std::ios_base::failure);
    source.clear();
    source.str("");
    source << "(10°, 20°, 30°)";
    source >> orientation_3d;
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D<double>(math::tAngle<double, math::angle::Degree>(10), math::tAngle<double, math::angle::Degree>(20), math::tAngle<double, math::angle::Degree>(30)), orientation_3d);
    source.clear();
    source << "(120)";
    RRLIB_UNIT_TESTS_EXCEPTION(source >> orientation_3d, std::ios_base::failure);

    source.clear();
    source.str("");
    source << "1°, 2°, ende";
    for (size_t i = 0; i < 2; ++i)
    {
      source >> orientation_2d;
      math::tAngle<double, math::angle::Degree> yaw(i + 1.0);
      RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D<double>(yaw), orientation_2d);
      char delim;
      source >> delim;
    }

    source.clear();
    source.str("");
    source << "(1°, 2°, 3°), (2°, 3°, 4°), ende";
    for (size_t i = 0; i < 2; ++i)
    {
      source >> orientation_3d;
      math::tAngle<double, math::angle::Degree> roll(i + 1.0);
      math::tAngle<double, math::angle::Degree> pitch(i + 2.0);
      math::tAngle<double, math::angle::Degree> yaw(i + 3.0);
      RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D<double>(roll, pitch, yaw), orientation_3d);
      char delim;
      source >> delim;
    }

    serialization::tMemoryBuffer memory_buffer;
    serialization::tOutputStream output_stream(memory_buffer);
    serialization::tInputStream input_stream(memory_buffer);

    typedef math::tAngle<double, math::angle::Radian> tAngle;

    output_stream << tOrientation2D<double>(tAngle(5 * M_PI_2));
    output_stream << tOrientation2D<float>(tAngle(M_PI_2));
    output_stream << tOrientation3D<double>(tAngle(1), tAngle(2), tAngle(3));
    output_stream << tOrientation3D<float>(tAngle(1.5), tAngle(2.5), tAngle(3.5));
    output_stream.Flush();

    tOrientation2D<float> orientation_2d_float;
    tOrientation3D<float> orientation_3d_float;

    input_stream >> orientation_2d;
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D<double>(tAngle(M_PI_2)), orientation_2d);
    input_stream >> orientation_2d_float;
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation2D<float>(tAngle(M_PI_2)), orientation_2d_float);
    input_stream >> orientation_3d;
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D<double>(tAngle(1), tAngle(2), tAngle(3)), orientation_3d);
    input_stream >> orientation_3d_float;
    RRLIB_UNIT_TESTS_EQUALITY(tOrientation3D<float>(tAngle(1.5), tAngle(2.5), tAngle(3.5)), orientation_3d_float);
  }

  void UnitChanges()
  {
    typedef si_units::tQuantity<si_units::tNoUnit, math::tAngle<double, math::angle::Degree, math::angle::NoWrap>> tAngle;
    typedef si_units::tQuantity<si_units::tHertz, math::tAngle<double, math::angle::Degree, math::angle::NoWrap>> tAngularVelocity;
    typedef si_units::tQuantity < si_units::tSIUnit < 0, 0, -2, 0, 0, 0, 0 > , math::tAngle<double, math::angle::Degree, math::angle::NoWrap >> tAngularAcceleration;
    tOrientation2D<> orientation_2d(tAngle(20));
    tOrientationChange2D<> orientation_change_2d(tAngularVelocity(2));
    tOrientation < 2, double, si_units::tSIUnit < 0, 0, -2, 0, 0, 0, 0 > , math::angle::NoWrap > acceleration_2d(tAngularAcceleration(0.2));

    RRLIB_UNIT_TESTS_EQUALITY(orientation_2d, tOrientation2D<>(orientation_change_2d * si_units::tTime<float>(10)));
    RRLIB_UNIT_TESTS_EQUALITY(orientation_2d, tOrientation2D<>(si_units::tTime<float>(10) * orientation_change_2d));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(orientation_change_2d, orientation_2d / si_units::tTime<float>(10), 0));

    RRLIB_UNIT_TESTS_EQUALITY(orientation_change_2d, acceleration_2d * si_units::tTime<float>(10));
    RRLIB_UNIT_TESTS_EQUALITY(orientation_change_2d, si_units::tTime<float>(10) * acceleration_2d);
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(acceleration_2d, orientation_change_2d / si_units::tTime<float>(10), 0));

    tOrientation3D<> orientation_3d(tAngle(10), tAngle(20), tAngle(30));
    tOrientationChange3D<> orientation_change_3d(tAngularVelocity(1), tAngularVelocity(2), tAngularVelocity(3));
    tOrientation < 3, double, si_units::tSIUnit < 0, 0, -2, 0, 0, 0, 0 > , math::angle::NoWrap > acceleration_3d(tAngularAcceleration(0.1), tAngularAcceleration(0.2), tAngularAcceleration(0.3));

    RRLIB_UNIT_TESTS_EQUALITY(orientation_3d, tOrientation3D<>(orientation_change_3d * si_units::tTime<float>(10)));
    RRLIB_UNIT_TESTS_EQUALITY(orientation_3d, tOrientation3D<>(si_units::tTime<float>(10) * orientation_change_3d));
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(orientation_change_3d, orientation_3d / si_units::tTime<float>(10), 0));

    RRLIB_UNIT_TESTS_EQUALITY(orientation_change_3d, acceleration_3d * si_units::tTime<float>(10));
    RRLIB_UNIT_TESTS_EQUALITY(orientation_change_3d, si_units::tTime<float>(10) * acceleration_3d);
    RRLIB_UNIT_TESTS_ASSERT(IsEqual(acceleration_3d, orientation_change_3d / si_units::tTime<float>(10), 0));
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(TestOrientation);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
