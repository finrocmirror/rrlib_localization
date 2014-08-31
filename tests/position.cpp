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
/*!\file    rrlib/localization/tests/position.cpp
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

#include "rrlib/localization/tPosition.h"

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

class TestPosition : public util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(TestPosition);
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
    RRLIB_UNIT_TESTS_EQUALITY(2 * sizeof(double), sizeof(tPosition2D<double>));

    double raw[2] = { 0.0, 0.0 };

    tPosition2D<double> zero;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero, &raw, sizeof(raw)) == 0);

    raw[0] = 1.0, raw[1] = 2.0;
    tPosition2D<double> position(1.0, 2.0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&position, &raw, sizeof(raw)) == 0);

    tPosition2D<double> copy(position);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &position, sizeof(position)) == 0);

    tPosition2D<float> converted(position);
    float converted_raw[2] = { 1.0, 2.0 };
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted, &converted_raw, sizeof(converted_raw)) == 0);
  }

  void Constructors3D()
  {
    RRLIB_UNIT_TESTS_EQUALITY(3 * sizeof(double), sizeof(tPosition3D<double>));

    double raw[3] = { 0.0, 0.0, 0.0 };

    tPosition3D<double> zero;
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&zero, &raw, sizeof(raw)) == 0);

    raw[0] = 1.0, raw[1] = 2.0, raw[2] = 3.0;
    tPosition3D<double> position(1.0, 2.0, 3.0);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&position, &raw, sizeof(raw)) == 0);

    tPosition3D<double> copy(position);
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&copy, &position, sizeof(position)) == 0);

    tPosition3D<float> converted(position);
    float converted_raw[3] = { 1.0, 2.0, 3.0 };
    RRLIB_UNIT_TESTS_ASSERT(std::memcmp(&converted, &converted_raw, sizeof(converted_raw)) == 0);
  }

  void AccessOperators()
  {
    tPosition2D<double> position_2d(1, 2);
    RRLIB_UNIT_TESTS_EQUALITY(1.0, position_2d.X().Value());
    RRLIB_UNIT_TESTS_EQUALITY(2.0, position_2d.Y().Value());
    position_2d.X() = 10;
    position_2d.Y() = 20;
    RRLIB_UNIT_TESTS_EQUALITY(10.0, position_2d.X().Value());
    RRLIB_UNIT_TESTS_EQUALITY(20.0, position_2d.Y().Value());

    tPosition3D<double> position_3d(1, 2, 3);
    RRLIB_UNIT_TESTS_EQUALITY(1.0, position_3d.X().Value());
    RRLIB_UNIT_TESTS_EQUALITY(2.0, position_3d.Y().Value());
    RRLIB_UNIT_TESTS_EQUALITY(3.0, position_3d.Z().Value());
    position_3d.X() = 10;
    position_3d.Y() = 20;
    position_3d.Z() = 30;
    RRLIB_UNIT_TESTS_EQUALITY(10.0, position_3d.X().Value());
    RRLIB_UNIT_TESTS_EQUALITY(20.0, position_3d.Y().Value());
    RRLIB_UNIT_TESTS_EQUALITY(30.0, position_3d.Z().Value());
  }

  void ComparisonOperators()
  {
    typedef localization::tPosition2D<double> tPosition2D;
    RRLIB_UNIT_TESTS_ASSERT(tPosition2D(1, 2) == tPosition2D(1, 2));
    RRLIB_UNIT_TESTS_ASSERT(tPosition2D(1, 2) != tPosition2D(2, 3));

    typedef localization::tPosition3D<double> tPosition3D;
    RRLIB_UNIT_TESTS_ASSERT(tPosition3D(1, 2, 3) == tPosition3D(1, 2, 3));
    RRLIB_UNIT_TESTS_ASSERT(tPosition3D(1, 2, 3) != tPosition3D(2, 3, 4));
  }

  void AssignmentOperators()
  {
    tPosition2D<double> position_2d;
    position_2d = tPosition2D<double>(si_units::tLength<>(1), si_units::tLength<>(2));
    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D<double>(1, 2), position_2d);
    position_2d = tPosition2D<float>(2, 3);
    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D<double>(2, 3), position_2d);
    position_2d.Set(3, 4);
    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D<double>(3, 4), position_2d);

    tPosition3D<double> position_3d;
    position_3d = tPosition3D<double>(1, 2, 3);
    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D<double>(1, 2, 3), position_3d);
    position_3d = tPosition3D<float>(2, 3, 4);
    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D<double>(2, 3, 4), position_3d);
    position_3d.Set(3, 4, 5);
    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D<double>(3, 4, 5), position_3d);
  }

  void ArithmeticOperators()
  {
    typedef localization::tPosition2D<double> tPosition2D;
    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D(-1, -2), -tPosition2D(1, 2));
    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D(1 + 2, 2 + 3), tPosition2D(1, 2) + tPosition2D(2, 3));
    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D(1 - 2, 2 - 3), tPosition2D(1, 2) - tPosition2D(2, 3));
    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D(1 * 2, 2 * 2), tPosition2D(1, 2) * 2.0);
    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D(1, 2) * 2.0, 2.0 * tPosition2D(1, 2));

    typedef localization::tPosition3D<double> tPosition3D;
    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D(-1, -2, -3), -tPosition3D(1, 2, 3));
    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D(1 + 2, 2 + 3, 3 + 4), tPosition3D(1, 2, 3) + tPosition3D(2, 3, 4));
    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D(1 - 2, 2 - 3, 3 - 4), tPosition3D(1, 2, 3) - tPosition3D(2, 3, 4));
    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D(1 * 2, 2 * 2, 3 * 2), tPosition3D(1, 2, 3) * 2.0);
    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D(1, 2, 3) * 2.0, 2.0 * tPosition3D(1, 2, 3));
  }

  void Streaming()
  {
//    std::stringstream actual;
//    std::stringstream expected;
//
//    actual << tPosition2D<double>(1, 2);
//    expected << "(1, 2)";
//    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());
//
//    actual << tPosition3D<double>(math::tAngle<double, math::angle::Degree>(1), math::tAngle<double, math::angle::Degree>(2), math::tAngle<double, math::angle::Degree>(3));
//    expected << "(1°, 2°, 3°)";
//    RRLIB_UNIT_TESTS_EQUALITY(expected.str(), actual.str());
//
//    tPosition2D<double> position_2d;
//    tPosition3D<double> position_3d;
//    std::stringstream source;
//    source.exceptions(std::istream::failbit);
//    RRLIB_UNIT_TESTS_EXCEPTION(source >> position_2d, std::ios_base::failure);
//    source.clear();
//    RRLIB_UNIT_TESTS_EXCEPTION(source >> position_3d, std::ios_base::failure);
//    source.clear();
//    source << "120°";
//    source >> position_2d;
//    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D<double>(math::tAngle<double, math::angle::Degree>(120)), position_2d);
//    source.clear();
//    source << "(120)";
//    RRLIB_UNIT_TESTS_EXCEPTION(source >> position_2d, std::ios_base::failure);
//    source.clear();
//    source.str("");
//    source << "(10°, 20°, 30°)";
//    source >> position_3d;
//    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D<double>(math::tAngle<double, math::angle::Degree>(10), math::tAngle<double, math::angle::Degree>(20), math::tAngle<double, math::angle::Degree>(30)), position_3d);
//    source.clear();
//    source << "(120)";
//    RRLIB_UNIT_TESTS_EXCEPTION(source >> position_3d, std::ios_base::failure);
//
//    source.clear();
//    source.str("");
//    source << "1°, 2°, ende";
//    for (size_t i = 0; i < 2; ++i)
//    {
//      source >> position_2d;
//      math::tAngle<double, math::angle::Degree> degree(i + 1.0);
//      RRLIB_UNIT_TESTS_EQUALITY(tPosition2D<double>(degree), position_2d);
//      char delim;
//      source >> delim;
//    }
//
//    source.clear();
//    source.str("");
//    source << "(1°, 2°, 3°), (2°, 3°, 4°), ende";
//    for (size_t i = 0; i < 2; ++i)
//    {
//      source >> position_3d;
//      math::tAngle<double, math::angle::Degree> degree_1(i + 1.0);
//      math::tAngle<double, math::angle::Degree> degree_2(i + 2.0);
//      math::tAngle<double, math::angle::Degree> degree_3(i + 3.0);
//      RRLIB_UNIT_TESTS_EQUALITY(tPosition3D<double>(degree_1, degree_2, degree_3), position_3d);
//      char delim;
//      source >> delim;
//    }
//
//    serialization::tMemoryBuffer memory_buffer;
//    serialization::tOutputStream output_stream(memory_buffer);
//    serialization::tInputStream input_stream(memory_buffer);
//
//    output_stream << tPosition2D<double>(math::tAngle<double, math::angle::Radian>(5 * M_PI_2));
//    output_stream << tPosition2D<float>(math::tAngle<double, math::angle::Radian>(0));
//    output_stream << tPosition3D<double>(math::tAngle<double, math::angle::Radian>(1), math::tAngle<double, math::angle::Radian>(2), math::tAngle<double, math::angle::Radian>(3));
//    output_stream << tPosition3D<float>(math::tAngle<double, math::angle::Radian>(1.5), math::tAngle<double, math::angle::Radian>(2.5), math::tAngle<double, math::angle::Radian>(3.5));
//    output_stream.Flush();
//
//    input_stream >> position_2d;
//    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D<double>(math::tAngle<double, math::angle::Radian>(M_PI_2)), position_2d);
//    input_stream >> position_2d;
//    RRLIB_UNIT_TESTS_EQUALITY(tPosition2D<double>(math::tAngle<double, math::angle::Radian>(0)), position_2d);
//    input_stream >> position_3d;
//    RRLIB_UNIT_TESTS_EQUALITY(tPosition3D<double>(math::tAngle<double, math::angle::Radian>(1), math::tAngle<double, math::angle::Radian>(2), math::tAngle<double, math::angle::Radian>(3)), position_3d);
//    input_stream >> position_3d;
//    RRLIB_UNIT_TESTS_ASSERT(IsEqual(tPosition3D<double>(math::tAngle<double, math::angle::Radian>(1.5), math::tAngle<double, math::angle::Radian>(2.5), math::tAngle<double, math::angle::Radian>(3.5)), position_3d, 1E-6));
  }

  void UnitChanges()
  {
    tPosition2D<> position_2d(si_units::tLength<>(10), si_units::tLength<>(20));
    tPositionChange2D<> position_change_2d(si_units::tVelocity<>(1), si_units::tVelocity<>(2));

    RRLIB_UNIT_TESTS_EQUALITY(position_2d, position_change_2d * si_units::tTime<float>(10));
    RRLIB_UNIT_TESTS_EQUALITY(position_2d, si_units::tTime<float>(10) * position_change_2d);

    RRLIB_UNIT_TESTS_EQUALITY(position_change_2d, position_2d / si_units::tTime<float>(10));

    tPosition3D<> position_3d(si_units::tLength<>(10), si_units::tLength<>(20), si_units::tLength<>(30));
    tPositionChange3D<> position_change_3d(si_units::tVelocity<>(1), si_units::tVelocity<>(2), si_units::tVelocity<>(3));

    RRLIB_UNIT_TESTS_EQUALITY(position_3d, position_change_3d * si_units::tTime<float>(10));
    RRLIB_UNIT_TESTS_EQUALITY(position_3d, si_units::tTime<float>(10) * position_change_3d);

    RRLIB_UNIT_TESTS_EQUALITY(position_change_3d, position_3d / si_units::tTime<float>(10));
  }
};

RRLIB_UNIT_TESTS_REGISTER_SUITE(TestPosition);

//----------------------------------------------------------------------
// End of namespace declaration
//----------------------------------------------------------------------
}
}
