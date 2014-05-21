//
// You received this file as part of RRLib
// Robotics Research Library
//
// Copyright (C) Finroc GbR (finroc.org)
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//
//----------------------------------------------------------------------
/*!\file    unit_test_pose.cpp
 *
 * \author  Michael Arndt
 *
 * \date    2014-04-17
 *
 */
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// External includes (system with <>, local with "")
//----------------------------------------------------------------------
#include <cstdlib>
#include <iostream>

#include "rrlib/util/tUnitTestSuite.h"

//----------------------------------------------------------------------
// Internal includes with ""
//----------------------------------------------------------------------
#include "rrlib/localization/tPose.h"
#include "rrlib/localization/tPoseWithUncertainty.h"

//----------------------------------------------------------------------
// Debugging
//----------------------------------------------------------------------
#include <cassert>

//----------------------------------------------------------------------
// Namespace usage
//----------------------------------------------------------------------
using namespace rrlib::localization;

//----------------------------------------------------------------------
// Forward declarations / typedefs / enums
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Const values
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Implementation
//----------------------------------------------------------------------

class tTestPose : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestPose);
  RRLIB_UNIT_TESTS_ADD_TEST(Test);
  RRLIB_UNIT_TESTS_ADD_TEST(TestSerialization);
  RRLIB_UNIT_TESTS_ADD_TEST(TestUncertainty);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests()
  {
  }
  virtual void CleanUp() {}

  void Test()
  {

    tPose<> p;
    typedef tPose<>::tLinear tLinear;
    typedef tPose<>::tAngular tAngular;


    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), p.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), p.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), p.Z());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0.0), p.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0.0), p.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0.0), p.Yaw());

    p.X() = 1;
    p.Y() = 2;
    p.Z() = 3;

    p.Roll() = 0.5;
    p.Pitch() = 0.2;
    p.Yaw() = 0.1;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(1), p.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(2), p.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(3), p.Z());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.5), p.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.2), p.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.1), p.Yaw());




    // test constructors
    tPose<> p2(1, 2, 3, 4, 5, 6);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After constructing, value must be correct", tLinear(1), p2.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After constructing, value must be correct", tLinear(2), p2.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After constructing, value must be correct", tLinear(3), p2.Z());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After constructing, value must be correct", tAngular(4), p2.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After constructing, value must be correct", tAngular(5), p2.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After constructing, value must be correct", tAngular(6), p2.Yaw());

  }

  void TestSerialization()
  {
    {

      tPose<3, double, rrlib::math::tAngleDeg> p;

      p.X() = 1;
      p.Y() = 2;
      p.Z() = 3;

      p.Roll() = 45;
      p.Pitch() = -32;
      p.Yaw() = 185;


      ///////


      std::stringstream s;
      s << p;

      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("ostream serialization must be correct", std::string("(1, 2, 3, 45°, -32°, -175°)"), s.str());

      ///////

      rrlib::serialization::tStringOutputStream so;
      so << p;

      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("tStringOutputStream serialization must be correct", std::string("(1, 2, 3, 45°, -32°, -175°)"), so.ToString());

      tPose<3, double, rrlib::math::tAngleDeg> p2;
      rrlib::serialization::tStringInputStream si("(5, 4, 1, 45°, -42°, 170°)");
      si >> p2;

      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("tStringInputStream deserialization must be correct", 5., p2.X());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("tStringInputStream deserialization must be correct", 4., p2.Y());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("tStringInputStream deserialization must be correct", 1., p2.Z());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("tStringInputStream deserialization must be correct", rrlib::math::tAngleDeg(45), p2.Roll());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("tStringInputStream deserialization must be correct", rrlib::math::tAngleDeg(-42), p2.Pitch());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("tStringInputStream deserialization must be correct", rrlib::math::tAngleDeg(170), p2.Yaw());


    }

    {

      tPose<3, double, rrlib::math::tAngleDeg> p;

      p.X() = 8;
      p.Y() = 6;
      p.Z() = 4.3;

      p.Roll() = -45;
      p.Pitch() = -2;
      p.Yaw() = 144;


      ///////


      // serialize
      rrlib::serialization::tMemoryBuffer mb;
      rrlib::serialization::tOutputStream os(mb);
      os << p;

      os.Flush();

      // deserialize
      rrlib::serialization::tInputStream is(mb);
      tPose<3, double, rrlib::math::tAngleDeg> p2;
      is >> p2;

      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After serialization/deserialization, value  must be correct", 8., p2.X());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After serialization/deserialization, value  must be correct", 6., p2.Y());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After serialization/deserialization, value  must be correct", 4.3, p2.Z());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After serialization/deserialization, value  must be correct", rrlib::math::tAngleDeg(-45), p2.Roll());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After serialization/deserialization, value  must be correct", rrlib::math::tAngleDeg(-2), p2.Pitch());
      RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After serialization/deserialization, value  must be correct", rrlib::math::tAngleDeg(144), p2.Yaw());



    }

  }

  void TestUncertainty()
  {

    tPoseWithUncertainty<> p;
    typedef tPoseWithUncertainty<>::tLinear tLinear;
    typedef tPoseWithUncertainty<>::tAngular tAngular;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0.0), p.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0.0), p.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0.0), p.Z());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0.0), p.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0.0), p.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0.0), p.Yaw());

    p.X() = 1;
    p.Y() = 2;
    p.Z() = 3;

    p.Roll() = 0.5;
    p.Pitch() = 0.2;
    p.Yaw() = 0.1;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(1.), p.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(2.), p.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(3.), p.Z());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.5), p.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.2), p.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.1), p.Yaw());

  }

};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestPose);
