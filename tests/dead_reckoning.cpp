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
/*!\file    unit_test_twist.cpp
 *
 * \author  Michael Arndt
 *
 * \date    2014-06-05
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
#include "rrlib/localization/tDeadReckoning.h"

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

class tTestDeadReckoning : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestDeadReckoning);
  RRLIB_UNIT_TESTS_ADD_TEST(TestLinear);
  RRLIB_UNIT_TESTS_ADD_TEST(TestAngular);
  RRLIB_UNIT_TESTS_ADD_TEST(TestCombined);
  RRLIB_UNIT_TESTS_ADD_TEST(TestCombinedInstance);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests()
  {

  }
  virtual void CleanUp() {}

  void TestLinear()
  {

    typedef tUncertainPose3D<> tPose;
    typedef tUncertainTwist3D<> tTwist;

    tPose p;
    tTwist t;
    rrlib::time::tDuration time_delta;

    // travel forward (in X direction) with 1 m/s for 0 s
    p.Reset();
    time_delta = std::chrono::milliseconds(0);
    t.SetPosition(1, 0, 0);
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0),  tPose::tPosition<>::tElement(0)), p);

    // travel forward (in X direction) with 1 m/s for 1 s
    time_delta = std::chrono::milliseconds(1000);
    t.SetPosition(1, 0, 0);
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(1), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0)), p);


    // travel forward (in X direction) with 0.5 m/s for 0.5 s
    time_delta = std::chrono::milliseconds(500);
    t.SetPosition(0.5, 0, 0);
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(1.25), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0)), p);



    // travel in X direction with 0.5 m/s and in Y direction with 1 m/s for 0.5 s
    p.Reset();
    time_delta = std::chrono::milliseconds(500);
    t.SetPosition(0.5, 1, 0);
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(0.25), tPose::tPosition<>::tElement(0.5), tPose::tPosition<>::tElement(0)), p);

    // travel in X direction with 0.5 m/s, in Y direction with 1 m/s and in Z direction with -2 m/s for 0.5 s
    p.Reset();
    time_delta = std::chrono::milliseconds(500);
    t.SetPosition(0.5, 1, -2);
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(0.25), tPose::tPosition<>::tElement(0.5), tPose::tPosition<>::tElement(-1)), p);

  }

  void TestAngular()
  {

    typedef tUncertainPose3D<> tPose;
    typedef tUncertainTwist3D<> tTwist;

    tPose p;
    tTwist t;
    rrlib::time::tDuration time_delta;

    // turn with Yaw = 1 rad/s for 0 s
    time_delta = std::chrono::milliseconds(0);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(1));
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0)), p);

    // turn with Yaw = 1 rad/s for 1 s
    p.Reset();
    time_delta = std::chrono::milliseconds(1000);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(1));
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(1)), p);

    // turn with Yaw = pi rad/s for 2 s
    p.Reset();
    time_delta = std::chrono::milliseconds(2000);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(M_PI));
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0)), p);


    // turn with Roll = 0.25 rad/s, Pitch = 0.5 rad/s, Yaw = 1 rad/s for 0.5 s
    p.Reset();
    time_delta = std::chrono::milliseconds(500);
    t.SetOrientation(tTwist::tOrientationComponent<>(0.25),
                     tTwist::tOrientationComponent<>(0.5),
                     tTwist::tOrientationComponent<>(1));
    tDeadReckoning::UpdatePose(p, t, time_delta);
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tOrientation<>::tComponent<>(0.125), tPose::tOrientation<>::tComponent<>(0.25), tPose::tOrientation<>::tComponent<>(0.5)), p);


  }

  void TestCombined()
  {

    typedef tUncertainPose3D<> tPose;
    typedef tUncertainTwist3D<> tTwist;

    tPose p;
    tTwist t;
    rrlib::time::tDuration time_delta;

    // move with X = 1 m/s, Yaw = pi rad/s for 1 s
    p.Reset();
    time_delta = std::chrono::milliseconds(1000);
    t.SetPosition(1, 0, 0);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(M_PI));
    tDeadReckoning::UpdatePose(p, t, time_delta);
    // NOTE: (1, 0, 0, 0, 0, pi) is correct here, because the DeadReckoning first translated, then rotates.
    // FIXME: the -M_PI is because of nowrap-policy .. we need to think about this
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(1), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tOrientationComponent<>(0), tPose::tOrientationComponent<>(0), tPose::tOrientationComponent<>(-M_PI)), p);

    // move with X = 1 m/s, Yaw = pi rad/s for 1 ms
    p.Reset();
    time_delta = std::chrono::milliseconds(1);
    t.SetPosition(1, 0, 0);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(M_PI));
    tDeadReckoning::UpdatePose(p, t, time_delta);
    // NOTE: (1, 0, 0, 0, 0, pi) is correct here, because the DeadReckoning first translated, then rotates.
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(1 / 1000.), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(M_PI / 1000.)), p);


    // now, we like to move on a circle and complete a quarter circle.
    // starting from yaw = 0 this will lead to yaw = 0.5 pi
    // the linear distance traveled along the arc will be t.X() * time_delta
    // the resulting circle has a radius of r = v/omega = t.X() / t.Yaw()
    // the time needed to complete this the quarter circle can be calculated as follows:
    // s = v * t
    // s = 1/4 * 2 * r * pi
    // => v * t = 1/4 * 2 * r * pi
    // => t = 1/4 * 2 * r * pi * 1/v = 1/2 * v / omega * pi * 1/v = 1/2 * pi * 1/omega
    // Inserting everything with yaw = pi rad/s leads to t = 1/2 s

    // The resulting X/Y coordinates are identical and are equal to the radius, so: x = y = 1 / pi

    // Now here, sample every 10 microseconds to get a result which is close to the theoretical result
    p.Reset();
    time_delta = std::chrono::microseconds(10);
    size_t samples = 0.5 * 100000; // see calculation above
    t.SetPosition(1, 0, 0);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(M_PI));

    for (size_t i = 0; i < samples; ++i)
    {
      tDeadReckoning::UpdatePose(p, t, time_delta);
    }
    // unfortunately, has to be checked for each double
    //RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(1/M_PI), tPose::tPosition<>::tElement(1/M_PI), tPose::tPosition<>::tElement(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0.5 * M_PI)), p);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 1 / M_PI, static_cast<double>(p.X()), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 1 / M_PI, static_cast<double>(p.Y()), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 0 , static_cast<double>(p.Z()), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 0 , static_cast<double>(rrlib::math::tAngleRad(p.Roll())), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 0 , static_cast<double>(rrlib::math::tAngleRad(p.Pitch())), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 0.5 * M_PI, static_cast<double>(rrlib::math::tAngleRad(p.Yaw())), 1E-3);


  }

  void TestCombinedInstance()
  {

    typedef tUncertainPose3D<> tPose;
    typedef tUncertainTwist3D<> tTwist;

    tTwist t;
    rrlib::time::tDuration time_delta;

    tDeadReckoning obj;

    // move with X = 1 m/s, Yaw = pi rad/s for 1 s
    time_delta = std::chrono::milliseconds(1000);
    t.SetPosition(1, 0, 0);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(M_PI));
    obj.UpdatePose(t, time_delta);
    // NOTE: (1, 0, 0, 0, 0, pi) is correct here, because the DeadReckoning first translated, then rotates.
    // FIXME: the -M_PI is because of nowrap-policy .. we need to think about this
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(1), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tOrientationComponent<>(0), tPose::tOrientationComponent<>(0), tPose::tOrientationComponent<>(-M_PI)), obj.GetPose());

    // move with X = 1 m/s, Yaw = pi rad/s for 1 ms
    time_delta = std::chrono::milliseconds(1);
    t.SetPosition(1, 0, 0);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(M_PI));
    obj.SetPose(tPose());
    obj.ResetTwist();
    obj.UpdatePose(t, time_delta);

    // NOTE: (1, 0, 0, 0, 0, pi) is correct here, because the DeadReckoning first translated, then rotates.
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(1 / 1000.), tPose::tPosition<>::tElement(0), tPose::tPosition<>::tElement(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(M_PI / 1000.)), obj.GetPose());


    // now, we like to move on a circle and complete a quarter circle.
    // starting from yaw = 0 this will lead to yaw = 0.5 pi
    // the linear distance traveled along the arc will be t.X() * time_delta
    // the resulting circle has a radius of r = v/omega = t.X() / t.Yaw()
    // the time needed to complete this the quarter circle can be calculated as follows:
    // s = v * t
    // s = 1/4 * 2 * r * pi
    // => v * t = 1/4 * 2 * r * pi
    // => t = 1/4 * 2 * r * pi * 1/v = 1/2 * v / omega * pi * 1/v = 1/2 * pi * 1/omega
    // Inserting everything with yaw = pi rad/s leads to t = 1/2 s

    // The resulting X/Y coordinates are identical and are equal to the radius, so: x = y = 1 / pi

    // Now here, sample every 10 microseconds to get a result which is close to the theoretical result
    time_delta = std::chrono::microseconds(10);
    size_t samples = 0.5 * 100000; // see calculation above
    t.SetPosition(1, 0, 0);
    t.SetOrientation(tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(0),
                     tTwist::tOrientationComponent<>(M_PI));

    obj.SetPose(tPose());
    obj.ResetTwist();
    for (size_t i = 0; i < samples; ++i)
    {
      obj.UpdatePose(t, time_delta);
    }
    // unfortunately, has to be checked for each double
    //RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After updating, value must be correct", tPose(tPose::tPosition<>::tElement(1/M_PI), tPose::tPosition<>::tElement(1/M_PI), tPose::tPosition<>::tElement(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0), tPose::tOrientation<>::tComponent<>(0.5 * M_PI)), p);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 1 / M_PI, static_cast<double>(obj.GetPose().X()), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 1 / M_PI, static_cast<double>(obj.GetPose().Y()), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 0 , static_cast<double>(obj.GetPose().Z()), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 0 , static_cast<double>(rrlib::math::tAngleRad(obj.GetPose().Roll())), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 0 , static_cast<double>(rrlib::math::tAngleRad(obj.GetPose().Pitch())), 1E-3);
    RRLIB_UNIT_TESTS_EQUALITY_DOUBLE_MESSAGE("After updating, value must be correct", 0.5 * M_PI, static_cast<double>(rrlib::math::tAngleRad(obj.GetPose().Yaw())), 1E-3);


  }


};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestDeadReckoning);
