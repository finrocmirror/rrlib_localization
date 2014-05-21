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
#include "rrlib/si_units/si_units.h"
#include "rrlib/localization/tTwist.h"
#include "rrlib/localization/tTwistWithUncertainty.h"

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

class tTestTwist : public rrlib::util::tUnitTestSuite
{
  RRLIB_UNIT_TESTS_BEGIN_SUITE(tTestTwist);
  RRLIB_UNIT_TESTS_ADD_TEST(Test);
  //RRLIB_UNIT_TESTS_ADD_TEST(TestDouble);
  RRLIB_UNIT_TESTS_ADD_TEST(TestUncertainty);
  RRLIB_UNIT_TESTS_END_SUITE;

private:

  virtual void InitializeTests()
  {

  }
  virtual void CleanUp() {}

  void Test()
  {

    tTwist<> t;
    typedef tTwist<>::tAngular tAngular;
    typedef tTwist<>::tLinear tLinear;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), t.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), t.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), t.Z());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0), t.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0), t.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0), t.Yaw());

    t.X() = 1;
    t.Y() = 2;
    t.Z() = 3;

    t.Roll() = tAngular(0.5);
    t.Pitch() = tAngular(0.2);
    t.Yaw() = tAngular(0.1);

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(1), t.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(2), t.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(3), t.Z());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.5), t.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.2), t.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.1), t.Yaw());

  }

#if 0
  // temporary disabled because of ambiguities when instantiating tTwist with two doubles
  void TestDouble()
  {

    tTwist<3, double, double> t;


    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", 0.0, t.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", 0.0, t.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", 0.0, t.Z());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", 0.0, t.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", 0.0, t.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", 0.0, t.Yaw());

    t.X() = 1;
    t.Y() = 2;
    t.Z() = 3;

    t.Roll() = 0.5;
    t.Pitch() = 0.2;
    t.Yaw() = 0.1;

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", 1., t.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", 2., t.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", 3., t.Z());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", 0.5, t.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", 0.2, t.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", 0.1, t.Yaw());

  }
#endif

  void TestUncertainty()
  {

    tTwistWithUncertainty<> t;
    typedef tTwistWithUncertainty<>::tAngular tAngular;
    typedef tTwistWithUncertainty<>::tLinear tLinear;

    typedef tTwistWithUncertainty<>::tLinearSquared tLinearVelocitySquared;
    typedef tTwistWithUncertainty<>::tAngularSquared tAngularVelocitySquared;
    typedef tTwistWithUncertainty<>::tLinearCrossAngular tLinearCrossAngularVelocity;


    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), t.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), t.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinear(0), t.Z());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0), t.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0), t.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngular(0), t.Yaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetVarianceX());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetVarianceY());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetVarianceZ());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngularVelocitySquared(0), t.GetVarianceRoll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngularVelocitySquared(0), t.GetVariancePitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tAngularVelocitySquared(0), t.GetVarianceYaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetCovarianceXX());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetCovarianceXY());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetCovarianceXZ());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetCovarianceYY());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetCovarianceYZ());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("Initial value must be zero", tLinearVelocitySquared(0), t.GetCovarianceZZ());

    t.X() = 1;
    t.Y() = 2;
    t.Z() = 3;

    t.Roll() = tAngular(0.5);
    t.Pitch() = tAngular(0.2);
    t.Yaw() = tAngular(0.1);

    t.SetVarianceX(tLinearVelocitySquared(3.4));
    t.SetVarianceY(tLinearVelocitySquared(1.2));
    t.SetVarianceZ(tLinearVelocitySquared(4.4));

    t.SetVarianceRoll(tAngularVelocitySquared(0.33));
    t.SetVariancePitch(tAngularVelocitySquared(0.44));
    t.SetVarianceYaw(tAngularVelocitySquared(0.55));

    t.SetCovarianceXY(tLinearVelocitySquared(1.9));
    t.SetCovarianceXZ(tLinearVelocitySquared(8.2));
    t.SetCovarianceXRoll(tLinearCrossAngularVelocity(6.4));
    t.SetCovarianceXPitch(tLinearCrossAngularVelocity(7.4));
    t.SetCovarianceXYaw(tLinearCrossAngularVelocity(8.4));

    t.SetCovarianceYZ(tLinearVelocitySquared(3.7));
    t.SetCovarianceYRoll(tLinearCrossAngularVelocity(3.8));
    t.SetCovarianceYPitch(tLinearCrossAngularVelocity(3.9));
    t.SetCovarianceYYaw(tLinearCrossAngularVelocity(4.0));

    t.SetCovarianceZRoll(tLinearCrossAngularVelocity(4.8));
    t.SetCovarianceZPitch(tLinearCrossAngularVelocity(4.9));
    t.SetCovarianceZYaw(tLinearCrossAngularVelocity(5.0));

    t.SetCovarianceRollPitch(tAngularVelocitySquared(6.0));
    t.SetCovarianceRollYaw(tAngularVelocitySquared(6.1));

    t.SetCovariancePitchYaw(tAngularVelocitySquared(7.0));



    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(1), t.X());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(2), t.Y());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinear(3), t.Z());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.5), t.Roll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.2), t.Pitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngular(0.1), t.Yaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearVelocitySquared(3.4), t.GetVarianceX());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearVelocitySquared(1.2), t.GetVarianceY());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearVelocitySquared(4.4), t.GetVarianceZ());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngularVelocitySquared(0.33), t.GetVarianceRoll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngularVelocitySquared(0.44), t.GetVariancePitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngularVelocitySquared(0.55), t.GetVarianceYaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("var(x) must equal cov(x, x)", t.GetVarianceX(), t.GetCovarianceXX());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("var(y) must equal cov(y, y)", t.GetVarianceY(), t.GetCovarianceYY());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("var(z) must equal cov(z, z)", t.GetVarianceZ(), t.GetCovarianceZZ());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("var(roll) must equal cov(roll, roll)", t.GetVarianceRoll(), t.GetCovarianceRollRoll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("var(pitch) must equal cov(pitch, pitch)", t.GetVariancePitch(), t.GetCovariancePitchPitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("var(yaw) must equal cov(yaw, yaw)", t.GetVarianceYaw(), t.GetCovarianceYawYaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearVelocitySquared(1.9), t.GetCovarianceXY());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearVelocitySquared(8.2), t.GetCovarianceXZ());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(6.4), t.GetCovarianceXRoll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(7.4), t.GetCovarianceXPitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(8.4), t.GetCovarianceXYaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearVelocitySquared(3.7), t.GetCovarianceYZ());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(3.8), t.GetCovarianceYRoll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(3.9), t.GetCovarianceYPitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(4.0), t.GetCovarianceYYaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(4.8), t.GetCovarianceZRoll());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(4.9), t.GetCovarianceZPitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tLinearCrossAngularVelocity(5.0), t.GetCovarianceZYaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngularVelocitySquared(6.0), t.GetCovarianceRollPitch());
    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngularVelocitySquared(6.1), t.GetCovarianceRollYaw());

    RRLIB_UNIT_TESTS_EQUALITY_MESSAGE("After setting, value must be correct", tAngularVelocitySquared(7.0), t.GetCovariancePitchYaw());

  }



};

RRLIB_UNIT_TESTS_REGISTER_SUITE(tTestTwist);
