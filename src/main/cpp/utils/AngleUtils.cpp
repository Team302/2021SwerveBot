
//====================================================================================================================================================
// Copyright 2020 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <cmath>

// FRC includes

// Team 302 includes
#include <utils/AngleUtils.h>
#include <utils/Logger.h>

// Third Party Includes



/// @brief find the angle from the startingAngle to the targetAngle
/// @param [in] startingAngle - angle to start from
/// @param [in] targetAngle - angle to go to
/// @returns units::angle::degree_t the angle to traverse to get from the startingAngle
///                                 to the targetAngle within 1 rotation
units::angle::degree_t AngleUtils::GetDeltaAngle
(
    units::angle::degree_t  startingAngle,
    units::angle::degree_t  targetAngle
)
{
    // start by making sure the angles are between -180 and 180 degrees
    auto normTarget = GetEquivAngle(targetAngle);
    auto normStart  = GetEquivAngle(startingAngle);

    Logger::GetLogger()->ToNtTable("AngleUtils", "target angle", targetAngle.to<double>());
    Logger::GetLogger()->ToNtTable("AngleUtils", "target angle - normalized", normTarget.to<double>());
    Logger::GetLogger()->ToNtTable("AngleUtils", "starting angle", startingAngle.to<double>());
    Logger::GetLogger()->ToNtTable("AngleUtils", "starting angle - normalized", normStart.to<double>());

    // compute delta which is between 0 and 360 degrees
    auto delta = units::angle::degree_t(normTarget - normStart);

    Logger::GetLogger()->ToNtTable("AngleUtils", "delta", delta.to<double>());

    return delta;
}


/// @brief make sure the angle is between -180.0_deg and 180.0_deg and if it isn't find
///        its equivalent angle within that range
/// @param [in] units::angle::degree_t  angle - angle to check
/// @returns units::angle::degree_t angle within range of -180_deg to 180.0_deg
units::angle::degree_t AngleUtils::GetEquivAngle
(
    units::angle::degree_t  angle
)
{
    units::angle::degree_t angleInRange = angle;
  // make sure the delta is between -180.0 and 180.0
    if (angleInRange < -180.0_deg)
    {
        while (angleInRange < -180.0_deg )
        {
            angleInRange = angleInRange + 360.0_deg;
        }
    }
    else if (angleInRange > 180.0_deg)
    {
        while (angleInRange > 180.0_deg)
        {
            angleInRange = angleInRange - 360.0_deg;
        }
    }
    return angleInRange;
}

