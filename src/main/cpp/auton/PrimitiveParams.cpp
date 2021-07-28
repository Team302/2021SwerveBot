
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

#include <auton/PrimitiveEnums.h>
#include <auton/PrimitiveParams.h>

PrimitiveParams::PrimitiveParams
(
    PRIMITIVE_IDENTIFIER								id,
    float                       						time,
    float                       						distance,
    float                       						xLoc,
    float                       						yLoc,
    float                       						heading,
    float                       						startDriveSpeed,
    float                       						endDriveSpeed,
	bool												runIntake,
	std::string                                         pathName,
	std::string											driveMode
):	//Pass over parameters to class variables
		m_id(id), //Primitive ID
		m_time(time),
		m_distance(distance),
		m_xLoc( xLoc ),
		m_yLoc( yLoc ),
		m_heading(heading),
		m_startDriveSpeed( startDriveSpeed ),
		m_endDriveSpeed( endDriveSpeed ),
		m_pathName ( pathName),
		m_runIntake ( runIntake ),
		m_driveMode ( driveMode )
{
}

PRIMITIVE_IDENTIFIER PrimitiveParams::GetID() const
{
	return m_id;
}

float PrimitiveParams::GetTime() const
{
	return m_time;
}

float PrimitiveParams::GetDistance() const
{
	return m_distance;
}

float PrimitiveParams::GetXLocation() const
{
    return m_xLoc;
}

float PrimitiveParams::GetYLocation() const
{
    return m_yLoc;
}

float PrimitiveParams::GetHeading() const
{
	return m_heading;
}

float PrimitiveParams::GetDriveSpeed() const
{
    return m_startDriveSpeed;
}

float PrimitiveParams::GetEndDriveSpeed() const
{
    return m_endDriveSpeed;
}


std::string PrimitiveParams::GetPathName() const
{
	return m_pathName;
}

bool PrimitiveParams::GetIntakeState() const
{
	return m_runIntake;
}

std::string PrimitiveParams::GetDriveMode() const
{
	return m_driveMode;
}

//Setters
void PrimitiveParams::SetDistance(float distance)
{
	m_distance = distance;
}
