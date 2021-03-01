
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

// C++ includes


// FRC includes 


// 302 includes

#include <auton/PrimitiveEnums.h>
#include <auton/PrimitiveFactory.h>
#include <auton/PrimitiveParser.h>

#include <auton/primitives/DoNothing.h>
#include <auton/primitives/DriveDistance.h>
#include <auton/primitives/DrivePath.h>
#include <auton/primitives/DriveTime.h>
#include <auton/primitives/HoldPosition.h>
#include <auton/primitives/IPrimitive.h>
#include <auton/primitives/ResetPosition.h>
#include <auton/primitives/TurnAngle.h>

PrimitiveFactory* PrimitiveFactory::m_instance = nullptr;

PrimitiveFactory* PrimitiveFactory::GetInstance() 
{
	if (PrimitiveFactory::m_instance == nullptr) 
	{																//If we do not have an instance
		PrimitiveFactory::m_instance = new PrimitiveFactory();		//Create a new instance
	}
	return PrimitiveFactory::m_instance;							//Return said instance
}

PrimitiveFactory::PrimitiveFactory() :
				m_doNothing(nullptr),
				m_driveTime(nullptr),
				m_driveDistance(nullptr),
				m_turnAngle(nullptr),
				m_holdPosition(nullptr),
				m_drivePath(nullptr),
				m_resetPosition(nullptr)
{
}

PrimitiveFactory::~PrimitiveFactory() 
{
	PrimitiveFactory::m_instance = nullptr; //todo: do we have to delete this pointer?
}

IPrimitive* PrimitiveFactory::GetIPrimitive(PrimitiveParams* primitivePasser)
{
	IPrimitive* primitive = nullptr;
	switch (primitivePasser->GetID())				//Decides which primitive to get or make
	{
		case DO_NOTHING:
			if (m_doNothing == nullptr)
			{
				m_doNothing = new DoNothing();
			}
			primitive =  m_doNothing;
			break;

		case RESET_POSITION:
			if (m_resetPosition == nullptr)
			{
				m_resetPosition = new ResetPosition();
			}
			primitive = m_resetPosition;
			break;

		case DRIVE_TIME:
			if (m_driveTime == nullptr)
			{
				m_driveTime = new DriveTime();
			}
			primitive = m_driveTime;
			break;

		case DRIVE_DISTANCE:
			if (m_driveDistance == nullptr)
			{
				m_driveDistance = new DriveDistance();
			}
			primitive = m_driveDistance;
			break;

		case TURN_ANGLE_ABS:
			if (m_turnAngle == nullptr)
			{
				m_turnAngle = new TurnAngle();
			}
			primitive = m_turnAngle;
			break;

		case TURN_ANGLE_REL:
			if (m_turnAngle == nullptr)
			{
				m_turnAngle = new TurnAngle();
			}
			primitive = m_turnAngle;
			break;

		case HOLD_POSITION:
			if (m_holdPosition == nullptr)
			{
				m_holdPosition = new HoldPosition();
			}
			primitive = m_holdPosition;
			break;

		case DRIVE_PATH:
			if (m_drivePath == nullptr)
			{
				m_drivePath = new DrivePath();
			}
			primitive = m_drivePath;
			break;

		default:
			break;
	}
	return primitive;

}
