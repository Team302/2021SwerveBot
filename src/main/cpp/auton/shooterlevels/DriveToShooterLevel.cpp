//====================================================================================================================================================
// Copyright 2021 Lake Orion Robotics FIRST Team 302
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

//C++ includes
#include <memory>

//Team 302 includes
#include <auton/PrimitiveParams.h>
#include <subsys/SwerveChassisFactory.h>
#include <auton/primitives/IPrimitive.h>
#include <auton/shooterlevels/DriveToShooterLevel.h>
#include <auton/PrimitiveFactory.h>

using namespace std;

DriveToShooterLevel::DriveToShooterLevel() : m_primitive(nullptr),
                                             m_primFactory(PrimitiveFactory::GetInstance())
{
}

void DriveToShooterLevel::DriveToLevel()
{

   
    m_primitive->Run();
}

void DriveToShooterLevel::Init(double distance, double startSpeed)
{
    if (m_primitive == nullptr)
    {

    auto params = new PrimitiveParams( DRIVE_DISTANCE,  //identifer
                                       0.0,             //time
                                       distance,        //distance  in inches
                                       0.0,             //target x location
                                       0.0,             //target y location
                                       0.0,             //heading in degrees
                                       startSpeed,      //start drive speed in inches
                                       0.0,             //end drive speed
                                       false,           //run the intake
                                       string(""),       //pathname
                                       string("RAMSETE") //drive mode
                                      );                                       
        m_primitive = m_primFactory->GetIPrimitive(params);
        m_primitive->Init(params);
    }
}

void DriveToShooterLevel::Run()
{
    m_primitive->Run();
}

bool DriveToShooterLevel::IsDone()
{
    return m_primitive->IsDone();
}

     DriveToShooterLevel::~DriveToShooterLevel()
{
    delete m_primitive;
    delete m_primFactory;
}