//====================================================================================================================================================
// Copyright 2019 Lake Orion Robotics FIRST Team 302
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

//Team 302 includes
#include <auton/PrimitiveParams.h>
#include <auton/primitives/IPrimitive.h>
#include <subsys/SwerveChassisFactory.h>
#include <auton/primitives/ZeroWheels.h>
#include <subsys/SwerveChassis.h>

using namespace std;
using namespace frc;

ZeroWheels::ZeroWheels() : IPrimitive(),
                            m_chassis( SwerveChassisFactory::GetSwerveChassisFactory()->GetSwerveChassis())
{
}

void ZeroWheels::Init(PrimitiveParams* params)
{
    //m_chassis.get()->ZeroAlignSwerveModules();
}

void ZeroWheels::Run()
{
    m_chassis.get()->ZeroAlignSwerveModules();
}

bool ZeroWheels::IsDone()
{
    //return true;
    return false;
}