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

//C++ Includes
#include <memory>
#include <units/time.h>

//Team 302 Includes
#include <states/IState.h>
#include <states/ballhopper/BallHopperSlowRelease.h>
#include <states/ballhopper/BallHopperStateMgr.h>
#include <states/ballhopper/BallHopperState.h>
#include <states/Mech1MotorState.h>
#include <subsys/MechanismFactory.h>
#include <controllers/MechanismTargetData.h>
#include <xmlmechdata/StateDataDefn.h>
#include <hw/usages/DigitalInputUsage.h>
#include <hw/DragonDigitalInput.h>

using namespace std;

BallHopperSlowRelease::BallHopperSlowRelease
(
    ControlData*        control,
    double              target
) : Mech1MotorState ( MechanismFactory::GetMechanismFactory()->GetBallHopper().get(), control, target )
{
   m_ballHopper = MechanismFactory::GetMechanismFactory()->GetBallHopper();
}

void BallHopperSlowRelease::Init()
{
    //Initialize default values
    m_isHolding = false;
    m_canDetect = true;
    m_timesSeen = 0;

    auto StateMgr = BallHopperStateMgr::GetInstance();

    m_holdState = StateMgr->GetState(BallHopperStateMgr::BALL_HOPPER_STATE::HOLD);
    m_releaseState = StateMgr->GetState(BallHopperStateMgr::BALL_HOPPER_STATE::RAPID_RELEASE);
}

void BallHopperSlowRelease::Run()
{
    //Check if we are sensing the ball and we have not already see the same ball 
    //(canDetect allows us to not check if we have seen the ball but the holdBall timer hasn't ran out yet)
   if (m_ballHopper.get()->isBallDetected() && m_canDetect)
   {
       m_timer.Start();
       m_isHolding = true;
       m_canDetect = false;
       m_timesSeen++;
   } 
   else if ( !m_ballHopper.get()->isBallDetected() && !m_isHolding )
   {
       m_releaseState->Run();
   } 
   else if ( m_isHolding )
   {
       m_holdState->Run();
   }

    //this waits until our shooter cooldown has passed so the shooter motors can get back up to full speed
   if ( m_timer.HasPeriodPassed(units::second_t(m_waitTime)))
   {
       m_isHolding = false;
       m_canDetect = true;
       m_timer.Stop();
       m_timer.Reset();
   }
}

bool BallHopperSlowRelease::AtTarget()
{
    //check the timesSeen variable until we have seen 3 balls, after we have released 3 balls, we're done
    if ( m_timesSeen == 3)
    {
        return true;
    }
    else
    {
        return false;
    }
    
}
