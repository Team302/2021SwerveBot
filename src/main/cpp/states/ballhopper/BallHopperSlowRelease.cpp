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
    BallHopperStateMgr::BALL_HOPPER_STATE m_stateEnum = BallHopperStateMgr::BALL_HOPPER_STATE::RAPID_RELEASE;
}

void BallHopperSlowRelease::Run()
{
    m_stateVector[m_stateEnum]->Run();

    // check if banner sensor sees ball


    // add hold and release state to constructor using ballhopperstatemgr::GetState(BallHopperStateMgr::BALL_HOPPER_STATE)
    //change how states are ran to include new holdState and releaseState variables
    //holdState and releaseState are IState*
    //incorporate this class into the existing ballhopperstatemgr case block for slowreleasestate



    if (m_ballHopper.get()->isBallDetected()) 
    { 
        m_stateEnum = BallHopperStateMgr::BALL_HOPPER_STATE::HOLD;
        m_timer.Start();
    }

   if ( m_timer.HasPeriodPassed(units::second_t(3)))
   {
       m_stateEnum = BallHopperStateMgr::BALL_HOPPER_STATE::RAPID_RELEASE;
   }

}
/*
void SetState( BallHopperStateMgr::BALL_HOPPER_STATE stateEnum)
{
    auto state = m_stateVector[stateEnum];
    m_currentState = state;
    m_currentStateEnum = stateEnum;
}
*/