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


#include <map>
#include <vector>

#include "states/turret/TurretStateMgr.h"
#include <states/IState.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>

#include <states/turret/LimelightAim.h>
#include <states/turret/HoldTurretPosition.h>
#include <hw/factories/LimelightFactory.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>
#include <subsys/Turret.h>

using namespace std;

TurretStateMgr* TurretStateMgr::m_instance = nullptr;
TurretStateMgr* TurretStateMgr::GetInstance()
{
	if ( TurretStateMgr::m_instance == nullptr )
	{
		TurretStateMgr::m_instance = new TurretStateMgr();
	}
	return TurretStateMgr::m_instance;
}

TurretStateMgr::TurretStateMgr() : m_states(),
                                   m_currentState(),
                                   m_approxTargetAngle( 0.0 )
{
    auto turret = MechanismFactory::GetMechanismFactory()->GetTurret();
    m_approxTargetAngle = turret->GetPosition();

    // Parse the configuration file 
    auto stateXML = make_unique<StateDataDefn>();
    vector<MechanismTargetData*> targetData = stateXML.get()->ParseXML( MechanismTypes::MECHANISM_TYPE::TURRET );

    map<string, TURRET_STATE> stateMap;
    stateMap["TURRETHOLD"] = TURRET_STATE::HOLD;
    stateMap["TURRETAUTOAIM"] = TURRET_STATE::LIMELIGHT_AIM;
    for (auto inx=0; inx<MAX_TURRET_STATES; ++inx)
    {
       m_states[inx] = nullptr;
    }

    for ( auto td: targetData )
    {
        auto stateString = td->GetStateString();
        auto stateItr = stateMap.find( stateString );
        if ( stateItr != stateMap.end() )
        {
            auto stateEnum = stateItr->second;
            if ( m_states[stateEnum] == nullptr )
            {
                auto controlData = td->GetController();
                auto target = td->GetTarget();
                switch ( stateEnum )
                {
                    case TURRET_STATE::HOLD:
                    {
                        auto thisState = new HoldTurretPosition(controlData, m_approxTargetAngle);
                        m_states[stateEnum] = thisState;
                        m_currentState = thisState;
                        m_currentStateEnum = stateEnum;
                        m_currentState->Init();
                    }
                    break;

                    case TURRET_STATE::LIMELIGHT_AIM:
                    {
                        auto thisState = new LimelightAim(controlData, target);
                        m_states[stateEnum] = thisState;
                    }
                    break;

                    default:
                    {
                        Logger::GetLogger()->LogError( string("TurretHoodStateMgr::TurretHoodStateMgr"), string("unknown state"));
                    }
                    break;
                }
            }
            {
                
            }
            
        }
    }
}

void TurretStateMgr::RunCurrentState()
{
    if ( MechanismFactory::GetMechanismFactory()->GetTurret().get() != nullptr )
    {
        // process teleop/manual interrupts
        auto controller = TeleopControl::GetInstance();
        if ( controller != nullptr )
        {
            if (controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::TURRET_LIMELIGHT_AIM))
            {
                SetCurrentState( TURRET_STATE::LIMELIGHT_AIM, false);
            }
        }
        if ( m_currentState != nullptr )
        {
            m_currentState->Run();
        }
    }
}
void TurretStateMgr::SetCurrentState
(
    TURRET_STATE stateEnum,
    bool         run
)
{
    auto state = m_states[stateEnum];
    if ( state != nullptr && state != m_currentState )
    {
        m_currentState = state;
        m_currentStateEnum = stateEnum;
        m_currentState->Init();
        if ( stateEnum == LIMELIGHT_AIM )
        {
            auto llAim = dynamic_cast<LimelightAim*>(m_currentState);
            if ( llAim != nullptr )
            {
                llAim->UpdateTarget( m_approxTargetAngle );
            }
        }

        
        if ( run )
        {
            if ( MechanismFactory::GetMechanismFactory()->GetTurret().get() != nullptr )
            {
                m_currentState->Run();
            }
        }
        
    }
}