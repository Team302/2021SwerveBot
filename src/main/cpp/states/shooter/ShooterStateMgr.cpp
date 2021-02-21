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
#include <map>
#include <memory>
#include <vector>

// FRC includes
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

// Team 302 includes
#include <states/IState.h>
#include <states/balltransfer/BallTransferStateMgr.h>
#include <states/ballhopper/BallHopperStateMgr.h>
#include <states/shooter/ShooterStateMgr.h>
#include <states/shooter/ShooterState.h>
#include <states/turret/TurretStateMgr.h>
#include <xmlmechdata/StateDataDefn.h>
#include <controllers/MechanismTargetData.h>
#include <utils/Logger.h>
#include <gamepad/TeleopControl.h>
#include <subsys/MechanismFactory.h>
#include <subsys/MechanismTypes.h>


// Third Party Includes

using namespace std;

ShooterStateMgr* ShooterStateMgr::m_instance = nullptr;
ShooterStateMgr* ShooterStateMgr::GetInstance()
{
	if ( ShooterStateMgr::m_instance == nullptr )
	{
        // Create the shooter state manager, ball transfer state manager, turret state manager and hopper state manager
		ShooterStateMgr::m_instance = new ShooterStateMgr();
        BallTransferStateMgr::GetInstance();
        TurretStateMgr::GetInstance();
        BallHopperStateMgr::GetInstance();
	}
	return ShooterStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
ShooterStateMgr::ShooterStateMgr() : m_stateVector(),
                                     m_currentState()
{
    // Parse the configuration file 
    auto stateXML = make_unique<StateDataDefn>();
    vector<MechanismTargetData*> targetData = stateXML.get()->ParseXML( MechanismTypes::MECHANISM_TYPE::SHOOTER );

    // initialize the xml string to state map
    map<string, SHOOTER_STATE> stateStringToEnumMap;
    stateStringToEnumMap["SHOOTEROFF"] = SHOOTER_STATE::OFF;
    stateStringToEnumMap["SHOOTERGETREADY"]  = SHOOTER_STATE::GET_READY;
    stateStringToEnumMap["SHOOTERSHOOTGREEN"] = SHOOTER_STATE::SHOOTGREEN;
    stateStringToEnumMap["SHOOTERSHOOTYELLOW"] = SHOOTER_STATE::SHOOTYELLOW;
    stateStringToEnumMap["SHOOTERSHOOTBLUE"] = SHOOTER_STATE::SHOOTBLUE;
    stateStringToEnumMap["SHOOTERSHOOTRED"] = SHOOTER_STATE::SHOOTRED;
    m_stateVector.resize(6);

    // create the states passing the configuration data
    for ( auto td: targetData )
    {
        auto stateString = td->GetStateString();
        auto stateStringToEnumMapItr = stateStringToEnumMap.find( stateString );
        if ( stateStringToEnumMapItr != stateStringToEnumMap.end() )
        {
            auto stateEnum = stateStringToEnumMapItr->second;
            if ( m_stateVector[stateEnum] == nullptr )
            {
                auto controlData = td->GetController();
                auto target1 = td->GetTarget();
                auto target2 = td->GetSecondTarget();

                switch ( stateEnum )
                {
                    case SHOOTER_STATE::OFF:
                    {   
                        auto thisState = new ShooterState( controlData, target1, target2 );
                        m_stateVector[stateEnum] = thisState;
                        m_currentState = thisState;
                        m_currentStateEnum = stateEnum;
                        m_currentState->Init();
                    }
                    break;

                    case SHOOTER_STATE::GET_READY:
                    {   
                        auto thisState = new ShooterState( controlData, target1, target2 );
                        m_stateVector[stateEnum] = thisState;
                    }
                    break;

                    case SHOOTER_STATE::SHOOTGREEN:
                    {   
                        auto thisState = new ShooterState( controlData, target1, target2 );
                        m_stateVector[stateEnum] = thisState;
                    }
                    break;

                    case SHOOTER_STATE::SHOOTYELLOW:
                    {   
                        auto thisState = new ShooterState( controlData, target1, target2 );
                        m_stateVector[stateEnum] = thisState;
                    }
                    break;

                    case SHOOTER_STATE::SHOOTBLUE:
                    {   
                        auto thisState = new ShooterState( controlData, target1, target2 );
                        m_stateVector[stateEnum] = thisState;
                    }
                    break;
                    
                    case SHOOTER_STATE::SHOOTRED:
                    {   
                        auto thisState = new ShooterState( controlData, target1, target2 );
                        m_stateVector[stateEnum] = thisState;
                    }
                    break;

                    default:
                    {
                        Logger::GetLogger()->LogError( string("ShooterStateMgr::ShooterStateMgr"), string("unknown state"));
                    }
                    break;
                }
            }
            else
            {
                Logger::GetLogger()->LogError( string("ShooterStateMgr::ShooterStateMgr"), string("multiple mechanism state info for state"));
            }
        }
        else
        {
            Logger::GetLogger()->LogError( string("ShooterStateMgr::ShooterStateMgr"), string("state not found"));
        }
    }
}

/// @brief  run the current state
/// @return void
void ShooterStateMgr::RunCurrentState()
{
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable(string("Shooter State Manager"));
    auto controller = TeleopControl::GetInstance();

//    if ( MechanismFactory::GetMechanismFactory()->GetShooter().get() != nullptr && controller != nullptr )
    if (controller != nullptr)
    {
        if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_PREPARE_TO_SHOOT ))
        {
            nt.get()->PutString("Current State", "Prepare to Shoot");
            SetCurrentState( SHOOTER_STATE::GET_READY, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
        }
        else if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_MANUAL_SHOOT_GREEN ))
        {
            nt.get()->PutString("Current State", "Shoot Green");
            SetCurrentState( SHOOTER_STATE::SHOOTGREEN, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::RAPID_RELEASE, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
        }
        else if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_MANUAL_SHOOT_YELLOW ))
        {
            nt.get()->PutString("Current State", "Shoot Yellow");
            SetCurrentState( SHOOTER_STATE::SHOOTYELLOW, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::RAPID_RELEASE, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
        }
        else if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_MANUAL_SHOOT_BLUE ))
        {
            nt.get()->PutString("Current State", "Shoot Blue");
            SetCurrentState( SHOOTER_STATE::SHOOTBLUE, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::RAPID_RELEASE, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
        }
        else if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_MANUAL_SHOOT_RED ))
        {
            nt.get()->PutString("Current State", "Shoot Red");
            SetCurrentState( SHOOTER_STATE::SHOOTRED, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::RAPID_RELEASE, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
        }
    }

    // run the current state
    if ( m_currentState != nullptr )
    {
        m_currentState->Run();
    }
    BallHopperStateMgr::GetInstance()->RunCurrentState();
    BallTransferStateMgr::GetInstance()->RunCurrentState();
}

/// @brief  set the current state, initialize it and run it
/// @return void
void ShooterStateMgr::SetCurrentState
(
    SHOOTER_STATE  stateEnum,
    bool            run
)
{
    auto state = m_stateVector[stateEnum];
    if ( state != nullptr && state != m_currentState )
    {
        auto nt = nt::NetworkTableInstance::GetDefault().GetTable(string("Shooter State Manager"));
        nt.get()->PutString("Current State", "none");
        
        m_currentState = state;
        m_currentStateEnum = stateEnum;
        m_currentState->Init();
        if ( stateEnum == SHOOTER_STATE::GET_READY )
        {
            nt.get()->PutString("Current State", "Prepare to Shoot");
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, run );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, run);
        }
        else if ( stateEnum == SHOOTER_STATE::SHOOTBLUE || stateEnum == SHOOTER_STATE::SHOOTGREEN ||
                  stateEnum == SHOOTER_STATE::SHOOTRED  || stateEnum == SHOOTER_STATE::SHOOTYELLOW )
        {
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, run );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::RAPID_RELEASE, run);
            if ( stateEnum == SHOOTER_STATE::SHOOTBLUE)
            {
                nt.get()->PutString("Current State", "Shoot Blue");
            }
            else if ( stateEnum == SHOOTER_STATE::SHOOTGREEN)
            {
                nt.get()->PutString("Current State", "Shoot Green");
            }
            else if ( stateEnum == SHOOTER_STATE::SHOOTRED)
            {
                nt.get()->PutString("Current State", "Shoot Red");
            }
            else
            {
                nt.get()->PutString("Current State", "Shoot Yellow");
            }
        }
        if ( run )
        {
            //if ( MechanismFactory::GetMechanismFactory()->GetShooter().get() != nullptr)
            //{
                m_currentState->Run();
            //}
        }
    }
}


