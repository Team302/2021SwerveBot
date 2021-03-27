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
#include <hw/factories/LimelightFactory.h>
#include <hw/DragonLimelight.h>
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
        BallHopperStateMgr::GetInstance();
	    TurretStateMgr::GetInstance();
    }
	return ShooterStateMgr::m_instance;
}

/// @brief    initialize the state manager, parse the configuration file and create the states.
ShooterStateMgr::ShooterStateMgr() : m_states(),
                                     m_currentState(),
                                     m_prevStateEnum(ShooterStateMgr::SHOOTER_STATE::OFF),
                                     m_nt(nt::NetworkTableInstance::GetDefault().GetTable(string("Shooter State Manager")))    
{
    auto shooter = MechanismFactory::GetMechanismFactory()->GetShooter();
    if ( shooter.get() != nullptr)
    {
        m_nt = nt::NetworkTableInstance::GetDefault().GetTable("fred");
    }

    // Parse the configuration file 
    auto stateXML = make_unique<StateDataDefn>();
    vector<MechanismTargetData*> targetData = stateXML.get()->ParseXML( MechanismTypes::MECHANISM_TYPE::SHOOTER );

    // initialize the xml string to state map
    map<string, SHOOTER_STATE> stateStringToEnumMap;
    stateStringToEnumMap["SHOOTEROFF"] = SHOOTER_STATE::OFF;
    stateStringToEnumMap["SHOOTERON"] = SHOOTER_STATE::SHOOT;
    stateStringToEnumMap["SHOOTERSHOOTGREEN"] = SHOOTER_STATE::GET_READY_SHOOTGREEN;
    stateStringToEnumMap["SHOOTERSHOOTYELLOW"] = SHOOTER_STATE::GET_READY_SHOOTYELLOW;
    stateStringToEnumMap["SHOOTERSHOOTBLUE"] = SHOOTER_STATE::GET_READY_SHOOTBLUE;
    stateStringToEnumMap["SHOOTERSHOOTRED"] = SHOOTER_STATE::GET_READY_SHOOTRED;

    Logger::GetLogger()->ToNtTable(m_nt, string("SHOOTEROFF"), string("not created"));
    Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERON", "not created");
    Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERSHOOTGREEN", "not created");
    Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERSHOOTYELLOW", "not created");
    Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERSHOOTBLUE", "not created");
    Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERSHOOTRED", "not created");
    for (auto inx=0; inx<MAX_SHOOTER_STATES; ++inx)
    {
       m_states[inx] = nullptr;
    }

    // create the states passing the configuration data
    for ( auto td: targetData )
    {
        auto stateString = td->GetStateString();
        auto stateStringToEnumMapItr = stateStringToEnumMap.find( stateString );
        if ( stateStringToEnumMapItr != stateStringToEnumMap.end() )
        {
            auto stateEnum = stateStringToEnumMapItr->second;
            if ( m_states[stateEnum] == nullptr )
            {
                auto controlData = td->GetController();
                auto controlData2 = td->GetController2();
                auto target1 = td->GetTarget();
                auto target2 = td->GetSecondTarget();

                switch ( stateEnum )
                {
                    case SHOOTER_STATE::OFF:
                    {   
                        m_states[stateEnum] = new ShooterState( controlData, controlData2, target1, target2 );
                        m_currentState = m_states[SHOOTER_STATE::OFF];
                        m_currentStateEnum = stateEnum;
                        m_currentState->Init();
                        Logger::GetLogger()->ToNtTable(m_nt, "SHOOTEROFF", "created");
                    }
                    break;

                    case SHOOTER_STATE::SHOOT:
                    {   
                        m_states[stateEnum] = new ShooterState( controlData, controlData2, target1, target2 );
                        Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERON", "created");
                    }
                    break;

                    case SHOOTER_STATE::GET_READY_SHOOTGREEN:
                    {   
                        m_states[stateEnum] = new ShooterState( controlData, controlData2, target1, target2 );
                        Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERSHOOTGREEN", "created");
                    }
                    break;

                    case SHOOTER_STATE::GET_READY_SHOOTYELLOW:
                    {   
                        m_states[stateEnum] = new ShooterState( controlData, controlData2, target1, target2 );
                        Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERSHOOTYELLOW", "created");
                    }
                    break;

                    case SHOOTER_STATE::GET_READY_SHOOTBLUE:
                    {   
                        m_states[stateEnum] = new ShooterState( controlData, controlData2, target1, target2 );
                        Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERSHOOTBLUE", "created");
                    }
                    break;
                    
                    case SHOOTER_STATE::GET_READY_SHOOTRED:
                    {   
                        m_states[stateEnum] = new ShooterState( controlData, controlData2, target1, target2 );
                        Logger::GetLogger()->ToNtTable(m_nt, "SHOOTERSHOOTRED", "created");
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
    auto controller = TeleopControl::GetInstance();
    if (controller != nullptr)
    {
        auto cam = LimelightFactory::GetLimelightFactory()->GetLimelight();
        
        if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_SHOOT ))
        {
            Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot");
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::RAPID_RELEASE, false);
            TurretStateMgr::GetInstance()->SetCurrentState(TurretStateMgr::TURRET_STATE::HOLD, false);
        }
        else if ( m_currentStateEnum==ShooterStateMgr::SHOOTER_STATE::OFF)
        {
                // keep everything the same
        }
        else    // revert to holding the ball
        {
            Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot");
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, false);
        }

        if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_PREPARE_TO_SHOOT_GREEN ))
        {
            Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot Green");
            if ( cam != nullptr )
            {
                cam->SetPipeline(1);
            }
            SetCurrentState( SHOOTER_STATE::GET_READY_SHOOTGREEN, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
            TurretStateMgr::GetInstance()->SetCurrentState(TurretStateMgr::TURRET_STATE::LIMELIGHT_AIM, false);
        }
        else if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_PREPARE_TO_SHOOT_YELLOW ))
        {
            Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot Yellow");
            if ( cam != nullptr )
            {
                cam->SetPipeline(2);
            }
            SetCurrentState( SHOOTER_STATE::GET_READY_SHOOTYELLOW, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
            TurretStateMgr::GetInstance()->SetCurrentState(TurretStateMgr::TURRET_STATE::LIMELIGHT_AIM, false);
        }
        else if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_PREPARE_TO_SHOOT_BLUE ))
        {
            Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot Blue");
            if ( cam != nullptr )
            {
                cam->SetPipeline(3);
            }
            SetCurrentState( SHOOTER_STATE::GET_READY_SHOOTBLUE, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
            TurretStateMgr::GetInstance()->SetCurrentState(TurretStateMgr::TURRET_STATE::LIMELIGHT_AIM, false);
        }
        else if ( controller->IsButtonPressed( TeleopControl::FUNCTION_IDENTIFIER::SHOOTER_PREPARE_TO_SHOOT_RED ))
        {
            Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot Red");
            if ( cam != nullptr )
            {
                cam->SetPipeline(4);
            }
            SetCurrentState( SHOOTER_STATE::GET_READY_SHOOTRED, false );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, false);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, false );
            TurretStateMgr::GetInstance()->SetCurrentState(TurretStateMgr::TURRET_STATE::LIMELIGHT_AIM, false);
        }
//        else
//        {
//            SetCurrentState( m_prevStateEnum, false);
//        }
    }

    // run the current state
    if ( m_currentState != nullptr )
    {
        m_currentState->Run();
    }
    BallHopperStateMgr::GetInstance()->RunCurrentState();
    BallTransferStateMgr::GetInstance()->RunCurrentState();
    TurretStateMgr::GetInstance()->RunCurrentState();
}

/// @brief  set the current state, initialize it and run it
/// @return void
void ShooterStateMgr::SetCurrentState
(
    SHOOTER_STATE   stateEnum,
    bool            run
)
{
    auto state = m_states[stateEnum];
    if ( state != nullptr )
    {
        Logger::GetLogger()->ToNtTable(m_nt, "Current State", "none");
        
        m_prevStateEnum = (m_currentStateEnum == stateEnum) ? m_prevStateEnum : m_currentStateEnum;

        auto limelight = LimelightFactory::GetLimelightFactory()->GetLimelight();
        m_currentState = state;
        m_currentStateEnum = stateEnum;
        m_currentState->Init();
        if ( stateEnum == SHOOTER_STATE::SHOOT )
        {
            if ( limelight != nullptr )
            {
                limelight->SetPipeline(1);
            }
            Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot");
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, run );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::RAPID_RELEASE, run);
            TurretStateMgr::GetInstance()->SetCurrentState(TurretStateMgr::TURRET_STATE::HOLD, run);
        }
        else if ( stateEnum == SHOOTER_STATE::GET_READY_SHOOTBLUE || stateEnum == SHOOTER_STATE::GET_READY_SHOOTGREEN ||
                  stateEnum == SHOOTER_STATE::GET_READY_SHOOTRED  || stateEnum == SHOOTER_STATE::GET_READY_SHOOTYELLOW )
        {
            if ( limelight != nullptr )
            {
                limelight->SetPipeline(1);
            }
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, run );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, run);
            TurretStateMgr::GetInstance()->SetCurrentState(TurretStateMgr::TURRET_STATE::LIMELIGHT_AIM, run);
            if ( stateEnum == SHOOTER_STATE::GET_READY_SHOOTBLUE)
            {
                Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot Blue");
            }
            else if ( stateEnum == SHOOTER_STATE::GET_READY_SHOOTGREEN)
            {
                Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot Green");
            }
            else if ( stateEnum == SHOOTER_STATE::GET_READY_SHOOTRED)
            {
                Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot Red");
            }
            else
            {
                Logger::GetLogger()->ToNtTable(m_nt, "Current State", "Shoot Yellow");
            }
        }
        /**
        else
        {
            ShooterStateMgr::GetInstance()->SetCurrentState(ShooterStateMgr::SHOOTER_STATE::OFF, run);
            BallTransferStateMgr::GetInstance()->SetCurrentState( BallTransferStateMgr::BALL_TRANSFER_STATE::TO_SHOOTER, run );
            BallHopperStateMgr::GetInstance()->SetCurrentState( BallHopperStateMgr::HOLD, run);
            TurretStateMgr::GetInstance()->SetCurrentState(TurretStateMgr::TURRET_STATE::HOLD, run);
        }
        **/
        
        if ( run )
        {
            m_currentState->Run();
        }
    }
    else
    {
        Logger::GetLogger()->ToNtTable(m_nt, "state", "nullptr" );
    }
    
}


