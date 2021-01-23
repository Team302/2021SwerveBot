
#pragma once

// C++ Includes
#include <vector>

// FRC includes

// Team 302 includes
#include <auton/PrimitiveEnums.h>
#include <states/BallManipulator.h>
//#include <utils/DragonMath.h>

// Third Party Includes





class PrimitiveParams
{
    public:

        PrimitiveParams
        (
                PRIMITIVE_IDENTIFIER                                id,
                float                                               time,
                float                                               distance,
                float                                               xLoc,
                float                                               yLoc,
                float                                               heading,
                float                                               startDriveSpeed,
                float                                               endDriveSpeed,
                BallManipulator::BALL_MANIPULATOR_STATE             ballState,
                float                                               turretAngle
        );//Constructor. Takes in all parameters

        PrimitiveParams() = delete;
        virtual ~PrimitiveParams() = default;//Destructor


        //Some getters
        PRIMITIVE_IDENTIFIER GetID() const;
        float GetTime() const;
        float GetDistance() const;
        float GetXLocation() const;
        float GetYLocation() const;
        float GetHeading() const;
        float GetDriveSpeed() const;
        float GetEndDriveSpeed() const;
        BallManipulator::BALL_MANIPULATOR_STATE GetBallState() const;
        float GetTurretAngle() const;

        //Setters
        void SetDistance(float distance);

    private:
        //Primitive Parameters
        PRIMITIVE_IDENTIFIER                                m_id; //Primitive ID
        float                                               m_time;
        float                                               m_distance;
        float                                               m_xLoc;
        float                                               m_yLoc;
        float                                               m_heading;
        float                                               m_startDriveSpeed;
        float                                               m_endDriveSpeed;
        BallManipulator::BALL_MANIPULATOR_STATE             m_ballState;
        float                                               m_turretAngle;
};

typedef std::vector<PrimitiveParams*> PrimitiveParamsVector;
