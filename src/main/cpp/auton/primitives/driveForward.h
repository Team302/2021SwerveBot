/*#pragma once
#include <frc/geometry/Pose2d.h>

class PrimitiveParams;
class driveForward
{
    public:
        bool IsDone();
        void Init(PrimitiveParams* params);
        void Run();
        driveForward();
        virtual ~driveForward() = default;

    protected:
        void SetDistance(double distance);

    private:
        void CalculateSlowDownDistance();
        PrimitiveParams* m_params;
        Pose2d* m_pose;

        float m_targetDistance;
        float m_initialDistance;
	    float m_timeRemaining;

        float m_minSpeedCountTime;
        int m_underSpeedCounts;
        float m_startHeading;
        float m_endHeading;
        float m_minSpeed;
        bool m_arcing;

        const double SPEED_THRESHOLD = 1.5;
        const double MIN_SPEED_COUNT_TIME = 0.5; //seconds before we start checking for wall collisions
        const int UNDER_SPEED_COUNT_THRESHOLD = 4;
        const double DECEL_TIME_MULTIPLIER = 0.85; //0.75

};*/