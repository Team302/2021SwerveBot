#pragma once

class PrimitiveParams;

class IPrimitive
{
    public:
         static const  double LOOP_LENGTH = 0.020; //Program loop duration in seconds

        IPrimitive() = default;
        virtual ~IPrimitive() = default;
        virtual void Init(PrimitiveParams*	Parms) = 0;
        virtual void Run() = 0;
        virtual bool IsDone() = 0;

};