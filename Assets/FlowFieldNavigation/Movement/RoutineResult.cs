﻿using Unity.Mathematics;


namespace FlowFieldNavigation
{
    internal struct RoutineResult
    {
        internal float2 NewDirection;
        internal float3 NewDirection3;
        internal float2 NewSeperation;
        internal float HeightChange;
        internal byte NewSplitInterval;
        internal byte NewSplitInfo;
        internal AvoidanceStatus NewAvoidance;
        internal MovingAvoidanceStatus NewMovingAvoidance;
        internal bool HasForeignInFront;
    }


}