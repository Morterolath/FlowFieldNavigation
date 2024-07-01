using System;

namespace FlowFieldNavigation
{
    [Flags]
    internal enum PathSectorState : byte
    {
        Included = 1,
        Source = 2,
        FlowCalculated = 4,
        IntegrationCalculated = 8,
        PossibleGoal = 16,
    }


}