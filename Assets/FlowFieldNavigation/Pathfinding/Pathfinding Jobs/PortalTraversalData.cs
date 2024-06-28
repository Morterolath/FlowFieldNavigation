using Unity.Burst;
using Unity.VisualScripting.YamlDotNet.Core;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct PortalTraversalData
    {
        internal int OriginIndex;
        internal int NextIndex;
        internal int PortalCountToGoal;
        internal float GCost;
        internal float HCost;
        internal float FCost;
        internal float DistanceFromTarget;
        internal PortalTraversalMark Mark;
        internal void Reset()
        {
            this = new PortalTraversalData()
            {
                NextIndex = -1,
                OriginIndex = -1,
                PortalCountToGoal = -1,
                GCost = 0,
                HCost = 0,
                FCost = 0,
                DistanceFromTarget = float.MaxValue,
                Mark = 0
            };
        }
        internal bool HasMark(PortalTraversalMark mark)
        {
            return (Mark & mark) == mark;
        }
    }

}

