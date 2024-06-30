using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct GoalNeighborPortalDeterminationJob : IJobParallelFor
    {
        public void Execute(int index)
        {
            throw new System.NotImplementedException();
        }
    }
}