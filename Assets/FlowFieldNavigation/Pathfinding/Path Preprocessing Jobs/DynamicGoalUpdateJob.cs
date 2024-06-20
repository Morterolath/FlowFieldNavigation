using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct DynamicGoalUpdateJob : IJobParallelFor
    {
        internal float TileSize;
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        internal int SectorRowAmount;
        internal int SectorTileAmount;
        internal int FieldRowAmount;
        internal int FieldColAmount;
        internal float FieldMinXIncluding;
        internal float FieldMinYIncluding;
        internal float FieldMaxXExcluding;
        internal float FieldMaxYExcluding;
        internal float2 FieldGridStartPos;
        [ReadOnly] internal NativeArray<int> PathIslandSeedsAsFieldIndices;
        [ReadOnly] internal NativeArray<PathState> PathStateArray;
        [ReadOnly] internal NativeArray<float3> AgentPositions;
        [ReadOnly] internal NativeArray<IslandFieldProcessor> IslandFieldProcessors;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<byte>> CostFields;
        [ReadOnly] internal NativeArray<float> PathDesiredRanges;
        internal NativeArray<float> PathRanges;
        internal NativeArray<PathRoutineData> PathOrganizationDataArray;
        internal NativeArray<PathDestinationData> PathDestinationDataArray;
        public void Execute(int index)
        {
            PathState pathState = PathStateArray[index];
            bool isBeingReconstructed = (PathOrganizationDataArray[index].Task & PathTask.Reconstruct) == PathTask.Reconstruct;
            if (pathState == PathState.Removed || isBeingReconstructed)
            {
                return;
            }
            PathDestinationData destinationData = PathDestinationDataArray[index];
            if (destinationData.DestinationType == DestinationType.DynamicDestination)
            {
                //Data structures
                UnsafeListReadOnly<byte> costs = CostFields[destinationData.Offset];
                IslandFieldProcessor islandFieldProcessor = IslandFieldProcessors[destinationData.Offset];

                //Get targets
                float3 targetAgentPos = AgentPositions[destinationData.TargetAgentIndex];
                float2 targetAgentPos2 = new float2(targetAgentPos.x, targetAgentPos.z);

                //Get ranges
                float desiredRange = PathDesiredRanges[index];
                float oldRange = PathRanges[index];

                //Clamp destination to bounds
                targetAgentPos2.x = math.select(targetAgentPos2.x, FieldMinXIncluding, targetAgentPos2.x < FieldMinXIncluding);
                targetAgentPos2.y = math.select(targetAgentPos2.y, FieldMinYIncluding, targetAgentPos2.y < FieldMinYIncluding);
                targetAgentPos2.x = math.select(targetAgentPos2.x, FieldMaxXExcluding - TileSize / 2, targetAgentPos2.x >= FieldMaxXExcluding);
                targetAgentPos2.y = math.select(targetAgentPos2.y, FieldMaxYExcluding - TileSize / 2, targetAgentPos2.y >= FieldMaxYExcluding);

                float2 oldDestination = destinationData.Destination;
                float2 newDesiredDestination = targetAgentPos2;
                int2 oldDestinationIndex = FlowFieldUtilities.PosTo2D(oldDestination, TileSize, FieldGridStartPos);
                int2 newDesiredDestinationIndex = FlowFieldUtilities.PosTo2D(newDesiredDestination, TileSize, FieldGridStartPos);
                LocalIndex1d newDesiredDestinationLocal = FlowFieldUtilities.GetLocal1D(newDesiredDestinationIndex, SectorColAmount, SectorMatrixColAmount);
                byte newDesiredDestinationCost = costs[newDesiredDestinationLocal.sector * SectorTileAmount + newDesiredDestinationLocal.index];


                //Get old and new islands
                int islandSeedAsFieldIndex = PathIslandSeedsAsFieldIndices[index];
                int2 islandSeed2d = FlowFieldUtilities.To2D(islandSeedAsFieldIndex, FieldColAmount);
                int oldDestinationIsland = islandFieldProcessor.GetIsland(islandSeed2d);
                int newDesiredDestinationIsland = islandFieldProcessor.GetIsland(newDesiredDestinationIndex);

                //Test
                float2 newActualDestination = newDesiredDestination;
                int2 newActualDestinationIndex = newDesiredDestinationIndex;
                float newActualRange = desiredRange;

                bool shouldExpandNewDestination = newDesiredDestinationCost == byte.MaxValue || oldDestinationIsland != newDesiredDestinationIsland;
                if (shouldExpandNewDestination)
                {
                    int desiredIsland = oldDestinationIsland;
                    newActualDestination = GetExtendedGoal(newDesiredDestination, desiredIsland, islandFieldProcessor, costs);
                    newActualDestinationIndex = FlowFieldUtilities.PosTo2D(newActualDestination, TileSize, FieldGridStartPos);
                    newActualRange = 0;
                }
                bool newActualDestinationIndexIsSameAsDesired = newActualDestinationIndex.Equals(newDesiredDestinationIndex);
                bool newActualDestinationIsCloseEnough = math.distancesq(newActualDestination, newDesiredDestination) <= desiredRange * desiredRange;
                if(newActualDestinationIndexIsSameAsDesired || newActualDestinationIsCloseEnough)
                {
                    newActualDestination = newDesiredDestination;
                    newActualDestinationIndex = newDesiredDestinationIndex;
                    newActualRange = desiredRange;
                }

                bool rangeChanged = oldRange != newActualRange;
                if (rangeChanged)
                {
                    PathRanges[index] = newActualRange;
                }
                destinationData.DesiredDestination = newDesiredDestination;
                destinationData.Destination = newActualDestination;
                PathDestinationDataArray[index] = destinationData;

                PathRoutineData organizationData = PathOrganizationDataArray[index];
                bool goalIndexChanged = !oldDestinationIndex.Equals(newActualDestinationIndex);
                bool goalPosChanged = !oldDestination.Equals(newActualDestinationIndex);
                bool goalIsRanged = newActualRange != 0;
                bool goalMoved = goalIndexChanged || (goalIsRanged && goalPosChanged);
                organizationData.DestinationState =  goalMoved ? DynamicDestinationState.Moved : DynamicDestinationState.None;
                organizationData.DestinationState = rangeChanged ? DynamicDestinationState.OutOfReach : organizationData.DestinationState;
                PathOrganizationDataArray[index] = organizationData;
            }
        }
        float2 GetExtendedGoal(float2 goal, int desiredIsland, IslandFieldProcessor islandFieldProcessor, UnsafeListReadOnly<byte> costs)
        {
            int2 goalIndex = FlowFieldUtilities.PosTo2D(goal, TileSize, FieldGridStartPos);
            int2 extendedGoalIndex = ClosestIndexWithIslandQuery.GetClosestIndexWithIsland(
                goal,
                desiredIsland,
                TileSize,
                FieldGridStartPos,
                SectorTileAmount,
                SectorColAmount,
                SectorRowAmount,
                SectorMatrixColAmount,
                FieldRowAmount,
                FieldColAmount,
                islandFieldProcessor,
                costs);
            float2 extendedGoalPos = FlowFieldUtilities.IndexToPos(extendedGoalIndex, TileSize, FieldGridStartPos);
            return math.select(extendedGoalPos,goal, extendedGoalIndex.Equals(goalIndex));
        }
    }
}