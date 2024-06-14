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
        [ReadOnly] internal NativeArray<PathState> PathStateArray;
        [ReadOnly] internal NativeArray<float3> AgentPositions;
        [ReadOnly] internal NativeArray<IslandFieldProcessor> IslandFieldProcessors;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<byte>> CostFields;
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

                //Clamp destination to bounds
                targetAgentPos2.x = math.select(targetAgentPos2.x, FieldMinXIncluding, targetAgentPos2.x < FieldMinXIncluding);
                targetAgentPos2.y = math.select(targetAgentPos2.y, FieldMinYIncluding, targetAgentPos2.y < FieldMinYIncluding);
                targetAgentPos2.x = math.select(targetAgentPos2.x, FieldMaxXExcluding - TileSize / 2, targetAgentPos2.x >= FieldMaxXExcluding);
                targetAgentPos2.y = math.select(targetAgentPos2.y, FieldMaxYExcluding - TileSize / 2, targetAgentPos2.y >= FieldMaxYExcluding);

                float2 oldDestination = destinationData.Destination;
                float2 newDestination = targetAgentPos2;
                int2 oldDestinationIndex = FlowFieldUtilities.PosTo2D(oldDestination, TileSize, FieldGridStartPos);
                int2 newDestinationIndex = FlowFieldUtilities.PosTo2D(newDestination, TileSize, FieldGridStartPos);
                LocalIndex1d newDestinationLocal = FlowFieldUtilities.GetLocal1D(newDestinationIndex, SectorColAmount, SectorMatrixColAmount);
                byte newDestinationCost = costs[newDestinationLocal.sector * SectorTileAmount + newDestinationLocal.index];
                int oldDestinationIsland = islandFieldProcessor.GetIsland(oldDestinationIndex);
                int newDestinationIsland = islandFieldProcessor.GetIsland(newDestinationIndex);

                //Test
                bool shouldExpandNewDestination = newDestinationCost == byte.MaxValue || oldDestinationIsland != newDestinationIsland;
                if (shouldExpandNewDestination)
                {
                    int desiredIsland = oldDestinationIsland;
                    newDestination = GetExtendedGoal(newDestination, desiredIsland, islandFieldProcessor, costs);
                }

                //Output
                destinationData.DesiredDestination = targetAgentPos2;
                destinationData.Destination = newDestination;
                PathDestinationDataArray[index] = destinationData;

                PathRoutineData organizationData = PathOrganizationDataArray[index];
                organizationData.DestinationState = oldDestinationIndex.Equals(newDestinationIndex) ? DynamicDestinationState.None : DynamicDestinationState.Moved;
                PathOrganizationDataArray[index] = organizationData;
            }
        }
        float2 GetExtendedGoal(float2 goal, int desiredIsland, IslandFieldProcessor islandFieldProcessor, UnsafeListReadOnly<byte> costs)
        {
            int2 oldGoalIndex = FlowFieldUtilities.PosTo2D(goal, TileSize, FieldGridStartPos);
            int2 newGoalIndex = ClosestIndexWithIslandQuery.GetClosestIndexWithIsland(
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
            float2 oldGoalPos = goal;
            float2 newGoalPos = FlowFieldUtilities.IndexToPos(newGoalIndex, TileSize, FieldGridStartPos);
            return math.select(newGoalPos,oldGoalPos, newGoalIndex.Equals(oldGoalIndex));
        }
    }
}