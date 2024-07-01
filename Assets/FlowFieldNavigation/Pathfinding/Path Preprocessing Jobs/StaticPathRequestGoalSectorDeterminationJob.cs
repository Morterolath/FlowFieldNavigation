using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    internal struct StaticPathRequestGoalSectorDeterminationJob : IJob
    {
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        internal int SectorMatrixRowAmount;
        internal int SectorTileAmount;
        internal float TileSize;
        internal float2 FieldGridStartPos;
        [ReadOnly] internal NativeList<FinalPathRequest> FinalPathRequests;
        internal NativeParallelMultiHashMap<int, int> PathIndexToGoalSectors;
        internal NativeList<float> StaticGoalSectorBfsGrids;
        internal NativeList<int> StaticGoalSectorOffsets;
        internal NativeList<int> StaticGoalSectors;
        public void Execute()
        {
            NativeArray<FinalPathRequest> finalPathRequestsAsArray = FinalPathRequests.AsArray();
            for(int i = 0; i < finalPathRequestsAsArray.Length; i++)
            {
                FinalPathRequest request = finalPathRequestsAsArray[i];
                if (!request.IsValid()) { continue; }
                if (request.Type != DestinationType.DynamicDestination) { continue; }
                request.GoalSectorStartIndex = StaticGoalSectors.Length;
                int pathIndex = request.PathIndex;
                float2 goal = request.Destination;
                float goalRange = request.Range;

                //Submit sectors
                int goalSectorCount = 0;
                GridCircleBorderOverlapper rangeBoderOverlapper = new GridCircleBorderOverlapper(goal, goalRange, SectorColAmount, TileSize, FieldGridStartPos);
                rangeBoderOverlapper.Start();
                while(rangeBoderOverlapper.TryGetCurrent(out int2 currentSector))
                {
                    if(!FlowFieldUtilities.SectorOutOfBounds(currentSector, SectorMatrixColAmount, SectorMatrixRowAmount))
                    {
                        int currentSector1d = FlowFieldUtilities.To1D(currentSector, SectorMatrixColAmount);
                        PathIndexToGoalSectors.Add(pathIndex, currentSector1d);
                        StaticGoalSectors.Add(currentSector1d);
                        StaticGoalSectorBfsGrids.Length += SectorTileAmount;
                        goalSectorCount++;
                    }
                }
                request.GoalSectorCount = goalSectorCount;
                finalPathRequestsAsArray[i] = request;

                StaticGoalSectorOffsets.Length += goalSectorCount;
                for(int j = 0; j < request.GoalSectorCount; j++)
                {
                    int staticGoalSectorIndex = j + request.GoalSectorStartIndex;
                    StaticGoalSectorOffsets[staticGoalSectorIndex] = request.Offset;
                }
            }
        }
    }
}