using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct StaticPathRequestGoalSectorDeterminationJob : IJob
    {
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        internal int SectorMatrixRowAmount;
        internal int SectorTileAmount;
        internal float TileSize;
        internal float2 FieldGridStartPos;
        internal NativeList<FinalPathRequest> FinalPathRequests;
        internal NativeParallelMultiHashMap<int, int> PathIndexToGoalSectors;
        internal NativeList<float> StaticGoalSectorBfsGrids;
        internal NativeList<int> StaticGoalSectors;
        internal NativeList<int> StaticGoalSectorIndexToFinalPathRequestIndex;
        public void Execute()
        {
            NativeArray<FinalPathRequest> finalPathRequestsAsArray = FinalPathRequests.AsArray();
            for(int i = 0; i < finalPathRequestsAsArray.Length; i++)
            {
                FinalPathRequest request = finalPathRequestsAsArray[i];
                if (!request.IsValid()) { continue; }
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
                    rangeBoderOverlapper.MoveNext();
                    if(!FlowFieldUtilities.SectorOutOfBounds(currentSector, SectorMatrixColAmount, SectorMatrixRowAmount))
                    {
                        int currentSector1d = FlowFieldUtilities.To1D(currentSector, SectorMatrixColAmount);
                        PathIndexToGoalSectors.Add(pathIndex, currentSector1d);
                        StaticGoalSectors.Add(currentSector1d);
                        StaticGoalSectorIndexToFinalPathRequestIndex.Add(i);
                        StaticGoalSectorBfsGrids.Length += SectorTileAmount;
                        goalSectorCount++;
                    }
                }
                request.GoalSectorCount = goalSectorCount;
                finalPathRequestsAsArray[i] = request;
            }
        }
    }
}