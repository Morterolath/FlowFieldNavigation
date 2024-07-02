using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEditor.PackageManager.Requests;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct PathRequestGoalSectorDeterminationJob : IJob
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
        internal NativeList<float> DynamicGoalSectorBfsGrids;
        internal NativeList<int> DynamicGoalSectors;
        internal NativeList<int> DynamicGoalSectorIndexToFinalPathRequestIndex;
        public void Execute()
        {
            NativeArray<FinalPathRequest> finalPathRequestsAsArray = FinalPathRequests.AsArray();
            for(int i = 0; i < finalPathRequestsAsArray.Length; i++)
            {
                FinalPathRequest request = finalPathRequestsAsArray[i];
                if (!request.IsValid()) { continue; }
                if (request.Type == DestinationType.StaticDestination)
                {
                    finalPathRequestsAsArray[i] = RunForRequest(request, i, StaticGoalSectors, StaticGoalSectorIndexToFinalPathRequestIndex, StaticGoalSectorBfsGrids);
                }
                else if(request.Type == DestinationType.DynamicDestination)
                {
                    finalPathRequestsAsArray[i] = RunForRequest(request, i, DynamicGoalSectors, DynamicGoalSectorIndexToFinalPathRequestIndex, DynamicGoalSectorBfsGrids);
                }
            }
        }
        FinalPathRequest RunForRequest(
            FinalPathRequest request, 
            int requestIndex, 
            NativeList<int> goalSectors, 
            NativeList<int> goalSectorIndexToFinalPathRequestIndex, 
            NativeList<float> goalSectorBfsGrids)
        {
            request.GoalSectorStartIndex = goalSectors.Length;
            int pathIndex = request.PathIndex;
            float2 goal = request.Destination;
            float goalRange = request.Range;

            //Submit sectors
            int goalSectorCount = 0;
            GridCircleBorderOverlapper rangeBoderOverlapper = new GridCircleBorderOverlapper(goal, goalRange, SectorColAmount, TileSize, FieldGridStartPos);
            rangeBoderOverlapper.Start();
            while (rangeBoderOverlapper.TryGetCurrent(out int2 currentSector))
            {
                rangeBoderOverlapper.MoveNext();
                if (!FlowFieldUtilities.SectorOutOfBounds(currentSector, SectorMatrixColAmount, SectorMatrixRowAmount))
                {
                    int currentSector1d = FlowFieldUtilities.To1D(currentSector, SectorMatrixColAmount);
                    PathIndexToGoalSectors.Add(pathIndex, currentSector1d);
                    goalSectors.Add(currentSector1d);
                    goalSectorIndexToFinalPathRequestIndex.Add(requestIndex);
                    goalSectorBfsGrids.Length += SectorTileAmount;
                    goalSectorCount++;
                }
            }
            request.GoalSectorCount = goalSectorCount;
            return request;
        }
    }
}