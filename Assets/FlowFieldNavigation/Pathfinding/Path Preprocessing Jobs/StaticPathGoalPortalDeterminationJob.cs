using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct StaticPathGoalPortalDeterminationJob : IJob
    {
        internal int SectorTileAmount;
        internal int SectorMatrixColAmount;
        internal int SectorColAmount;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<SectorNode>> SectorNodesPerOffset;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<int>> SecToWinPtrsPerOffset;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<WindowNode>> WindowNodesPerOffset;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<PortalNode>> PortalNodesPerOffset;
        [ReadOnly] internal NativeList<FinalPathRequest> FinalPathRequests;
        [ReadOnly] internal NativeList<int> StaticGoalSectors;
        [ReadOnly] internal NativeList<float> StaticGoalSectorBfsGrid;
        [WriteOnly] internal NativeList<UnsafeList<GoalNeighborPortal>> PathGoalNeighborPortals;
        public void Execute()
        {
            NativeHashMap<int, int> duplicateEliminationSet = new NativeHashMap<int, int>(0, Allocator.Temp);
            NativeArray<FinalPathRequest> finalPathRequestsAsArray = FinalPathRequests.AsArray();
            NativeArray<int> staticGoalSectorsAsArray = StaticGoalSectors.AsArray();
            NativeArray<float> staticGoalSectorBfsGridAsArray = StaticGoalSectorBfsGrid.AsArray();
            for (int i = 0; i < finalPathRequestsAsArray.Length; i++)
            {
                FinalPathRequest request = finalPathRequestsAsArray[i];
                if (!request.IsValid()) { continue; }
                int goalSectorStart = request.GoalSectorStartIndex;
                int goalSectorCount = request.GoalSectorCount;
                int pathIndex = request.PathIndex;
                int offset = request.Offset;
                UnsafeList<GoalNeighborPortal> goalNeighbors = new UnsafeList<GoalNeighborPortal>(0, Allocator.Persistent);
                UnsafeListReadOnly<SectorNode> sectorNodes = SectorNodesPerOffset[offset];
                UnsafeListReadOnly<int> secToWinPtrs = SecToWinPtrsPerOffset[offset];
                UnsafeListReadOnly<WindowNode> windowNodes = WindowNodesPerOffset[offset];
                UnsafeListReadOnly<PortalNode> portalNodes = PortalNodesPerOffset[offset];
                for(int j = goalSectorStart; j < goalSectorStart + goalSectorCount; j++)
                {
                    int sectorIndex = staticGoalSectorsAsArray[j];
                    NativeSlice<float> sectorBfsResults = new NativeSlice<float>(staticGoalSectorBfsGridAsArray, j * SectorTileAmount, SectorTileAmount);
                    SectorNode sectorNode = sectorNodes[sectorIndex];
                    int winPtr = sectorNode.SecToWinPtr;
                    int winCnt = sectorNode.SecToWinCnt;
                    for (int k = 0; k < winCnt; k++)
                    {
                        WindowNode windowNode = windowNodes[secToWinPtrs[winPtr + k]];
                        int porPtr = windowNode.PorPtr;
                        int porCnt = windowNode.PorCnt;
                        for (int l = 0; l < porCnt; l++)
                        {
                            int portalIndex = l + porPtr;
                            PortalNode portalNode = portalNodes[portalIndex];
                            int localIndex = FlowFieldUtilities.GetLocal1dInSector(portalNode, sectorIndex, SectorMatrixColAmount, SectorColAmount);

                            float cost = sectorBfsResults[localIndex];
                            if (cost == float.MaxValue) { continue; }
                            if(duplicateEliminationSet.TryGetValue(portalIndex, out int goalNeighborListIndex))
                            {
                                GoalNeighborPortal existing = goalNeighbors[goalNeighborListIndex];
                                if(existing.Distance > cost)
                                {
                                    goalNeighbors[goalNeighborListIndex] = new GoalNeighborPortal(portalIndex, cost);
                                }
                            }
                            else
                            {
                                duplicateEliminationSet.Add(portalIndex, goalNeighbors.Length);
                                goalNeighbors.Add(new GoalNeighborPortal(portalIndex, cost));
                            }
                        }
                    }
                }
                duplicateEliminationSet.Clear();
                PathGoalNeighborPortals[pathIndex] = goalNeighbors;
            }
        }
    }
}
