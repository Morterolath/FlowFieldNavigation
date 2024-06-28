using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct NewPortalTraversalJob : IJob
    {
        internal int2 Target;
        internal float TileSize;
        internal float2 FieldGridStartPos;
        internal int IslandSeed;
        internal int FieldColAmount;
        internal int SectorColAmount;
        internal int NewPortalSliceStartIndex;
        internal int SectorMatrixColAmount;
        internal int SectorMatrixRowAmount;
        internal int SectorTileAmount;
        internal int LosRange;
        internal int NewPickedSectorStartIndex;

        internal NativeReference<SectorsWihinLOSArgument> SectorWithinLosRange;
        internal NativeArray<PortalTraversalData> PortalTraversalDataArray;
        internal NativeList<ActivePortal> PortalSequence;
        internal NativeList<Slice> PortalSequenceSlices;
        internal UnsafeList<PathSectorState> SectorStateTable;
        internal NativeList<int> PickedSectorIndicies;
        internal NativeList<IntegrationTile> IntegrationField;
        internal NativeList<int> SourcePortalIndexList;
        internal NativeHashSet<int> PossibleGoalSectors;
        internal NativeList<PickedPortalDataRecord> PickedPortalDataRecords;

        [ReadOnly] internal NativeSlice<float2> SourcePositions;
        [ReadOnly] internal NativeArray<byte> Costs;
        [ReadOnly] internal NativeArray<SectorNode> SectorNodes;
        [ReadOnly] internal NativeArray<int> SecToWinPtrs;
        [ReadOnly] internal NativeArray<WindowNode> WindowNodes;
        [ReadOnly] internal NativeArray<PortalNode> PortalNodes;
        [ReadOnly] internal NativeArray<PortalToPortal> PorPtrs;
        [ReadOnly] internal NativeArray<UnsafeList<int>> IslandFields;
        public void Execute()
        {
            PortalTraversalProc.Run(
                Target,
                IslandSeed,
                FieldColAmount,
                SectorColAmount,
                SectorTileAmount,
                TileSize,
                FieldGridStartPos,
                SectorMatrixColAmount,
                SectorMatrixRowAmount,
                NewPortalSliceStartIndex,
                NewPickedSectorStartIndex,
                LosRange,
                PickedPortalDataRecords,
                Costs,
                IntegrationField,
                PortalSequence,
                PortalSequenceSlices,
                PickedSectorIndicies,
                SourcePositions,
                SectorStateTable,
                SourcePortalIndexList,
                PorPtrs,
                PortalTraversalDataArray,
                SectorNodes,
                WindowNodes,
                SecToWinPtrs,
                PortalNodes,
                IslandFields,
                PossibleGoalSectors,
                SectorWithinLosRange);
        }
    }
}