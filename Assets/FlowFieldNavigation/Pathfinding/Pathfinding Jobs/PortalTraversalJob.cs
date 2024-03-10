﻿using System;
using System.IO;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct PortalTraversalJob : IJob
    {
        internal int2 Target;
        internal int FieldColAmount;
        internal int SectorColAmount;
        internal int AddedPortalSequenceBorderStartIndex;
        internal int SectorMatrixColAmount;
        internal int SectorMatrixRowAmount;
        internal int SectorTileAmount;
        internal int LOSRange;
        internal NativeArray<PortalTraversalData> PortalTraversalDataArray;
        internal NativeList<ActivePortal> PortalSequence;
        internal NativeList<int> PortalSequenceBorders;
        internal UnsafeList<PathSectorState> SectorStateTable;
        internal NativeList<int> PickedToSector;
        internal NativeReference<int> FlowFieldLength;

        [ReadOnly] internal NativeArray<SectorNode> SectorNodes;
        [ReadOnly] internal NativeArray<WindowNode> WindowNodes;
        [ReadOnly] internal NativeArray<int> WinToSecPtrs;
        [ReadOnly] internal NativeArray<PortalNode> PortalNodes;
        [ReadOnly] internal NativeArray<PortalToPortal> PorPtrs;
        [ReadOnly] internal NativeReference<int> NewPickedSectorStartIndex;

        [ReadOnly] internal NativeList<int> SourcePortalIndexList;
        [ReadOnly] internal NativeList<int> DijkstraStartIndicies;
        [ReadOnly] internal NativeList<int> TargetNeighbourPortalIndicies;
        internal NativeReference<SectorsWihinLOSArgument> SectorWithinLOSState;
        int _targetSectorIndex1d;
        public void Execute()
        {
            //TARGET DATA
            int2 targetSectorIndex2d = new int2(Target.x / SectorColAmount, Target.y / SectorColAmount);
            _targetSectorIndex1d = targetSectorIndex2d.y * SectorMatrixColAmount + targetSectorIndex2d.x;

            bool isPathNew = PortalSequenceBorders.Length == 0;
            if(PortalSequenceBorders.Length == 0) { PortalSequenceBorders.Add(0); }
            RunDijkstra(isPathNew);
            NativeArray<int> sourcePortalsAsArray = SourcePortalIndexList.AsArray();
            for (int i = 0; i < sourcePortalsAsArray.Length; i++)
            {
                PickPortalSequence(sourcePortalsAsArray[i]);
            }
            PickSectorsFromPortalSequence();

            int newAddedSectorStart = NewPickedSectorStartIndex.Value;
            int newAddedSectorCount = PickedToSector.Length - newAddedSectorStart;
            NativeSlice<int> newAddedSectors = new NativeSlice<int>(PickedToSector.AsArray(), newAddedSectorStart, newAddedSectorCount);
            if (ContainsSectorsWithinLOSRange(newAddedSectors))
            {
                SectorsWihinLOSArgument argument = SectorWithinLOSState.Value;
                argument |= SectorsWihinLOSArgument.AddedSectorWithinLOS;
                SectorWithinLOSState.Value = argument;
            }
            AddTargetSector();
        }
        void RunDijkstra(bool isPathNew)
        {
            SingleFloatUnsafeHeap<int> travHeap = new SingleFloatUnsafeHeap<int>(0, Allocator.Temp);
            NativeArray<PortalTraversalData> portalTraversalDataArray = PortalTraversalDataArray;
            NativeArray<PortalNode> portalNodes = PortalNodes;
            NativeArray<PortalToPortal> porPtrs = PorPtrs;

            if (isPathNew)
            {
                for (int i = 0; i < TargetNeighbourPortalIndicies.Length; i++)
                {
                    int index = TargetNeighbourPortalIndicies[i];
                    PortalTraversalData targetNeighbour = PortalTraversalDataArray[index];
                    targetNeighbour.Mark |= PortalTraversalMark.DijkstraTraversed;
                    targetNeighbour.DistanceFromTarget++;
                    PortalTraversalDataArray[index] = targetNeighbour;
                }

                for (int i = 0; i < TargetNeighbourPortalIndicies.Length; i++)
                {
                    int index = TargetNeighbourPortalIndicies[i];
                    float distanceFromTarget = portalTraversalDataArray[index].DistanceFromTarget;
                    EnqueueNeighbours(index, distanceFromTarget);
                }
            }
            else
            {
                for (int i = 0; i < DijkstraStartIndicies.Length; i++)
                {
                    int index = DijkstraStartIndicies[i];
                    float distanceFromTarget = portalTraversalDataArray[index].DistanceFromTarget;
                    EnqueueNeighbours(index, distanceFromTarget);
                }
            }

            int curIndex = GetNextIndex();
            while (curIndex != -1)
            {
                float distanceFromTarget = portalTraversalDataArray[curIndex].DistanceFromTarget;
                EnqueueNeighbours(curIndex, distanceFromTarget);
                curIndex = GetNextIndex();
            }
            int GetNextIndex()
            {
                if (travHeap.IsEmpty) { return -1; }
                int nextMinIndex = travHeap.ExtractMin();
                PortalTraversalData nextMinTraversalData = portalTraversalDataArray[nextMinIndex];
                while (nextMinTraversalData.HasMark(PortalTraversalMark.DijstraExtracted))
                {
                    if (travHeap.IsEmpty) { return -1; }
                    nextMinIndex = travHeap.ExtractMin();
                    nextMinTraversalData = portalTraversalDataArray[nextMinIndex];
                }
                nextMinTraversalData.Mark |= PortalTraversalMark.DijstraExtracted;
                portalTraversalDataArray[nextMinIndex] = nextMinTraversalData;
                return nextMinIndex;
            }
            void EnqueueNeighbours(int index, float gCost)
            {
                PortalNode portal = portalNodes[index];
                int por1Ptr = portal.Portal1.PorToPorPtr;
                int por1Cnt = portal.Portal1.PorToPorCnt;
                int por2Ptr = portal.Portal2.PorToPorPtr;
                int por2Cnt = portal.Portal2.PorToPorCnt;

                for (int i = por1Ptr; i < por1Ptr + por1Cnt; i++)
                {
                    int portalIndex = porPtrs[i].Index;
                    float portalDistance = porPtrs[i].Distance;
                    PortalTraversalData porData = portalTraversalDataArray[portalIndex];
                    if (!porData.HasMark(PortalTraversalMark.Reduced) || porData.HasMark(PortalTraversalMark.DijstraExtracted)) { continue; }
                    if (porData.HasMark(PortalTraversalMark.DijkstraTraversed))
                    {
                        float newGCost = gCost + portalDistance + 1;
                        if (porData.DistanceFromTarget <= newGCost) { continue; }
                        porData.DistanceFromTarget = newGCost;
                        porData.NextIndex = index;
                        portalTraversalDataArray[portalIndex] = porData;
                        travHeap.Add(portalIndex, newGCost);
                        continue;
                    }
                    porData.Mark |= PortalTraversalMark.DijkstraTraversed;
                    porData.DistanceFromTarget = gCost + portalDistance + 1;
                    porData.NextIndex = index;
                    portalTraversalDataArray[portalIndex] = porData;
                    travHeap.Add(portalIndex, gCost + portalDistance + 1);
                }
                for (int i = por2Ptr; i < por2Ptr + por2Cnt; i++)
                {
                    int portalIndex = porPtrs[i].Index;
                    float portalDistance = porPtrs[i].Distance;
                    PortalTraversalData porData = portalTraversalDataArray[portalIndex];
                    if (!porData.HasMark(PortalTraversalMark.Reduced) || porData.HasMark(PortalTraversalMark.DijstraExtracted)) { continue; }
                    if (porData.HasMark(PortalTraversalMark.DijkstraTraversed))
                    {
                        float newGCost = gCost + portalDistance + 1;
                        if (porData.DistanceFromTarget <= newGCost) { continue; }
                        porData.DistanceFromTarget = newGCost;
                        porData.NextIndex = index;
                        portalTraversalDataArray[portalIndex] = porData;
                        travHeap.Add(portalIndex, newGCost);
                        continue;
                    }
                    porData.Mark |= PortalTraversalMark.DijkstraTraversed;
                    porData.DistanceFromTarget = gCost + portalDistance + 1;
                    porData.NextIndex = index;
                    portalTraversalDataArray[portalIndex] = porData;
                    travHeap.Add(portalIndex, gCost + portalDistance + 1);
                }
            }
        }
        bool ContainsSectorsWithinLOSRange(NativeSlice<int> sectors)
        {
            int losRange = LOSRange;
            int sectorColAmount = SectorColAmount;
            int sectorMatrixColAmount = SectorMatrixColAmount;
            int sectorMatrixRowAmount = SectorMatrixRowAmount;
            int sectorTileAmount = SectorTileAmount;

            int2 targetSector2d = FlowFieldUtilities.GetSector2D(Target, sectorColAmount);
            int extensionLength = losRange / sectorColAmount + math.select(0, 1, losRange % sectorColAmount > 0);
            int2 rangeTopRightSector = targetSector2d + new int2(extensionLength, extensionLength);
            int2 rangeBotLeftSector = targetSector2d - new int2(extensionLength, extensionLength);
            rangeTopRightSector = new int2()
            {
                x = math.select(rangeTopRightSector.x, sectorMatrixColAmount - 1, rangeTopRightSector.x >= sectorMatrixColAmount),
                y = math.select(rangeTopRightSector.y, sectorMatrixRowAmount - 1, rangeTopRightSector.y >= sectorMatrixRowAmount)
            };
            rangeBotLeftSector = new int2()
            {
                x = math.select(rangeBotLeftSector.x, 0, rangeBotLeftSector.x < 0),
                y = math.select(rangeBotLeftSector.y, 0, rangeBotLeftSector.y < 0)
            };
            for (int i = 0; i < sectors.Length; i++)
            {
                int sector1d = sectors[i];
                int sectorCol = sector1d % sectorMatrixColAmount;
                int sectorRow = sector1d / sectorMatrixColAmount;

                bool withinColRange = sectorCol >= rangeBotLeftSector.x && sectorCol <= rangeTopRightSector.x;
                bool withinRowRange = sectorRow >= rangeBotLeftSector.y && sectorRow <= rangeTopRightSector.y;
                if (withinColRange && withinRowRange) { return true; }
            }
            return false;
        }
        void AddTargetSector()
        {
            int sectorTileAmount = SectorColAmount * SectorColAmount;
            if ((SectorStateTable[_targetSectorIndex1d] & PathSectorState.Included) != PathSectorState.Included)
            {
                PickedToSector.Add(_targetSectorIndex1d);
                SectorStateTable[_targetSectorIndex1d] |= PathSectorState.Included;
            }
            FlowFieldLength.Value = PickedToSector.Length * sectorTileAmount + 1;
        }
        void PickSectorsFromPortalSequence()
        {
            AddedPortalSequenceBorderStartIndex = math.select(AddedPortalSequenceBorderStartIndex - 1, 0, AddedPortalSequenceBorderStartIndex == 0);
            for (int i = AddedPortalSequenceBorderStartIndex; i < PortalSequenceBorders.Length - 1; i++)
            {
                int start = PortalSequenceBorders[i];
                int end = PortalSequenceBorders[i + 1];
                for (int j = start; j < end - 1; j++)
                {
                    PickSectorsBetweenportals(PortalSequence[j], PortalSequence[j + 1]);
                }
            }
        }
        void PickSectorsBetweenportals(ActivePortal portal1, ActivePortal portal2)
        {
            int sectorTileAmount = SectorColAmount * SectorColAmount;
            int win1Sec1Index = FlowFieldUtilities.GetSector1D(portal1.FieldIndex1, FieldColAmount, SectorColAmount, SectorMatrixColAmount);
            int win1Sec2Index = FlowFieldUtilities.GetSector1D(portal1.FieldIndex2, FieldColAmount, SectorColAmount, SectorMatrixColAmount);
            int win2Sec1Index = FlowFieldUtilities.GetSector1D(portal2.FieldIndex1, FieldColAmount, SectorColAmount, SectorMatrixColAmount);
            int win2Sec2Index = FlowFieldUtilities.GetSector1D(portal2.FieldIndex2, FieldColAmount, SectorColAmount, SectorMatrixColAmount);
            bool sector1Included = (SectorStateTable[win1Sec1Index] & PathSectorState.Included) == PathSectorState.Included;
            bool sector2Included = (SectorStateTable[win1Sec2Index] & PathSectorState.Included) == PathSectorState.Included;
            if ((win1Sec1Index == win2Sec1Index || win1Sec1Index == win2Sec2Index) && !sector1Included)
            {
                PickedToSector.Add(win1Sec1Index);
                SectorStateTable[win1Sec1Index] |= PathSectorState.Included;
            }
            if ((win1Sec2Index == win2Sec1Index || win1Sec2Index == win2Sec2Index) && !sector2Included)
            {
                PickedToSector.Add(win1Sec2Index);
                SectorStateTable[win1Sec2Index] |= PathSectorState.Included;

            }
        }
        void PickPortalSequence(int sourcePortalIndex)
        {
            int curPortalIndex = sourcePortalIndex;
            while(curPortalIndex != -1)
            {
                PortalTraversalData curPortalData = PortalTraversalDataArray[curPortalIndex];
                bool curPortalDijkstraPicked = curPortalData.HasMark(PortalTraversalMark.DijkstraPicked);
                bool curPortalTargetNeighbour = curPortalData.HasMark(PortalTraversalMark.TargetNeighbour);

                PortalNode curPortalNode = PortalNodes[curPortalIndex];
                ActivePortal curActivePortal = new ActivePortal()
                {
                    FieldIndex1 = FlowFieldUtilities.To1D(curPortalNode.Portal1.Index, FieldColAmount),
                    FieldIndex2 = FlowFieldUtilities.To1D(curPortalNode.Portal2.Index, FieldColAmount),
                    Distance = curPortalData.DistanceFromTarget,
                };
                PortalSequence.Add(curActivePortal);

                curPortalData.Mark |= PortalTraversalMark.DijkstraPicked;
                PortalTraversalDataArray[curPortalIndex] = curPortalData;
                if (curPortalDijkstraPicked)
                {
                    curPortalIndex = -1;
                }
                else if (curPortalTargetNeighbour)
                {
                    ActivePortal targetActivePortal = new ActivePortal()
                    {
                        FieldIndex1 = FlowFieldUtilities.To1D(Target, FieldColAmount),
                        FieldIndex2 = FlowFieldUtilities.To1D(Target, FieldColAmount),
                        Distance = 0,
                    };
                    PortalSequence.Add(targetActivePortal);
                    curPortalIndex = -1;
                }
                else
                {
                    curPortalIndex = curPortalData.NextIndex;
                }
            }
            PortalSequenceBorders.Add(PortalSequence.Length);
        }
    }
}
