using System;
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
        internal int NewPortalSliceStartIndex;
        internal int SectorMatrixColAmount;
        internal int SectorMatrixRowAmount;
        internal int SectorTileAmount;
        internal int LOSRange;
        internal NativeArray<PortalTraversalData> PortalTraversalDataArray;
        internal NativeList<ActivePortal> PortalSequence;
        internal NativeList<Slice> PortalSequenceSlices;
        internal UnsafeList<PathSectorState> SectorStateTable;
        internal NativeList<int> PickedSectorIndicies;
        internal NativeList<PortalTraversalDataRecord> PortalDataRecords;
        internal NativeReference<SectorsWihinLOSArgument> SectorWithinLOSState;
        internal NativeList<IntegrationTile> IntegrationField;

        [ReadOnly] internal NativeArray<SectorNode> SectorNodes;
        [ReadOnly] internal NativeArray<WindowNode> WindowNodes;
        [ReadOnly] internal NativeArray<int> WinToSecPtrs;
        [ReadOnly] internal NativeArray<PortalNode> PortalNodes;
        [ReadOnly] internal NativeArray<PortalToPortal> PorPtrs;
        [ReadOnly] internal NativeReference<int> NewPickedSectorStartIndex;

        [ReadOnly] internal NativeList<int> SourcePortalIndexList;
        [ReadOnly] internal NativeList<int> DijkstraStartIndicies;
        [ReadOnly] internal NativeList<int> NewReducedPortalIndicies;
        int _targetSectorIndex1d;
        public void Execute()
        {
            //TARGET DATA
            int2 targetSectorIndex2d = new int2(Target.x / SectorColAmount, Target.y / SectorColAmount);
            _targetSectorIndex1d = targetSectorIndex2d.y * SectorMatrixColAmount + targetSectorIndex2d.x;

            RunDijkstra();
            NativeArray<int> sourcePortalsAsArray = SourcePortalIndexList.AsArray();
            for (int i = 0; i < sourcePortalsAsArray.Length; i++)
            {
                PickPortalSequence(sourcePortalsAsArray[i]);
            }
            PickSectorsFromPortalSequence();

            int newAddedSectorStart = NewPickedSectorStartIndex.Value;
            int newAddedSectorCount = PickedSectorIndicies.Length - newAddedSectorStart;
            NativeSlice<int> newAddedSectors = new NativeSlice<int>(PickedSectorIndicies.AsArray(), newAddedSectorStart, newAddedSectorCount);
            if (ContainsSectorsWithinLOSRange(newAddedSectors))
            {
                SectorsWihinLOSArgument argument = SectorWithinLOSState.Value;
                argument |= SectorsWihinLOSArgument.AddedSectorWithinLOS;
                SectorWithinLOSState.Value = argument;
            }
            AddTargetSector();
            SetNewIntegrationFieldLength();
            SubmitPortalDataRecords();
            ResetPortalDataArray();
        }
        void SubmitPortalDataRecords()
        {
            for(int i = 0; i < NewReducedPortalIndicies.Length; i++)
            {
                int portalIndex = NewReducedPortalIndicies[i];
                PortalTraversalData portalData = PortalTraversalDataArray[portalIndex];
                PortalTraversalDataRecord record = new PortalTraversalDataRecord()
                {
                    PortalIndex = portalIndex,
                    DistanceFromTarget = portalData.DistanceFromTarget,
                    Mark = portalData.Mark,
                    NextIndex = portalData.NextIndex,
                };
                PortalDataRecords.Add(record);
            }
        }
        void ResetPortalDataArray()
        {
            for(int i = 0; i < PortalDataRecords.Length; i++)
            {
                int portalIndex = PortalDataRecords[i].PortalIndex;
                PortalTraversalData portalData = PortalTraversalDataArray[portalIndex];
                portalData.Reset();
                PortalTraversalDataArray[portalIndex] = portalData;
            }
        }
        void RunDijkstra()
        {
            SingleFloatUnsafeHeap<int> travHeap = new SingleFloatUnsafeHeap<int>(0, Allocator.Temp);
            NativeArray<PortalTraversalData> portalTraversalDataArray = PortalTraversalDataArray;
            NativeArray<PortalNode> portalNodes = PortalNodes;
            NativeArray<PortalToPortal> porPtrs = PorPtrs;


            //Mark starting portals
            for (int i = 0; i < DijkstraStartIndicies.Length; i++)
            {
                int index = DijkstraStartIndicies[i];
                PortalTraversalData portalData = PortalTraversalDataArray[index];
                portalData.Mark |= PortalTraversalMark.DijkstraTraversed;
                portalTraversalDataArray[index] = portalData;
            }

            //Enqueue neighbours of starting portals
            for (int i = 0; i < DijkstraStartIndicies.Length; i++)
            {
                int index = DijkstraStartIndicies[i];
                EnqueueNeighbours(index, PortalTraversalDataArray[index].DistanceFromTarget);
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
                PickedSectorIndicies.Add(_targetSectorIndex1d);
                SectorStateTable[_targetSectorIndex1d] |= PathSectorState.Included;
            }
            
        }
        void SetNewIntegrationFieldLength()
        {
            int oldIntegrationFieldLength = IntegrationField.Length;
            int newIntegrationFieldLength = PickedSectorIndicies.Length * SectorTileAmount + 1;
            IntegrationField.Length = newIntegrationFieldLength;
            for(int i = oldIntegrationFieldLength; i < newIntegrationFieldLength; i++)
            {
                IntegrationTile tile = IntegrationField[i];
                tile.Reset();
                IntegrationField[i] = tile;
            }
        }
        void PickSectorsFromPortalSequence()
        {
            for (int i = NewPortalSliceStartIndex; i < PortalSequenceSlices.Length; i++)
            {
                Slice slice = PortalSequenceSlices[i];
                int start = slice.Index;
                int end = start + slice.Count;
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
                PickedSectorIndicies.Add(win1Sec1Index);
                SectorStateTable[win1Sec1Index] |= PathSectorState.Included;
            }
            if ((win1Sec2Index == win2Sec1Index || win1Sec2Index == win2Sec2Index) && !sector2Included)
            {
                PickedSectorIndicies.Add(win1Sec2Index);
                SectorStateTable[win1Sec2Index] |= PathSectorState.Included;

            }
        }
        void PickPortalSequence(int sourcePortalIndex)
        {
            int sliceStart = PortalSequence.Length;

            int curPortalIndex = sourcePortalIndex;
            while(curPortalIndex != -1)
            {
                PortalTraversalData curPortalData = PortalTraversalDataArray[curPortalIndex];
                bool curPortalDijkstraPicked = curPortalData.HasMark(PortalTraversalMark.DijkstraPicked);
                bool curPortalTargetNeighbour = curPortalData.HasMark(PortalTraversalMark.GoalNeighbour);

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

            int sliceEnd = PortalSequence.Length;
            int sliceLength = sliceEnd - sliceStart;
            PortalSequenceSlices.Add(new Slice(sliceStart, sliceLength));
        }
    }
}
