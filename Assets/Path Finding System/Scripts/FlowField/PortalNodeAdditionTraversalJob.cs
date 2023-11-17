﻿using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;


[BurstCompile]
public struct PortalNodeAdditionTraversalJob : IJob
{
    public int2 TargetIndex;
    public int FieldColAmount;
    public int FieldRowAmount;
    public float FieldTileSize;
    public int SectorColAmount;
    public int SectorMatrixColAmount;
    public int AddedPortalSequenceBorderStartIndex;

    public NativeArray<PortalTraversalData> PortalTraversalDataArray;
    public NativeList<ActivePortal> PortalSequence;
    public NativeList<int> PortalSequenceBorders;
    public UnsafeList<int> SectorToPicked;
    public UnsafeList<PathSectorState> SectorStateTable;
    public NativeList<int> PickedToSector;
    public NativeArray<DijkstraTile> TargetSectorCosts;
    public NativeArray<int> FlowFieldLength;

    [ReadOnly] public NativeSlice<float2> SourcePositions;
    [ReadOnly] public UnsafeList<SectorNode> SectorNodes;
    [ReadOnly] public NativeArray<int> SecToWinPtrs;
    [ReadOnly] public NativeArray<WindowNode> WindowNodes;
    [ReadOnly] public NativeArray<int> WinToSecPtrs;
    [ReadOnly] public UnsafeList<PortalNode> PortalNodes;
    [ReadOnly] public NativeArray<PortalToPortal> PorPtrs;
    [ReadOnly] public UnsafeList<UnsafeList<int>> IslandFields;

    public NativeList<int> TargetNeighbourPortalIndicies;
    public NativeList<int> AStarTraverseIndexList;
    public NativeList<int> SourcePortalIndexList;

    public NativeList<int> DijkstraStartIndicies;
    int _targetSectorIndex1d;
    public void Execute()
    {
        //TARGET DATA
        int2 targetSectorIndex2d = new int2(TargetIndex.x / SectorColAmount, TargetIndex.y / SectorColAmount);
        _targetSectorIndex1d = targetSectorIndex2d.y * SectorMatrixColAmount + targetSectorIndex2d.x;

        //START GRAPH WALKER
        DoubleUnsafeHeap<int> walkerHeap = new DoubleUnsafeHeap<int>(10, Allocator.Temp);

        UnsafeList<int> allSorucePortalIndicies = GetSourcePortalIndicies();
        for (int i = 0; i < allSorucePortalIndicies.Length; i++)
        {
            int stoppedIndex = RunReductionAStar(allSorucePortalIndicies[i], walkerHeap);
            if (stoppedIndex == -1) { continue; }
            PickAStarNodes(allSorucePortalIndicies[i], stoppedIndex);
            ResetTraversedIndicies();
            walkerHeap.Clear();
        }
        RunDijkstra();
        for (int i = 0; i < allSorucePortalIndicies.Length; i++)
        {
            PickPortalSequenceFromFastMarching(allSorucePortalIndicies[i]);
        }
        PickSectorsFromPortalSequence();
    }
    void PickPortalSequenceFromFastMarching(int sourcePortal)
    {
        //NOTE: NextIndex of portalTraversalData is used as:
        //1. NextIndex in portalTraversalDataArray
        //2. PortalSequence of corresponding portalTraversalData
        //For memory optimization reasons :/

        PortalTraversalData sourceData = PortalTraversalDataArray[sourcePortal];

        if (sourceData.HasMark(PortalTraversalMark.DijkstraPicked)) { return; }

        int nextDataIndex = sourceData.NextIndex;
        sourceData.Mark |= PortalTraversalMark.DijkstraPicked;
        sourceData.NextIndex = PortalSequence.Length;
        PortalTraversalDataArray[sourcePortal] = sourceData;

        ActivePortal sourceActivePortal = new ActivePortal()
        {
            Index = sourcePortal,
            Distance = sourceData.DistanceFromTarget,
            NextIndex = PortalSequence.Length + 1,
        };

        //IF SOURCE IS TARGET NEIGHBOUR
        if (nextDataIndex == -1)
        {
            sourceActivePortal.NextIndex = -1;
            PortalSequence.Add(sourceActivePortal);
            PortalSequenceBorders.Add(PortalSequence.Length);
            return;
        }

        //IF SOURCE IS NOT TARGET NEIGHBOUR
        PortalSequence.Add(sourceActivePortal);
        int curIndex = nextDataIndex;

        while (curIndex != -1)
        {
            PortalTraversalData curData = PortalTraversalDataArray[curIndex];
            if (curData.HasMark(PortalTraversalMark.DijkstraPicked))
            {
                ActivePortal previousNode = PortalSequence[PortalSequence.Length - 1];
                previousNode.NextIndex = curData.NextIndex;
                PortalSequence[PortalSequence.Length - 1] = previousNode;
                break;
            }
            //PUSH ACTIVE PORTAL
            ActivePortal curActivePortal = new ActivePortal()
            {
                Index = curIndex,
                Distance = curData.DistanceFromTarget,
                NextIndex = math.select(PortalSequence.Length + 1, -1, curData.NextIndex == -1),
            };
            PortalSequence.Add(curActivePortal);

            //MARK OR STOP
            int curDataIndex = curData.NextIndex;
            curData.Mark |= PortalTraversalMark.DijkstraPicked;
            curData.NextIndex = PortalSequence.Length - 1;
            PortalTraversalDataArray[curIndex] = curData;

            curIndex = curDataIndex;
        }
        PortalSequenceBorders.Add(PortalSequence.Length);
    }
    void ResetTraversedIndicies()
    {
        PortalTraversalMark bitsToSet = ~(PortalTraversalMark.AStarTraversed | PortalTraversalMark.AStarExtracted);
        for (int i = 0; i < AStarTraverseIndexList.Length; i++)
        {
            int index = AStarTraverseIndexList[i];
            PortalTraversalData travData = PortalTraversalDataArray[index];
            travData.Mark &= bitsToSet;
            PortalTraversalDataArray[index] = travData;
        }
        AStarTraverseIndexList.Clear();
    }
    void PickAStarNodes(int sourceNodeIndex, int stoppedIndex)
    {
        int originIndex = stoppedIndex;

        //FIRST STEP
        PortalTraversalData stoppedPortalData = PortalTraversalDataArray[stoppedIndex];
        stoppedPortalData.Mark |= PortalTraversalMark.AStarPicked;
        PortalTraversalDataArray[originIndex] = stoppedPortalData;

        originIndex = stoppedPortalData.OriginIndex;

        //REMAINING STEPS
        while (originIndex != sourceNodeIndex)
        {
            PortalTraversalData nextPortalData = PortalTraversalDataArray[originIndex];
            nextPortalData.Mark |= PortalTraversalMark.AStarPicked;
            PortalTraversalDataArray[originIndex] = nextPortalData;
            originIndex = nextPortalData.OriginIndex;
        }

        //LAST STEP
        PortalTraversalData sourcePortalData = PortalTraversalDataArray[originIndex];
        sourcePortalData.Mark |= PortalTraversalMark.AStarPicked;
        PortalTraversalDataArray[originIndex] = sourcePortalData;
    }
    UnsafeList<int> GetSourcePortalIndicies()
    {
        int targetIsland = GetIsland(TargetIndex);
        UnsafeList<int> indicies = new UnsafeList<int>(0, Allocator.Temp);
        int sectorTileAmount = SectorColAmount * SectorColAmount;
        for (int i = 0; i < SourcePositions.Length; i++)
        {
            float2 sourcePos = SourcePositions[i];
            int2 sourceIndex = new int2((int)math.floor(sourcePos.x / FieldTileSize), (int)math.floor(sourcePos.y / FieldTileSize));
            int2 sourceSectorIndex = sourceIndex / SectorColAmount;
            int sourceSectorIndexFlat = sourceSectorIndex.y * SectorMatrixColAmount + sourceSectorIndex.x;

            //ADD SOURCE SECTOR TO THE PICKED SECTORS
            if (SectorToPicked[sourceSectorIndexFlat] != 0) { continue; }
            SectorToPicked[sourceSectorIndexFlat] = PickedToSector.Length * sectorTileAmount + 1;
            PickedToSector.Add(sourceSectorIndexFlat);
            SectorStateTable[sourceSectorIndexFlat] |= PathSectorState.Included | PathSectorState.Source;
            //ADD SOURCE SECTOR TO THE PICKED SECTORS
            SetSectorPortalIndicies(sourceSectorIndexFlat, SourcePortalIndexList);

            for (int j = 0; j < SourcePortalIndexList.Length; j++)
            {
                int index = SourcePortalIndexList[j];
                if (PortalNodes[index].IslandIndex != targetIsland) { continue; }
                indicies.Add(index);
            }
        }
        return indicies;
    }
    void RunDijkstra()
    {
        SingleUnsafeHeap<int> travHeap = new SingleUnsafeHeap<int>(0, Allocator.Temp);
        NativeArray<PortalTraversalData> portalTraversalDataArray = PortalTraversalDataArray;
        UnsafeList<PortalNode> portalNodes = PortalNodes;
        NativeArray<PortalToPortal> porPtrs = PorPtrs;

        //MARK TARGET NEIGHBOURS
        for (int i = 0; i < DijkstraStartIndicies.Length; i++)
        {
            int index = DijkstraStartIndicies[i];
            //PortalTraversalData startData = PortalTraversalDataArray[index];
            //startData.Mark |= PortalTraversalMark.DijkstraTraversed;
            //PortalTraversalDataArray[index] = startData;
        }

        for (int i = 0; i < DijkstraStartIndicies.Length; i++)
        {
            int index = DijkstraStartIndicies[i];
            float distanceFromTarget = portalTraversalDataArray[index].DistanceFromTarget;
            EnqueueNeighbours(index, distanceFromTarget);
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
    int RunReductionAStar(int sourcePortalIndex, DoubleUnsafeHeap<int> traversalHeap)
    {
        UnsafeList<PortalNode> portalNodes = PortalNodes;
        NativeArray<PortalTraversalData> portalTraversalDataArray = PortalTraversalDataArray;
        PortalTraversalData curData = PortalTraversalDataArray[sourcePortalIndex];
        if (curData.HasMark(PortalTraversalMark.AStarPicked) || curData.HasMark(PortalTraversalMark.DijkstraTraversed))
        {
            return -1;
        }

        //SET INNITIAL MARK
        curData = new PortalTraversalData()
        {
            FCost = 0,
            GCost = 0,
            HCost = float.MaxValue,
            Mark = curData.Mark | PortalTraversalMark.AStarPicked | PortalTraversalMark.AStarTraversed | PortalTraversalMark.Reduced,
            OriginIndex = sourcePortalIndex,
            NextIndex = -1,
            DistanceFromTarget = curData.DistanceFromTarget,
        };
        PortalTraversalDataArray[sourcePortalIndex] = curData;

        //NODE DATA
        int curNodeIndex = sourcePortalIndex;
        PortalNode curNode = PortalNodes[curNodeIndex];
        int por1P2pIdx = curNode.Portal1.PorToPorPtr;
        int por2P2pIdx = curNode.Portal2.PorToPorPtr;
        int por1P2pCnt = curNode.Portal1.PorToPorCnt;
        int por2P2pCnt = curNode.Portal2.PorToPorCnt;

        //HANDLE NEIGHBOURS
        AStarTraverseIndexList.Add(curNodeIndex);
        bool isCurDijkstraTraversed = curData.HasMark(PortalTraversalMark.DijkstraTraversed);
        TraverseNeighbours(curData, ref traversalHeap, curNodeIndex, por1P2pIdx, por1P2pIdx + por1P2pCnt, isCurDijkstraTraversed);
        TraverseNeighbours(curData, ref traversalHeap, curNodeIndex, por2P2pIdx, por2P2pIdx + por2P2pCnt, isCurDijkstraTraversed);
        SetNextNode();

        while (!curData.HasMark(PortalTraversalMark.AStarPicked))
        {
            isCurDijkstraTraversed = curData.HasMark(PortalTraversalMark.DijkstraTraversed);
            TraverseNeighbours(curData, ref traversalHeap, curNodeIndex, por1P2pIdx, por1P2pIdx + por1P2pCnt, isCurDijkstraTraversed);
            TraverseNeighbours(curData, ref traversalHeap, curNodeIndex, por2P2pIdx, por2P2pIdx + por2P2pCnt, isCurDijkstraTraversed);
            SetNextNode();
        }
        return curNodeIndex;
        void SetNextNode()
        {
            if (traversalHeap.IsEmpty) { return; }
            int nextMinIndex = traversalHeap.ExtractMin();
            PortalTraversalData nextMinTraversalData = portalTraversalDataArray[nextMinIndex];
            while (nextMinTraversalData.HasMark(PortalTraversalMark.AStarExtracted))
            {
                nextMinIndex = traversalHeap.ExtractMin();
                nextMinTraversalData = portalTraversalDataArray[nextMinIndex];
            }
            nextMinTraversalData.Mark |= PortalTraversalMark.AStarExtracted;
            curData = nextMinTraversalData;
            portalTraversalDataArray[nextMinIndex] = curData;
            curNodeIndex = nextMinIndex;
            curNode = portalNodes[curNodeIndex];
            por1P2pIdx = curNode.Portal1.PorToPorPtr;
            por2P2pIdx = curNode.Portal2.PorToPorPtr;
            por1P2pCnt = curNode.Portal1.PorToPorCnt;
            por2P2pCnt = curNode.Portal2.PorToPorCnt;
        }
    }
    void TraverseNeighbours(PortalTraversalData curData, ref DoubleUnsafeHeap<int> traversalHeap, int curNodeIndex, int from, int to, bool curDijkstraTraversed)
    {
        for (int i = from; i < to; i++)
        {
            PortalToPortal neighbourConnection = PorPtrs[i];
            PortalNode portalNode = PortalNodes[neighbourConnection.Index];
            PortalTraversalData traversalData = PortalTraversalDataArray[neighbourConnection.Index];
            if (traversalData.HasMark(PortalTraversalMark.AStarTraversed))
            {
                float newGCost = curData.GCost + neighbourConnection.Distance;
                if (newGCost < traversalData.GCost)
                {
                    float newFCost = traversalData.HCost + newGCost;
                    traversalData.GCost = newGCost;
                    traversalData.FCost = newFCost;
                    traversalData.OriginIndex = curNodeIndex;
                    PortalTraversalDataArray[neighbourConnection.Index] = traversalData;
                    traversalHeap.Add(neighbourConnection.Index, traversalData.FCost, traversalData.HCost);
                }
            }
            else
            {
                float hCost = GetHCost(portalNode.Portal1.Index);
                float gCost = curData.GCost + neighbourConnection.Distance;
                float fCost = hCost * gCost;
                traversalData.HCost = hCost;
                traversalData.GCost = gCost;
                traversalData.FCost = fCost;
                traversalData.Mark |= PortalTraversalMark.AStarTraversed | PortalTraversalMark.Reduced;
                traversalData.OriginIndex = curNodeIndex;
                PortalTraversalDataArray[neighbourConnection.Index] = traversalData;
                traversalHeap.Add(neighbourConnection.Index, traversalData.FCost, traversalData.HCost);
                AStarTraverseIndexList.Add(neighbourConnection.Index);

                bool neighbourDijkstraTraversed = traversalData.HasMark(PortalTraversalMark.DijkstraTraversed);
                if (!curDijkstraTraversed && neighbourDijkstraTraversed)
                {
                    DijkstraStartIndicies.Add(neighbourConnection.Index);
                }
            }
        }
        if (curData.HasMark(PortalTraversalMark.TargetNeighbour))
        {
            int targetNodeIndex = PortalNodes.Length - 1;
            PortalTraversalData traversalData = PortalTraversalDataArray[targetNodeIndex];
            if (traversalData.HasMark(PortalTraversalMark.AStarTraversed))
            {
                float newGCost = curData.GCost + GetGCostBetweenTargetAndTargetNeighbour(curNodeIndex);
                if (newGCost < traversalData.GCost)
                {
                    float newFCost = traversalData.HCost + newGCost;
                    traversalData.GCost = newGCost;
                    traversalData.FCost = newFCost;
                    traversalData.OriginIndex = curNodeIndex;
                    PortalTraversalDataArray[targetNodeIndex] = traversalData;
                    traversalHeap.Add(targetNodeIndex, traversalData.FCost, traversalData.HCost);
                }
            }
            else
            {
                float hCost = 0f;
                float gCost = curData.GCost + GetGCostBetweenTargetAndTargetNeighbour(curNodeIndex);
                float fCost = hCost + gCost;
                traversalData.HCost = hCost;
                traversalData.GCost = gCost;
                traversalData.FCost = fCost;
                traversalData.Mark |= PortalTraversalMark.AStarTraversed;
                traversalData.OriginIndex = curNodeIndex;
                PortalTraversalDataArray[targetNodeIndex] = traversalData;
                traversalHeap.Add(targetNodeIndex, traversalData.FCost, traversalData.HCost);
                AStarTraverseIndexList.Add(targetNodeIndex);
            }
        }
    }
    float GetHCost(Index2 nodePos)
    {
        int2 newNodePos = new int2(nodePos.C, nodePos.R);
        int2 targetPos = TargetIndex;

        int xDif = math.abs(newNodePos.x - targetPos.x);
        int yDif = math.abs(newNodePos.y - targetPos.y);
        int smallOne = math.min(xDif, yDif);
        int bigOne = math.max(xDif, yDif);
        return (bigOne - smallOne) * 1f + smallOne * 1.4f;
    }
    float GetGCostBetweenTargetAndTargetNeighbour(int targetNeighbourIndex)
    {
        int portalLocalIndexAtSector = FlowFieldUtilities.GetLocal1dInSector(PortalNodes[targetNeighbourIndex], _targetSectorIndex1d, SectorMatrixColAmount, SectorColAmount);
        return TargetSectorCosts[portalLocalIndexAtSector].IntegratedCost;
    }
    void SetSectorPortalIndicies(int targetSectorIndexF, NativeList<int> destinationList)
    {
        SectorNode sectorNode = SectorNodes[targetSectorIndexF];
        int winPtr = sectorNode.SecToWinPtr;
        int winCnt = sectorNode.SecToWinCnt;
        for (int i = 0; i < winCnt; i++)
        {
            WindowNode windowNode = WindowNodes[SecToWinPtrs[winPtr + i]];
            int porPtr = windowNode.PorPtr;
            int porCnt = windowNode.PorCnt;
            for (int j = 0; j < porCnt; j++)
            {
                destinationList.Add(j + porPtr);
            }
        }
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
                int portalIndex1 = PortalSequence[j].Index;
                int portalIndex2 = PortalSequence[j + 1].Index;
                PickSectorsBetweenportals(portalIndex1, portalIndex2);
            }
            ActivePortal lastActivePortalInBorder = PortalSequence[end - 1];
            if (lastActivePortalInBorder.NextIndex != -1)
            {
                int portalIndex1 = lastActivePortalInBorder.Index;
                int portalIndex2 = PortalSequence[lastActivePortalInBorder.NextIndex].Index;
                PickSectorsBetweenportals(portalIndex1, portalIndex2);
            }
        }

        int sectorTileAmount = SectorColAmount * SectorColAmount;
        FlowFieldLength[0] = PickedToSector.Length * sectorTileAmount + 1;
    }
    void PickSectorsBetweenportals(int portalIndex1, int portalIndex2)
    {
        int sectorTileAmount = SectorColAmount * SectorColAmount;
        int windowIndex1 = PortalNodes[portalIndex1].WinPtr;
        int windowIndex2 = PortalNodes[portalIndex2].WinPtr;
        WindowNode winNode1 = WindowNodes[windowIndex1];
        WindowNode winNode2 = WindowNodes[windowIndex2];
        int win1Sec1Index = WinToSecPtrs[winNode1.WinToSecPtr];
        int win1Sec2Index = WinToSecPtrs[winNode1.WinToSecPtr + 1];
        int win2Sec1Index = WinToSecPtrs[winNode2.WinToSecPtr];
        int win2Sec2Index = WinToSecPtrs[winNode2.WinToSecPtr + 1];
        if ((win1Sec1Index == win2Sec1Index || win1Sec1Index == win2Sec2Index) && SectorToPicked[win1Sec1Index] == 0)
        {
            SectorToPicked[win1Sec1Index] = PickedToSector.Length * sectorTileAmount + 1;
            PickedToSector.Add(win1Sec1Index);
            SectorStateTable[win1Sec1Index] |= PathSectorState.Included;
        }
        if ((win1Sec2Index == win2Sec1Index || win1Sec2Index == win2Sec2Index) && SectorToPicked[win1Sec2Index] == 0)
        {
            SectorToPicked[win1Sec2Index] = PickedToSector.Length * sectorTileAmount + 1;
            PickedToSector.Add(win1Sec2Index);
            SectorStateTable[win1Sec2Index] |= PathSectorState.Included;

        }
    }
    public int GetIsland(int2 general2d)
    {
        int2 sector2d = FlowFieldUtilities.GetSector2D(general2d, SectorColAmount);
        int sector1d = FlowFieldUtilities.To1D(sector2d, SectorMatrixColAmount);
        SectorNode sector = SectorNodes[sector1d];

        if (sector.IsIslandValid())
        {
            return PortalNodes[sector.SectorIslandPortalIndex].IslandIndex;
        }
        else if (sector.IsIslandField)
        {
            int2 sectorStart = FlowFieldUtilities.GetSectorStartIndex(sector2d, SectorColAmount);
            int2 local2d = FlowFieldUtilities.GetLocal2D(general2d, sectorStart);
            int local1d = FlowFieldUtilities.To1D(local2d, SectorColAmount);
            int island = IslandFields[sector1d][local1d];
            switch (island)
            {
                case < 0:
                    return -island;
                case int.MaxValue:
                    return int.MaxValue;
                default:
                    return PortalNodes[island].IslandIndex;
            }
        }
        return int.MaxValue;
    }
    public struct SingleUnsafeHeap<T> where T : unmanaged
    {
        public UnsafeList<HeapElement<T>> _array;
        public T this[int index]
        {
            get
            {
                return _array[index].data;
            }
        }
        public bool IsEmpty
        {
            get
            {
                return _array.IsEmpty;
            }
        }
        public SingleUnsafeHeap(int size, Allocator allocator)
        {
            _array = new UnsafeList<HeapElement<T>>(size, allocator);
        }
        public void Clear()
        {
            _array.Clear();
        }
        public void Add(T element, float pri)
        {
            int elementIndex = _array.Length;
            _array.Add(new HeapElement<T>(element, pri));
            if (elementIndex != 0)
            {
                HeapifyUp(elementIndex);
            }
        }
        public T GetMin() => _array[0].data;
        public T ExtractMin()
        {
            T min = _array[0].data;
            HeapElement<T> last = _array[_array.Length - 1];
            _array[0] = last;
            _array.Length--;
            if (_array.Length > 1)
            {
                HeapifyDown(0);
            }
            return min;
        }
        public void SetPriority(int index, float pri)
        {
            int length = _array.Length;
            HeapElement<T> cur = _array[index];
            cur.pri = pri;
            _array[index] = cur;
            int parIndex = index / 2 - 1;
            int lcIndex = index * 2 + 1;
            int rcIndex = index * 2 + 2;
            parIndex = math.select(index, parIndex, parIndex >= 0);
            lcIndex = math.select(index, lcIndex, lcIndex < length);
            rcIndex = math.select(index, rcIndex, rcIndex < length);
            HeapElement<T> parent = _array[parIndex];
            if (cur.pri < parent.pri)
            {
                HeapifyUp(index);
            }
            else
            {
                HeapifyDown(index);
            }
        }
        public void Dispose()
        {
            _array.Dispose();
        }

        void HeapifyUp(int startIndex)
        {
            int curIndex = startIndex;
            int parIndex = (curIndex - 1) / 2;
            HeapElement<T> cur = _array[startIndex];
            HeapElement<T> par = _array[parIndex];
            bool isCurSmaller = cur.pri < par.pri;
            while (isCurSmaller)
            {
                _array[parIndex] = cur;
                _array[curIndex] = par;
                curIndex = parIndex;
                parIndex = math.select((curIndex - 1) / 2, 0, curIndex == 0);
                par = _array[parIndex];
                isCurSmaller = cur.pri < par.pri;
            }
        }
        void HeapifyDown(int startIndex)
        {
            int length = _array.Length;
            int curIndex = startIndex;
            int lcIndex = startIndex * 2 + 1;
            int rcIndex = lcIndex + 1;
            lcIndex = math.select(curIndex, lcIndex, lcIndex < length);
            rcIndex = math.select(curIndex, rcIndex, rcIndex < length);
            HeapElement<T> cur;
            HeapElement<T> lc;
            HeapElement<T> rc;
            while (lcIndex != curIndex)
            {
                cur = _array[curIndex];
                lc = _array[lcIndex];
                rc = _array[rcIndex];
                bool lcSmallerThanRc = lc.pri < rc.pri;
                bool lcSmallerThanCur = lc.pri < cur.pri;
                bool rcSmallerThanCur = rc.pri < cur.pri;

                if (lcSmallerThanRc && lcSmallerThanCur)
                {
                    _array[curIndex] = lc;
                    _array[lcIndex] = cur;
                    curIndex = lcIndex;
                    lcIndex = curIndex * 2 + 1;
                    rcIndex = lcIndex + 1;
                    lcIndex = math.select(lcIndex, curIndex, lcIndex >= length);
                    rcIndex = math.select(rcIndex, curIndex, rcIndex >= length);
                }
                else if (!lcSmallerThanRc && rcSmallerThanCur)
                {
                    _array[curIndex] = rc;
                    _array[rcIndex] = cur;
                    curIndex = rcIndex;
                    lcIndex = curIndex * 2 + 1;
                    rcIndex = lcIndex + 1;
                    lcIndex = math.select(lcIndex, curIndex, lcIndex >= length);
                    rcIndex = math.select(rcIndex, curIndex, rcIndex >= length);
                }
                else
                {
                    break;
                }
            }
        }
        public struct HeapElement<T> where T : unmanaged
        {
            public T data;
            public float pri;

            public HeapElement(T data, float pri)
            {
                this.data = data;
                this.pri = pri;
            }
        }
    }
    public struct DoubleUnsafeHeap<T> where T : unmanaged
    {
        public UnsafeList<HeapElement<T>> _array;
        public T this[int index]
        {
            get
            {
                return _array[index].data;
            }
        }
        public bool IsEmpty
        {
            get
            {
                return _array.IsEmpty;
            }
        }
        public DoubleUnsafeHeap(int size, Allocator allocator)
        {
            _array = new UnsafeList<HeapElement<T>>(size, allocator);
        }
        public void Clear()
        {
            _array.Clear();
        }
        public void Add(T element, float pri1, float pri2)
        {
            int elementIndex = _array.Length;
            _array.Add(new HeapElement<T>(element, pri1, pri2));
            if (elementIndex != 0)
            {
                HeapifyUp(elementIndex);
            }
        }
        public T GetMin() => _array[0].data;
        public T ExtractMin()
        {
            T min = _array[0].data;
            HeapElement<T> last = _array[_array.Length - 1];
            _array[0] = last;
            _array.Length--;
            if (_array.Length > 1)
            {
                HeapifyDown(0);
            }
            return min;
        }
        public void SetPriority(int index, float pri1)
        {
            int length = _array.Length;
            HeapElement<T> cur = _array[index];
            cur.pri1 = pri1;
            _array[index] = cur;
            int parIndex = index / 2 - 1;
            int lcIndex = index * 2 + 1;
            int rcIndex = index * 2 + 2;
            parIndex = math.select(index, parIndex, parIndex >= 0);
            lcIndex = math.select(index, lcIndex, lcIndex < length);
            rcIndex = math.select(index, rcIndex, rcIndex < length);
            HeapElement<T> parent = _array[parIndex];
            if (cur.pri1 < parent.pri1 || (cur.pri1 == parent.pri1 && cur.pri2 < parent.pri2))
            {
                HeapifyUp(index);
            }
            else
            {
                HeapifyDown(index);
            }
        }
        public void Dispose()
        {
            _array.Dispose();
        }

        void HeapifyUp(int startIndex)
        {
            int curIndex = startIndex;
            int parIndex = (curIndex - 1) / 2;
            HeapElement<T> cur = _array[startIndex];
            HeapElement<T> par = _array[parIndex];
            bool isCurSmaller = cur.pri1 < par.pri1 || (cur.pri1 == par.pri1 && cur.pri2 < par.pri2);
            while (isCurSmaller)
            {
                _array[parIndex] = cur;
                _array[curIndex] = par;
                curIndex = parIndex;
                parIndex = math.select((curIndex - 1) / 2, 0, curIndex == 0);
                par = _array[parIndex];
                isCurSmaller = cur.pri1 < par.pri1 || (cur.pri1 == par.pri1 && cur.pri2 < par.pri2);
            }
        }
        void HeapifyDown(int startIndex)
        {
            int length = _array.Length;
            int curIndex = startIndex;
            int lcIndex = startIndex * 2 + 1;
            int rcIndex = lcIndex + 1;
            lcIndex = math.select(curIndex, lcIndex, lcIndex < length);
            rcIndex = math.select(curIndex, rcIndex, rcIndex < length);
            HeapElement<T> cur;
            HeapElement<T> lc;
            HeapElement<T> rc;
            while (lcIndex != curIndex)
            {
                cur = _array[curIndex];
                lc = _array[lcIndex];
                rc = _array[rcIndex];
                bool lcSmallerThanRc = lc.pri1 < rc.pri1 || (lc.pri1 == rc.pri1 && lc.pri2 < rc.pri2);
                bool lcSmallerThanCur = lc.pri1 < cur.pri1 || (lc.pri1 == cur.pri1 && lc.pri2 < cur.pri2);
                bool rcSmallerThanCur = rc.pri1 < cur.pri1 || (rc.pri1 == cur.pri1 && rc.pri2 < cur.pri2);

                if (lcSmallerThanRc && lcSmallerThanCur)
                {
                    _array[curIndex] = lc;
                    _array[lcIndex] = cur;
                    curIndex = lcIndex;
                    lcIndex = curIndex * 2 + 1;
                    rcIndex = lcIndex + 1;
                    lcIndex = math.select(lcIndex, curIndex, lcIndex >= length);
                    rcIndex = math.select(rcIndex, curIndex, rcIndex >= length);
                }
                else if (!lcSmallerThanRc && rcSmallerThanCur)
                {
                    _array[curIndex] = rc;
                    _array[rcIndex] = cur;
                    curIndex = rcIndex;
                    lcIndex = curIndex * 2 + 1;
                    rcIndex = lcIndex + 1;
                    lcIndex = math.select(lcIndex, curIndex, lcIndex >= length);
                    rcIndex = math.select(rcIndex, curIndex, rcIndex >= length);
                }
                else
                {
                    break;
                }
            }
        }
        public struct HeapElement<T> where T : unmanaged
        {
            public T data;
            public float pri1;
            public float pri2;

            public HeapElement(T data, float pri1, float pri2)
            {
                this.data = data;
                this.pri1 = pri1;
                this.pri2 = pri2;
            }
        }
    }
}