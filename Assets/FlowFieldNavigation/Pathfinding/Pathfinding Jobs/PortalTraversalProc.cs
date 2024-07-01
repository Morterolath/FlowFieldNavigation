using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections.LowLevel.Unsafe;
using static UnityEngine.GraphicsBuffer;

namespace FlowFieldNavigation
{
    internal struct PortalTraversalProc
    {
        internal static void Run(
            int pathIndex,
            int2 goalIndex,
            int islandSeed,
            int fieldColAmount,
            int sectorColAmount,
            int sectorTileAmount,
            float fieldTileSize,
            float2 fieldGridStartPos,
            int sectorMatrixColAmount,
            int sectorMatrixRowAmount,
            int newPortalSliceStartIndex,
            int newPickedSectorStartIndex,
            int losRange,


            NativeList<PickedPortalDataRecord> pickedPortalDataRecord,
            NativeList<IntegrationTile> integrationField,
            NativeList<ActivePortal> portalSequence,
            NativeList<Slice> portalSequenceSlices,
            NativeList<int> pickedSectorList,
            NativeSlice<float2> sourcePositions,
            UnsafeList<PathSectorState> sectorStateTable,
            NativeList<int> sourcePortalIndexList,
            NativeArray<PortalToPortal> porPtrs,
            NativeArray<PortalTraversalData> portalTraversalDataArray,
            NativeArray<SectorNode> sectorNodes,
            NativeArray<WindowNode> windowNodes,
            NativeArray<int> secToWinPtrs,
            NativeArray<PortalNode> portalNodes,
            NativeArray<UnsafeList<int>> islandFields,
            NativeReference<SectorsWihinLOSArgument> sectorWithinLosRange,
            UnsafeList<GoalNeighborPortal> goalNeighborPortals,
            NativeParallelMultiHashMap<int, int> pathToGoalSectorMap
            )
        {
            sourcePortalIndexList.Clear();
            SubmitPickedMarks();
            SetSourcePortalIndicies();
            NativeList<int> indiciesToClear = new NativeList<int>(Allocator.Temp);
            NativeHeap<int> traversalHeap = new NativeHeap<int>(0, Allocator.Temp);
            for (int i = 0; i < sourcePortalIndexList.Length; i++)
            {
                int currentSourcePortalIndex = sourcePortalIndexList[i];
                int2 curSourcePortalFieldIndex = portalNodes[currentSourcePortalIndex].Portal1.Index;
                RefreshPortalsInHeap(curSourcePortalFieldIndex);
                bool isFirstSource = i == 0;
                if (isFirstSource) { AddGoalNeihborPortalsToTheHeap(goalNeighborPortals, traversalHeap, curSourcePortalFieldIndex); }
                RunAStar(currentSourcePortalIndex);
            }

            //pick nodes and submit them as records
            for (int i = 0; i < sourcePortalIndexList.Length; i++)
            {
                int sourcePortalIndex = sourcePortalIndexList[i];
                PickPortalSequence(sourcePortalIndex);
            }

            //pick sectors
            PickSectorsFromPortalSequence();

            //add goal sector
            AddGoalSector();

            //check if new sectors contain a sector in los range
            CheckLOSUpadte();

            //set new integration field length
            SetNewIntegrationFieldLength();

            //clear portalDataArray
            ClearPortalTraversalDataArray();

            void CheckLOSUpadte()
            {
                int newAddedSectorStart = newPickedSectorStartIndex;
                int newAddedSectorCount = pickedSectorList.Length - newAddedSectorStart;
                NativeSlice<int> newAddedSectors = new NativeSlice<int>(pickedSectorList.AsArray(), newAddedSectorStart, newAddedSectorCount);
                if (ContainsSectorsWithinLOSRange(newAddedSectors))
                {
                    SectorsWihinLOSArgument argument = sectorWithinLosRange.Value;
                    argument |= SectorsWihinLOSArgument.AddedSectorWithinLOS;
                    sectorWithinLosRange.Value = argument;
                }

            }
            bool ContainsSectorsWithinLOSRange(NativeSlice<int> sectors)
            {
                int2 targetSector2d = FlowFieldUtilities.GetSector2D(goalIndex, sectorColAmount);
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
            void SubmitPickedMarks()
            {
                for (int i = 0; i < pickedPortalDataRecord.Length; i++)
                {
                    PickedPortalDataRecord record = pickedPortalDataRecord[i];
                    PortalTraversalData travData = new PortalTraversalData();
                    travData.Reset();
                    travData.Mark = PortalTraversalMark.AStarPicked;
                    portalTraversalDataArray[record.PortalIndex] = travData;
                }
            }
            void RefreshPortalsInHeap(int2 newHeuristicPosition)
            {
                NativeList<int> portalsInHeap = new NativeList<int>(Allocator.Temp);
                for(int i = 0; i < traversalHeap._array.Length; i++)
                {
                    NativeHeap<int>.HeapElement<int> portalHeapElement = traversalHeap._array[i];
                    int portalIndex = portalHeapElement.data;

                    PortalTraversalData portalTravData = portalTraversalDataArray[portalIndex];
                    if(portalTravData.HasMark(PortalTraversalMark.AStarExtracted)) { continue; }
                    portalTravData.Mark |= PortalTraversalMark.AStarExtracted;
                    portalTraversalDataArray[portalIndex] = portalTravData;

                    portalsInHeap.Add(portalIndex);
                }
                traversalHeap.Clear();

                for(int i = 0; i < portalsInHeap.Length; i++)
                {
                    int portalIndex = portalsInHeap[i];
                    PortalTraversalData portalTravData = portalTraversalDataArray[portalIndex];
                    PortalNode portalNode = portalNodes[portalIndex];
                    portalTravData.HCost = GetHeuristicDistance(portalNode.Portal1.Index, newHeuristicPosition);
                    portalTravData.FCost = portalTravData.GCost + portalTravData.HCost;
                    portalTravData.Mark = ~((~portalTravData.Mark) | PortalTraversalMark.AStarExtracted);
                    portalTraversalDataArray[portalIndex] = portalTravData;
                    traversalHeap.Add(portalIndex, portalTravData.FCost);
                }
            }
            void AddGoalSector()
            {
                NativeParallelMultiHashMap<int, int>.Enumerator goalSectors = pathToGoalSectorMap.GetValuesForKey(pathIndex);
                while (goalSectors.MoveNext())
                {
                    int sector = goalSectors.Current;
                    PathSectorState mark = PathSectorState.PossibleGoal;
                    if ((sectorStateTable[sector] & PathSectorState.Included) != PathSectorState.Included)
                    {
                        pickedSectorList.Add(sector);
                        mark |= PathSectorState.Included;
                    }
                    sectorStateTable[sector] |= mark;
                }
            }
            void ClearPortalTraversalDataArray()
            {
                PortalTraversalData clearedData = new PortalTraversalData();
                clearedData.Reset();
                for(int i = 0; i < indiciesToClear.Length; i++)
                {
                    portalTraversalDataArray[indiciesToClear[i]] = clearedData;
                }
                for(int i = 0; i < pickedPortalDataRecord.Length; i++)
                {
                    portalTraversalDataArray[pickedPortalDataRecord[i].PortalIndex] = clearedData;
                }
            }
            void SetNewIntegrationFieldLength()
            {
                int oldIntegrationFieldLength = integrationField.Length;
                int newIntegrationFieldLength = pickedSectorList.Length * sectorTileAmount + 1;
                integrationField.Length = newIntegrationFieldLength;
                for (int i = oldIntegrationFieldLength; i < newIntegrationFieldLength; i++)
                {
                    IntegrationTile tile = integrationField[i];
                    tile.Reset();
                    integrationField[i] = tile;
                }
            }
            void PickSectorsFromPortalSequence()
            {
                for (int i = newPortalSliceStartIndex; i < portalSequenceSlices.Length; i++)
                {
                    Slice slice = portalSequenceSlices[i];
                    int start = slice.Index;
                    int end = start + slice.Count;
                    for (int j = start; j < end - 1; j++)
                    {
                        PickSectorsBetweenportals(portalSequence[j], portalSequence[j + 1]);
                    }
                }

            }
            void PickSectorsBetweenportals(ActivePortal portal1, ActivePortal portal2)
            {
                int win1Sec1Index = FlowFieldUtilities.GetSector1D(portal1.FieldIndex1, fieldColAmount, sectorColAmount, sectorMatrixColAmount);
                int win1Sec2Index = FlowFieldUtilities.GetSector1D(portal1.FieldIndex2, fieldColAmount, sectorColAmount, sectorMatrixColAmount);
                int win2Sec1Index = FlowFieldUtilities.GetSector1D(portal2.FieldIndex1, fieldColAmount, sectorColAmount, sectorMatrixColAmount);
                int win2Sec2Index = FlowFieldUtilities.GetSector1D(portal2.FieldIndex2, fieldColAmount, sectorColAmount, sectorMatrixColAmount);
                bool sector1Included = (sectorStateTable[win1Sec1Index] & PathSectorState.Included) == PathSectorState.Included;
                bool sector2Included = (sectorStateTable[win1Sec2Index] & PathSectorState.Included) == PathSectorState.Included;
                if ((win1Sec1Index == win2Sec1Index || win1Sec1Index == win2Sec2Index) && !sector1Included)
                {
                    pickedSectorList.Add(win1Sec1Index);
                    sectorStateTable[win1Sec1Index] |= PathSectorState.Included;
                }
                if ((win1Sec2Index == win2Sec1Index || win1Sec2Index == win2Sec2Index) && !sector2Included)
                {
                    pickedSectorList.Add(win1Sec2Index);
                    sectorStateTable[win1Sec2Index] |= PathSectorState.Included;
                }
            }
            void PickPortalSequence(int startPortalIndex)
            {
                if (portalTraversalDataArray[startPortalIndex].HasMark(PortalTraversalMark.AStarPicked)) { return; }

                int sliceStart = portalSequence.Length;
                int currentPortalIndex = startPortalIndex;
                while(currentPortalIndex != -1)
                {
                    PortalTraversalData curTravData = portalTraversalDataArray[currentPortalIndex];
                    bool portalAlreadyPicked = curTravData.HasMark(PortalTraversalMark.AStarPicked);
                    bool portalGoalNeighbor = curTravData.HasMark(PortalTraversalMark.GoalNeighbour);
                    curTravData.Mark |= PortalTraversalMark.AStarPicked;
                    portalTraversalDataArray[currentPortalIndex] = curTravData;

                    PortalNode curNode = portalNodes[currentPortalIndex];
                    ActivePortal activePortal = new ActivePortal()
                    {
                        Distance = curTravData.GCost + curTravData.PortalCountToGoal,
                        FieldIndex1 = FlowFieldUtilities.To1D(curNode.Portal1.Index, fieldColAmount),
                        FieldIndex2 = FlowFieldUtilities.To1D(curNode.Portal2.Index, fieldColAmount),
                    };
                    portalSequence.Add(activePortal);
                    if (portalGoalNeighbor)
                    {
                        ActivePortal goalActivePortal = new ActivePortal()
                        {
                            FieldIndex1 = FlowFieldUtilities.To1D(goalIndex, fieldColAmount),
                            FieldIndex2 = FlowFieldUtilities.To1D(goalIndex, fieldColAmount),
                            Distance = 0,
                        };
                        portalSequence.Add(goalActivePortal);
                    }

                    //Record submission
                    if (!portalAlreadyPicked)
                    {
                        PickedPortalDataRecord record = new PickedPortalDataRecord()
                        {
                            PortalCountToGoal = curTravData.PortalCountToGoal,
                            GCost = curTravData.GCost,
                            OriginIndex = curTravData.OriginIndex,
                            PortalIndex = currentPortalIndex,
                        };
                        pickedPortalDataRecord.Add(record);
                    }

                    currentPortalIndex = math.select(curTravData.OriginIndex, -1, portalAlreadyPicked);
                }

                int sliceEnd = portalSequence.Length;
                int sliceLength = sliceEnd - sliceStart;
                portalSequenceSlices.Add(new Slice(sliceStart, sliceLength));
            }
            void AddGoalNeihborPortalsToTheHeap(UnsafeList<GoalNeighborPortal> goalNeighborPortals, NativeHeap<int> traversalHeap, int2 goalIndex)
            {
                for(int i = 0; i < goalNeighborPortals.Length; i++)
                {
                    GoalNeighborPortal goalNeighbor = goalNeighborPortals[i];
                    PortalNode goalNeighborNode = portalNodes[goalNeighbor.PortalIndex];
                    PortalTraversalData goalNeighborData = portalTraversalDataArray[goalNeighbor.PortalIndex];
                    goalNeighborData.GCost = goalNeighbor.Distance;
                    goalNeighborData.HCost = GetHeuristicDistance(goalNeighborNode.Portal1.Index, goalIndex);
                    goalNeighborData.FCost = goalNeighborData.GCost + goalNeighborData.HCost;
                    goalNeighborData.Mark |= PortalTraversalMark.AStarTraversed | PortalTraversalMark.GoalNeighbour;
                    goalNeighborData.PortalCountToGoal = 1;
                    portalTraversalDataArray[goalNeighbor.PortalIndex] = goalNeighborData; 
                    traversalHeap.Add(goalNeighbor.PortalIndex, goalNeighborData.FCost);
                    indiciesToClear.Add(goalNeighbor.PortalIndex);
                }
            }
            void RunAStar(int goalPortalIndex)
            {
                if (portalTraversalDataArray[goalPortalIndex].HasMark(PortalTraversalMark.AStarExtracted)) { return; }
                PortalNode goalPortalNode = portalNodes[goalPortalIndex];
                int2 goalIndex = goalPortalNode.Portal1.Index;
                while (Dequeue(out int dequeuedPortalIndex))
                {
                    bool goalFound = dequeuedPortalIndex == goalPortalIndex;
                    PortalNode curPortalNode = portalNodes[dequeuedPortalIndex];
                    PortalTraversalData curPorTravData = portalTraversalDataArray[dequeuedPortalIndex];
                    NativeSlice<PortalToPortal> portalNeighbors1 = new NativeSlice<PortalToPortal>(porPtrs, curPortalNode.Portal1.PorToPorPtr, curPortalNode.Portal1.PorToPorCnt);
                    NativeSlice<PortalToPortal> portalNeighbors2 = new NativeSlice<PortalToPortal>(porPtrs, curPortalNode.Portal2.PorToPorPtr, curPortalNode.Portal2.PorToPorCnt);
                    TraverseNeighbors(dequeuedPortalIndex, curPorTravData.PortalCountToGoal, goalIndex, curPorTravData.GCost, portalNeighbors1);
                    TraverseNeighbors(dequeuedPortalIndex, curPorTravData.PortalCountToGoal, goalIndex, curPorTravData.GCost, portalNeighbors2);
                    if (goalFound) { return; }
                }
            }
            void TraverseNeighbors(int originPortal, int originPortalCountToGoal, int2 goalIndex, float originPortalGCost, NativeSlice<PortalToPortal> neighborPointers)
            {
                for(int i = 0; i  < neighborPointers.Length; i++)
                {
                    PortalToPortal neighborPointer = neighborPointers[i];
                    PortalNode neighborPortalNode = portalNodes[neighborPointer.Index];
                    PortalTraversalData neighborTravData = portalTraversalDataArray[neighborPointer.Index];
                    if (neighborTravData.HasMark(PortalTraversalMark.AStarTraversed))
                    {
                        float newGCost = originPortalGCost + neighborPointer.Distance;
                        if(neighborTravData.GCost > newGCost)
                        {
                            neighborTravData.GCost = newGCost;
                            neighborTravData.FCost = newGCost + neighborTravData.HCost;
                            neighborTravData.OriginIndex = originPortal;
                            neighborTravData.PortalCountToGoal = originPortalCountToGoal + 1;
                            portalTraversalDataArray[neighborPointer.Index] = neighborTravData;
                            traversalHeap.Add(neighborPointer.Index, neighborTravData.FCost);
                        }
                    }
                    else
                    {
                        neighborTravData.GCost = originPortalGCost + neighborPointer.Distance;
                        neighborTravData.HCost = GetHeuristicDistance(neighborPortalNode.Portal1.Index, goalIndex);
                        neighborTravData.FCost = neighborTravData.GCost + neighborTravData.HCost;
                        neighborTravData.Mark |= PortalTraversalMark.AStarTraversed;
                        neighborTravData.OriginIndex = originPortal;
                        neighborTravData.PortalCountToGoal = originPortalCountToGoal + 1;
                        portalTraversalDataArray[neighborPointer.Index] = neighborTravData;
                        traversalHeap.Add(neighborPointer.Index, neighborTravData.FCost);
                        indiciesToClear.Add(neighborPointer.Index);
                    }
                }
            }
            float GetHeuristicDistance(int2 index1, int2 index2)
            {
                float2 pos1 = FlowFieldUtilities.IndexToPos(index1, fieldTileSize, fieldGridStartPos);
                float2 pos2 = FlowFieldUtilities.IndexToPos(index2, fieldTileSize, fieldGridStartPos);
                return math.distance(pos1, pos2);
            }
            bool Dequeue(out int portalIndex)
            {
                portalIndex = -1;
                while (!traversalHeap.IsEmpty)
                {
                    int dequeuedPortalIndex = traversalHeap.ExtractMin();
                    PortalTraversalData dequeuedPortalTravData = portalTraversalDataArray[dequeuedPortalIndex];
                    if (dequeuedPortalTravData.HasMark(PortalTraversalMark.AStarExtracted)) { continue; }
                    portalIndex = dequeuedPortalIndex;
                    dequeuedPortalTravData.Mark |= PortalTraversalMark.AStarExtracted;
                    portalTraversalDataArray[dequeuedPortalIndex] = dequeuedPortalTravData;
                    return true;
                }
                return false;
            }
            int GetIsland(int2 general2d)
            {
                int2 sector2d = FlowFieldUtilities.GetSector2D(general2d, sectorColAmount);
                int sector1d = FlowFieldUtilities.To1D(sector2d, sectorMatrixColAmount);
                SectorNode sector = sectorNodes[sector1d];

                if (sector.IsIslandValid())
                {
                    return portalNodes[sector.SectorIslandPortalIndex].IslandIndex;
                }
                else if (sector.IsIslandField)
                {
                    int2 sectorStart = FlowFieldUtilities.GetSectorStartIndex(sector2d, sectorColAmount);
                    int2 local2d = FlowFieldUtilities.GetLocal2D(general2d, sectorStart);
                    int local1d = FlowFieldUtilities.To1D(local2d, sectorColAmount);
                    int island = islandFields[sector1d][local1d];
                    switch (island)
                    {
                        case < 0:
                            return -island;
                        case int.MaxValue:
                            return int.MaxValue;
                        default:
                            return portalNodes[island].IslandIndex;
                    }
                }
                return int.MaxValue;
            }
            void SetSourcePortalIndicies()
            {
                int2 islandSeed2d = FlowFieldUtilities.To2D(islandSeed, fieldColAmount);
                int targetIsland = GetIsland(islandSeed2d);
                for (int i = 0; i < sourcePositions.Length; i++)
                {
                    float2 sourcePos = sourcePositions[i];
                    int2 sourceIndex = FlowFieldUtilities.PosTo2D(sourcePos, fieldTileSize, fieldGridStartPos);
                    int2 sourceSectorIndex = sourceIndex / sectorColAmount;
                    int sourceSectorIndexFlat = sourceSectorIndex.y * sectorMatrixColAmount + sourceSectorIndex.x;
                    //ADD SOURCE SECTOR TO THE PICKED SECTORS
                    PathSectorState sectorState = sectorStateTable[sourceSectorIndexFlat];
                    if ((sectorState & PathSectorState.Included) != PathSectorState.Included)
                    {
                        pickedSectorList.Add(sourceSectorIndexFlat);
                        sectorStateTable[sourceSectorIndexFlat] |= PathSectorState.Included | PathSectorState.Source;
                        SetSectorPortalIndicies(sourceSectorIndexFlat, sourcePortalIndexList, targetIsland);
                    }
                    else if ((sectorState & PathSectorState.Source) != PathSectorState.Source)
                    {
                        sectorStateTable[sourceSectorIndexFlat] |= PathSectorState.Source;
                        SetSectorPortalIndicies(sourceSectorIndexFlat, sourcePortalIndexList, targetIsland);
                    }
                }
            }
            void SetSectorPortalIndicies(int targetSectorIndexF, NativeList<int> destinationList, int targetIsland)
            {
                SectorNode sectorNode = sectorNodes[targetSectorIndexF];
                int winPtr = sectorNode.SecToWinPtr;
                int winCnt = sectorNode.SecToWinCnt;
                for (int i = 0; i < winCnt; i++)
                {
                    WindowNode windowNode = windowNodes[secToWinPtrs[winPtr + i]];
                    int porPtr = windowNode.PorPtr;
                    int porCnt = windowNode.PorCnt;
                    for (int j = 0; j < porCnt; j++)
                    {
                        if (portalNodes[j + porPtr].IslandIndex != targetIsland) { continue; }
                        destinationList.Add(j + porPtr);
                    }
                }
            }
        }
    }
    internal struct GoalNeighborPortal
    {
        internal int PortalIndex;
        internal float Distance;

        internal GoalNeighborPortal(int portalIndex, float distance) { PortalIndex = portalIndex; Distance = distance; }
    }
}
