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
            int2 goalIndex,
            int islandSeed,
            int fieldColAmount,
            int sectorColAmount,
            int sectorTileAmount,
            float fieldTileSize,
            float2 fieldGridStartPos,
            int sectorMatrixColAmount,
            int newPortalSliceStartIndex,

            NativeList<PickedPortalDataRecord> pickedPortalDataRecord,
            NativeArray<byte> costs,
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
            NativeHashSet<int> possibleGoalSectors
            )
        {
            sourcePortalIndexList.Clear();
            SubmitPickedMarks();
            SetSourcePortalIndicies();
            NativeList<int> indiciesToClear = new NativeList<int>(Allocator.Temp);
            NativeArray<GoalNeighborPortal> goalNeighborPortals = GetGoalPortalIndicies();
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

            //set new integration field length
            SetNewIntegrationFieldLength();

            //clear portalDataArray
            ClearPortalTraversalDataArray();

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
                int targetSector = FlowFieldUtilities.GetSector1D(goalIndex, sectorColAmount, sectorMatrixColAmount);
                if ((sectorStateTable[targetSector] & PathSectorState.Included) != PathSectorState.Included)
                {
                    pickedSectorList.Add(targetSector);
                    sectorStateTable[targetSector] |= PathSectorState.Included;
                }
                possibleGoalSectors.Add(targetSector);
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
            void AddGoalNeihborPortalsToTheHeap(NativeArray<GoalNeighborPortal> goalNeighborPortals, NativeHeap<int> traversalHeap, int2 goalIndex)
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
            NativeArray<GoalNeighborPortal> GetGoalPortalIndicies()
            {
                NativeList<GoalNeighborPortal> goalNeighborPortals = new NativeList<GoalNeighborPortal>(Allocator.Temp);
                int goalSectorIndex = FlowFieldUtilities.GetSector1D(goalIndex, sectorColAmount, sectorMatrixColAmount);
                NativeArray<float> targetSectorCostGrid = RunFM(goalSectorIndex, FlowFieldUtilities.GetLocal1D(goalIndex, sectorColAmount));

                SectorNode sectorNode = sectorNodes[goalSectorIndex];
                int winPtr = sectorNode.SecToWinPtr;
                int winCnt = sectorNode.SecToWinCnt;
                for (int i = 0; i < winCnt; i++)
                {
                    WindowNode windowNode = windowNodes[secToWinPtrs[winPtr + i]];
                    int porPtr = windowNode.PorPtr;
                    int porCnt = windowNode.PorCnt;
                    for (int j = 0; j < porCnt; j++)
                    {
                        int portalIndex = j + porPtr;
                        PortalNode portalNode = portalNodes[portalIndex];
                        int portalLocalIndexAtSector = FlowFieldUtilities.GetLocal1dInSector(portalNode, goalSectorIndex, sectorMatrixColAmount, sectorColAmount);
                        float cost = targetSectorCostGrid[portalLocalIndexAtSector];
                        if(cost == float.MaxValue) { continue; }

                        goalNeighborPortals.Add(new GoalNeighborPortal(portalIndex, cost));
                    }
                }
                return goalNeighborPortals.AsArray();
            }
            NativeArray<float> RunFM(int sectorIndex, int startLocalIndex)
            {
                NativeArray<float> targetSectorCostsGrid = new NativeArray<float>(sectorTileAmount, Allocator.Temp);
                NativeSlice<byte> sectorCosts = new NativeSlice<byte>(costs, sectorIndex * sectorTileAmount, sectorTileAmount);
                NativeBitArray isBlocked = new NativeBitArray(sectorTileAmount, Allocator.Temp);
                NativeQueue<int> fastMarchingQueue = new NativeQueue<int>(Allocator.Temp);
                int4 directions_N_E_S_W;
                int4 directions_NE_SE_SW_NW;
                bool4 isBlocked_N_E_S_W;


                //Initialize grid
                for (int i = 0; i < targetSectorCostsGrid.Length; i++)
                {
                    bool tileIsUnwalkable = sectorCosts[i] == byte.MaxValue;
                    targetSectorCostsGrid[i] = float.MaxValue;
                    isBlocked.Set(i, tileIsUnwalkable);
                }

                targetSectorCostsGrid[startLocalIndex] = 0f;
                isBlocked.Set(startLocalIndex, true);

                SetNeighbourData(startLocalIndex);
                EnqueueNeighbours();

                //Remaining
                while (!fastMarchingQueue.IsEmpty())
                {
                    int curIndex = fastMarchingQueue.Dequeue();
                    SetNeighbourData(curIndex);
                    targetSectorCostsGrid[curIndex] = GetCost();
                    EnqueueNeighbours();
                }
                return targetSectorCostsGrid;

                void SetNeighbourData(int curIndex)
                {
                    directions_N_E_S_W = new int4()
                    {
                        x = sectorColAmount,
                        y = 1,
                        z = -sectorColAmount,
                        w = -1
                    };
                    directions_N_E_S_W += curIndex;
                    directions_NE_SE_SW_NW = new int4()
                    {
                        x = sectorColAmount,
                        y = -sectorColAmount,
                        z = -sectorColAmount,
                        w = sectorColAmount,
                    };
                    directions_NE_SE_SW_NW += new int4(1, 1, -1, -1);
                    directions_NE_SE_SW_NW += curIndex;
                    bool4 overflow_N_E_S_W = new bool4()
                    {
                        x = directions_N_E_S_W.x >= sectorTileAmount,
                        y = (directions_N_E_S_W.y % sectorColAmount) == 0,
                        z = directions_N_E_S_W.z < 0,
                        w = (curIndex % sectorColAmount) == 0,
                    };
                    bool4 overflow_NE_SE_SW_NW = new bool4()
                    {
                        x = overflow_N_E_S_W.x || overflow_N_E_S_W.y,
                        y = overflow_N_E_S_W.z || overflow_N_E_S_W.y,
                        z = overflow_N_E_S_W.z || overflow_N_E_S_W.w,
                        w = overflow_N_E_S_W.x || overflow_N_E_S_W.w,
                    };
                    directions_N_E_S_W = math.select(directions_N_E_S_W, curIndex, overflow_N_E_S_W);
                    directions_NE_SE_SW_NW = math.select(directions_NE_SE_SW_NW, curIndex, overflow_NE_SE_SW_NW);
                    isBlocked_N_E_S_W = new bool4()
                    {
                        x = isBlocked.IsSet(directions_N_E_S_W.x),
                        y = isBlocked.IsSet(directions_N_E_S_W.y),
                        z = isBlocked.IsSet(directions_N_E_S_W.z),
                        w = isBlocked.IsSet(directions_N_E_S_W.w),
                    };
                }
                float GetCost()
                {
                    float4 costs_N_E_S_W = new float4()
                    {
                        x = targetSectorCostsGrid[directions_N_E_S_W.x],
                        y = targetSectorCostsGrid[directions_N_E_S_W.y],
                        z = targetSectorCostsGrid[directions_N_E_S_W.z],
                        w = targetSectorCostsGrid[directions_N_E_S_W.w],
                    };
                    costs_N_E_S_W += 1f;
                    float4 costs_NE_SE_SW_NW = new float4()
                    {
                        x = targetSectorCostsGrid[directions_NE_SE_SW_NW.x],
                        y = targetSectorCostsGrid[directions_NE_SE_SW_NW.y],
                        z = targetSectorCostsGrid[directions_NE_SE_SW_NW.z],
                        w = targetSectorCostsGrid[directions_NE_SE_SW_NW.w],
                    };
                    costs_NE_SE_SW_NW += 1.4f;
                    float4 min4 = math.min(costs_N_E_S_W, costs_NE_SE_SW_NW);
                    return math.min(min4.w, math.min(min4.z, math.min(min4.x, min4.y)));
                }
                void EnqueueNeighbours()
                {
                    if (!isBlocked_N_E_S_W.x)
                    {
                        fastMarchingQueue.Enqueue(directions_N_E_S_W.x);
                        isBlocked.Set(directions_N_E_S_W.x, true);
                    }
                    if (!isBlocked_N_E_S_W.y)
                    {
                        fastMarchingQueue.Enqueue(directions_N_E_S_W.y);
                        isBlocked.Set(directions_N_E_S_W.y, true);
                    }
                    if (!isBlocked_N_E_S_W.z)
                    {
                        fastMarchingQueue.Enqueue(directions_N_E_S_W.z);
                        isBlocked.Set(directions_N_E_S_W.z, true);
                    }
                    if (!isBlocked_N_E_S_W.w)
                    {
                        fastMarchingQueue.Enqueue(directions_N_E_S_W.w);
                        isBlocked.Set(directions_N_E_S_W.w, true);
                    }
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
