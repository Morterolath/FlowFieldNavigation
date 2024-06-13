using Unity.Collections;
using Unity.Burst;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using Unity.VisualScripting;

namespace FlowFieldNavigation
{
    //The job is unsafe unfortunately. Amount of scheduled jobs depend on the output of prevıuos job.
    //And i dont want it to run on main thread (time waste + bad scaling)
    //Luckily, possible race conditions are easy to find, but no exceptions thrown :(
    [BurstCompile]
    internal struct FinalPathRequestDestinationExpansionJob : IJob
    {
        internal int TotalJobCount;
        internal int JobIndex;
        internal float TileSize;
        internal int SectorColAmount;
        internal int SectorRowAmount;
        internal int SectorMatrixColAmount;
        internal int SectorTileAmount;
        internal int FieldRowAmount;
        internal int FieldColAmount;
        internal float FieldMinXIncluding;
        internal float FieldMinYIncluding;
        internal float FieldMaxXExcluding;
        internal float FieldMaxYExcluding;
        internal float2 FieldGridStartPos;
        [NativeDisableContainerSafetyRestriction] internal NativeList<FinalPathRequest> FinalPathRequests;
        [ReadOnly] internal NativeArray<IslandFieldProcessor> IslandFieldProcessors;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<byte>> CostFields;
        public void Execute()
        {
            NativeSlice<FinalPathRequest> pickedFinalRequests = GetFinalPathRequestSlice();
            for (int index = 0; index < pickedFinalRequests.Length; index++)
            {
                FinalPathRequest request = pickedFinalRequests[index];
                IslandFieldProcessor islandProcessor = IslandFieldProcessors[request.Offset];
                int sourceIsland = request.SourceIsland;

                //Clamp destination to bounds
                request.Destination.x = math.select(request.Destination.x, FieldMinXIncluding, request.Destination.x < FieldMinXIncluding);
                request.Destination.y = math.select(request.Destination.y, FieldMinYIncluding, request.Destination.y < FieldMinYIncluding);
                request.Destination.x = math.select(request.Destination.x, FieldMaxXExcluding - TileSize / 2, request.Destination.x >= FieldMaxXExcluding);
                request.Destination.y = math.select(request.Destination.y, FieldMaxYExcluding - TileSize / 2, request.Destination.y >= FieldMaxYExcluding);
                request.DesiredDestination.x = math.select(request.DesiredDestination.x, FieldMinXIncluding, request.DesiredDestination.x < FieldMinXIncluding);
                request.DesiredDestination.y = math.select(request.DesiredDestination.y, FieldMinYIncluding, request.DesiredDestination.y < FieldMinYIncluding);
                request.DesiredDestination.x = math.select(request.DesiredDestination.x, FieldMaxXExcluding - TileSize / 2, request.DesiredDestination.x >= FieldMaxXExcluding);
                request.DesiredDestination.y = math.select(request.DesiredDestination.y, FieldMaxYExcluding - TileSize / 2, request.DesiredDestination.y >= FieldMaxYExcluding);

                int destinationIsland = islandProcessor.GetIsland(request.Destination);
                int2 destination2d = FlowFieldUtilities.PosTo2D(request.Destination, TileSize, FieldGridStartPos);
                LocalIndex1d destinationLocal = FlowFieldUtilities.GetLocal1D(destination2d, SectorColAmount, SectorMatrixColAmount);
                if (sourceIsland == destinationIsland && CostFields[request.Offset][destinationLocal.sector * SectorTileAmount + destinationLocal.index] != byte.MaxValue)
                {
                    pickedFinalRequests[index] = request;
                    continue;
                }
                float2 newDestination = GetClosestIndex(
                    request.Destination, 
                    sourceIsland, 
                    islandProcessor, 
                    CostFields[request.Offset],
                    out int2 lastTopLeft,
                    out int2 lastTopRight,
                    out int2 lastBotLeft,
                    out int2 lastBotRight);
                newDestination = MakeSureExpandedDestinationIsClosest(
                    lastBotLeft,
                    lastBotRight,
                    lastTopLeft,
                    lastTopRight,
                    newDestination,
                    request.Destination,
                    sourceIsland,
                    CostFields[request.Offset],
                    islandProcessor);
                request.Destination = newDestination;
                pickedFinalRequests[index] = request;
            }
        }

        NativeSlice<FinalPathRequest> GetFinalPathRequestSlice()
        {
            NativeSlice<FinalPathRequest> sliceToReturn;
            int finalPathRequestCount = FinalPathRequests.Length;
            if (finalPathRequestCount < TotalJobCount)
            {
                int partitionSize = math.select(1, 0, JobIndex >= finalPathRequestCount);
                int partitionStart = math.select(JobIndex, 0, JobIndex >= finalPathRequestCount);
                sliceToReturn = new NativeSlice<FinalPathRequest>(FinalPathRequests.AsArray(), partitionStart, partitionSize);
            }
            else
            {
                int partitionSize = finalPathRequestCount / TotalJobCount;
                int partitionStart = JobIndex * partitionSize;
                int partitionSizeOverflow = partitionStart + partitionSize - finalPathRequestCount;
                partitionSizeOverflow = math.select(partitionSizeOverflow, 0, partitionSizeOverflow < 0);
                int partitionSizeClamped = partitionSize - partitionSizeOverflow;
                partitionSizeClamped = math.select(partitionSizeClamped, finalPathRequestCount - partitionStart, JobIndex + 1 == TotalJobCount);
                sliceToReturn = new NativeSlice<FinalPathRequest>(FinalPathRequests.AsArray(), partitionStart, partitionSizeClamped);
            }
            return sliceToReturn;
        }
        float2 MakeSureExpandedDestinationIsClosest(
            int2 lastBotLeft,
            int2 lastBotRight,
            int2 lastTopLeft,
            int2 lastTopRight,
            float2 pickedPosition,
            float2 goalPosition,
            int desiredIsland,
            UnsafeListReadOnly<byte> costField,
            IslandFieldProcessor islandFieldProcessor)
        {
            float tileSize = TileSize;
            float2 fieldGridStartPos = FieldGridStartPos;
            int sectorColAmount = SectorColAmount;
            int sectorMatrixColAmount = SectorMatrixColAmount;
            int sectorTileAmount = SectorTileAmount;

            float curGoalDistSq = math.distancesq(pickedPosition, goalPosition);
            float curGoalDist = math.sqrt(curGoalDistSq);

            int topSearchStartRow = math.min(FieldRowAmount - 1, lastTopLeft.y + 1);
            int topSearchEndRow;
            int topSearchStartCol = lastTopLeft.x;
            int topSearchEndCol = lastTopRight.x;

            int botSearchStartRow = math.max(0, lastBotLeft.y - 1);
            int botSearchEndRow;
            int botSearchStartCol = lastBotLeft.x;
            int botSearchEndCol = lastBotRight.x;

            int rightSearchStartRow = lastBotRight.y;
            int rightSearchEndRow = lastTopRight.y;
            int rightSearchStartCol = math.min(FieldColAmount - 1, lastTopRight.x + 1);
            int rightSearchEndCol;

            int leftSearchStartRow = lastBotLeft.y;
            int leftSearchEndRow = lastTopLeft.y;
            int leftSearchStartCol = math.max(0, lastTopLeft.x - 1);
            int leftSearchEndCol;

            SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);

            int curTopSearchRow = topSearchStartRow;
            int curBotSearchRow = botSearchStartRow;
            int curRightSearchCol = rightSearchStartCol;
            int curLeftSearchCol = leftSearchStartCol;
            bool topExhausted = false;
            bool botExhausted = false;
            bool rightExhausted = false;
            bool leftExhausted = false;

            bool exhausted = false;
            float2 foundPosition = pickedPosition;
            while (!exhausted)
            {
                topExhausted = topSearchEndRow < curTopSearchRow;
                botExhausted = botSearchEndRow > curBotSearchRow;
                rightExhausted = rightSearchEndCol < curRightSearchCol;
                leftExhausted = leftSearchEndCol > curLeftSearchCol;
                exhausted = topExhausted && botExhausted && rightExhausted && leftExhausted;

                if (!topExhausted)
                {
                    for (int c = topSearchStartCol; c <= topSearchEndCol; c++)
                    {
                        int2 curIndex = new int2(c, curTopSearchRow);
                        if (!IndexSuitable(curIndex, out float2 newPickedPos, out float newGoalDistSq)) { continue; }
                        foundPosition = newPickedPos;
                        curGoalDistSq = newGoalDistSq;
                        curGoalDist = math.sqrt(curGoalDistSq);
                        SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);
                    }
                }
                if (!botExhausted)
                {
                    for (int c = botSearchStartCol; c <= botSearchEndCol; c++)
                    {
                        int2 curIndex = new int2(c, curBotSearchRow);
                        if (!IndexSuitable(curIndex, out float2 newPickedPos, out float newGoalDistSq)) { continue; }
                        foundPosition = newPickedPos;
                        curGoalDistSq = newGoalDistSq;
                        curGoalDist = math.sqrt(curGoalDistSq);
                        SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);
                    }
                }
                if (!rightExhausted)
                {
                    for (int r = rightSearchStartRow; r <= rightSearchEndRow; r++)
                    {
                        int2 curIndex = new int2(curRightSearchCol, r);
                        if (!IndexSuitable(curIndex, out float2 newPickedPos, out float newGoalDistSq)) { continue; }
                        foundPosition = newPickedPos;
                        curGoalDistSq = newGoalDistSq;
                        curGoalDist = math.sqrt(curGoalDistSq);
                        SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);
                    }
                }
                if (!leftExhausted)
                {
                    for (int r = leftSearchStartRow; r <= leftSearchEndRow; r++)
                    {
                        int2 curIndex = new int2(curLeftSearchCol, r);
                        if (!IndexSuitable(curIndex, out float2 newPickedPos, out float newGoalDistSq)) { continue; }
                        foundPosition = newPickedPos;
                        curGoalDistSq = newGoalDistSq;
                        curGoalDist = math.sqrt(curGoalDistSq);
                        SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);
                    }
                }
                curTopSearchRow++;
                curBotSearchRow--;
                curRightSearchCol++;
                curLeftSearchCol--;
            }
            return foundPosition;
            bool IndexSuitable(int2 curIndex, out float2 indexPos, out float indexGoalDistSq)
            {
                indexPos = FlowFieldUtilities.IndexToPos(curIndex, tileSize, fieldGridStartPos);
                indexGoalDistSq = math.distancesq(indexPos, goalPosition);
                bool closeEnough = indexGoalDistSq < curGoalDistSq;
                LocalIndex1d curLocalIndex = FlowFieldUtilities.GetLocal1D(curIndex, sectorColAmount, sectorMatrixColAmount);
                int curCostFieldIndex = curLocalIndex.sector * sectorTileAmount + curLocalIndex.index;
                byte curCost = costField[curCostFieldIndex];
                bool walkable = curCost != byte.MaxValue;
                int curIsland = islandFieldProcessor.GetIsland(curLocalIndex.sector, curLocalIndex.index);
                bool onDesiredIsland = curIsland == desiredIsland;
                return onDesiredIsland && closeEnough && walkable;
            }
        }
        void SetEndFor_Top_Right_Bot_Left(float2 goalPosition, float curGoalDist, out int topEndRow, out int botEndRow, out int rightEndCol, out int leftEndCol)
        {
            int2 topIndex = FlowFieldUtilities.PosTo2D(new float2(goalPosition.x, goalPosition.y + curGoalDist), TileSize, FieldGridStartPos);
            int2 botIndex = FlowFieldUtilities.PosTo2D(new float2(goalPosition.x, goalPosition.y - curGoalDist), TileSize, FieldGridStartPos);
            int2 rightIndex = FlowFieldUtilities.PosTo2D(new float2(goalPosition.x + curGoalDist, goalPosition.y), TileSize, FieldGridStartPos);
            int2 leftIndex = FlowFieldUtilities.PosTo2D(new float2(goalPosition.x - curGoalDist, goalPosition.y), TileSize, FieldGridStartPos);

            topEndRow = math.min(FieldRowAmount - 1, topIndex.y);
            botEndRow = math.max(0, botIndex.y);
            rightEndCol = math.min(FieldColAmount - 1, rightIndex.x);
            leftEndCol = math.max(0, leftIndex.x);
        }
        float2 GetClosestIndex(
            float2 destination, 
            int desiredIsland, 
            IslandFieldProcessor islandFieldProcessors, 
            UnsafeListReadOnly<byte> costField,
            out int2 lastTopLeft,
            out int2 lastTopRight,
            out int2 lastBotLeft,
            out int2 lastBotRight)
        {
            float tileSize = TileSize;
            float2 fieldGridStartPos = FieldGridStartPos;
            int sectorTileAmount = SectorTileAmount;
            int sectorColAmount = SectorColAmount;
            int sectorMatrixColAmount = SectorMatrixColAmount;

            int2 destinationIndex = FlowFieldUtilities.PosTo2D(destination, TileSize, FieldGridStartPos);
            LocalIndex1d destinationLocal = FlowFieldUtilities.GetLocal1D(destinationIndex, SectorColAmount, SectorMatrixColAmount);
            int destinationLocalIndex = destinationLocal.index;
            int destinationSector = destinationLocal.sector;

            int offset = 1;

            float pickedExtensionIndexDistance = float.MaxValue;
            int pickedExtensionIndexLocalIndex = 0;
            int pickedExtensionIndexSector = 0;


            int2 topLeft = destinationIndex;
            int2 topRight = destinationIndex;
            int2 botLeft = destinationIndex;
            int2 botRight = destinationIndex;
            while (pickedExtensionIndexDistance == float.MaxValue)
            {
                topLeft = destinationIndex + new int2(-offset, offset);
                topRight = destinationIndex + new int2(offset, offset);
                botLeft = destinationIndex + new int2(-offset, -offset);
                botRight = destinationIndex + new int2(offset, -offset);

                bool topOverflow = topLeft.y >= FieldRowAmount;
                bool botOverflow = botLeft.y < 0;
                bool rightOverflow = topRight.x >= FieldColAmount;
                bool leftOverflow = topLeft.x < 0;

                if (topOverflow && botOverflow && rightOverflow && leftOverflow)
                {
                    lastTopLeft = topLeft;
                    lastBotLeft = botLeft;
                    lastTopRight = topRight;
                    lastBotRight = botRight;
                    return destination;
                }

                if (topOverflow)
                {
                    topLeft.y = FieldRowAmount - 1;
                    topRight.y = FieldRowAmount - 1;
                }
                if (botOverflow)
                {
                    botLeft.y = 0;
                    botRight.y = 0;
                }
                if (rightOverflow)
                {
                    botRight.x = FieldColAmount - 1;
                    topRight.x = FieldColAmount - 1;
                }
                if (leftOverflow)
                {
                    topLeft.x = 0;
                    botLeft.x = 0;
                }

                int topLeftSector = FlowFieldUtilities.GetSector1D(topLeft, sectorColAmount, SectorMatrixColAmount);
                int topRightSector = FlowFieldUtilities.GetSector1D(topRight, sectorColAmount, SectorMatrixColAmount);
                int botRightSector = FlowFieldUtilities.GetSector1D(botRight, sectorColAmount, SectorMatrixColAmount);
                int botLeftSector = FlowFieldUtilities.GetSector1D(botLeft, sectorColAmount, SectorMatrixColAmount);
                if (!topOverflow)
                {
                    int rowToCheck = topLeft.y % SectorRowAmount;
                    for (int i = topLeftSector; i <= topRightSector; i++)
                    {
                        int colStart = math.select(0, topLeft.x % SectorColAmount, i == topLeftSector);
                        int colEnd = math.select(9, topRight.x % SectorColAmount, i == topRightSector);
                        ExtensionIndex checkedExtension = CheckSectorRow(i, rowToCheck, colStart, colEnd);
                        if (checkedExtension.IsValid() && checkedExtension.Distance < pickedExtensionIndexDistance)
                        {
                            pickedExtensionIndexDistance = checkedExtension.Distance;
                            pickedExtensionIndexLocalIndex = checkedExtension.LocalIndex;
                            pickedExtensionIndexSector = checkedExtension.SectorIndex;
                        }
                    }
                }
                if (!rightOverflow)
                {
                    int colToCheck = topRight.x % SectorColAmount;
                    for (int i = topRightSector; i >= botRightSector; i -= SectorMatrixColAmount)
                    {
                        int rowStart = math.select(0, botRight.y % SectorRowAmount, i == botRightSector);
                        int rowEnd = math.select(9, topRight.y % SectorRowAmount, i == topRightSector);
                        ExtensionIndex checkedExtension = CheckSectorCol(i, colToCheck, rowStart, rowEnd);
                        if (checkedExtension.IsValid() && checkedExtension.Distance < pickedExtensionIndexDistance)
                        {
                            pickedExtensionIndexDistance = checkedExtension.Distance;
                            pickedExtensionIndexLocalIndex = checkedExtension.LocalIndex;
                            pickedExtensionIndexSector = checkedExtension.SectorIndex;
                        }
                    }
                }
                if (!botOverflow)
                {
                    int rowToCheck = botRight.y % SectorRowAmount;
                    for (int i = botRightSector; i >= botLeftSector; i--)
                    {
                        int colStart = math.select(0, botLeft.x % SectorColAmount, i == botLeftSector);
                        int colEnd = math.select(9, botRight.x % SectorColAmount, i == botRightSector);
                        ExtensionIndex checkedExtension = CheckSectorRow(i, rowToCheck, colStart, colEnd);
                        if (checkedExtension.IsValid() && checkedExtension.Distance < pickedExtensionIndexDistance)
                        {
                            pickedExtensionIndexDistance = checkedExtension.Distance;
                            pickedExtensionIndexLocalIndex = checkedExtension.LocalIndex;
                            pickedExtensionIndexSector = checkedExtension.SectorIndex;
                        }
                    }
                }
                if (!leftOverflow)
                {
                    int colToCheck = topLeft.x % SectorColAmount;
                    for (int i = botLeftSector; i <= topLeftSector; i += SectorMatrixColAmount)
                    {
                        int rowStart = math.select(0, botLeft.y % SectorRowAmount, i == botLeftSector);
                        int rowEnd = math.select(9, topLeft.y % SectorRowAmount, i == topLeftSector);
                        ExtensionIndex checkedExtension = CheckSectorCol(i, colToCheck, rowStart, rowEnd);
                        if (checkedExtension.IsValid() && checkedExtension.Distance < pickedExtensionIndexDistance)
                        {
                            pickedExtensionIndexDistance = checkedExtension.Distance;
                            pickedExtensionIndexLocalIndex = checkedExtension.LocalIndex;
                            pickedExtensionIndexSector = checkedExtension.SectorIndex;
                        }
                    }
                }
                offset++;
            }

            int2 outputGeneral2d = FlowFieldUtilities.GetGeneral2d(pickedExtensionIndexLocalIndex, pickedExtensionIndexSector, sectorMatrixColAmount, sectorColAmount);
            lastTopLeft = topLeft;
            lastBotLeft = botLeft;
            lastTopRight = topRight;
            lastBotRight = botRight;
            return FlowFieldUtilities.IndexToPos(outputGeneral2d, TileSize, FieldGridStartPos);

            ExtensionIndex CheckSectorRow(int sectorToCheck, int rowToCheck, int colToStart, int colToEnd)
            {
                if (islandFieldProcessors.GetIslandIfNotField(sectorToCheck, out int islandOut))
                {
                    if (islandOut != desiredIsland) { return new ExtensionIndex() { Distance = float.MaxValue }; }
                }
                float currentExtensionIndexDistance = float.MaxValue;
                int currentExtensionLocalIndex = 0;
                for(int c = colToStart; c <= colToEnd; c++)
                {
                    int2 local2d = new int2(c, rowToCheck);
                    int local1d = FlowFieldUtilities.To1D(local2d, sectorColAmount);
                    int costFieldIndex = sectorToCheck * sectorTileAmount + local1d;
                    byte cost = costField[costFieldIndex];
                    if (cost == byte.MaxValue) { continue; }
                    int island = islandFieldProcessors.GetIsland(sectorToCheck, local1d);
                    if (island != desiredIsland) { continue; }
                    int2 curGeneralIndex = FlowFieldUtilities.GetGeneral2d(local1d, sectorToCheck, sectorMatrixColAmount, sectorColAmount);
                    float2 curIndexPos = FlowFieldUtilities.IndexToPos(curGeneralIndex, tileSize, fieldGridStartPos);
                    float newExtensionDistance = math.distance(curIndexPos, destination);
                    if (newExtensionDistance < currentExtensionIndexDistance) { currentExtensionIndexDistance = newExtensionDistance; currentExtensionLocalIndex = local1d; }

                }
                return new ExtensionIndex()
                {
                    SectorIndex = sectorToCheck,
                    LocalIndex = currentExtensionLocalIndex,
                    Distance = currentExtensionIndexDistance
                };
            }
            ExtensionIndex CheckSectorCol(int sectorToCheck, int colToCheck, int rowToStart, int rowToEnd)
            {
                if (islandFieldProcessors.GetIslandIfNotField(sectorToCheck, out int islandOut))
                {
                    if (islandOut != desiredIsland) { return new ExtensionIndex() { Distance = float.MaxValue }; }
                }
                float currentExtensionIndexDistance = float.MaxValue;
                int currentExtensionLocalIndex = 0;
                for(int r = rowToStart; r <= rowToEnd; r++)
                {
                    int2 local2d = new int2(colToCheck, r);
                    int local1d = FlowFieldUtilities.To1D(local2d, sectorColAmount);
                    int costFieldIndex = sectorToCheck * sectorTileAmount + local1d;
                    byte cost = costField[costFieldIndex];
                    if(cost == byte.MaxValue) { continue; }
                    int island = islandFieldProcessors.GetIsland(sectorToCheck, local1d);
                    if(island != desiredIsland) { continue; }
                    int2 curGeneralIndex = FlowFieldUtilities.GetGeneral2d(local1d, sectorToCheck, sectorMatrixColAmount, sectorColAmount);
                    float2 curIndexPos = FlowFieldUtilities.IndexToPos(curGeneralIndex, tileSize, fieldGridStartPos);
                    float newExtensionDistance = math.distance(curIndexPos, destination);
                    if (newExtensionDistance < currentExtensionIndexDistance) { currentExtensionIndexDistance = newExtensionDistance; currentExtensionLocalIndex = local1d; }
                }
                return new ExtensionIndex()
                {
                    SectorIndex = sectorToCheck,
                    LocalIndex = currentExtensionLocalIndex,
                    Distance = currentExtensionIndexDistance
                };
            }
        }


        private struct ExtensionIndex
        {
            internal int LocalIndex;
            internal int SectorIndex;
            internal float Distance;

            internal bool IsValid()
            {
                return Distance != float.MaxValue;
            }
        }
    }


}
