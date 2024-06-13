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
                float2 newDestination = GetClosestIndex(request.Destination, sourceIsland, islandProcessor, CostFields[request.Offset]);
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

        float2 GetClosestIndex(float2 destination, int desiredIsland, IslandFieldProcessor islandFieldProcessors, UnsafeListReadOnly<byte> costField)
        {
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


            while (pickedExtensionIndexDistance == float.MaxValue)
            {
                int2 topLeft = destinationIndex + new int2(-offset, offset);
                int2 topRight = destinationIndex + new int2(offset, offset);
                int2 botLeft = destinationIndex + new int2(-offset, -offset);
                int2 botRight = destinationIndex + new int2(offset, -offset);

                bool topOverflow = topLeft.y >= FieldRowAmount;
                bool botOverflow = botLeft.y < 0;
                bool rightOverflow = topRight.x >= FieldColAmount;
                bool leftOverflow = topLeft.x < 0;

                if (topOverflow && botOverflow && rightOverflow && leftOverflow) { return destination; }

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
                        int rowStart = math.select(9, topRight.y % SectorRowAmount, i == topRightSector);
                        int rowEnd = math.select(0, botRight.y % SectorRowAmount, i == botRightSector);
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
                        int colStart = math.select(9, botRight.x % SectorColAmount, i == botRightSector);
                        int colEnd = math.select(0, botLeft.x % SectorColAmount, i == botLeftSector);
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
            return FlowFieldUtilities.IndexToPos(outputGeneral2d, TileSize, FieldGridStartPos);

            ExtensionIndex CheckSectorRow(int sectorToCheck, int rowToCheck, int colToStart, int colToEnd)
            {
                if (islandFieldProcessors.GetIslandIfNotField(sectorToCheck, out int islandOut))
                {
                    if (islandOut != desiredIsland) { return new ExtensionIndex() { Distance = float.MaxValue }; }
                }
                float currentExtensionIndexDistance = float.MaxValue;
                int currentExtensionIndexLocalIndex = 0;
                int sectorCostStartIndex = sectorToCheck * sectorTileAmount;
                int startLocal = rowToCheck * sectorColAmount + colToStart;
                int checkRange = colToEnd - colToStart + 1;
                int checkCount = math.abs(checkRange);
                int checkCountNonZero = math.select(checkCount, 1, checkCount == 0);
                int checkUnit = checkRange / checkCountNonZero;

                int startIndex = sectorCostStartIndex + startLocal;
                for (int i = 0; i < checkCount; i++)
                {
                    int indexToCheck = startIndex + i * checkUnit;
                    int localIndex = indexToCheck - sectorCostStartIndex;
                    byte cost = costField[indexToCheck];
                    if (cost == byte.MaxValue) { continue; }
                    int island = islandFieldProcessors.GetIsland(sectorToCheck, localIndex);
                    if (island == desiredIsland)
                    {
                        float newExtensionCost = FlowFieldUtilities.GetCostBetween(sectorToCheck, localIndex, destinationSector, destinationLocalIndex, sectorColAmount, sectorMatrixColAmount);
                        if (newExtensionCost < currentExtensionIndexDistance) { currentExtensionIndexDistance = newExtensionCost; currentExtensionIndexLocalIndex = localIndex; }
                    }
                }
                return new ExtensionIndex()
                {
                    SectorIndex = sectorToCheck,
                    LocalIndex = currentExtensionIndexLocalIndex,
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
                int currentExtensionIndexLocalIndex = 0;
                int sectorCostStartIndex = sectorToCheck * sectorTileAmount;
                int startLocal = rowToStart * sectorColAmount + colToCheck;
                int checkRange = rowToEnd - rowToStart + 1;
                int checkCount = math.abs(checkRange);
                int checkCountNonZero = math.select(checkCount, 1, checkCount == 0);
                int checkUnit = checkRange / checkCountNonZero;

                int startIndex = sectorCostStartIndex + startLocal;
                for (int i = 0; i < checkCount; i++)
                {
                    int indexToCheck = startIndex + i * sectorColAmount * checkUnit;
                    int localIndex = indexToCheck - sectorCostStartIndex;
                    byte cost = costField[indexToCheck];
                    if (cost == byte.MaxValue) { continue; }
                    int island = islandFieldProcessors.GetIsland(sectorToCheck, localIndex);
                    if (island == desiredIsland)
                    {
                        float newExtensionCost = FlowFieldUtilities.GetCostBetween(sectorToCheck, localIndex, destinationSector, destinationLocalIndex, sectorColAmount, sectorMatrixColAmount);
                        if (newExtensionCost < currentExtensionIndexDistance) { currentExtensionIndexDistance = newExtensionCost; currentExtensionIndexLocalIndex = localIndex; }
                    }
                }
                return new ExtensionIndex()
                {
                    SectorIndex = sectorToCheck,
                    LocalIndex = currentExtensionIndexLocalIndex,
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
