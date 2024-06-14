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
                float2 newDestination = ClosestIndexWithIslandQuery.GetClosestIndexWithIsland(
                    request.Destination,
                    sourceIsland,
                    TileSize,
                    FieldGridStartPos,
                    SectorTileAmount,
                    SectorColAmount,
                    SectorRowAmount,
                    SectorMatrixColAmount,
                    FieldRowAmount,
                    FieldColAmount,
                    islandProcessor,
                    CostFields[request.Offset]);
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
    }
}
