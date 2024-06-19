using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using System;
using Codice.CM.Common;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct DynamicGoalPathReconstructionDeterminationJob : IJobParallelForBatch
    {
        internal float TileSize;
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        internal int SectorTileAmount;
        internal int FieldColAmount;
        internal float2 FieldGridStartPos;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<byte>> CostFields;
        [ReadOnly] internal NativeArray<PathDestinationData> PathDestinationDataArray;
        [ReadOnly] internal NativeParallelMultiHashMap<int, int> PathIndexToUpdateSeedMap;
        internal NativeArray<PathRoutineData> PathRoutineDataArray;

        public void Execute(int startIndex, int count)
        {
            NativeBitArray bfsArray = new NativeBitArray(SectorTileAmount, Allocator.Temp);
            NativeQueue<int> bfsQueue = new NativeQueue<int>(Allocator.Temp);

            for (int i = startIndex; i < startIndex + count; i++)
            {
                int curPathIndex = i;
                PathRoutineData routineData = PathRoutineDataArray[curPathIndex];
                bool alreadyBeingReconstructed = (routineData.Task & PathTask.Reconstruct) == PathTask.Reconstruct;
                bool alreadyOutOfRangeDueToRangeChange = (routineData.DestinationState & DynamicDestinationState.OutOfReach) == DynamicDestinationState.OutOfReach;
                if (alreadyBeingReconstructed ||alreadyOutOfRangeDueToRangeChange)
                {
                    continue;
                }
                NativeParallelMultiHashMap<int,int>.Enumerator updateSeedEnumerator = PathIndexToUpdateSeedMap.GetValuesForKey(curPathIndex);
                PathDestinationData destinationData = PathDestinationDataArray[curPathIndex];
                float2 destination = destinationData.Destination;
                int offset = destinationData.Offset;
                int2 goalIndex2d = FlowFieldUtilities.PosTo2D(destination, TileSize, FieldGridStartPos);
                int goalIndex1d = FlowFieldUtilities.To1D(goalIndex2d, FieldColAmount);
                bool canNotReach = false;
                while(updateSeedEnumerator.MoveNext() && !canNotReach)
                {
                    int seed = updateSeedEnumerator.Current;
                    canNotReach = IsOutOfReach(seed, goalIndex1d, offset, bfsArray, bfsQueue);
                    bfsArray.Clear();
                    bfsQueue.Clear();
                }
                if (canNotReach)
                {
                    routineData.DestinationState = DynamicDestinationState.OutOfReach;
                    PathRoutineDataArray[curPathIndex] = routineData;
                }
            }
        }

        bool IsOutOfReach(int seedTile, int targetTile, int costFieldOffset, NativeBitArray bfsArray, NativeQueue<int> bfsQueue)
        {
            LocalIndex1d seedLocal = FlowFieldUtilities.GetLocal1D(seedTile, FieldColAmount, SectorColAmount, SectorMatrixColAmount);
            LocalIndex1d targetLocal = FlowFieldUtilities.GetLocal1D(targetTile, FieldColAmount, SectorColAmount, SectorMatrixColAmount);
            if(seedLocal.sector != targetLocal.sector) { return true; }
            if(seedLocal.index == targetLocal.index) { return false; }
            int bfsSector = seedLocal.sector;
            int bfsSectorCostStartIndex = bfsSector * SectorTileAmount;
            UnsafeListReadOnly<byte> costs = CostFields[costFieldOffset];
            //Transfer unwalkable areas
            for(int i = 0; i < SectorTileAmount; i++)
            {
                bfsArray.Set(i, costs[bfsSectorCostStartIndex + i] == byte.MaxValue);
            }

            //Run bfs, search for targetLocal
            bfsArray.Set(seedLocal.index, true);
            bfsQueue.Enqueue(seedLocal.index);

            while (!bfsQueue.IsEmpty())
            {
                int curLocal1d = bfsQueue.Dequeue();

                //Calculate neighbour indicies
                int4 localIndicies_N_E_S_W = new int4(curLocal1d, curLocal1d, curLocal1d, curLocal1d);
                localIndicies_N_E_S_W += new int4(SectorColAmount, 1, -SectorColAmount, -1);
                bool4 localOverflow_N_E_S_W = new bool4()
                {
                    x = localIndicies_N_E_S_W.x >= SectorTileAmount,
                    y = localIndicies_N_E_S_W.y % SectorColAmount == 0,
                    z = localIndicies_N_E_S_W.z < 0,
                    w = (curLocal1d % SectorColAmount) == 0,
                };
                localIndicies_N_E_S_W = math.select(localIndicies_N_E_S_W, curLocal1d, localOverflow_N_E_S_W);

                //Look at neighbours
                bool4 targetFound4 = localIndicies_N_E_S_W == targetLocal.index;
                if(targetFound4.x || targetFound4.y || targetFound4.z || targetFound4.w) { return false; }

                //Enqueue neighbours if can
                bool nEnqueueable = !bfsArray.IsSet(localIndicies_N_E_S_W.x);
                bool eEnqueueable = !bfsArray.IsSet(localIndicies_N_E_S_W.y);
                bool sEnqueueable = !bfsArray.IsSet(localIndicies_N_E_S_W.z);
                bool wEnqueueable = !bfsArray.IsSet(localIndicies_N_E_S_W.w);
                if (nEnqueueable)
                {
                    bfsQueue.Enqueue(localIndicies_N_E_S_W.x);
                    bfsArray.Set(localIndicies_N_E_S_W.x, true);
                }
                if (eEnqueueable)
                {
                    bfsQueue.Enqueue(localIndicies_N_E_S_W.y);
                    bfsArray.Set(localIndicies_N_E_S_W.y, true);
                }
                if (sEnqueueable)
                {
                    bfsQueue.Enqueue(localIndicies_N_E_S_W.z);
                    bfsArray.Set(localIndicies_N_E_S_W.z, true);
                }
                if (wEnqueueable)
                {
                    bfsQueue.Enqueue(localIndicies_N_E_S_W.w);
                    bfsArray.Set(localIndicies_N_E_S_W.w, true);
                }
            }
            return true;
        }
    }
}