using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct StaticPathGoalSectorFMJob : IJobParallelForBatch
    {
        internal float2 FieldGridStartPos;
        internal float TileSize;
        internal int SectorMatrixColAmount;
        internal int SectorColAmount;
        internal int SectorTileAmount;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<byte>> CostFields;
        [ReadOnly] internal NativeList<FinalPathRequest> FinalPathRequests;
        [ReadOnly] internal NativeList<int> StaticGoalSectors;
        internal NativeList<float> StaticGoalSectorBfsGrid;
        internal NativeList<int> StaticGoalSectorIndexToFinalPathRequestIndex;
        public void Execute(int startIndex, int count)
        {
            NativeSlice<float> sectorBfsGrid = new NativeSlice<float>(StaticGoalSectorBfsGrid.AsArray(), startIndex, count);
            int sectorIndex = StaticGoalSectors[startIndex / SectorTileAmount];
            int requestIndex = StaticGoalSectorIndexToFinalPathRequestIndex[startIndex / SectorTileAmount];
            FinalPathRequest request = FinalPathRequests[requestIndex];
            int offset = request.Offset;
            float2 goal = request.Destination;
            float goalRangeSq = request.Range * request.Range;
            UnsafeListReadOnly<byte> costs = CostFields[offset];
            RunFM(sectorIndex, goal, goalRangeSq, costs, sectorBfsGrid);
        }
        void RunFM(int sectorIndex, float2 goal, float goalRangeSq, UnsafeListReadOnly<byte> costs, NativeSlice<float> targetSectorCostsGrid)
        {
            int2 goal2d = FlowFieldUtilities.PosTo2D(goal, TileSize, FieldGridStartPos);
            int sectorTileAmount = SectorTileAmount;
            int sectorColAmount = SectorColAmount;
            NativeBitArray isBlocked = new NativeBitArray(sectorTileAmount, Allocator.Temp);
            NativeQueue<int> fastMarchingQueue = new NativeQueue<int>(Allocator.Temp);
            int4 directions_N_E_S_W;
            int4 directions_NE_SE_SW_NW;
            bool4 isBlocked_N_E_S_W;


            //Initialize grid
            for (int i = 0; i < targetSectorCostsGrid.Length; i++)
            {
                bool tileIsUnwalkable = costs[sectorIndex * sectorTileAmount + i] == byte.MaxValue;
                targetSectorCostsGrid[i] = float.MaxValue;
                isBlocked.Set(i, tileIsUnwalkable);
            }

            //initialize start indicies
            for(int i = 0; i < targetSectorCostsGrid.Length; i++)
            {
                int localIndex = i;
                int2 general2d = FlowFieldUtilities.GetGeneral2d(i, sectorIndex, SectorMatrixColAmount, SectorColAmount);
                float2 indexPos = FlowFieldUtilities.IndexToPos(general2d, TileSize, FieldGridStartPos);
                if(math.distancesq(indexPos, goal) > goalRangeSq && !goal2d.Equals(general2d)) { continue; }
                targetSectorCostsGrid[localIndex] = 0f;
                isBlocked.Set(localIndex, true);
            }

            //Enqueue start index neighbour
            for (int i = 0; i < targetSectorCostsGrid.Length; i++)
            {
                int localIndex = i;
                int2 general2d = FlowFieldUtilities.GetGeneral2d(i, sectorIndex, SectorMatrixColAmount, SectorColAmount);
                float2 indexPos = FlowFieldUtilities.IndexToPos(general2d, TileSize, FieldGridStartPos);
                if (math.distancesq(indexPos, goal) > goalRangeSq && !goal2d.Equals(general2d)) { continue; }
                SetNeighbourData(localIndex);
                EnqueueNeighbours();
            }

            //Remaining
            while (!fastMarchingQueue.IsEmpty())
            {
                int curIndex = fastMarchingQueue.Dequeue();
                SetNeighbourData(curIndex);
                targetSectorCostsGrid[curIndex] = GetCost();
                EnqueueNeighbours();
            }

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
    }
}
