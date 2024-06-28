using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using Unity.Mathematics;
using System.Diagnostics;
namespace FlowFieldNavigation
{
    internal class beh : MonoBehaviour
    {
        public int sectorCount;
        public int batch;
        private void Start()
        {
            
        }
        private void Update()
        {
            /*
            NativeArray<int> sectors = new NativeArray<int>(sectorCount, Allocator.Persistent);
            NativeArray<bool> flag = new NativeArray<bool>(sectorCount, Allocator.Persistent);
            for(int i=0; i < sectors.Length; i++)
            {
                sectors[i] = i;
            }

            testSIMD tj = new testSIMD()
            {
                SectorColAmount = 16,
                FieldGridStartPos = 0,
                RangeSq = 123,
                Sectors = sectors,
                TileSize = 1,
                Flag = flag,
                Goal = 654,
                SectorMatrixColAmount = 100
            };

            Stopwatch sw = new Stopwatch();
            sw.Start();
            tj.Schedule(sectors.Length, batch).Complete();
            sw.Stop();
            UnityEngine.Debug.Log("simd: " + sw.Elapsed.TotalMilliseconds);
            bool[] simdresult = flag.ToArray();
            for(int i = 0; i < flag.Length; i++) { flag[i] = false; }
            TestNormal tn = new TestNormal()
            {
                SectorColAmount = 16,
                FieldGridStartPos = 0,
                RangeSq = 123,
                Sectors = sectors,
                TileSize = 1,
                Flag = flag,
                Goal = 654,
                SectorMatrixColAmount = 100
            };

            sw = new Stopwatch();
            sw.Start();
            tn.Schedule(sectors.Length, batch).Complete();
            sw.Stop();
            UnityEngine.Debug.Log("normal: " + sw.Elapsed.TotalMilliseconds);
            bool[] normalresult = flag.ToArray();

            for(int i = 0; i< normalresult.Length; i++)
            {
                if (normalresult[i] != simdresult[i])
                {
                    UnityEngine.Debug.Log("different");
                    return;
                }
            }*/
        }
    }


    [BurstCompile]
    public struct testSIMD : IJobParallelFor
    {
        internal float TileSize;
        internal float2 FieldGridStartPos;
        internal float2 Goal;
        internal float RangeSq;
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        [ReadOnly] internal NativeArray<int> Sectors;
        [WriteOnly] internal NativeArray<bool> Flag;

        public void Execute(int index)
        {
            bool flag = false;
            int sector = Sectors[index];
            float halfTileSize = TileSize / 2;
            int2 sectorStart = FlowFieldUtilities.GetSectorStartIndex(FlowFieldUtilities.To2D(sector, SectorMatrixColAmount), SectorColAmount);
            for (int r = sectorStart.y; r < sectorStart.y + SectorColAmount; r+=4)
            {
                for (int c = sectorStart.x; c < sectorStart.x + SectorColAmount; c+=4)
                {
                    int4 index_x = new int4(c, c + 1, c + 2, c + 3);
                    int4 index_y = new int4(r, r + 1, r + 2, r + 3);
                    float4 pos_x = FieldGridStartPos.x + (math.asfloat(index_x) * TileSize) + halfTileSize;
                    float4 pos_y = FieldGridStartPos.y + (math.asfloat(index_y) * TileSize) + halfTileSize;
                    float4 pos_to_goal_x = pos_x - Goal.x;
                    float4 pos_to_goal_y = pos_y - Goal.y;
                    float4 x_sq = pos_to_goal_x * pos_to_goal_x;
                    float4 y_sq = pos_to_goal_y * pos_to_goal_y;
                    float4 distsq = x_sq + y_sq;
                    bool4 closeEnough = distsq < RangeSq;
                    flag = math.any(closeEnough);
                }
            }
            Flag[index] = flag;
        }
    }
    [BurstCompile]
    internal struct TestNormal : IJobParallelFor
    {
        internal float TileSize;
        internal float2 FieldGridStartPos;
        internal float2 Goal;
        internal float RangeSq;
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        [ReadOnly] internal NativeArray<int> Sectors;
        [WriteOnly] internal NativeArray<bool> Flag;
        public void Execute(int index)
        {
            bool flag = false;
            int sector = Sectors[index];
            int2 sectorStart = FlowFieldUtilities.GetSectorStartIndex(FlowFieldUtilities.To2D(sector, SectorMatrixColAmount), SectorColAmount);
            for (int r = sectorStart.y; r < sectorStart.y + SectorColAmount; r++)
            {
                for (int c = sectorStart.x; c < sectorStart.x + SectorColAmount; c++)
                {
                    int2 index2 = new int2(c, r);
                    float2 indexPos = FlowFieldUtilities.IndexToPos(index2, TileSize, FieldGridStartPos);
                    flag ^= math.distancesq(indexPos, Goal) < RangeSq;
                }
            }
            Flag[index] = flag;
        }
    }
}
