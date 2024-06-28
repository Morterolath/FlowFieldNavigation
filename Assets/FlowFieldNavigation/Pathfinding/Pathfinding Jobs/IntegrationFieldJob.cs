using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace FlowFieldNavigation
{

    [BurstCompile]
    internal struct IntegrationFieldJob : IJob
    {
        internal int2 TargetIndex;
        internal float2 Goal;
        internal float GoalRange;
        internal float TileSize;
        internal float2 FieldGridStartPos;
        internal int FieldColAmount;
        internal int FieldRowAmount;
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        internal int SectorTileAmount;

        [ReadOnly] internal NativeArray<int> SectorIndiciesToCalculateIntegration;
        [ReadOnly] internal NativeArray<byte> CostField;
        [ReadOnly] internal NativeParallelMultiHashMap<int, ActiveWaveFront> SectorToWaveFrontsMap;
        [ReadOnly] internal NativeArray<int> SectorFlowStartTable;
        [ReadOnly] internal NativeHashSet<int> PossibleGoalSectors;
        internal NativeArray<IntegrationTile> IntegrationField;
        public void Execute()
        {
            for(int i = 0; i < SectorIndiciesToCalculateIntegration.Length; i++)
            {
                int sectorIndex = SectorIndiciesToCalculateIntegration[i];
                int sectorFlowStart = SectorFlowStartTable[sectorIndex];
                NativeList<int> goalTileList = new NativeList<int>(Allocator.Temp);
                NativeQueue<LocalIndex1d> integrationQueue = new NativeQueue<LocalIndex1d>(Allocator.Temp);
                NativeSlice<IntegrationTile> integrationSectorSlice = new NativeSlice<IntegrationTile>(IntegrationField, sectorFlowStart, SectorTileAmount);
                NativeSlice<byte> costSectorSlice = new NativeSlice<byte>(CostField, sectorIndex * SectorTileAmount, SectorTileAmount);
                Integrate(integrationSectorSlice, costSectorSlice, goalTileList, integrationQueue, sectorIndex);
            }
        }
        void Integrate(
            NativeSlice<IntegrationTile> integrationFieldSector, 
            NativeSlice<byte> costs, 
            NativeList<int> goalTileList, 
            NativeQueue<LocalIndex1d> integrationQueue,
            int sectorIndex)
        {
            integrationQueue.Clear();
            goalTileList.Clear();
            int sectorColAmount = SectorColAmount;
            int sectorTileAmount = sectorColAmount * sectorColAmount;

            ///////////LOOKUP TABLE////////////////
            ///////////////////////////////////////
            int nLocal1d;
            int eLocal1d;
            int sLocal1d;
            int wLocal1d;
            int neLocal1d;
            int seLocal1d;
            int swLocal1d;
            int nwLocal1d;
            bool nBlocked;
            bool eBlocked;
            bool sBlocked;
            bool wBlocked;
            float curIntCost;
            float nIntCost;
            float eIntCost;
            float sIntCost;
            float wIntCost;
            float neIntCost;
            float seIntCost;
            float swIntCost;
            float nwIntCost;
            bool nAvailable;
            bool eAvailable;
            bool sAvailable;
            bool wAvailable;
            ///////////////////////////////////////////////
            //CODE

            //Submit start tile costs
            NativeParallelMultiHashMap<int, ActiveWaveFront>.Enumerator enumerator = SectorToWaveFrontsMap.GetValuesForKey(sectorIndex);
            while (enumerator.MoveNext())
            {
                ActiveWaveFront front = enumerator.Current;
                IntegrationTile startTile = integrationFieldSector[front.LocalIndex];
                integrationFieldSector[front.LocalIndex] = new IntegrationTile()
                {
                    Cost = front.Distance,
                    Mark = startTile.Mark,
                };
            }
            if (PossibleGoalSectors.Contains(sectorIndex))
            {
                for (int i = 0; i < integrationFieldSector.Length; i++)
                {
                    byte tileCost = costs[i];
                    if(tileCost == byte.MaxValue) { continue; }
                    int2 general2d = FlowFieldUtilities.GetGeneral2d(i, sectorIndex, SectorMatrixColAmount, sectorColAmount);
                    float2 centerPos = FlowFieldUtilities.IndexToPos(general2d, TileSize, FieldGridStartPos);
                    float distanceToGoal = math.distance(Goal, centerPos);
                    if (distanceToGoal > GoalRange && !TargetIndex.Equals(general2d)) { continue; }

                    IntegrationTile goalTile = integrationFieldSector[i];
                    goalTile.Cost = 0f;
                    integrationFieldSector[i] = goalTile;
                    goalTileList.Add(i);
                }
            }

            //Start fast marching from start indicies
            enumerator.Reset();
            while (enumerator.MoveNext())
            {
                int index = enumerator.Current.LocalIndex;
                SetLookupTable(index);
                Enqueue();
            }
            for (int i = 0; i < goalTileList.Length; i++)
            {
                int goalTileIndex = goalTileList[i];
                SetLookupTable(goalTileIndex);
                Enqueue();
            }

            //Rests of fast marching
            while (!integrationQueue.IsEmpty())
            {
                LocalIndex1d cur = integrationQueue.Dequeue();
                SetLookupTable(cur.index);
                float newCost = GetCost();
                IntegrationTile tile = integrationFieldSector[cur.index];
                tile.Cost = newCost;
                tile.Mark = ~((~tile.Mark) | IntegrationMark.Awaiting);
                curIntCost = newCost;
                integrationFieldSector[cur.index] = tile;
                Enqueue();
            }
            //HELPERS
            void SetLookupTable(int curLocal1d)
            {
                //LOCAL INDICIES
                nLocal1d = curLocal1d + sectorColAmount;
                eLocal1d = curLocal1d + 1;
                sLocal1d = curLocal1d - sectorColAmount;
                wLocal1d = curLocal1d - 1;
                neLocal1d = nLocal1d + 1;
                seLocal1d = sLocal1d + 1;
                swLocal1d = sLocal1d - 1;
                nwLocal1d = nLocal1d - 1;

                //OVERFLOWS
                bool nLocalOverflow = nLocal1d >= sectorTileAmount;
                bool eLocalOverflow = (eLocal1d % sectorColAmount) == 0;
                bool sLocalOverflow = sLocal1d < 0;
                bool wLocalOverflow = (curLocal1d % sectorColAmount) == 0;

                nLocal1d = math.select(nLocal1d, curLocal1d, nLocalOverflow);
                eLocal1d = math.select(eLocal1d, curLocal1d, eLocalOverflow);
                sLocal1d = math.select(sLocal1d, curLocal1d, sLocalOverflow);
                wLocal1d = math.select(wLocal1d, curLocal1d, wLocalOverflow);
                neLocal1d = math.select(neLocal1d, curLocal1d, nLocalOverflow || eLocalOverflow);
                seLocal1d = math.select(seLocal1d, curLocal1d, sLocalOverflow || eLocalOverflow);
                swLocal1d = math.select(swLocal1d, curLocal1d, sLocalOverflow || wLocalOverflow);
                nwLocal1d = math.select(nwLocal1d, curLocal1d, nLocalOverflow || wLocalOverflow);

                //COSTS
                nBlocked = costs[nLocal1d] == byte.MaxValue;
                eBlocked = costs[eLocal1d] == byte.MaxValue;
                sBlocked = costs[sLocal1d] == byte.MaxValue;
                wBlocked = costs[wLocal1d] == byte.MaxValue;

                IntegrationMark nMark = IntegrationMark.None;
                IntegrationMark eMark = IntegrationMark.None;
                IntegrationMark sMark = IntegrationMark.None;
                IntegrationMark wMark = IntegrationMark.None;

                nMark = integrationFieldSector[nLocal1d].Mark;
                eMark = integrationFieldSector[eLocal1d].Mark;
                sMark = integrationFieldSector[sLocal1d].Mark;
                wMark = integrationFieldSector[wLocal1d].Mark;

                //INTEGRATED COSTS
                curIntCost = integrationFieldSector[curLocal1d].Cost;
                nIntCost = integrationFieldSector[nLocal1d].Cost;
                eIntCost = integrationFieldSector[eLocal1d].Cost;
                sIntCost = integrationFieldSector[sLocal1d].Cost;
                wIntCost = integrationFieldSector[wLocal1d].Cost;
                neIntCost = integrationFieldSector[neLocal1d].Cost;
                seIntCost = integrationFieldSector[seLocal1d].Cost;
                swIntCost = integrationFieldSector[swLocal1d].Cost;
                nwIntCost = integrationFieldSector[nwLocal1d].Cost;

                //AVAILABILITY
                nAvailable = !nBlocked && !nLocalOverflow && (nMark & IntegrationMark.Awaiting) != IntegrationMark.Awaiting;
                eAvailable = !eBlocked && !eLocalOverflow && (eMark & IntegrationMark.Awaiting) != IntegrationMark.Awaiting;
                sAvailable = !sBlocked && !sLocalOverflow && (sMark & IntegrationMark.Awaiting) != IntegrationMark.Awaiting;
                wAvailable = !wBlocked && !wLocalOverflow && (wMark & IntegrationMark.Awaiting) != IntegrationMark.Awaiting;
            }
            void Enqueue()
            {
                float nDif = nIntCost - curIntCost;
                float eDif = eIntCost - curIntCost;
                float sDif = sIntCost - curIntCost;
                float wDif = wIntCost - curIntCost;
                if (nAvailable && nDif > 1f)
                {
                    IntegrationTile tile = integrationFieldSector[nLocal1d];
                    tile.Mark |= IntegrationMark.Awaiting;
                    integrationFieldSector[nLocal1d] = tile;
                    integrationQueue.Enqueue(new LocalIndex1d(nLocal1d, 0));
                }
                if (eAvailable && eDif > 1f)
                {
                    IntegrationTile tile = integrationFieldSector[eLocal1d];
                    tile.Mark |= IntegrationMark.Awaiting;
                    integrationFieldSector[eLocal1d] = tile;
                    integrationQueue.Enqueue(new LocalIndex1d(eLocal1d, 0));
                }
                if (sAvailable && sDif > 1f)
                {
                    IntegrationTile tile = integrationFieldSector[sLocal1d];
                    tile.Mark |= IntegrationMark.Awaiting;
                    integrationFieldSector[sLocal1d] = tile;
                    integrationQueue.Enqueue(new LocalIndex1d(sLocal1d, 0));
                }
                if (wAvailable && wDif > 1f)
                {
                    IntegrationTile tile = integrationFieldSector[wLocal1d];
                    tile.Mark |= IntegrationMark.Awaiting;
                    integrationFieldSector[wLocal1d] = tile;
                    integrationQueue.Enqueue(new LocalIndex1d(wLocal1d, 0));
                }
            }
            float GetCost()
            {
                float costToReturn = float.MaxValue;
                float nCost = nIntCost + 1f;
                float eCost = eIntCost + 1f;
                float sCost = sIntCost + 1f;
                float wCost = wIntCost + 1f;
                float neCost = math.select(neIntCost + 1.4f, float.MaxValue, nBlocked && eBlocked);
                float seCost = math.select(seIntCost + 1.4f, float.MaxValue, sBlocked && eBlocked);
                float swCost = math.select(swIntCost + 1.4f, float.MaxValue, sBlocked && wBlocked);
                float nwCost = math.select(nwIntCost + 1.4f, float.MaxValue, nBlocked && wBlocked);

                costToReturn = math.select(costToReturn, nCost, nCost < costToReturn);
                costToReturn = math.select(costToReturn, eCost, eCost < costToReturn);
                costToReturn = math.select(costToReturn, sCost, sCost < costToReturn);
                costToReturn = math.select(costToReturn, wCost, wCost < costToReturn);
                costToReturn = math.select(costToReturn, neCost, neCost < costToReturn);
                costToReturn = math.select(costToReturn, seCost, seCost < costToReturn);
                costToReturn = math.select(costToReturn, swCost, swCost < costToReturn);
                costToReturn = math.select(costToReturn, nwCost, nwCost < costToReturn);
                return costToReturn;
            }

        }
    }

}