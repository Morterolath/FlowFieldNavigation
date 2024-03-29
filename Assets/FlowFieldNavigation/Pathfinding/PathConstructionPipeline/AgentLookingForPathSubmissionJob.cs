﻿using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Burst;
using System;

namespace FlowFieldNavigation
{

    [BurstCompile]
    internal struct AgentLookingForPathSubmissionJob : IJob
    {
        internal float TileSize;
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        internal int SectorTileAmount;
        internal float2 FieldGridStartPos;
        [ReadOnly] internal NativeArray<float> AgentRadii;
        [ReadOnly] internal NativeArray<float3> AgentPositions;
        [ReadOnly] internal NativeArray<UnsafeListReadOnly<byte>> CostFields;
        [ReadOnly] internal NativeList<PathRequest> InitialPathRequests;
        internal NativeArray<int> AgentNewPathIndicies;
        internal NativeList<int> AgentsLookingForPath;
        internal NativeList<PathRequestRecord> AgentsLookingForPathRequestRecords;
        internal NativeList<PathRequestRecord> ReadyAgentsLookingForPathRequestRecords;
        internal NativeList<int> ReadyAgentsLookingForPath;
        public void Execute()
        {
            NativeArray<PathRequest> intitialPathRequestsAsArray = InitialPathRequests.AsArray();
            //Unsubmit
            for (int i = AgentsLookingForPath.Length - 1; i >= 0; i--)
            {
                int agentIndex = AgentsLookingForPath[i];
                int newPathIndex = AgentNewPathIndicies[agentIndex];
                if (newPathIndex != -1)
                {
                    AgentsLookingForPath.RemoveAtSwapBack(i);
                    AgentsLookingForPathRequestRecords.RemoveAtSwapBack(i);
                    continue;
                }
                float3 agentpos3 = AgentPositions[agentIndex];
                float2 agentPos2 = new float2(agentpos3.x, agentpos3.z);
                int agentOffset = FlowFieldUtilities.RadiusToOffset(AgentRadii[agentIndex], TileSize);
                int2 agentIndex2d = FlowFieldUtilities.PosTo2D(agentPos2, TileSize, FieldGridStartPos);
                LocalIndex1d agentLocal = FlowFieldUtilities.GetLocal1D(agentIndex2d, SectorColAmount, SectorMatrixColAmount);
                byte agentIndexCost = CostFields[agentOffset][agentLocal.sector * SectorTileAmount + agentLocal.index];
                if (agentIndexCost != byte.MaxValue)
                {
                    PathRequestRecord lookingForPathDestination = AgentsLookingForPathRequestRecords[i];
                    ReadyAgentsLookingForPath.Add(agentIndex);
                    ReadyAgentsLookingForPathRequestRecords.Add(lookingForPathDestination);
                    AgentsLookingForPath.RemoveAtSwapBack(i);
                    AgentsLookingForPathRequestRecords.RemoveAtSwapBack(i);
                }
            }

            //Submit
            for (int index = 0; index < AgentNewPathIndicies.Length; index++)
            {
                int newPathIndex = AgentNewPathIndicies[index];
                if (newPathIndex == -1) { continue; }

                float3 agentpos3 = AgentPositions[index];
                float2 agentPos2 = new float2(agentpos3.x, agentpos3.z);
                int agentOffset = FlowFieldUtilities.RadiusToOffset(AgentRadii[index], TileSize);
                int2 agentIndex = FlowFieldUtilities.PosTo2D(agentPos2, TileSize, FieldGridStartPos);
                LocalIndex1d agentLocal = FlowFieldUtilities.GetLocal1D(agentIndex, SectorColAmount, SectorMatrixColAmount);
                byte agentIndexCost = CostFields[agentOffset][agentLocal.sector * SectorTileAmount + agentLocal.index];
                if (agentIndexCost == byte.MaxValue)
                {
                    AgentsLookingForPath.Add(index);
                    PathRequest pointedPathRequest = intitialPathRequestsAsArray[newPathIndex];
                    PathRequestRecord lookingForPathDestination = new PathRequestRecord(pointedPathRequest);
                    AgentsLookingForPathRequestRecords.Add(lookingForPathDestination);
                    AgentNewPathIndicies[index] = -1;
                    continue;
                }
            }
        }
    }

}