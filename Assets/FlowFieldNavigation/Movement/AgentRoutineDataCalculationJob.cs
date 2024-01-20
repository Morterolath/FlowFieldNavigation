﻿using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct AgentRoutineDataCalculationJob : IJobParallelFor
{
    public float TileSize;
    public int FieldColAmount;
    public int SectorColAmount;
    public int SectorMatrixColAmount;
    public float SectorSize;
    [ReadOnly] public NativeArray<AgentData> AgentDataArray;
    [ReadOnly] public NativeArray<int> AgentCurPathIndicies;
    [ReadOnly] public NativeArray<PathLocationData> ExposedPathLocationDataArray;
    [ReadOnly] public NativeArray<PathFlowData> ExposedPathFlowDataArray;
    [ReadOnly] public NativeArray<float2> ExposedPathDestinationArray;
    [ReadOnly] public NativeArray<int> HashedToNormal;

    public NativeArray<AgentMovementData> AgentMovementData;
    public void Execute(int index)
    {
        //FIRST
        AgentMovementData data = AgentMovementData[index];
        float2 agentPos = new float2(data.Position.x, data.Position.z);
        int2 agentSector2d = new int2((int)math.floor(agentPos.x / (SectorColAmount * TileSize)), (int)math.floor(agentPos.y / (SectorColAmount * TileSize)));
        int2 general2d = new int2((int)math.floor(agentPos.x / TileSize), (int)math.floor(agentPos.y / TileSize));
        int2 sectorStart2d = agentSector2d * SectorColAmount;
        int2 local2d = general2d - sectorStart2d;
        int local1d = local2d.y * SectorColAmount + local2d.x;


        //IF NOT MOVING
        if ((data.Status & AgentStatus.Moving) != AgentStatus.Moving)
        {
            data.DesiredDirection = data.CurrentDirection;
            AgentMovementData[index] = data;
            return;
        }

        int agentNormalIndex = HashedToNormal[index];
        int agentCurPathIndex = AgentCurPathIndicies[agentNormalIndex];

        //IF NOT HAVE PATH
        if (agentCurPathIndex == -1)
        {
            data.DesiredDirection = data.CurrentDirection;
            AgentMovementData[index] = data;
            return;
        }

        //FLOW CALCULATION
        float2 pathDestination = ExposedPathDestinationArray[agentCurPathIndex];
        PathLocationData pathLocationData = ExposedPathLocationDataArray[agentCurPathIndex];
        PathFlowData pathFlowData = ExposedPathFlowDataArray[agentCurPathIndex];
        UnsafeList<FlowData> flowField = pathFlowData.FlowField;
        UnsafeLOSBitmap losMap = pathFlowData.LOSMap;
        UnsafeList<int> sectorFlowStarts = pathLocationData.SectorToPicked;
        int agentSector1d = FlowFieldUtilities.To1D(agentSector2d, SectorMatrixColAmount);
        int agentSectorFlowStart = sectorFlowStarts[agentSector1d];

        if (agentSectorFlowStart == 0 || agentSectorFlowStart >= flowField.Length || agentSectorFlowStart >= losMap.BitLength)
        {
            data.PathId = agentCurPathIndex;
            data.Destination = pathDestination;
            data.DesiredDirection = 0;
            AgentMovementData[index] = data;
            return;
        }

        float2 flow;
        data.AlignmentMultiplierPercentage = 1f;
        if (losMap.IsLOS(agentSectorFlowStart + local1d))
        {
            float2 posToDestination = pathDestination - agentPos;
            float distanceBetweenDestination = math.length(posToDestination);
            flow = math.select(posToDestination / distanceBetweenDestination, 0, distanceBetweenDestination == 0);
            float alignmentMultiplierPercentage = math.lerp(1f, 0f, (20f - distanceBetweenDestination) / 20f);
            alignmentMultiplierPercentage = math.select(alignmentMultiplierPercentage, 0, alignmentMultiplierPercentage < 0);
            alignmentMultiplierPercentage = math.select(alignmentMultiplierPercentage, 1f, alignmentMultiplierPercentage > 1f);
            data.AlignmentMultiplierPercentage = alignmentMultiplierPercentage;
        }
        else if (GetSectorDynamicFlowStartIfExists(pathLocationData.DynamicAreaPickedSectorFlowStarts, agentSector1d, out int dynamicSectorFlowStart))
        {
            FlowData flowData = pathFlowData.DynamicAreaFlowField[dynamicSectorFlowStart + local1d];
            flow = math.select(0f, flowData.GetFlow(TileSize), flowData.IsValid());
        }
        else
        {
            FlowData flowData = pathFlowData.FlowField[agentSectorFlowStart + local1d];
            flow = math.select(0f, flowData.GetFlow(TileSize), flowData.IsValid());
        }

        

        //FLOW CALCULATION
        flow = math.select(GetSmoothFlow(data.DesiredDirection, flow, data.Speed), flow, math.dot(data.DesiredDirection, flow) < 0.7f);
        data.DesiredDirection = flow;
        data.PathId = agentCurPathIndex;
        data.Destination = pathDestination;
        AgentMovementData[index] = data;
    }
    float2 GetSmoothFlow(float2 currentDirection, float2 desiredDirection, float speed)
    {
        float2 steeringToSeek = desiredDirection - currentDirection;
        float steeringToSeekLen = math.length(steeringToSeek);
        float2 steeringForce = math.select(steeringToSeek / steeringToSeekLen, 0f, steeringToSeekLen == 0) * math.select(speed / 1000, steeringToSeekLen, steeringToSeekLen < speed / 1000);
        return math.normalizesafe(currentDirection + steeringForce);
    }
    bool GetSectorDynamicFlowStartIfExists(UnsafeList<SectorFlowStart> dynamicFlowSectosStarts, int agentSectorIndex, out int sectorFlowStart)
    {
        for(int i = 0; i < dynamicFlowSectosStarts.Length; i++)
        {
            SectorFlowStart sectorFlowStartElement = dynamicFlowSectosStarts[i];
            if(sectorFlowStartElement.SectorIndex == agentSectorIndex)
            {
                sectorFlowStart = sectorFlowStartElement.FlowStartIndex;
                return true;
            }
        }
        sectorFlowStart = 0;
        return false;
    }
}