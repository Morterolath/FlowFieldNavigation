﻿using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct CollisionResolutionJob : IJobParallelFor
{
    [ReadOnly] public AgentSpatialHashGrid AgentSpatialHashGrid;
    [ReadOnly] public NativeArray<RoutineResult> RoutineResultArray;
    public NativeArray<float3> AgentPositionChangeBuffer;
    public void Execute(int index)
    {
        AgentMovementData agentData = AgentSpatialHashGrid.RawAgentMovementDataArray[index];
        bool hasForeignInFront = RoutineResultArray[index].HasForeignInFront;
        float3 agentPos3 = agentData.Position;
        float2 agentPos2 = new float2(agentData.Position.x, agentData.Position.z);
        float3 totalResolution = 0;
        float agentRadius = agentData.Radius;
        float maxResolutionLength = 0;
        float checkRange = agentRadius;
        for (int i = 0; i < AgentSpatialHashGrid.GetGridCount(); i++)
        {
            SpatialHashGridIterator iterator = AgentSpatialHashGrid.GetIterator(agentPos2, checkRange, i);
            while (iterator.HasNext())
            {
                NativeSlice<AgentMovementData> agentsToCheck = iterator.GetNextRow(out int sliceStart);
                for (int j = 0; j < agentsToCheck.Length; j++)
                {
                    if (j + sliceStart == index) { continue; }

                    AgentMovementData mateData = agentsToCheck[j];
                    AgentStatus mateStatus = mateData.Status;
                    float mateRadius = mateData.Radius;
                    float3 mastePos3 = mateData.Position;
                    float2 matePos2 = new float2(mastePos3.x, mastePos3.z);
                    float desiredDistance = checkRange + mateRadius;
                    float distance = math.distance(agentPos3, mastePos3);
                    float overlapping = desiredDistance - distance;
                    if (overlapping <= 0) { continue; }
                    bool mateInFront = math.dot(agentData.CurrentDirection, matePos2 - agentPos2) > 0;
                    bool agentInFront = math.dot(mateData.CurrentDirection, agentPos2 - matePos2) > 0;
                    float resoltionMultiplier = GetMultiplier(agentData.Status, mateStatus, agentRadius, mateRadius, hasForeignInFront, mateInFront, agentInFront);
                    if (resoltionMultiplier == 0f) { continue; }
                    resoltionMultiplier *= overlapping;

                    float3 resolutionForce = math.normalizesafe(agentPos3 - mastePos3) * resoltionMultiplier;
                    resolutionForce = math.select(resolutionForce, new float3(sliceStart + j, 0, 1), distance == 0 && index < sliceStart + j);
                    totalResolution += resolutionForce;
                    maxResolutionLength = math.select(resoltionMultiplier, maxResolutionLength, maxResolutionLength >= resoltionMultiplier);
                }
            }
        }
        float totalResolutionLen = math.length(totalResolution);

        //Enable if you want speed to be considered while calculating maxResolutionLength. More realistic, but increased overlapping.
        //maxResolutionLength = math.select(agentData.Speed * 0.02f, maxResolutionLength, maxResolutionLength < agentData.Speed * 0.016f);
        totalResolution = math.select(totalResolution, totalResolution / totalResolutionLen * maxResolutionLength, totalResolutionLen > maxResolutionLength);
        AgentPositionChangeBuffer[index] += totalResolution;
    }
    float GetMultiplier(AgentStatus agentStatus, AgentStatus mateStatus, float agentRadius, float mateRadius, bool hasForeignInFront, bool mateInFront, bool agentInFront)
    {
        bool agentMoving = (agentStatus & AgentStatus.Moving) == AgentStatus.Moving;
        bool agentHoldGround = (agentStatus & AgentStatus.HoldGround) == AgentStatus.HoldGround;
        bool agentStopped = agentStatus == 0;
        bool mateMoving = (mateStatus & AgentStatus.Moving) == AgentStatus.Moving;
        bool mateHoldGround = (mateStatus & AgentStatus.HoldGround) == AgentStatus.HoldGround;
        bool mateStopped = mateStatus == 0;
        const float fullMultiplier = 1f;
        const float noneMultiplier = 0f;
        const float stoppedMultiplier = 0.9f;
        if(agentMoving && mateMoving)
        {
            float multiplier = math.select(1f, 0.2f, !mateInFront);
            multiplier = math.select(multiplier, 0.55f, hasForeignInFront);
            return mateRadius / (agentRadius + mateRadius) * multiplier;
        }
        if(agentStopped && mateStopped)
        {
            if (!mateInFront) { return 0.7f; }
            if(mateInFront && agentInFront) { return 0.5f; }
            return 0.1f;
        }
        if (agentStopped && (mateMoving || mateHoldGround))
        {
            return stoppedMultiplier;
        }
        if (agentMoving && mateHoldGround)
        {
            return fullMultiplier;
        }
        if (agentMoving && mateStopped)
        {
            return 0.05f;
        }
        return noneMultiplier;
    }
}