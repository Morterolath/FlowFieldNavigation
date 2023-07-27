﻿using Unity.Collections;
using UnityEngine;
using UnityEngine.Jobs;
using Unity.Mathematics;
using System.Runtime.InteropServices.WindowsRuntime;
using Unity.Burst;

[BurstCompile]
public struct AgentMovementUpdateJob : IJobParallelForTransform
{
    public float DeltaTime;
    [ReadOnly] public NativeArray<AgentData> AgentDataArray;
    [WriteOnly] public NativeArray<Vector3> AgentPositions;
    public void Execute(int index, TransformAccess transform)
    {
        AgentData data = AgentDataArray[index];
        if(data.Direction.x == 0 && data.Direction.y == 0){ return; }
        else
        {
            float3 direction = new float3(data.Direction.x, 0f, data.Direction.y);
            float3 pos = transform.position;
            float3 newPos = pos + (direction * data.Speed * DeltaTime);
            transform.position = newPos;
            AgentPositions[index] = newPos;
        }
    }
}