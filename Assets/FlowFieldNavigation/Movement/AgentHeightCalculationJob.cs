﻿using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using System.Numerics;

[BurstCompile]
public struct AgentHeightCalculationJob : IJobParallelFor
{
    [ReadOnly] public TriangleSpatialHashGrid TriangleSpatialHashGrid;
    [ReadOnly] public NativeArray<AgentMovementData> AgentMovementDataArray;
    [ReadOnly] public NativeArray<float3> Verticies;
    public NativeArray<float3> AgentPositionChangeArray;
    public void Execute(int index)
    {
        float3 agentPos3 = AgentMovementDataArray[index].Position;
        float currentHeight = agentPos3.y;
        float2 agentPos2 = new float2(agentPos3.x, agentPos3.z);
        float desiredHeight = float.MinValue;
        for(int i = 0; i < TriangleSpatialHashGrid.GetGridCount(); i++)
        {
            TriangleSpatialHashGridIterator triangleGridIterator = TriangleSpatialHashGrid.GetIterator(agentPos2, i);
            while (triangleGridIterator.HasNext())
            {
                NativeSlice<int> triangles = triangleGridIterator.GetNextRow();
                for(int j = 0; j < triangles.Length; j += 3)
                {
                    int v1Index = triangles[j];
                    int v2Index = triangles[j + 1];
                    int v3Index = triangles[j + 2];
                    float3 v13d = Verticies[v1Index];
                    float3 v23d = Verticies[v2Index];
                    float3 v33d = Verticies[v3Index];
                    float2 v1 = new float2(v13d.x, v13d.z);
                    float2 v2 = new float2(v23d.x, v23d.z);
                    float2 v3 = new float2(v33d.x, v33d.z);

                    //Point inside triangle check
                    if(!IsPOintInsideTriangle(agentPos2, v1, v2, v3)) { continue; }
                    //Height check
                    float newHeight = GetNewHeight(v13d, v23d, v33d, agentPos3);
                    desiredHeight = math.select(desiredHeight, newHeight, newHeight > desiredHeight);
                }
            }
        }
        desiredHeight = math.select(desiredHeight, 1f, desiredHeight == float.MinValue);
        float3 agentPositionChange = AgentPositionChangeArray[index];
        agentPositionChange.y = desiredHeight - currentHeight;
        AgentPositionChangeArray[index] = agentPositionChange;
    }
    float GetNewHeight(float3 a3, float3 b3, float3 c3, float3 p3)
    {
        float2 a = new float2(a3.x, a3.z);
        float2 b = new float2(b3.x, b3.z);
        float2 c = new float2(c3.x, c3.z);
        float2 p = new float2(p3.x, p3.z);
        float2 v0 = b - a, v1 = c - a, v2 = p - a;
        float d00 = math.dot(v0, v0);
        float d01 = math.dot(v0, v1);
        float d11 = math.dot(v1, v1);
        float d20 = math.dot(v2, v0);
        float d21 = math.dot(v2, v1);
        float denom = d00 * d11 - d01 * d01;
        float v = (d11 * d20 - d01 * d21) / denom;
        float w = (d00 * d21 - d01 * d20) / denom;
        float u = 1.0f - v - w;
        return a3.y * u + b3.y * v + c3.y * w + 0.55f;
    }
    bool IsPOintInsideTriangle(float2 agentPos2, float2 v1, float2 v2, float2 v3)
    {
        float2 trigMins = math.min(math.min(v1, v2), v3);
        float2 trigMaxs = math.max(math.max(v1, v2), v3);
        if (agentPos2.x < trigMins.x || agentPos2.x > trigMaxs.x) { return false; }
        float2x2 line1 = new float2x2()
        {
            c0 = math.select(v2, v1, v1.x < v2.x),
            c1 = math.select(v1, v2, v1.x < v2.x),
        };
        bool line1ContainsX = agentPos2.x >= line1.c0.x && agentPos2.x <= line1.c1.x;
        float2x2 line2 = new float2x2()
        {
            c0 = math.select(v3, v2, v2.x < v3.x),
            c1 = math.select(v2, v3, v2.x < v3.x),
        };
        bool line2ContainsX = agentPos2.x >= line2.c0.x && agentPos2.x <= line2.c1.x;
        float2x2 line3 = new float2x2()
        {
            c0 = math.select(v1, v3, v3.x < v1.x),
            c1 = math.select(v3, v1, v3.x < v1.x),
        };
        bool line3ContainsX = agentPos2.x >= line3.c0.x && agentPos2.x <= line3.c1.x;
        float tLine1 = (agentPos2.x - line1.c0.x) / (line1.c1.x - line1.c0.x);
        float tLine2 = (agentPos2.x - line2.c0.x) / (line2.c1.x - line2.c0.x);
        float tLine3 = (agentPos2.x - line3.c0.x) / (line3.c1.x - line3.c0.x);
        float yLine1 = line1.c0.y + (line1.c1.y - line1.c0.y) * tLine1;
        float yLine2 = line2.c0.y + (line2.c1.y - line2.c0.y) * tLine2;
        float yLine3 = line3.c0.y + (line3.c1.y - line3.c0.y) * tLine3;
        float yMin = float.MaxValue;
        float yMax = float.MinValue;
        yMin = math.select(yMin, yLine1, yLine1 < yMin && line1ContainsX);
        yMin = math.select(yMin, yLine2, yLine2 < yMin && line2ContainsX);
        yMin = math.select(yMin, yLine3, yLine3 < yMin && line3ContainsX);
        yMax = math.select(yMax, yLine1, yLine1 > yMax && line1ContainsX);
        yMax = math.select(yMax, yLine2, yLine2 > yMax && line2ContainsX);
        yMax = math.select(yMax, yLine3, yLine3 > yMax && line3ContainsX);
        bool isPointInsideTriangle = agentPos2.y <= yMax && agentPos2.y >= yMin;
        return isPointInsideTriangle;
    }
}