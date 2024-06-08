using Unity.Mathematics;
using System.Collections.Generic;
using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using System.Diagnostics;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal class GoalIndexDebugMeshBuilder
    {
        FlowFieldNavigationManager _navManager;
        List<Mesh> _debugMeshes;
        List<Mesh> _cornerDebugMeshes;
        float2 _lastDebuggedGoal;
        float _lastDebuggedRange;
        int _lastPathIndex;

        internal GoalIndexDebugMeshBuilder(FlowFieldNavigationManager navManager)
        {
            _navManager = navManager;
            _debugMeshes = new List<Mesh>();
            _cornerDebugMeshes = new List<Mesh>();
            _lastPathIndex = -1;
        }

        internal void GetDebugMeshes(float2 goal, float goalRange, int pathIndex, out List<Mesh> debugMeshes, out List<Mesh> borderDebugMeshes)
        {
            bool isGoalPointChanged = !_lastDebuggedGoal.Equals(goal);
            bool isGoalRangeChanged = _lastDebuggedRange != goalRange;
            bool isPathIndexChanged = _lastPathIndex != pathIndex ||_lastPathIndex == -1;
            if (isGoalPointChanged || isGoalRangeChanged || isPathIndexChanged)
            {
                CreateDebugMeshes(goal, goalRange);
                _lastDebuggedGoal = goal;
                _lastDebuggedRange = goalRange;
                _lastPathIndex = pathIndex;
            }
            debugMeshes = _debugMeshes;
            borderDebugMeshes = _cornerDebugMeshes;
        }

        void CreateDebugMeshes(float2 goal, float goalRange)
        {
            _debugMeshes.Clear();
            _cornerDebugMeshes.Clear();
            const int RowAndColCountPerMesh = 200;
            NativeList<float3> verts = new NativeList<float3>(Allocator.Temp);
            NativeList<int> trigs = new NativeList<int>(Allocator.Temp);
            NativeList<int> borderTrigs = new NativeList<int>(Allocator.Temp);
            TriangleSpatialHashGrid trigSpatHashGrid = _navManager.FieldDataContainer.HeightMeshGenerator.GetTriangleSpatialHashGrid();
            NativeArray<float3> heightMeshVerts = _navManager.FieldDataContainer.HeightMeshGenerator.Verticies.AsArray();
            float rangeMinX = goal.x - goalRange;
            float rangeMaxX = goal.x + goalRange;
            float rangeMinY = goal.y - goalRange;
            float rangeMaxY = goal.y + goalRange;
            int2 rangeMinIndex = FlowFieldUtilities.PosTo2D(new float2(rangeMinX, rangeMinY), FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
            int2 rangeMaxIndex = FlowFieldUtilities.PosTo2D(new float2(rangeMaxX, rangeMaxY), FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
            rangeMinIndex = math.max(rangeMinIndex, 0);
            rangeMaxIndex = math.min(rangeMaxIndex, new int2(FlowFieldUtilities.FieldColAmount, FlowFieldUtilities.FieldRowAmount));
            for (int y = rangeMinIndex.y; y <= rangeMaxIndex.y; y += RowAndColCountPerMesh)
            {
                for (int x = rangeMinIndex.x; x <= rangeMaxIndex.x; x += RowAndColCountPerMesh)
                {
                    int2 meshStartTileIndex = new int2(x, y);
                    int2 meshEndTileIndex = math.min(meshStartTileIndex + RowAndColCountPerMesh - 1, rangeMaxIndex);
                    verts.Length = 0;
                    trigs.Length = 0;
                    borderTrigs.Length = 0;
                    ComputeMesh(ref verts, 
                        ref trigs,
                        ref borderTrigs,
                        trigSpatHashGrid,
                        heightMeshVerts,
                        goal, 
                        goalRange, 
                        FlowFieldUtilities.TileSize, 
                        meshStartTileIndex, 
                        meshEndTileIndex, 
                        FlowFieldUtilities.FieldGridStartPosition);
                    Mesh mesh = CreateMesh(verts.AsArray(), trigs.AsArray());
                    _debugMeshes.Add(mesh);
                    Mesh borderMesh = CreateMesh(verts.AsArray(), borderTrigs.AsArray());
                    _cornerDebugMeshes.Add(borderMesh);
                }
            }
        }
        Mesh CreateMesh(NativeArray<float3> verts, NativeArray<int> trigs)
        {
            Mesh mesh = new Mesh();
            mesh.Clear();
            mesh.SetVertices(verts);
            mesh.triangles = new int[0];
            mesh.RecalculateNormals();
            mesh.SetIndices(trigs, MeshTopology.Lines, 0);
            return mesh;
        }

        [BurstCompile]
        static void ComputeMesh(ref NativeList<float3> verts,
            ref NativeList<int> trigs,
            ref NativeList<int> borderTrigs,
            in TriangleSpatialHashGrid trigSpatHashGrid,
            in NativeArray<float3> heightMeshVerts,
            in float2 goal,
            in float range,
            in float tileSize, 
            in int2 startIndex, 
            in int2 endIndex, 
            in float2 fieldGridStartPos)
        {
            int indexRowAmount = endIndex.y - startIndex.y + 1;
            int indexColAmount = endIndex.x - startIndex.x + 1;
            int vertColAmount = indexColAmount + 1;

            //set verticies
            for (int y = startIndex.y; y <= endIndex.y + 1; y++)
            {
                for(int x = startIndex.x; x <= endIndex.x + 1; x++)
                {
                    int2 curIndex = new int2(x, y);
                    float2 vert2 = FlowFieldUtilities.IndexToStartPos(curIndex, tileSize, fieldGridStartPos);
                    float3 vert3 = new float3(vert2.x, 0, vert2.y);
                    verts.Add(vert3);
                }
            }

            //set vertex heights
            NativeArray<float3> vertsAsArray = verts.AsArray();
            HeightMeshImmediateQueryManager.SetHeightsAsYBurst(ref vertsAsArray, trigSpatHashGrid, heightMeshVerts);

            //Set triangles
            NativeHashSet<int> usedIndexMap = new NativeHashSet<int>(0, Allocator.Temp);
            for (int y = 0; y < indexRowAmount; y++)
            {
                for (int x = 0; x < indexColAmount; x++)
                {
                    int2 curLocalIndex = new int2(x, y);
                    int2 curFieldIndex = startIndex + curLocalIndex;
                    float2 indexCenterPos = FlowFieldUtilities.IndexToPos(curFieldIndex, tileSize, fieldGridStartPos);
                    if (math.distance(indexCenterPos, goal) > range) { continue; }
                    int curIndex1d = FlowFieldUtilities.To1D(curLocalIndex, indexColAmount);
                    int nIndex1d = curIndex1d + indexColAmount;
                    int eIndex1d = curIndex1d + 1;
                    int sIndex1d = curIndex1d - indexColAmount;
                    int wIndex1d = curIndex1d - 1;
                    int botLeftVertIndex = FlowFieldUtilities.To1D(curLocalIndex, vertColAmount);
                    int topLeftVertIndex = botLeftVertIndex + vertColAmount;
                    int topRightVertIndex = topLeftVertIndex + 1;
                    int botRightVertIndex = botLeftVertIndex + 1;
                    if (IsTileBorder(goal, range, curFieldIndex, tileSize, fieldGridStartPos))
                    {
                        borderTrigs.Add(topLeftVertIndex);
                        borderTrigs.Add(topRightVertIndex);
                        borderTrigs.Add(topRightVertIndex);
                        borderTrigs.Add(botRightVertIndex);
                        borderTrigs.Add(botLeftVertIndex);
                        borderTrigs.Add(botRightVertIndex);
                        borderTrigs.Add(botLeftVertIndex);
                        borderTrigs.Add(topLeftVertIndex);
                        usedIndexMap.Add(curIndex1d);
                        continue;
                    }
                    if (!usedIndexMap.Contains(nIndex1d))
                    {
                        trigs.Add(topLeftVertIndex);
                        trigs.Add(topRightVertIndex);
                    }
                    if (!usedIndexMap.Contains(eIndex1d))
                    {
                        trigs.Add(topRightVertIndex);
                        trigs.Add(botRightVertIndex);
                    }
                    if (!usedIndexMap.Contains(sIndex1d))
                    {
                        trigs.Add(botLeftVertIndex);
                        trigs.Add(botRightVertIndex);
                    }
                    if (!usedIndexMap.Contains(wIndex1d))
                    {
                        trigs.Add(botLeftVertIndex);
                        trigs.Add(topLeftVertIndex);
                    }
                    usedIndexMap.Add(curIndex1d);
                }
            }
        }
        static bool IsTileBorder(float2 goal, float goalRange, int2 tileGeneral2d, float tileSize, float2 fieldGridStartPos)
        {
            float2 tilePos = FlowFieldUtilities.IndexToPos(tileGeneral2d, tileSize, fieldGridStartPos);
            float dx = math.abs(goal.x - tilePos.x);
            float dy = math.abs(goal.y - tilePos.y);
            float goalRangeSq = goalRange * goalRange;
            float tileDistSq = dx * dx + dy * dy;
            float tileXIncDistSq = (dx + tileSize) * (dx + tileSize) + dy * dy;
            float tileYIncDistSq = dx * dx + (dy + tileSize) * (dy + tileSize);
            return tileDistSq <= goalRangeSq && (tileXIncDistSq > goalRangeSq || tileYIncDistSq > goalRangeSq);
        }
    }
}