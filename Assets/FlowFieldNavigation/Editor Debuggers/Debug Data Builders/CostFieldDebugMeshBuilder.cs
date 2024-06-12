using Unity.Collections;
using UnityEngine;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    internal class CostFieldDebugMeshBuilder
    {
        FlowFieldNavigationManager _navigationManager;
        bool _isCreated;
        List<Mesh> _debugMeshes;
        uint _lastFieldState;
        int _lastOffset;
        internal CostFieldDebugMeshBuilder(FlowFieldNavigationManager navigationManager)
        {
            _navigationManager = navigationManager;
            _debugMeshes = new List<Mesh>();
            _isCreated = false;
            _lastFieldState = 0;
            _lastOffset = 0;
        }
        internal List<Mesh> GetDebugMesh(int offset)
        {
            uint curFieldState = _navigationManager.FieldEditManager.FieldState;
            if (!_isCreated || _lastOffset != offset || _lastFieldState != curFieldState)
            {
                _lastOffset = offset;
                _lastFieldState = curFieldState;
                Create(offset);
            }
            return _debugMeshes;
        }
        internal void Create(int offset)
        {
            _isCreated = true;
            _debugMeshes.Clear();
            int fieldColAmount = FlowFieldUtilities.FieldColAmount;
            int fieldRowAmount = FlowFieldUtilities.FieldRowAmount;
            const int maxRowEachMesh = 100;
            const int maxColEachMesh = 100;
            NativeArray<byte> costs = _navigationManager.FieldDataContainer.GetCostFieldWithOffset(offset).Costs;
            for (int r = 0; r < fieldRowAmount; r += maxRowEachMesh)
            {
                for (int c = 0; c < fieldColAmount; c += maxColEachMesh)
                {
                    int2 startIndex = new int2(c, r);
                    int2 endIndex = new int2(math.min(c + maxColEachMesh - 1, fieldColAmount - 1), math.min(r + maxRowEachMesh - 1, fieldRowAmount - 1));

                    NativeList<Vector3> verts = new NativeList<Vector3>(Allocator.TempJob);
                    NativeList<int> trigs = new NativeList<int>(Allocator.TempJob);
                    CostFieldDebugMeshBuildJob debugMeshCalculation = new CostFieldDebugMeshBuildJob()
                    {
                        SectorColAmount = FlowFieldUtilities.SectorColAmount,
                        SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                        SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                        FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                        TileSize = FlowFieldUtilities.TileSize,
                        FieldColAmount = FlowFieldUtilities.FieldColAmount,
                        StartFieldIndex = startIndex,
                        EndFieldIndex = endIndex,
                        Costs = costs,
                        TriangleSpatialHashGrid = _navigationManager.FieldDataContainer.HeightMeshGenerator.GetTriangleSpatialHashGrid(),
                        HeightMeshVerts = _navigationManager.FieldDataContainer.HeightMeshGenerator.Verticies.AsArray(),
                        Trigs = trigs,
                        Verts = verts,
                    };
                    debugMeshCalculation.Schedule().Complete();
                    if (verts.Length >= 3)
                    {
                        Mesh mesh = CreateMesh(verts, trigs);
                        _debugMeshes.Add(mesh);
                    }
                    verts.Dispose();
                    trigs.Dispose();
                }
            }
        }
        Mesh CreateMesh(NativeList<Vector3> verts, NativeList<int> trigs)
        {
            Mesh mesh = new Mesh();
            mesh.Clear();
            mesh.vertices = verts.AsArray().ToArray();
            mesh.triangles = trigs.AsArray().ToArray();
            mesh.RecalculateNormals();
            return mesh;
        }
    }

}
