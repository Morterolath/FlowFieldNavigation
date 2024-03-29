﻿using Unity.Collections;
using Unity.Jobs;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace FlowFieldNavigation
{
    internal class TileIslandDebugMeshBuilder
    {
        FlowFieldNavigationManager _navigationManager;
        List<Mesh> _debugMeshes;
        List<int> _debugMeshColorIndicies;
        bool _isCreated;
        uint _lastFieldState;
        int _lastOffset;

        internal TileIslandDebugMeshBuilder(FlowFieldNavigationManager navigationManager)
        {
            _navigationManager = navigationManager;
            _debugMeshes = new List<Mesh>();
            _debugMeshColorIndicies = new List<int>();
            _isCreated = false;
            _lastFieldState = 0;
            _lastOffset = 0;
        }

        internal void GetTileIslandDebugMesh(int offset, out List<Mesh> debugMeshes, out List<int> debugMeshColorIndicies)
        {
            uint curFieldState = _navigationManager.FieldEditManager.FieldState;
            if (!_isCreated || _lastOffset != offset || _lastFieldState != curFieldState)
            {
                _lastOffset = offset;
                _lastFieldState = curFieldState;
                CreateTileIslandDebugMesh(offset);
            }
            debugMeshes = _debugMeshes;
            debugMeshColorIndicies = _debugMeshColorIndicies;
        }
        void CreateTileIslandDebugMesh(int offset)
        {
            _isCreated = true;
            _debugMeshes.Clear();
            _debugMeshColorIndicies.Clear();
            IslandFieldProcessor islandFieldProcessor = _navigationManager.FieldDataContainer.GetFieldGraphWithOffset(offset).GetIslandFieldProcessor();
            NativeArray<byte> costField = _navigationManager.FieldDataContainer.GetCostFieldWithOffset(offset).Costs;
            NativeList<IndexIslandPair> indiciesWithValidIsland = new NativeList<IndexIslandPair>(Allocator.TempJob);
            IndiciesWithIslandIndexJob indiciesWithIslandIndexJob = new IndiciesWithIslandIndexJob()
            {
                FieldColAmount = FlowFieldUtilities.FieldColAmount,
                FieldRowAmount = FlowFieldUtilities.FieldRowAmount,
                SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                SectorColAmount = FlowFieldUtilities.SectorColAmount,
                SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                CostField = costField,
                IslandFieldProcessor = islandFieldProcessor,
                IndiciesWithValidIslands = indiciesWithValidIsland,
            };
            indiciesWithIslandIndexJob.Schedule().Complete();

            //Seperate indicies for each islands
            List<NativeList<int2>> tilesPerIsland = new List<NativeList<int2>>();
            Dictionary<int, int> islandToListIndex = new Dictionary<int, int>();
            for (int i = 0; i < indiciesWithValidIsland.Length; i++)
            {
                IndexIslandPair pair = indiciesWithValidIsland[i];
                if (islandToListIndex.TryGetValue(pair.Island, out int listIndex))
                {
                    tilesPerIsland[listIndex].Add(pair.FieldIndex);
                    continue;
                }
                NativeList<int2> newList = new NativeList<int2>(Allocator.TempJob);
                newList.Add(pair.FieldIndex);
                islandToListIndex.Add(pair.Island, tilesPerIsland.Count);
                tilesPerIsland.Add(newList);
            }

            const int tilePerMesh = 5000 / 4;
            NativeList<Vector3> verts = new NativeList<Vector3>(Allocator.TempJob);
            NativeList<int> trigs = new NativeList<int>(Allocator.TempJob);
            for (int i = 0; i < tilesPerIsland.Count; i++)
            {
                NativeList<int2> indicies = tilesPerIsland[i];
                for (int j = 0; j < indicies.Length; j += tilePerMesh)
                {
                    int sliceStart = j;
                    int sliceCount = math.min(indicies.Length - j, tilePerMesh);
                    NativeSlice<int2> indiciesForMesh = new NativeSlice<int2>(indicies.AsArray(), sliceStart, sliceCount);
                    IslandTileMeshBuildJob meshBuildJob = new IslandTileMeshBuildJob()
                    {
                        FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                        TileSize = FlowFieldUtilities.TileSize,
                        FieldColAmount = FlowFieldUtilities.FieldColAmount,
                        FieldRowAmount = FlowFieldUtilities.FieldRowAmount,
                        TriangleSpatialHashGrid = _navigationManager.FieldDataContainer.HeightMeshGenerator.GetTriangleSpatialHashGrid(),
                        HeightMeshVerts = _navigationManager.FieldDataContainer.HeightMeshGenerator.Verticies.AsArray(),
                        IndiciesToCreateMesh = indiciesForMesh,
                        Verts = verts,
                        Trigs = trigs,
                    };
                    meshBuildJob.Schedule().Complete();
                    if (verts.Length >= 3)
                    {
                        Mesh mesh = CreateMesh(verts.AsArray(), trigs.AsArray());
                        _debugMeshes.Add(mesh);
                        _debugMeshColorIndicies.Add(i);
                        verts.Clear();
                        trigs.Clear();
                    }
                }
            }
            verts.Dispose();
            trigs.Dispose();
            for (int i = 0; i < tilesPerIsland.Count; i++)
            {
                tilesPerIsland[i].Dispose();
            }
            indiciesWithValidIsland.Dispose();
        }
        Mesh CreateMesh(NativeArray<Vector3> verts, NativeArray<int> trigs)
        {
            Mesh mesh = new Mesh();
            mesh.Clear();
            mesh.vertices = verts.ToArray();
            mesh.triangles = trigs.ToArray();
            mesh.RecalculateNormals();
            return mesh;
        }
    }


}