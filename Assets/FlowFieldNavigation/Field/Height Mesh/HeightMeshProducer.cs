﻿using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using System.Diagnostics;

namespace FlowFieldNavigation
{
    internal class HeightMeshProducer
    {
        internal NativeList<float3> Verticies;
        internal NativeList<int> Triangles;
        NativeList<UnsafeList<HashTile>> SpatialHashGrids;
        NativeHashMap<float, int> TileSizeToGridIndex;
        NativeHashMap<int, float> GridIndexToTileSize;
        internal HeightMeshProducer()
        {
            Verticies = new NativeList<float3>(Allocator.Persistent);
            Triangles = new NativeList<int>(Allocator.Persistent);
            SpatialHashGrids = new NativeList<UnsafeList<HashTile>>(Allocator.Persistent);
            TileSizeToGridIndex = new NativeHashMap<float, int>(0, Allocator.Persistent);
            GridIndexToTileSize = new NativeHashMap<int, float>(0, Allocator.Persistent);
        }
        internal void GenerateHeightMesh(NativeArray<float3> surfaceMeshVerticiesInput, NativeArray<int> surfaceMeshTrianglesInput)
        {
            //Eliminate wrong normals
            TriangleNormalTestJob heightMapJob = new TriangleNormalTestJob()
            {
                UpDirection = new float3(0, 1f, 0f),
                InputTriangles = surfaceMeshTrianglesInput,
                InputVertecies = surfaceMeshVerticiesInput,
                OutputTriangles = Triangles,
                OutputVerticies = Verticies,
            };
            heightMapJob.Schedule().Complete();

            NativeReference<float2> baseTranslation = new NativeReference<float2>(0, Allocator.TempJob);
            HeightMeshStartPositionDeterminationJob meshStartPosJob = new HeightMeshStartPositionDeterminationJob()
            {
                BaseTranslationOut = baseTranslation,
                Verticies = Verticies.AsArray(),
            };
            meshStartPosJob.Schedule().Complete();
            FlowFieldUtilities.HeightMeshStartPosition = baseTranslation.Value;
            baseTranslation.Dispose();


            //Get grid tile sizes
            NativeList<float> gridTileSizes = new NativeList<float>(Allocator.TempJob);
            TriangleSpatialHashingTileSizeCalculationJob spatialHashingTileSizeCalculation = new TriangleSpatialHashingTileSizeCalculationJob()
            {
                BaseSpatialGridSize = FlowFieldUtilities.BaseTriangleSpatialGridSize,
                Triangles = Triangles.AsArray(),
                Verticies = Verticies.AsArray(),
                GridTileSizes = gridTileSizes,
            };
            spatialHashingTileSizeCalculation.Schedule().Complete();

            //Create grid according to grid tile sizes
            CreateHashGrids(gridTileSizes.AsArray().AsReadOnly());
            gridTileSizes.Dispose();

            //Submit triangles to spatial hashing
            NativeList<int> newTriangles = new NativeList<int>(Allocator.Persistent);
            SpatialHashingTriangleSubmissionJob spatialHashingTriangleSubmission = new SpatialHashingTriangleSubmissionJob()
            {
                HeightMeshStartPos = FlowFieldUtilities.HeightMeshStartPosition,
                BaseSpatialGridSize = FlowFieldUtilities.BaseTriangleSpatialGridSize,
                FieldHorizontalSize = FlowFieldUtilities.TileSize * FlowFieldUtilities.FieldColAmount,
                FieldVerticalSize = FlowFieldUtilities.TileSize * FlowFieldUtilities.FieldRowAmount,
                SpatialHashGrids = SpatialHashGrids,
                TileSizeToGridIndex = TileSizeToGridIndex,
                Triangles = Triangles.AsArray(),
                Verticies = Verticies.AsArray(),
                NewTriangles = newTriangles,
            };
            spatialHashingTriangleSubmission.Schedule().Complete();
            Triangles.Dispose();
            Triangles = newTriangles;
        }
        internal TriangleSpatialHashGrid GetTriangleSpatialHashGrid()
        {
            return new TriangleSpatialHashGrid()
            {
                HeightMapStartPosition = FlowFieldUtilities.HeightMeshStartPosition,
                BaseSpatialGridSize = FlowFieldUtilities.BaseTriangleSpatialGridSize,
                FieldHorizontalSize = FlowFieldUtilities.FieldColAmount * FlowFieldUtilities.TileSize,
                FieldVerticalSize = FlowFieldUtilities.FieldRowAmount * FlowFieldUtilities.TileSize,
                GridIndexToTileSize = GridIndexToTileSize,
                HashedTriangles = Triangles.AsArray(),
                TriangleHashGrids = SpatialHashGrids.AsArray(),
            };
        }
        internal void DisposeAll()
        {
            Verticies.Dispose();
            Triangles.Dispose();
            for (int i = 0; i < SpatialHashGrids.Length; i++)
            {
                UnsafeList<HashTile> hashTiles = SpatialHashGrids[i];
                hashTiles.Dispose();
                SpatialHashGrids[i] = hashTiles;
            }
            SpatialHashGrids.Dispose();
            TileSizeToGridIndex.Dispose();
            GridIndexToTileSize.Dispose();
        }
        void CreateHashGrids(NativeArray<float>.ReadOnly gridTileSizes)
        {
            float fieldMinXIncluding = FlowFieldUtilities.FieldMinXIncluding;
            float fieldMinYIncluding = FlowFieldUtilities.FieldMinYIncluding;
            float fieldMaxXExcluding = FlowFieldUtilities.FieldMaxXExcluding;
            float fieldMaxYExcluding = FlowFieldUtilities.FieldMaxYExcluding;
            float fieldHorizontalSize = fieldMaxXExcluding - fieldMinXIncluding;
            float fieldVerticalSize = fieldMaxYExcluding - fieldMinYIncluding;
            for (int i = 0; i < gridTileSizes.Length; i++)
            {
                float tileSize = gridTileSizes[i];
                int rowAmount = (int)math.ceil(fieldVerticalSize / tileSize);
                int colAmount = (int)math.ceil(fieldHorizontalSize / tileSize);
                UnsafeList<HashTile> hashTiles = new UnsafeList<HashTile>(rowAmount * colAmount, Allocator.Persistent, NativeArrayOptions.ClearMemory);
                hashTiles.Length = rowAmount * colAmount;
                SpatialHashGrids.Add(hashTiles);
                TileSizeToGridIndex.Add(tileSize, SpatialHashGrids.Length - 1);
                GridIndexToTileSize.Add(SpatialHashGrids.Length - 1, tileSize);
            }
        }

    }

}
