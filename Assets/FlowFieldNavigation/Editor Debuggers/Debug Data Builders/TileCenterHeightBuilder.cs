﻿
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using static UnityEngine.ParticleSystem;

namespace FlowFieldNavigation
{
    internal class TileCenterHeightBuilder
    {
        FlowFieldNavigationManager _navigationManager;
        NativeArray<float> _tileCenterHeights;
        bool _isCreated;
        internal TileCenterHeightBuilder(FlowFieldNavigationManager navigationManager)
        {
            _navigationManager = navigationManager;
            _isCreated = false;
        }
        internal NativeArray<float> GetTileCenterHeights()
        {
            if (!_isCreated)
            {
                Create();
            }
            return _tileCenterHeights;
        }

        void Create()
        {
            _isCreated = true;
            if (!_tileCenterHeights.IsCreated) { _tileCenterHeights = new NativeArray<float>(FlowFieldUtilities.FieldTileAmount, Allocator.Persistent); }

            float tileSize = FlowFieldUtilities.TileSize;
            float2 fieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition;
            NativeArray<float2> tileCenters = new NativeArray<float2>(FlowFieldUtilities.FieldTileAmount, Allocator.TempJob);
            int tileCentersIndex = 0;
            for (int r = 0; r < FlowFieldUtilities.FieldRowAmount; r++)
            {
                for (int c = 0; c < FlowFieldUtilities.FieldColAmount; c++)
                {
                    int2 index2d = new int2(c, r);
                    tileCenters[tileCentersIndex++] = FlowFieldUtilities.IndexToPos(index2d, tileSize, fieldGridStartPos);
                }
            }

            PointHeightCalculationJob poitnHeights = new PointHeightCalculationJob()
            {
                TriangleSpatialHashGrid = _navigationManager.FieldDataContainer.HeightMeshGenerator.GetTriangleSpatialHashGrid(),
                HeightMeshVerts = _navigationManager.FieldDataContainer.HeightMeshGenerator.Verticies.AsArray(),
                Heights = _tileCenterHeights,
                Points = tileCenters,
            };
            poitnHeights.Schedule(tileCenters.Length, 64).Complete();
            tileCenters.Dispose();
        }
    }

}
