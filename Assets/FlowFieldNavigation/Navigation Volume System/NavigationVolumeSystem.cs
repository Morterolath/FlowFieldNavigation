using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Collections.LowLevel.Unsafe;
using System.Diagnostics;

namespace FlowFieldNavigation
{
    internal class NavigationVolumeSystem
    {
        internal void AnalyzeVolume(NativeArray<float3> navigationSurfaceVerticies,
            NativeArray<int> navigationSurfaceTriangles,
            NativeArray<StaticObstacle> staticObstacles,
            float voxelHorizontalSize,
            float voxelVerticalSize,
            float maxSurfaceHeightDifference,
            float maxTileHeight,
            NativeArray<byte> costsToWriteOnTopOf)
        {
            float fieldHorizontalSize = FlowFieldUtilities.TileSize * FlowFieldUtilities.FieldColAmount;
            float fieldVerticalSize = FlowFieldUtilities.TileSize * FlowFieldUtilities.FieldRowAmount;
            const int sectorComponentVoxelCount = 10;
            float sectorHorizontalSize = sectorComponentVoxelCount * voxelHorizontalSize;
            float sectorVerticalSize = sectorComponentVoxelCount * voxelVerticalSize;

            NativeArray<HeightTile> HighestVoxelSaveTable = new NativeArray<HeightTile>(FlowFieldUtilities.FieldTileAmount, Allocator.TempJob);
            HighestVoxSaveTableResetJob highestVoxelReset = new HighestVoxSaveTableResetJob() { Table = HighestVoxelSaveTable };
            highestVoxelReset.Schedule().Complete();

            NativeReference<float3> volumeStartPos = new NativeReference<float3>(0, Allocator.TempJob);
            NativeReference<int> xAxisVoxelCount = new NativeReference<int>(0, Allocator.TempJob);
            NativeReference<int> yAxisVoxelCount = new NativeReference<int>(0, Allocator.TempJob);
            NativeReference<int> zAxisVoxelCount = new NativeReference<int>(0, Allocator.TempJob);
            NativeReference<int> xAxisSectorCount = new NativeReference<int>(0, Allocator.TempJob);
            NativeReference<int> yAxisSectorCount = new NativeReference<int>(0, Allocator.TempJob);
            NativeReference<int> zAxisSectorCount = new NativeReference<int>(0, Allocator.TempJob);

            NavVolumeBoundingJob volumeBounding = new NavVolumeBoundingJob()
            {
                FieldGridSize = new float2(fieldHorizontalSize, fieldVerticalSize),
                FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                VoxelHorizontalSize = voxelHorizontalSize,
                VoxelVerticalSize = voxelVerticalSize,
                SectorComponentVoxelCount = sectorComponentVoxelCount,
                SectorHorizontalSize = sectorHorizontalSize,
                SectorVerticalSize = sectorVerticalSize,
                Verticies = navigationSurfaceVerticies,
                StaticObstacles = staticObstacles,
                VolumeStartPos = volumeStartPos,
                XAxisVoxelCount = xAxisVoxelCount,
                YAxisVoxelCount = yAxisVoxelCount,
                ZAxisVoxelCount = zAxisVoxelCount,
                XAxisSectorCount = xAxisSectorCount,
                YAxisSectorCount = yAxisSectorCount,
                ZAxisSectorCount = zAxisSectorCount,
            };
            volumeBounding.Schedule().Complete();

            FlowFieldVolumeUtilities.XAxisVoxelCount = xAxisVoxelCount.Value;
            FlowFieldVolumeUtilities.YAxisVoxelCount = yAxisVoxelCount.Value;
            FlowFieldVolumeUtilities.ZAxisVoxelCount = zAxisVoxelCount.Value;
            FlowFieldVolumeUtilities.VolumeStartPos = volumeStartPos.Value;
            FlowFieldVolumeUtilities.VoxelHorizontalSize = voxelHorizontalSize;
            FlowFieldVolumeUtilities.VoxelVerticalSize = voxelVerticalSize;
            FlowFieldVolumeUtilities.XAxisSectorCount = xAxisSectorCount.Value;
            FlowFieldVolumeUtilities.YAxisSectorCount = yAxisSectorCount.Value;
            FlowFieldVolumeUtilities.ZAxisSectorCount = zAxisSectorCount.Value;
            FlowFieldVolumeUtilities.SectorComponentVoxelCount = sectorComponentVoxelCount;
            FlowFieldVolumeUtilities.SectorHorizontalSize = sectorHorizontalSize;
            FlowFieldVolumeUtilities.SectorVerticalSize = sectorVerticalSize;
            NativeHashSet<int> detectedSectorSet = new NativeHashSet<int>(0, Allocator.TempJob);

            NavSurfaceSectorDetectionJob surfaceSectorDetection = new NavSurfaceSectorDetectionJob()
            {
                VolumeStartPos = FlowFieldVolumeUtilities.VolumeStartPos,
                SecHorSize = FlowFieldVolumeUtilities.SectorHorizontalSize,
                SecVerSize = FlowFieldVolumeUtilities.SectorVerticalSize,
                XSecCount = FlowFieldVolumeUtilities.XAxisSectorCount,
                YSecCount = FlowFieldVolumeUtilities.YAxisSectorCount,
                ZSecCount = FlowFieldVolumeUtilities.ZAxisSectorCount,
                Trigs = navigationSurfaceTriangles,
                Verts = navigationSurfaceVerticies,
                SectorIndexSet = detectedSectorSet,
            };
            surfaceSectorDetection.Schedule().Complete();
            NativeHashMap<int, UnsafeBitArray> SurfaceVolumeBits = AllocateSectors(detectedSectorSet, Allocator.TempJob);

            NavSurfaceMarkingJob surfaceMarking = new NavSurfaceMarkingJob()
            {
                VolumeStartPos = FlowFieldVolumeUtilities.VolumeStartPos,
                VoxHorSize = FlowFieldVolumeUtilities.VoxelHorizontalSize,
                VoxVerSize = FlowFieldVolumeUtilities.VoxelVerticalSize,
                XVoxCount = FlowFieldVolumeUtilities.XAxisVoxelCount,
                YVoxCount = FlowFieldVolumeUtilities.YAxisVoxelCount,
                ZVoxCount = FlowFieldVolumeUtilities.ZAxisVoxelCount,
                SectorCompVoxCount = FlowFieldVolumeUtilities.SectorComponentVoxelCount,
                XSecCount = FlowFieldVolumeUtilities.XAxisSectorCount,
                ZSecCount = FlowFieldVolumeUtilities.ZAxisSectorCount,
                Trigs = navigationSurfaceTriangles,
                Verts = navigationSurfaceVerticies,
                SectorBits = SurfaceVolumeBits,
                HighestVoxelTable = HighestVoxelSaveTable,
            };
            surfaceMarking.Schedule().Complete();

            HeightTileStackCountJob heightTileStackCount = new HeightTileStackCountJob()
            {
                SecCompVoxCount = FlowFieldVolumeUtilities.SectorComponentVoxelCount,
                XSecCount = FlowFieldVolumeUtilities.XAxisSectorCount,
                ZSecCount = FlowFieldVolumeUtilities.ZAxisSectorCount,
                HeightTiles = HighestVoxelSaveTable,
                SectorBits = SurfaceVolumeBits,
            };
            heightTileStackCount.Schedule(HighestVoxelSaveTable.Length, 64).Complete();

            NativeList<int3> collidedIndicies = new NativeList<int3>(Allocator.TempJob);

            NavObstacleDetectionJob obstacleDetection = new NavObstacleDetectionJob()
            {
                SecCompVoxCount = FlowFieldVolumeUtilities.SectorComponentVoxelCount,
                VolStartPos = FlowFieldVolumeUtilities.VolumeStartPos,
                VoxHorSize = FlowFieldVolumeUtilities.VoxelHorizontalSize,
                VoxVerSize = FlowFieldVolumeUtilities.VoxelVerticalSize,
                XSecCount = FlowFieldVolumeUtilities.XAxisSectorCount,
                ZSecCount = FlowFieldVolumeUtilities.ZAxisSectorCount,
                XVoxCount = FlowFieldVolumeUtilities.XAxisVoxelCount,
                YVoxCount = FlowFieldVolumeUtilities.YAxisVoxelCount,
                ZVoxCount = FlowFieldVolumeUtilities.ZAxisVoxelCount,
                HighestVoxelsEachTile = HighestVoxelSaveTable,
                StaticObstacles = staticObstacles,
                CollidedIndicies = collidedIndicies,
            };
            obstacleDetection.Schedule().Complete();

            NativeList<int> slopeExcludedTrigs = new NativeList<int>(Allocator.TempJob);

            TriangleSlopeExclusionJob trigSlopeExclusion = new TriangleSlopeExclusionJob()
            {
                SlopeExcludedTriangles = slopeExcludedTrigs,
                Triangles = navigationSurfaceTriangles,
                Verticies = navigationSurfaceVerticies,
            };
            trigSlopeExclusion.Schedule().Complete();

            SlopeExcludedTriangleVoxelOccupationJob slopeExcludedTrigVoxelOccupation = new SlopeExcludedTriangleVoxelOccupationJob()
            {
                VolumeStartPos = FlowFieldVolumeUtilities.VolumeStartPos,
                VoxHorSize = FlowFieldVolumeUtilities.VoxelHorizontalSize,
                VoxVerSize = FlowFieldVolumeUtilities.VoxelVerticalSize,
                XVoxCount = FlowFieldVolumeUtilities.XAxisVoxelCount,
                YVoxCount = FlowFieldVolumeUtilities.YAxisVoxelCount,
                ZVoxCount = FlowFieldVolumeUtilities.ZAxisVoxelCount,
                Verts = navigationSurfaceVerticies,
                SlopeExcludedTrigs = slopeExcludedTrigs.AsArray(),
                CollidedIndicies = collidedIndicies,
                HighestVoxelsEachTile = HighestVoxelSaveTable,
            };
            slopeExcludedTrigVoxelOccupation.Schedule().Complete();

            CollidedIndexToCostField collisionToCost = new CollidedIndexToCostField()
            {
                FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                FieldTileSize = FlowFieldUtilities.TileSize,
                FieldColAmount = FlowFieldUtilities.FieldColAmount,
                FieldRowAmount = FlowFieldUtilities.FieldRowAmount,
                VolStartPos = FlowFieldVolumeUtilities.VolumeStartPos,
                VoxHorSize = FlowFieldVolumeUtilities.VoxelHorizontalSize,
                VoxVerSize = FlowFieldVolumeUtilities.VoxelVerticalSize,
                CollidedIndicies = collidedIndicies,
                Costs = costsToWriteOnTopOf,
            };
            collisionToCost.Schedule().Complete();

            HeightDifToCostField heightDifToCostEdit = new HeightDifToCostField()
            {
                VoxVerSize = FlowFieldVolumeUtilities.VoxelVerticalSize,
                MaxSurfaceHeightDifference = maxSurfaceHeightDifference,
                XVoxCount = FlowFieldVolumeUtilities.XAxisVoxelCount,
                ZVoxCount = FlowFieldVolumeUtilities.ZAxisVoxelCount,
                Costs = costsToWriteOnTopOf,
                HighestVoxelTable = HighestVoxelSaveTable,
            };
            heightDifToCostEdit.Schedule().Complete();

            TileHeightExclusionJob tileHeightExclusionJob = new TileHeightExclusionJob()
            {
                VolumeStartPos = FlowFieldVolumeUtilities.VolumeStartPos,
                VoxVerSize = FlowFieldVolumeUtilities.VoxelVerticalSize,
                CostField = costsToWriteOnTopOf,
                MaxTileHeight = maxTileHeight,
                TileHeights = HighestVoxelSaveTable,
            };
            tileHeightExclusionJob.Schedule(costsToWriteOnTopOf.Length, 64).Complete();

            //Deallocation
            NativeHashMap<int, UnsafeBitArray>.Enumerator surfaceVolumeBitMapEnumerator = SurfaceVolumeBits.GetEnumerator();
            while (surfaceVolumeBitMapEnumerator.MoveNext())
            {
                surfaceVolumeBitMapEnumerator.Current.Value.Dispose();
            }
            SurfaceVolumeBits.Dispose();
            HighestVoxelSaveTable.Dispose();
            volumeStartPos.Dispose();
            xAxisSectorCount.Dispose();
            yAxisSectorCount.Dispose();
            zAxisSectorCount.Dispose();
            xAxisVoxelCount.Dispose();
            yAxisVoxelCount.Dispose();
            zAxisVoxelCount.Dispose();
            detectedSectorSet.Dispose();
            collidedIndicies.Dispose();
            slopeExcludedTrigs.Dispose();
        }
        NativeHashMap<int, UnsafeBitArray> AllocateSectors(NativeHashSet<int> sectorToAllocate, Allocator allocator)
        {
            int sectorComponentVoxelCount = FlowFieldVolumeUtilities.SectorComponentVoxelCount;
            int sectorVoxelCount = sectorComponentVoxelCount * sectorComponentVoxelCount * sectorComponentVoxelCount;
            NativeHashMap<int, UnsafeBitArray> volumeBits = new NativeHashMap<int, UnsafeBitArray>(0, allocator);

            NativeHashSet<int>.Enumerator setEnumerator = sectorToAllocate.GetEnumerator();
            while (setEnumerator.MoveNext())
            {
                int curSector = setEnumerator.Current;
                UnsafeBitArray bitArray = new UnsafeBitArray(sectorVoxelCount, allocator);
                volumeBits.Add(curSector, bitArray);
            }
            return volumeBits;
        }
    }

}