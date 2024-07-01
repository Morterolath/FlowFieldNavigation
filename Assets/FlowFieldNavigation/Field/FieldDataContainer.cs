using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

namespace FlowFieldNavigation
{

    internal class FieldDataContainer
    {
        internal ObstacleContainer ObstacleContainer { get; private set; }
        internal HeightMeshProducer HeightMeshGenerator { get; private set; }
        internal NavigationVolumeSystem NavigationVolumeSystem { get; private set; }
        CostFieldProducer _costFieldProducer;
        FieldGraphProducer _fieldGraphProducer;
        internal FieldDataContainer(NativeArray<float3> surfaceMeshVerticies, NativeArray<int> surfaceMeshTriangles)
        {
            _costFieldProducer = new CostFieldProducer();
            _fieldGraphProducer = new FieldGraphProducer();
            ObstacleContainer = new ObstacleContainer();
            HeightMeshGenerator = new HeightMeshProducer();
            HeightMeshGenerator.GenerateHeightMesh(surfaceMeshVerticies, surfaceMeshTriangles);
            NavigationVolumeSystem = new NavigationVolumeSystem();
        }
        internal void CreateField(NativeArray<StaticObstacle> staticObstacles,
            float voxelHorizontalSize,
            float voxelVerticalSize,
            float maxSurfaceHeightDifference,
            float maxWalkableHeight)
        {
            NativeArray<byte> baseCostField = new NativeArray<byte>(FlowFieldUtilities.FieldTileAmount, Allocator.TempJob);
            for (int i = 0; i < baseCostField.Length; i++) { baseCostField[i] = 1; }

            NativeArray<float3> heightMeshVerts = HeightMeshGenerator.Verticies.AsArray();
            NativeArray<int> heightMeshTrigs = HeightMeshGenerator.Triangles.AsArray();
            NavigationVolumeSystem.AnalyzeVolume(heightMeshVerts, heightMeshTrigs, staticObstacles, voxelHorizontalSize, voxelVerticalSize, maxSurfaceHeightDifference, maxWalkableHeight, baseCostField);
            _costFieldProducer.ProduceCostFields(FlowFieldUtilities.MaxCostFieldOffset, baseCostField);
            _fieldGraphProducer.ProduceFieldGraphs(_costFieldProducer.GetAllCostFields());
            baseCostField.Dispose();
        }
        internal NativeArray<UnsafeListReadOnly<PortalNode>> GetAllPortalNodeArrays(Allocator allocator)
        {
            return _fieldGraphProducer.GetAllPortalNodeArrays(allocator);
        }
        internal NativeArray<UnsafeListReadOnly<SectorNode>> GetAllSectorNodeArrays(Allocator allocator)
        {
            return _fieldGraphProducer.GetAllSectorNodeArrays(allocator);
        }
        internal NativeArray<UnsafeListReadOnly<WindowNode>> GetAllWindowNodeArrays(Allocator allocator)
        {
            return _fieldGraphProducer.GetAllWindowNodeArrays(allocator);
        }
        internal NativeArray<UnsafeListReadOnly<int>> GetAllSecToWinPtrArrays(Allocator allocator)
        {
            return _fieldGraphProducer.GetAllSecToWinPtrArrays(allocator);
        }
        internal FieldGraph GetFieldGraphWithOffset(int offset)
        {
            return _fieldGraphProducer.GetFieldGraphWithOffset(offset);
        }
        internal CostField GetCostFieldWithOffset(int offset)
        {
            return _costFieldProducer.GetCostFieldWithOffset(offset);
        }
        internal FieldGraph[] GetAllFieldGraphs()
        {
            return _fieldGraphProducer.GetAllFieldGraphs();
        }
        internal CostField[] GetAllCostFields()
        {
            return _costFieldProducer.GetAllCostFields();
        }
        internal NativeArray<SectorDirectionData> GetSectorDirections()
        {
            return _costFieldProducer.SectorDirections;
        }
        internal NativeArray<IslandFieldProcessor> GetAllIslandFieldProcessors(Allocator allocator)
        {
            return _fieldGraphProducer.GetAllIslandFieldProcessors(allocator);
        }
        internal UnsafeListReadOnly<byte>[] GetAllCostFieldCostsAsUnsafeListReadonly()
        {
            return _costFieldProducer.GetAllCostsAsUnsafeListReadonly();
        }
        internal void DisposeAll()
        {
            ObstacleContainer.DisposeAll();
            HeightMeshGenerator.DisposeAll();
            _costFieldProducer.DisposeAll();
            _fieldGraphProducer.DisposeAll();
        }
    }

}