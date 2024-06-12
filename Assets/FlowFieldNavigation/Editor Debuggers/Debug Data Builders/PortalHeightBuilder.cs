using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;

namespace FlowFieldNavigation
{

    internal class PortalHeightBuilder
    {
        FlowFieldNavigationManager _navigationManager;
        NativeList<float> _heights;
        bool _isCreated;
        uint _lastFieldState;
        int _lastOffset;

        internal PortalHeightBuilder(FlowFieldNavigationManager navigationManager)
        {
            _navigationManager = navigationManager;
            _isCreated = false;
            _heights = new NativeList<float>(Allocator.Persistent);
            _lastFieldState = 0;
            _lastOffset = 0;
        }
        internal NativeArray<float> GetPortalHeights(int offset)
        {
            uint curFieldState = _navigationManager.FieldEditManager.FieldState;
            if (!_isCreated || _lastOffset != offset || _lastFieldState != curFieldState)
            {
                _lastOffset = offset;
                _lastFieldState = curFieldState;
                Create(offset);
            }
            return _heights.AsArray();
        }

        void Create(int offset)
        {
            _isCreated = true;
            _heights.Clear();

            float tileSize = FlowFieldUtilities.TileSize;
            float2 fieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition;
            NativeArray<PortalNode> portalNodes = _navigationManager.FieldDataContainer.GetFieldGraphWithOffset(offset).PortalNodes;
            NativeArray<float2> portalPositions = new NativeArray<float2>(portalNodes.Length, Allocator.TempJob);
            _heights.Length = portalNodes.Length;

            PortalNodeToPosJob portaltopos = new PortalNodeToPosJob()
            {
                FieldGridStartPos = fieldGridStartPos,
                TileSize = tileSize,
                Nodes = portalNodes,
                Positions = portalPositions,
            };
            portaltopos.Schedule(portalNodes.Length, 64).Complete();

            PointHeightCalculationJob pointHeight = new PointHeightCalculationJob()
            {
                TriangleSpatialHashGrid = _navigationManager.FieldDataContainer.HeightMeshGenerator.GetTriangleSpatialHashGrid(),
                HeightMeshVerts = _navigationManager.FieldDataContainer.HeightMeshGenerator.Verticies.AsArray(),
                Heights = _heights.AsArray(),
                Points = portalPositions,
            };
            pointHeight.Schedule(portalNodes.Length, 64).Complete();
            portalPositions.Dispose();
        }
    }
}