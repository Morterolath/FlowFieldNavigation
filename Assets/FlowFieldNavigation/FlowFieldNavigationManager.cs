using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    public class FlowFieldNavigationManager : MonoBehaviour
    {
        public bool SimulationStarted { get; private set; }
        public FlowFieldNavigationInterface Interface { get; private set; }

        internal FieldDataContainer FieldDataContainer;
        internal PathDataContainer PathDataContainer;
        internal AgentDataContainer AgentDataContainer;
        internal FlockDataContainer FlockDataContainer;
        internal RequestAccumulator RequestAccumulator;
        internal PathfindingManager PathfindingManager;
        internal MovementManager MovementManager;
        internal CostFieldEditManager FieldEditManager;
        internal FieldImmediateQueryManager FieldImmediateQueryManager;
        internal HeightMeshImmediateQueryManager HeightMeshImmediateQueryManager;
        internal AgentDataReferanceManager AgentReferanceManager;
        internal AgentAdditionSystem AgentAdditionSystem;
        internal AgentRemovingSystem AgentRemovingSystem;
        internal AgentDataWriteSystem AgentDataWriteSystem;
        internal AgentDataReadSystem AgentDataReadSystem;
        internal PathUpdateSeedContainer PathUpdateSeedContainer;
        NavigationUpdater _navigationUpdater;
        void Awake()
        {
            Interface = new FlowFieldNavigationInterface(this);
        }
        void Update()
        {
            if (!SimulationStarted) { return; }
            _navigationUpdater.IntermediateUpdate();
            _navigationUpdater.RoutineFixedUpdate();
        }
        void FixedUpdate()
        {
            if (!SimulationStarted) { return; }
        }
        void LateUpdate()
        {
            if (!SimulationStarted) { return; }
            _navigationUpdater.IntermediateUpdate();
        }

        //Handle inappropriate col/row counts
        void SetFlowFieldUtilities(SimulationInputs startInputs)
        {
            const int sectorColAmount = 10;
            const int minRowCount = sectorColAmount * 2; //Does not work below. Can be improved.
            const int minColCount = sectorColAmount * 2; //Does not work below. Can be improved.
            int columnAmount = startInputs.ColumnCount;
            int rowAmount = startInputs.RowCount;
            columnAmount = columnAmount + (sectorColAmount - (columnAmount % sectorColAmount));
            rowAmount = rowAmount + (sectorColAmount - (rowAmount % sectorColAmount));
            rowAmount = math.select(rowAmount, minRowCount, rowAmount < minRowCount);
            columnAmount = math.select(columnAmount, minColCount, columnAmount < minColCount);

            float baseAgentSpatialGridSize = startInputs.BaseAgentSpatialGridSize;
            float tileSize = startInputs.TileSize;
            int sectorMatrixColAmount = columnAmount / sectorColAmount;
            int sectorMatrixRowAmount = rowAmount / sectorColAmount;
            FlowFieldUtilities.DebugMode = false;
            FlowFieldUtilities.SectorMatrixTileAmount = sectorMatrixColAmount * sectorMatrixRowAmount;
            FlowFieldUtilities.SectorMatrixRowAmount = sectorMatrixRowAmount;
            FlowFieldUtilities.SectorMatrixColAmount = sectorMatrixColAmount;
            FlowFieldUtilities.SectorColAmount = sectorColAmount;
            FlowFieldUtilities.SectorRowAmount = sectorColAmount;
            FlowFieldUtilities.SectorTileAmount = sectorColAmount * sectorColAmount;
            FlowFieldUtilities.TileSize = tileSize;
            FlowFieldUtilities.FieldColAmount = columnAmount;
            FlowFieldUtilities.FieldRowAmount = rowAmount;
            FlowFieldUtilities.FieldTileAmount = columnAmount * rowAmount;
            FlowFieldUtilities.BaseAgentSpatialGridSize = baseAgentSpatialGridSize;
            FlowFieldUtilities.BaseTriangleSpatialGridSize = startInputs.BaseTriangleSpatialGridSize;
            FlowFieldUtilities.MinAgentSize = 0;
            FlowFieldUtilities.MaxAgentSize = startInputs.MaxAgentRadius;
            FlowFieldUtilities.LOSRange = startInputs.LineOfSightRange;
            FlowFieldUtilities.FieldMinXIncluding = startInputs.FieldStartPositionXZ.x + 0.01f;
            FlowFieldUtilities.FieldMinYIncluding = startInputs.FieldStartPositionXZ.y + 0.01f;
            FlowFieldUtilities.FieldMaxXExcluding = startInputs.FieldStartPositionXZ.x + FlowFieldUtilities.FieldColAmount * FlowFieldUtilities.TileSize - 0.01f;
            FlowFieldUtilities.FieldMaxYExcluding = startInputs.FieldStartPositionXZ.y + FlowFieldUtilities.FieldRowAmount * FlowFieldUtilities.TileSize - 0.01f;
            FlowFieldUtilities.MaxCostFieldOffset = FlowFieldUtilities.RadiusToOffset(startInputs.MaxAgentRadius, tileSize);
            FlowFieldUtilities.FieldGridStartPosition = startInputs.FieldStartPositionXZ;
        }
        internal void StartSimulation(SimulationInputs startInputs)
        {
            SimulationStarted = true;
            SetFlowFieldUtilities(startInputs);
            FieldDataContainer = new FieldDataContainer(startInputs.NavigationSurfaceVerticies, startInputs.NavigationSurfaceTriangles);
            FieldDataContainer.CreateField(startInputs.StaticObstacles,
                startInputs.TileSize,
                startInputs.VerticalVoxelSize,
                startInputs.MaxSurfaceHeightDifference,
                startInputs.MaxWalkableHeight);
            PathUpdateSeedContainer = new PathUpdateSeedContainer();
            AgentDataContainer = new AgentDataContainer(this);
            AgentRemovingSystem = new AgentRemovingSystem(this);
            AgentDataWriteSystem = new AgentDataWriteSystem(this);
            PathDataContainer = new PathDataContainer(this);
            RequestAccumulator = new RequestAccumulator(this);
            PathfindingManager = new PathfindingManager(this);
            MovementManager = new MovementManager(AgentDataContainer, this);
            FieldEditManager = new CostFieldEditManager(this);
            _navigationUpdater = new NavigationUpdater(this, RequestAccumulator);
            FlockDataContainer = new FlockDataContainer();
            FieldImmediateQueryManager = new FieldImmediateQueryManager(this);
            HeightMeshImmediateQueryManager = new HeightMeshImmediateQueryManager(this);
            AgentReferanceManager = new AgentDataReferanceManager();
            AgentAdditionSystem = new AgentAdditionSystem(this);
            AgentDataReadSystem = new AgentDataReadSystem(this);
        }
        public void StopSimulation()
        {
            if (!SimulationStarted)
            {
                UnityEngine.Debug.Log("Request Denied. Simulation is already not started");
                return;
            }
            SimulationStarted = false;
            FieldDataContainer.DisposeAll();
            PathDataContainer.DisposeAll();
            AgentDataContainer.DisposeAll();
            FlockDataContainer.DisposeAll();
            RequestAccumulator.DisposeAll();
            _navigationUpdater.DisposeAll();
        }
        internal NativeArray<UnsafeList<HashTile>> GetSpatialHashGridArray()
        {
            return MovementManager.HashGridArray;
        }
        internal NativeArray<int> GetNormalToHashed()
        {
            return MovementManager.NormalToHashed.AsArray();
        }
        internal NativeArray<AgentMovementData> GetAgentMovementData()
        {
            return MovementManager.AgentMovementDataList.AsArray();
        }
        internal UnsafeListReadOnly<byte>[] GetAllCostFieldCostsAsUnsafeListReadonly()
        {
            return FieldDataContainer.GetAllCostFieldCostsAsUnsafeListReadonly();
        }
    }
}