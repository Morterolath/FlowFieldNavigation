using System.Collections.Generic;
using System.IO;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using System.Diagnostics;

namespace FlowFieldNavigation
{
    internal class FlowCalculationScheduler
    {
        FlowFieldNavigationManager _navigationManager;
        PathDataContainer _pathContainer;
        LOSIntegrationScheduler _losIntegrationScheduler;

        internal FlowCalculationScheduler(FlowFieldNavigationManager navigationManager, LOSIntegrationScheduler losIntegrationScheduler)
        {
            _navigationManager = navigationManager;
            _pathContainer = navigationManager.PathDataContainer;
            _losIntegrationScheduler = losIntegrationScheduler;
        }
        internal void DisposeAll()
        {
            _losIntegrationScheduler.DisposeAll();
            _losIntegrationScheduler = null;
        }
        internal JobHandle ScheduleFlow(NativeArray<FlowRequest> flowRequestsUnique)
        {
            NativeArray<JobHandle> tempHandleArray = new NativeArray<JobHandle>(flowRequestsUnique.Length, Allocator.Temp);
            for(int i = 0; i < flowRequestsUnique.Length; i++)
            {
                FlowRequest req = flowRequestsUnique[i];
                int pathIndex = req.PathIndex;
                PathfindingInternalData pathInternalData = _pathContainer.PathfindingInternalDataList[pathIndex];
                PathDestinationData destinationData = _pathContainer.PathDestinationDataList[pathIndex];
                float goalRange = _pathContainer.PathRanges[pathIndex];
                NativeArray<int> sectorToFlowStartTable = _pathContainer.SectorToFlowStartTables[pathIndex];
                UnsafeList<PathSectorState> sectorStateTable = _pathContainer.PathSectorStateTableList[pathIndex];
                CostField pickedCostField = _navigationManager.FieldDataContainer.GetCostFieldWithOffset(destinationData.Offset);
                int2 targetIndex = FlowFieldUtilities.PosTo2D(destinationData.Destination, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);

                //SCHEDULE INTEGRATION FIELDS
                NativeArray<int> sectorIndiciesToCalculateIntegration = pathInternalData.SectorIndiciesToCalculateIntegration.AsArray();
                NativeArray<int> sectorIndiciesToCalculateFlow = pathInternalData.SectorIndiciesToCalculateFlow.AsArray();

                IntegrationFieldJob integrationJob = new IntegrationFieldJob()
                {
                    TargetIndex = targetIndex,
                    Goal = destinationData.Destination,
                    GoalRange = goalRange,
                    FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                    TileSize = FlowFieldUtilities.TileSize,
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    FieldColAmount = FlowFieldUtilities.FieldColAmount,
                    FieldRowAmount = FlowFieldUtilities.FieldRowAmount,
                    SectorStateTable = sectorStateTable,
                    SectorFlowStartTable = sectorToFlowStartTable,
                    SectorIndiciesToCalculateIntegration = sectorIndiciesToCalculateIntegration,
                    CostField = pickedCostField.Costs,
                    IntegrationField = pathInternalData.IntegrationField.AsArray(),
                    SectorToWaveFrontsMap = pathInternalData.SectorToWaveFrontsMap,
                };
                JobHandle integrationHandle = integrationJob.Schedule();

                FlowFieldJob flowFieldJob = new FlowFieldJob()
                {
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorMatrixRowAmount = FlowFieldUtilities.SectorMatrixRowAmount,
                    SectorMatrixTileAmount = FlowFieldUtilities.SectorMatrixTileAmount,
                    SectorRowAmount = FlowFieldUtilities.SectorRowAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    TileSize = FlowFieldUtilities.TileSize,
                    FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                    FieldColAmount = FlowFieldUtilities.FieldColAmount,
                    FieldTileAmount = FlowFieldUtilities.FieldTileAmount,
                    SectorIndiciesToCalculateFlow = sectorIndiciesToCalculateFlow,
                    SectorToPicked = sectorToFlowStartTable,
                    PickedToSector = pathInternalData.PickedSectorList.AsArray(),
                    Costs = pickedCostField.Costs,
                    IntegrationField = pathInternalData.IntegrationField.AsArray(),
                    FlowFieldCalculationBuffer = pathInternalData.FlowFieldCalculationBuffer,
                };
                JobHandle flowFieldHandle = flowFieldJob.Schedule(integrationHandle);

                if (FlowFieldUtilities.DebugMode) { flowFieldHandle.Complete(); }
                tempHandleArray[i] = flowFieldHandle;
            }
            return JobHandle.CombineDependencies(tempHandleArray);
        }
    }


}