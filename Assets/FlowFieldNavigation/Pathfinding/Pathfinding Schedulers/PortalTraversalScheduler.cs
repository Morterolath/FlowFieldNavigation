using Codice.Client.BaseCommands.Download;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using System.Diagnostics;

namespace FlowFieldNavigation
{
    internal class PortalTraversalScheduler
    {
        FlowFieldNavigationManager _navigationManager;
        PathDataContainer _pathContainer;
        RequestedSectorCalculationScheduler _requestedSectorCalculationScheduler;
        PortalTraversalDataProvider _porTravDataProvider;

        internal PortalTraversalScheduler(FlowFieldNavigationManager navManager, RequestedSectorCalculationScheduler reqSecCalcScheduler)
        {
            _navigationManager = navManager;
            _pathContainer = _navigationManager.PathDataContainer;
            _requestedSectorCalculationScheduler = reqSecCalcScheduler;
            _porTravDataProvider = new PortalTraversalDataProvider(navManager);
        }
        internal void DisposeAll()
        {
            _requestedSectorCalculationScheduler.DisposeAll();
            _requestedSectorCalculationScheduler = null;
        }
        internal JobHandle SchedulePortalTraversalFor(NativeArray<PortalTraversalRequest> portalTraversalRequestsUnique, NativeArray<float2> sources)
        {
            NativeArray<JobHandle> handleArrayTemp = new NativeArray<JobHandle>(portalTraversalRequestsUnique.Length, Allocator.Temp);
            for(int i = 0; i < portalTraversalRequestsUnique.Length; i++)
            {
                PortalTraversalRequest req = portalTraversalRequestsUnique[i];
                int pathIndex = req.PathIndex;
                Slice pathReqSourceSlice = req.SourceSlice;

                PathfindingInternalData pathInternalData = _navigationManager.PathDataContainer.PathfindingInternalDataList[pathIndex];
                PathDestinationData destinationData = _pathContainer.PathDestinationDataList[pathIndex];
                PathPortalTraversalData portalTraversalData = _pathContainer.PathPortalTraversalDataList[pathIndex];
                UnsafeList<PathSectorState> sectorStateTable = _pathContainer.PathSectorStateTableList[pathIndex];
                NativeArray<OverlappingDirection> sectorOverlappingDirectionTable = _pathContainer.SectorOverlappingDirectionTableList[pathIndex];
                SectorBitArray sectorBitArray = _pathContainer.PathSectorBitArrays[pathIndex];
                NativeArray<int> sectorToFlowStartTable = _pathContainer.SectorToFlowStartTables[pathIndex];
                NativeHashMap<int,int> goalNeighborIndexToGoalIndexMap = _pathContainer.PathGoalNeighbourIndexToGoalIndexMaps[pathIndex];
                NativeList<int> goalTraversalDataFieldIndexList = _pathContainer.PathGoalTraversalDataFieldIndexLists[pathIndex];
                NativeHashSet<int> alreadyConsideredGoalSectorIndexMap = _pathContainer.PathAlreadyConsideredSectorIndexMaps[pathIndex];
                float goalRange = _pathContainer.PathRanges[pathIndex];
                int islandSeed = _pathContainer.PathIslandSeedsAsFieldIndicies[pathIndex];
                int2 destinationIndex = FlowFieldUtilities.PosTo2D(destinationData.Destination, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
                CostField pickedCostField = _navigationManager.FieldDataContainer.GetCostFieldWithOffset(destinationData.Offset);
                FieldGraph pickedFieldGraph = _navigationManager.FieldDataContainer.GetFieldGraphWithOffset(destinationData.Offset);
                portalTraversalData.PathAdditionSequenceSliceStartIndex.Value = portalTraversalData.PortalSequenceSlices.Length;
                portalTraversalData.NewPickedSectorStartIndex.Value = pathInternalData.PickedSectorList.Length;

                NativeArray<PortalTraversalData> porTravDataArray = _porTravDataProvider.GetAvailableData(out JobHandle dependency);
                NativeSlice<float2> pathRequestSource = new NativeSlice<float2>(sources, pathReqSourceSlice.Index, pathReqSourceSlice.Count);

                //Graph Reduction
                PortalReductionJob reductionJob = new PortalReductionJob()
                {
                    GoalPosition = destinationData.Destination,
                    GoalRange = goalRange,
                    IslandSeed = islandSeed,
                    TileSize = FlowFieldUtilities.TileSize,
                    GoalIndex = destinationIndex,
                    FieldColAmount = FlowFieldUtilities.FieldColAmount,
                    FieldRowAmount = FlowFieldUtilities.FieldRowAmount,
                    FieldTileSize = FlowFieldUtilities.TileSize,
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                    PickedToSector = pathInternalData.PickedSectorList,
                    PortalNodes = pickedFieldGraph.PortalNodes,
                    SecToWinPtrs = pickedFieldGraph.SecToWinPtrs,
                    WindowNodes = pickedFieldGraph.WindowNodes,
                    WinToSecPtrs = pickedFieldGraph.WinToSecPtrs,
                    SourcePositions = pathRequestSource,
                    PorPtrs = pickedFieldGraph.PorToPorPtrs,
                    SectorNodes = pickedFieldGraph.SectorNodes,
                    Costs = pickedCostField.Costs,
                    LocalDirections = _navigationManager.FieldDataContainer.GetSectorDirections(),
                    PortalTraversalDataArray = porTravDataArray,
                    SourcePortalIndexList = portalTraversalData.SourcePortalIndexList,
                    IslandFields = pickedFieldGraph.IslandFields,
                    SectorStateTable = sectorStateTable,
                    DijkstraStartIndicies = portalTraversalData.DiskstraStartIndicies,
                    GoalTraversalDataList = portalTraversalData.GoalDataList,
                    NewExploredPortalIndicies = portalTraversalData.NewExploredPortalIndicies,
                    PortalDataRecords = portalTraversalData.PortalDataRecords.AsArray(),
                    NewExploredUpdateSeedIndicies = portalTraversalData.NewPathUpdateSeedIndicies,
                    GoalNeighborIndexToGoalIndexMap = goalNeighborIndexToGoalIndexMap,
                    GoalTraversalDataFieldIndexList = goalTraversalDataFieldIndexList,
                    AlreadyConsideredGoalSectorIndicies = alreadyConsideredGoalSectorIndexMap,
                };
                JobHandle reductHandle = reductionJob.Schedule(dependency);
                if (FlowFieldUtilities.DebugMode) { reductHandle.Complete(); }

                //Graph Traversal
                PortalTraversalJob traversalJob = new PortalTraversalJob()
                {
                    FieldColAmount = FlowFieldUtilities.FieldColAmount,
                    Target = destinationIndex,
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorMatrixRowAmount = FlowFieldUtilities.SectorMatrixRowAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    LOSRange = FlowFieldUtilities.LOSRange,
                    DijkstraStartIndicies = portalTraversalData.DiskstraStartIndicies,
                    SectorWithinLOSState = pathInternalData.SectorWithinLOSState,
                    NewPickedSectorStartIndex = portalTraversalData.NewPickedSectorStartIndex,
                    NewPortalSliceStartIndex = portalTraversalData.PathAdditionSequenceSliceStartIndex.Value,
                    PickedSectorIndicies = pathInternalData.PickedSectorList,
                    PortalSequenceSlices = portalTraversalData.PortalSequenceSlices,
                    PortalNodes = pickedFieldGraph.PortalNodes,
                    WindowNodes = pickedFieldGraph.WindowNodes,
                    WinToSecPtrs = pickedFieldGraph.WinToSecPtrs,
                    PorPtrs = pickedFieldGraph.PorToPorPtrs,
                    SectorNodes = pickedFieldGraph.SectorNodes,
                    PortalSequence = portalTraversalData.PortalSequence,
                    IntegrationField = pathInternalData.IntegrationField, 
                    PortalTraversalDataArray = porTravDataArray,
                    SectorStateTable = sectorStateTable,
                    SourcePortalIndexList = portalTraversalData.SourcePortalIndexList,
                    NewExploredPortalIndicies = portalTraversalData.NewExploredPortalIndicies,
                    PortalDataRecords = portalTraversalData.PortalDataRecords,
                    GoalTraversalDataFieldIndexList = goalTraversalDataFieldIndexList,
                    GoalNeighborIndexToGoalIndexMap = goalNeighborIndexToGoalIndexMap,
                };
                JobHandle travHandle = traversalJob.Schedule(reductHandle);
                _porTravDataProvider.IncerimentPointer(travHandle);
                if (FlowFieldUtilities.DebugMode) { travHandle.Complete(); }

                //Active wave front submission
                ActivePortalSubmitJob submitJob = new ActivePortalSubmitJob()
                {
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorMatrixRowAmount = FlowFieldUtilities.SectorMatrixRowAmount,
                    SectorRowAmount = FlowFieldUtilities.SectorRowAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    FieldColAmount = FlowFieldUtilities.FieldColAmount,
                    TargetIndex2D = FlowFieldUtilities.PosTo2D(destinationData.Destination, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition),
                    SequenceSliceListStartIndex = portalTraversalData.PathAdditionSequenceSliceStartIndex.Value,

                    PortalEdges = pickedFieldGraph.PorToPorPtrs,
                    SectorToPicked = sectorToFlowStartTable,
                    PickedSectorIndicies = pathInternalData.PickedSectorList,
                    PortalSequence = portalTraversalData.PortalSequence,
                    PortalSequenceSlices = portalTraversalData.PortalSequenceSlices,
                    WinToSecPtrs = pickedFieldGraph.WinToSecPtrs,
                    PortalNodes = pickedFieldGraph.PortalNodes,
                    WindowNodes = pickedFieldGraph.WindowNodes,
                    SectorToWaveFrontsMap = pathInternalData.SectorToWaveFrontsMap,
                    NotActivatedPortals = pathInternalData.NotActivePortalList,
                    SectorStateTable = sectorStateTable,
                    NewSectorStartIndex = portalTraversalData.NewPickedSectorStartIndex,
                    SectorBitArray = sectorBitArray,
                    SectorOverlappingDirectionTable = sectorOverlappingDirectionTable,
                };
                JobHandle activeFrontSubmissionHandle = submitJob.Schedule(travHandle);
                if (FlowFieldUtilities.DebugMode) { activeFrontSubmissionHandle.Complete(); }
                handleArrayTemp[i] = activeFrontSubmissionHandle;
            }

            return JobHandle.CombineDependencies(handleArrayTemp);
        }
    }

}