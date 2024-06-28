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

                NewPortalTraversalJob newPortalTraversalJob = new NewPortalTraversalJob()
                {
                    IslandSeed = islandSeed,
                    TileSize = FlowFieldUtilities.TileSize,
                    FieldColAmount = FlowFieldUtilities.FieldColAmount,
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                    SectorMatrixRowAmount = FlowFieldUtilities.SectorMatrixRowAmount,
                    PortalNodes = pickedFieldGraph.PortalNodes,
                    SecToWinPtrs = pickedFieldGraph.SecToWinPtrs,
                    WindowNodes = pickedFieldGraph.WindowNodes,
                    SourcePositions = pathRequestSource,
                    PorPtrs = pickedFieldGraph.PorToPorPtrs,
                    SectorNodes = pickedFieldGraph.SectorNodes,
                    Costs = pickedCostField.Costs,
                    PortalTraversalDataArray = porTravDataArray,
                    SourcePortalIndexList = portalTraversalData.SourcePortalIndexList,
                    IslandFields = pickedFieldGraph.IslandFields,
                    SectorStateTable = sectorStateTable,
                    Target = destinationIndex,
                    NewPortalSliceStartIndex = portalTraversalData.PathAdditionSequenceSliceStartIndex.Value,
                    PickedSectorIndicies = pathInternalData.PickedSectorList,
                    PortalSequenceSlices = portalTraversalData.PortalSequenceSlices,
                    PortalSequence = portalTraversalData.PortalSequence,
                    IntegrationField = pathInternalData.IntegrationField,
                    PossibleGoalSectors = alreadyConsideredGoalSectorIndexMap,
                    PickedPortalDataRecords = portalTraversalData.PickedPortalDataRecords,
                    LosRange = FlowFieldUtilities.LOSRange,
                    SectorWithinLosRange = pathInternalData.SectorWithinLOSState,
                    NewPickedSectorStartIndex = pathInternalData.PickedSectorList.Length,
                };
                JobHandle travHandle = newPortalTraversalJob.Schedule(dependency);

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