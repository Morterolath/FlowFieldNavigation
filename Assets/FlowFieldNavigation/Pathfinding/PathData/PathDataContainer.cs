using Codice.Client.BaseCommands.Download;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEditor.PackageManager.Requests;

namespace FlowFieldNavigation
{
    internal class PathDataContainer
    {
        internal List<NativeHashMap<int, int>> PathGoalNeighbourIndexToGoalIndexMaps;
        internal NativeList<float> PathRanges;
        internal NativeList<float> PathDesiredRanges;
        internal NativeList<FlowData> ExposedFlowData;
        internal NativeList<bool> ExposedLosData;
        internal PathSectorToFlowStartMapper SectorFlowStartMap;
        internal NativeList<float2> ExposedPathDestinations;
        internal NativeList<int> ExposedPathFlockIndicies;
        internal NativeList<float> ExposedPathReachDistanceCheckRanges;
        internal NativeList<PathState> ExposedPathStateList;
        internal NativeList<bool> ExposedPathAgentStopFlagList;
        internal List<NativeArray<int>> SectorToFlowStartTables;
        internal List<PathfindingInternalData> PathfindingInternalDataList;
        internal NativeList<UnsafeList<PathSectorState>> PathSectorStateTableList;
        internal NativeList<PathDestinationData> PathDestinationDataList;
        internal NativeList<PathRoutineData> PathRoutineDataList;
        internal NativeList<SectorBitArray> PathSectorBitArrays;
        internal List<PathPortalTraversalData> PathPortalTraversalDataList;
        internal NativeList<int> PathFlockIndicies;
        internal NativeList<int> PathSubscriberCounts;
        internal List<NativeArray<OverlappingDirection>> SectorOverlappingDirectionTableList;
        internal NativeList<int> RemovedExposedFlowAndLosIndicies;
        internal List<NativeList<int>> PathGoalTraversalDataFieldIndexLists;
        internal List<NativeHashSet<int>> PathAlreadyConsideredSectorIndexMaps;
        internal NativeList<int> PathIslandSeedsAsFieldIndicies;
        internal NativeParallelMultiHashMap<int, int> PathIndexToUpdateSeedsMap;
        internal NativeList<int> UnusedPathIndexList;
        internal NativeParallelMultiHashMap<int, int> PathIndexToGoalSectorsMap;

        FieldDataContainer _fieldProducer;
        PathPreallocator _preallocator;
        internal PathDataContainer(FlowFieldNavigationManager navigationManager)
        {
            _fieldProducer = navigationManager.FieldDataContainer;
            PathfindingInternalDataList = new List<PathfindingInternalData>();
            _preallocator = new PathPreallocator(_fieldProducer, FlowFieldUtilities.SectorTileAmount, FlowFieldUtilities.SectorMatrixTileAmount);
            UnusedPathIndexList = new NativeList<int>(Allocator.Persistent);
            PathSubscriberCounts = new NativeList<int>(Allocator.Persistent);
            PathSectorStateTableList = new NativeList<UnsafeList<PathSectorState>>(Allocator.Persistent);
            PathPortalTraversalDataList = new List<PathPortalTraversalData>();
            PathDestinationDataList = new NativeList<PathDestinationData>(Allocator.Persistent);
            ExposedPathStateList = new NativeList<PathState>(Allocator.Persistent);
            PathRoutineDataList = new NativeList<PathRoutineData>(Allocator.Persistent);
            PathSectorBitArrays = new NativeList<SectorBitArray>(Allocator.Persistent);
            SectorToFlowStartTables = new List<NativeArray<int>>();

            ExposedPathDestinations = new NativeList<float2>(Allocator.Persistent);
            PathFlockIndicies = new NativeList<int>(Allocator.Persistent);
            ExposedPathFlockIndicies = new NativeList<int>(Allocator.Persistent);
            ExposedPathReachDistanceCheckRanges = new NativeList<float>(Allocator.Persistent);
            ExposedPathAgentStopFlagList = new NativeList<bool>(Allocator.Persistent);
            SectorOverlappingDirectionTableList = new List<NativeArray<OverlappingDirection>>();

            RemovedExposedFlowAndLosIndicies = new NativeList<int>(Allocator.Persistent);
            ExposedFlowData = new NativeList<FlowData>(Allocator.Persistent);
            SectorFlowStartMap = new PathSectorToFlowStartMapper(0, Allocator.Persistent);
            ExposedLosData = new NativeList<bool>(Allocator.Persistent);
            PathRanges = new NativeList<float>(Allocator.Persistent);
            PathDesiredRanges = new NativeList<float>(Allocator.Persistent);
            PathGoalNeighbourIndexToGoalIndexMaps = new List<NativeHashMap<int, int>>();
            PathGoalTraversalDataFieldIndexLists = new List<NativeList<int>>();
            PathAlreadyConsideredSectorIndexMaps = new List<NativeHashSet<int>>();
            PathIslandSeedsAsFieldIndicies = new NativeList<int>(Allocator.Persistent);
            PathIndexToUpdateSeedsMap = new NativeParallelMultiHashMap<int, int>(0, Allocator.Persistent);
            PathIndexToGoalSectorsMap = new NativeParallelMultiHashMap<int, int>(0, Allocator.Persistent);
        }
        internal void DisposeAll()
        {
            if (ExposedPathDestinations.IsCreated) { ExposedPathDestinations.Dispose(); }
            if (ExposedPathFlockIndicies.IsCreated) { ExposedPathFlockIndicies.Dispose(); }
            if (ExposedPathReachDistanceCheckRanges.IsCreated) { ExposedPathReachDistanceCheckRanges.Dispose(); }
            if (ExposedPathStateList.IsCreated) { ExposedPathStateList.Dispose(); }
            if (ExposedPathAgentStopFlagList.IsCreated) { ExposedPathAgentStopFlagList.Dispose(); }
            if (PathSectorStateTableList.IsCreated) { PathSectorStateTableList.Dispose(); }
            if (PathDestinationDataList.IsCreated) { PathDestinationDataList.Dispose(); }
            if (PathRoutineDataList.IsCreated) { PathRoutineDataList.Dispose(); }
            if (PathSectorBitArrays.IsCreated) { PathSectorBitArrays.Dispose(); }
            if (PathFlockIndicies.IsCreated) { PathFlockIndicies.Dispose(); }
            if (PathSubscriberCounts.IsCreated) { PathSubscriberCounts.Dispose(); }
        }
        internal void Update()
        {
            const int maxDeallocationPerFrame = int.MaxValue;
            int deallcoated = 0;
            for (int i = 0; i < PathfindingInternalDataList.Count; i++)
            {
                PathfindingInternalData internalData = PathfindingInternalDataList[i];
                PathState pathState = ExposedPathStateList[i];
                if (pathState == PathState.Removed) { continue; }
                int subsciriber = PathSubscriberCounts[i];
                if (subsciriber == 0)
                {
                    if(deallcoated >= maxDeallocationPerFrame) { break; }
                    deallcoated++;
                    UnsafeList<PathSectorState> sectorStateTable = PathSectorStateTableList[i];
                    PathPortalTraversalData portalTraversalData = PathPortalTraversalDataList[i];
                    SectorBitArray sectorBitArray = PathSectorBitArrays[i];
                    NativeArray<OverlappingDirection> sectorOverlappingDirections = SectorOverlappingDirectionTableList[i];
                    NativeArray<int> sectorToFlowStartTable = SectorToFlowStartTables[i];
                    RemoveFromPathSectorToFlowStartMapper(internalData.PickedSectorList.AsArray(), i);
                    internalData.DynamicArea.SectorFlowStartCalculationBuffer.Dispose();
                    internalData.DynamicArea.FlowFieldCalculationBuffer.Dispose();
                    sectorOverlappingDirections.Dispose();
                    portalTraversalData.GoalDataList.Dispose();
                    internalData.SectorToWaveFrontsMap.Dispose();
                    internalData.IntegrationField.Dispose();
                    internalData.LOSCalculatedFlag.Dispose();
                    internalData.FlowFieldCalculationBuffer.Dispose();
                    sectorToFlowStartTable.Dispose();
                    portalTraversalData.NewPathUpdateSeedIndicies.Dispose();
                    portalTraversalData.PickedPortalDataRecords.Dispose();
                    PathIndexToUpdateSeedsMap.Remove(i);
                    PathGoalNeighbourIndexToGoalIndexMaps[i].Dispose();
                    PathGoalTraversalDataFieldIndexLists[i].Dispose();
                    ExposedPathStateList[i] = PathState.Removed;
                    PathAlreadyConsideredSectorIndexMaps[i].Dispose();
                    UnusedPathIndexList.Add(i);
                    PreallocationPack preallocations = new PreallocationPack()
                    {
                        PickedToSector = internalData.PickedSectorList,
                        PortalSequence = portalTraversalData.PortalSequence,
                        SourcePortalIndexList = portalTraversalData.SourcePortalIndexList,
                        TargetSectorPortalIndexList = portalTraversalData.TargetSectorPortalIndexList,
                        PortalTraversalFastMarchingQueue = internalData.PortalTraversalQueue,
                        SectorStateTable = sectorStateTable,
                        SectorStartIndexListToCalculateFlow = internalData.SectorIndiciesToCalculateFlow,
                        SectorStartIndexListToCalculateIntegration = internalData.SectorIndiciesToCalculateIntegration,
                        NotActivePortalList = internalData.NotActivePortalList,
                        NewPickedSectorStartIndex = portalTraversalData.NewPickedSectorStartIndex,
                        PathAdditionSequenceBorderStartIndex = portalTraversalData.PathAdditionSequenceSliceStartIndex,
                        DynamicAreaIntegrationField = internalData.DynamicArea.IntegrationField,
                        SectorsWithinLOSState = internalData.SectorWithinLOSState,
                        SectorBitArray = sectorBitArray,
                        DijkstraStartIndicies = portalTraversalData.DiskstraStartIndicies,
                    };
                    _preallocator.SendPreallocationsBack(ref preallocations);
                }
            }
            _preallocator.CheckForDeallocations();
        }
        void RemoveFromPathSectorToFlowStartMapper(NativeArray<int> pickedSectorList, int pathIndex)
        {
            for(int i = 0; i < pickedSectorList.Length; i++)
            {
                int sector = pickedSectorList[i];
                if(SectorFlowStartMap.TryGet(pathIndex, sector, out var flowStart))
                {
                    RemovedExposedFlowAndLosIndicies.Add(flowStart);
                    SectorFlowStartMap.TryRemove(pathIndex, sector);

                    for(int j = 0; j < FlowFieldUtilities.SectorTileAmount; j++)
                    {
                        ExposedFlowData[flowStart + j] = new FlowData();
                        ExposedLosData[flowStart + j] = false;
                    }
                }
                
            }
        }
        internal void ExposeBuffers(NativeArray<int> destinationUpdatedPathIndicies, NativeArray<int> newPathIndicies, NativeArray<int> expandedPathIndicies)
        {
            PathDataExposeJob dataExposeJob = new PathDataExposeJob()
            {
                DestinationUpdatedPathIndicies = destinationUpdatedPathIndicies,
                NewPathIndicies = newPathIndicies,
                ExpandedPathIndicies = expandedPathIndicies,
                ExposedPathDestinationList = ExposedPathDestinations,
                ExposedPathFlockIndicies = ExposedPathFlockIndicies,
                ExposedPathReachDistanceCheckRange = ExposedPathReachDistanceCheckRanges,
                PathStopFlagList = ExposedPathAgentStopFlagList,
                PathStateList = ExposedPathStateList,
                PathDestinationDataArray = PathDestinationDataList.AsArray(),
                PathFlockIndicies = PathFlockIndicies.AsArray(),
            };
            dataExposeJob.Schedule().Complete();
        }
        internal void ResizePathLists(int newLength)
        {
            int currentCount = PathfindingInternalDataList.Count;
            for (int i = currentCount; i < newLength; i++)
            {
                PathfindingInternalDataList.Add(new PathfindingInternalData());
                PathPortalTraversalDataList.Add(new PathPortalTraversalData());
                SectorOverlappingDirectionTableList.Add(new NativeArray<OverlappingDirection>());
                SectorToFlowStartTables.Add(new NativeArray<int>());
                PathGoalNeighbourIndexToGoalIndexMaps.Add(new NativeHashMap<int, int>());
                PathGoalTraversalDataFieldIndexLists.Add(new NativeList<int>());
                PathAlreadyConsideredSectorIndexMaps.Add(new NativeHashSet<int>());
            }
            PathSectorStateTableList.Length = newLength;
            PathDestinationDataList.Length = newLength;
            PathRoutineDataList.Length = newLength;
            PathSectorBitArrays.Length = newLength;
            PathSubscriberCounts.Length = newLength;
            PathFlockIndicies.Length = newLength;
            PathRanges.Length = newLength;
            PathDesiredRanges.Length = newLength;
            PathIslandSeedsAsFieldIndicies.Length = newLength;
        }
        internal void CreatePath(FinalPathRequest request, float2 anySourcePoint)
        {
            PreallocationPack preallocations = _preallocator.GetPreallocations();
            PathfindingInternalData internalData = new PathfindingInternalData()
            {
                PickedSectorList = preallocations.PickedToSector,
                PortalTraversalQueue = preallocations.PortalTraversalFastMarchingQueue,
                NotActivePortalList = preallocations.NotActivePortalList,
                SectorIndiciesToCalculateIntegration = preallocations.SectorStartIndexListToCalculateIntegration,
                SectorIndiciesToCalculateFlow = preallocations.SectorStartIndexListToCalculateFlow,
                SectorWithinLOSState = preallocations.SectorsWithinLOSState,
                SectorToWaveFrontsMap = new NativeParallelMultiHashMap<int, ActiveWaveFront>(0, Allocator.Persistent),
                IntegrationField = new NativeList<IntegrationTile>(Allocator.Persistent),
                LOSCalculatedFlag = new NativeReference<bool>(false, Allocator.Persistent),
                FlowFieldCalculationBuffer = new NativeList<FlowData>(Allocator.Persistent),
                DynamicArea = new DynamicArea()
                {
                    FlowFieldCalculationBuffer = new NativeList<FlowData>(Allocator.Persistent),
                    IntegrationField = preallocations.DynamicAreaIntegrationField,
                    SectorFlowStartCalculationBuffer = new NativeList<SectorFlowStart>(Allocator.Persistent),
                }
            };

            PathDestinationData destinationData = new PathDestinationData()
            {
                DestinationType = request.Type,
                TargetAgentIndex = request.TargetAgentIndex,
                Destination = request.Destination,
                DesiredDestination = request.DesiredDestination,
                Offset = request.Offset,
            };
            PathPortalTraversalData portalTraversalData = new PathPortalTraversalData()
            {
                PortalSequenceSlices = new NativeList<Slice>(Allocator.Persistent),
                PortalSequence = preallocations.PortalSequence,
                SourcePortalIndexList = preallocations.SourcePortalIndexList,
                TargetSectorPortalIndexList = preallocations.TargetSectorPortalIndexList,
                NewPickedSectorStartIndex = preallocations.NewPickedSectorStartIndex,
                PathAdditionSequenceSliceStartIndex = preallocations.PathAdditionSequenceBorderStartIndex,
                DiskstraStartIndicies = preallocations.DijkstraStartIndicies,
                GoalDataList = new NativeList<PortalTraversalData>(Allocator.Persistent),
                NewPathUpdateSeedIndicies = new NativeList<int>(Allocator.Persistent),
                PickedPortalDataRecords = new NativeList<PickedPortalDataRecord>(Allocator.Persistent),
            };

            NativeArray<OverlappingDirection> sectorOverlappingDirections = new NativeArray<OverlappingDirection>(FlowFieldUtilities.SectorMatrixTileAmount, Allocator.Persistent);
            int2 islandSeed2d = FlowFieldUtilities.PosTo2D(anySourcePoint, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
            int islandSeed1d = FlowFieldUtilities.To1D(islandSeed2d, FlowFieldUtilities.FieldColAmount);

            PathfindingInternalDataList[request.PathIndex] = internalData;
            PathSectorStateTableList[request.PathIndex] = preallocations.SectorStateTable;
            PathPortalTraversalDataList[request.PathIndex] = portalTraversalData;
            PathDestinationDataList[request.PathIndex] = destinationData;
            PathRoutineDataList[request.PathIndex] = new PathRoutineData();
            PathSectorBitArrays[request.PathIndex] = preallocations.SectorBitArray;
            PathSubscriberCounts[request.PathIndex] = request.SourceCount;
            PathFlockIndicies[request.PathIndex] = request.FlockIndex;
            SectorOverlappingDirectionTableList[request.PathIndex] = sectorOverlappingDirections;
            SectorToFlowStartTables[request.PathIndex] = new NativeArray<int>(FlowFieldUtilities.SectorMatrixTileAmount, Allocator.Persistent);
            PathRanges[request.PathIndex] = request.Range;
            PathDesiredRanges[request.PathIndex] = request.DesiredRange;
            PathGoalNeighbourIndexToGoalIndexMaps[request.PathIndex] = new NativeHashMap<int, int>(0, Allocator.Persistent);
            PathGoalTraversalDataFieldIndexLists[request.PathIndex] = new NativeList<int>(Allocator.Persistent);
            PathAlreadyConsideredSectorIndexMaps[request.PathIndex] = new NativeHashSet<int>(0, Allocator.Persistent);
            PathIslandSeedsAsFieldIndicies[request.PathIndex] = islandSeed1d;
        }
    }

}
