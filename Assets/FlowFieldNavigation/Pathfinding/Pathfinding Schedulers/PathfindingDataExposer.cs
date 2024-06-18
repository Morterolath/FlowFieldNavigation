using System.Collections.Generic;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    internal class PathfindingDataExposer
    {
        PathDataContainer _pathContainer;
        internal PathfindingDataExposer(PathDataContainer pathDataContainer)
        {
            _pathContainer = pathDataContainer;
        }
        internal void Expose(
            NativeArray<int> pathIndiciesOfScheduledDynamicAreas,
            NativeArray<PortalTraversalRequest> portalTraversalRequestedPaths,
            NativeArray<int> _losCalculatedPaths,
            NativeArray<FlowRequest> flowRequests, 
            NativeArray<int> destinationUpdatedPathIndicies, 
            NativeArray<int> newPathIndicies, 
            NativeArray<int> expandedPathIndicies
            )
        {
            RefreshResizedFlowFieldLengths(portalTraversalRequestedPaths);
            ScheduleLOSTransfers(_losCalculatedPaths);
            ScheduleFlowTransfers(flowRequests);
            UpdateDynamicArea(pathIndiciesOfScheduledDynamicAreas);
            _pathContainer.ExposeBuffers(destinationUpdatedPathIndicies, newPathIndicies, expandedPathIndicies);
            SubmitPathUpdateSeeds(portalTraversalRequestedPaths);
        }
        void SubmitPathUpdateSeeds(NativeArray<PortalTraversalRequest> portalTraversalRequestedPaths)
        {
            List<PathPortalTraversalData> pathPortalTraversalDataArray = _pathContainer.PathPortalTraversalDataList;
            NativeArray<PathDestinationData> pathDestinationDataArray = _pathContainer.PathDestinationDataList.AsArray();
            NativeParallelMultiHashMap<int,int> pathIndexToUpdateSeedMap = _pathContainer.PathIndexToUpdateSeedsMap;
            for(int i = 0; i < portalTraversalRequestedPaths.Length; i++)
            {
                int pathIndex = portalTraversalRequestedPaths[i].PathIndex;
                PathDestinationData destinationData = pathDestinationDataArray[pathIndex];
                if(destinationData.DestinationType != DestinationType.DynamicDestination) { continue; }
                NativeList<int> newPathUpdateSeedIndicies = pathPortalTraversalDataArray[pathIndex].NewPathUpdateSeedIndicies;
                for(int j = 0; j < newPathUpdateSeedIndicies.Length; j++)
                {
                    pathIndexToUpdateSeedMap.Add(pathIndex, newPathUpdateSeedIndicies[j]);
                }
                newPathUpdateSeedIndicies.Clear();
            }
        }
        void ScheduleFlowTransfers(NativeArray<FlowRequest> flowRequests)
        {
            float tileSize = FlowFieldUtilities.TileSize;
            float2 fieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition;
            int sectorTileAmount = FlowFieldUtilities.SectorTileAmount;
            int sectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount;
            PathSectorToFlowStartMapper newFlowStartMap = _pathContainer.SectorFlowStartMap;
            NativeList<FlowData> exposedFlowData = _pathContainer.ExposedFlowData;
            NativeArray<PathDestinationData> pathDestinationDataArray = _pathContainer.PathDestinationDataList.AsArray();
            for (int i = 0; i < flowRequests.Length; i++)
            {
                FlowRequest req = flowRequests[i];
                PathfindingInternalData pathInternalData = _pathContainer.PathfindingInternalDataList[req.PathIndex];
                PathDestinationData destinationData = pathDestinationDataArray[req.PathIndex];
                NativeArray<int> flowCalculatedSectorIndicies = pathInternalData.SectorIndiciesToCalculateFlow.AsArray();
                NativeArray<FlowData> calculationBuffer = pathInternalData.FlowFieldCalculationBuffer.AsArray();
                for (int j = 0; j < flowCalculatedSectorIndicies.Length; j++)
                {
                    int sectorIndex = flowCalculatedSectorIndicies[j];
                    newFlowStartMap.TryGet(req.PathIndex, sectorIndex, out int newSectorFlowStartIndex);
                    NativeSlice<FlowData> fromSlice = new NativeSlice<FlowData>(calculationBuffer, j * sectorTileAmount, sectorTileAmount);
                    TransferFlow(fromSlice, exposedFlowData.AsArray(), newSectorFlowStartIndex);

                    //if sector is within dynamic area, also transfer dynamic area
                    if(destinationData.DestinationType != DestinationType.DynamicDestination) { continue; }
                    int2 destinationIndex = FlowFieldUtilities.PosTo2D(destinationData.Destination, tileSize, fieldGridStartPos);
                    int2 destinationSector = FlowFieldUtilities.GetSector2D(destinationIndex, sectorMatrixColAmount);
                    int2 min = destinationSector - 1;
                    int2 max = destinationSector + 1;
                    bool2 greaterThanOrEquelToMin = destinationSector >= min;
                    bool2 lessThanOrEqualToMax = destinationSector <= max;
                    bool withinBounds = greaterThanOrEquelToMin.x & greaterThanOrEquelToMin.y & lessThanOrEqualToMax.x & lessThanOrEqualToMax.y;
                    if (withinBounds)
                    {
                        bool successfull = GetSectorStartForDynamicAreaSector(pathInternalData.DynamicArea.SectorFlowStartCalculationBuffer.AsArray(), sectorIndex, out int dynamicAreaSectorFlowStart);
                        if (!successfull) { continue; }
                        TransferDynamicArea(exposedFlowData.AsArray(), pathInternalData.DynamicArea.FlowFieldCalculationBuffer.AsArray(), newSectorFlowStartIndex, dynamicAreaSectorFlowStart);
                    }
                }
            }

            void TransferFlow(NativeSlice<FlowData> fromSlice, NativeArray<FlowData> toList, int listStartIndex)
            {
                for (int i = 0; i < fromSlice.Length; i++)
                {
                    toList[listStartIndex + i] = fromSlice[i];
                }
            }
            void TransferDynamicArea(NativeArray<FlowData> toArray, NativeArray<FlowData> fromArray, int toArrayStart, int fromArrayStart)
            {
                for(int i = 0; i < FlowFieldUtilities.SectorTileAmount; i++)
                {
                    int fromIndex = fromArrayStart + i;
                    int toIndex = toArrayStart + i;
                    FlowData flow = fromArray[fromIndex];
                    if (!flow.IsValid()) { continue; }
                    toArray[toIndex] =  flow;
                }
            }
            bool GetSectorStartForDynamicAreaSector(NativeArray<SectorFlowStart> sectorFlowStartPairs, int sectorToLookup, out int flowStart)
            {
                for (int i = 0; i < sectorFlowStartPairs.Length; i++)
                {
                    SectorFlowStart pair = sectorFlowStartPairs[i];
                    if (pair.SectorIndex == sectorToLookup)
                    {
                        flowStart = pair.FlowStartIndex;
                        return true;
                    }
                }
                flowStart = 0;
                return false;
            }
        }
        internal void ScheduleLOSTransfers(NativeArray<int> _losCalculatedPaths)
        {
            List<PathfindingInternalData> internalDataList = _pathContainer.PathfindingInternalDataList;
            NativeList<PathDestinationData> pathDestinationDataList = _pathContainer.PathDestinationDataList;
            PathSectorToFlowStartMapper newFlowStartMap = _pathContainer.SectorFlowStartMap;
            NativeArray<bool> exposedLosData = _pathContainer.ExposedLosData.AsArray();
            for (int i = 0; i < _losCalculatedPaths.Length; i++)
            {
                int pathIndex = _losCalculatedPaths[i];
                PathfindingInternalData internalData = internalDataList[pathIndex];
                PathDestinationData destinationData = pathDestinationDataList[pathIndex];
                NativeArray<int> sectorToFlowStartTable = _pathContainer.SectorToFlowStartTables[pathIndex];
                LOSTransferJob losTransfer = new LOSTransferJob()
                {
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorMatrixRowAmount = FlowFieldUtilities.SectorMatrixRowAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    LOSRange = FlowFieldUtilities.LOSRange,
                    PathIndex = pathIndex,
                    SectorFlowStartTable = sectorToFlowStartTable,
                    FlowStartMap = newFlowStartMap,
                    LosArray = exposedLosData,
                    IntegrationField = internalData.IntegrationField.AsArray(),
                    Target = FlowFieldUtilities.PosTo2D(destinationData.Destination, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition),
                };
                losTransfer.Schedule().Complete();
            }
        }
        void RefreshResizedFlowFieldLengths(NativeArray<PortalTraversalRequest> portalTraversalRequestedPaths)
        {
            List<PathfindingInternalData> pathfindingInternalDataList = _pathContainer.PathfindingInternalDataList;
            List<PathPortalTraversalData> pathPorTravDataList = _pathContainer.PathPortalTraversalDataList;
            PathSectorToFlowStartMapper newFlowStartMap = _pathContainer.SectorFlowStartMap;
            NativeList<FlowData> exposedFlowData = _pathContainer.ExposedFlowData;
            NativeList<bool> exposedLosData = _pathContainer.ExposedLosData;
            NativeList<int> removedFlowAndLosStartIndicies = _pathContainer.RemovedExposedFlowAndLosIndicies;
            for (int i = 0; i < portalTraversalRequestedPaths.Length; i++)
            {
                PortalTraversalRequest request = portalTraversalRequestedPaths[i];
                int pathIndex = request.PathIndex;
                NativeArray<int> allPickedSectors = pathfindingInternalDataList[pathIndex].PickedSectorList.AsArray();
                int newPickedSectorStartIndex = pathPorTravDataList[pathIndex].NewPickedSectorStartIndex.Value;
                for(int j = newPickedSectorStartIndex; j < allPickedSectors.Length; j++)
                {
                    int sectorIndex = allPickedSectors[j];
                    if(newFlowStartMap.Contains(pathIndex, sectorIndex)) { continue; }
                    int losAndFlowStartIndex;
                    if (removedFlowAndLosStartIndicies.IsEmpty)
                    {
                        losAndFlowStartIndex = exposedFlowData.Length;
                        exposedFlowData.Length += FlowFieldUtilities.SectorTileAmount;
                        exposedLosData.Length += FlowFieldUtilities.SectorTileAmount;
                    }
                    else
                    {
                        losAndFlowStartIndex = removedFlowAndLosStartIndicies[0];
                        removedFlowAndLosStartIndicies.RemoveAtSwapBack(0);
                    }
                    newFlowStartMap.TryAdd(pathIndex, sectorIndex, losAndFlowStartIndex);
                }
            }
        }
        void UpdateDynamicArea(NativeArray<int> pathIndiciesOfScheduledDynamicAreas)
        {
            List<PathfindingInternalData> pathfindingInternalDataList = _pathContainer.PathfindingInternalDataList;
            NativeList<JobHandle> handles = new NativeList<JobHandle>(Allocator.Temp);
            NativeList<UnsafeList<PathSectorState>> sectorStateTabelList = _pathContainer.PathSectorStateTableList;
            NativeArray<FlowData> exposedFlowData = _pathContainer.ExposedFlowData.AsArray();
            PathSectorToFlowStartMapper exposedFlowStartMap = _pathContainer.SectorFlowStartMap;
            for (int i = 0; i < pathIndiciesOfScheduledDynamicAreas.Length; i++)
            {
                int pathIndex = pathIndiciesOfScheduledDynamicAreas[i];
                PathfindingInternalData pathInternalData = pathfindingInternalDataList[pathIndex];
                NativeArray<FlowData> flowCalculationBuffer = pathInternalData.DynamicArea.FlowFieldCalculationBuffer.AsArray();
                NativeArray<SectorFlowStart> sectorFlowStartCalculationBuffer = pathInternalData.DynamicArea.SectorFlowStartCalculationBuffer.AsArray();
                UnsafeList<PathSectorState> sectorStateTable = sectorStateTabelList[pathIndex];
                for(int j = 0; j < sectorFlowStartCalculationBuffer.Length; j++)
                {
                    SectorFlowStart pair = sectorFlowStartCalculationBuffer[j];
                    int sectorIndex = pair.SectorIndex;
                    int flowStart = pair.FlowStartIndex;
                    if ((sectorStateTable[sectorIndex] & PathSectorState.FlowCalculated) != PathSectorState.FlowCalculated) { continue; }
                    exposedFlowStartMap.TryGet(pathIndex, sectorIndex, out int exposedFlowStart);
                    TransferDynamicArea(exposedFlowData, flowCalculationBuffer, exposedFlowStart, flowStart);
                }
            }
            JobHandle.CompleteAll(handles.AsArray());
            
            void TransferDynamicArea(NativeArray<FlowData> toArray, NativeArray<FlowData> fromArray, int toArrayStart, int fromArrayStart)
            {
                for (int i = 0; i < FlowFieldUtilities.SectorTileAmount; i++)
                {
                    int fromIndex = fromArrayStart + i;
                    int toIndex = toArrayStart + i;
                    FlowData flow = fromArray[fromIndex];
                    if (!flow.IsValid()) { continue; }
                    toArray[toIndex] = flow;
                }
            }
        }
    }
}