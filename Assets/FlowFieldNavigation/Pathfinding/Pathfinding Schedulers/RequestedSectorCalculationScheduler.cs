using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowFieldNavigation
{

    internal class RequestedSectorCalculationScheduler
    {
        PathDataContainer _pathContainer;

        internal RequestedSectorCalculationScheduler(FlowFieldNavigationManager navigationManager)
        {
            _pathContainer = navigationManager.PathDataContainer;
        }
        internal void DisposeAll()
        {
        }
        internal JobHandle ScheduleRequestedSectorCalculation(NativeArray<FlowRequest> flowRequestsUnique, JobHandle dependency, NativeArray<float2> sources)
        {
            NativeArray<JobHandle> tempHandleArray = new NativeArray<JobHandle>(flowRequestsUnique.Length, Allocator.Temp);
            for(int i = 0; i < flowRequestsUnique.Length; i++)
            {
                FlowRequest req = flowRequestsUnique[i];
                int pathIndex = req.PathIndex;
                Slice sourceSlice = req.SourceSlice;
                NativeSlice<float2> flowSources = new NativeSlice<float2>(sources, sourceSlice.Index, sourceSlice.Count);
                PathfindingInternalData pathInternalData = _pathContainer.PathfindingInternalDataList[pathIndex];
                PathDestinationData destinationData = _pathContainer.PathDestinationDataList[pathIndex];
                UnsafeList<PathSectorState> sectorStateTable = _pathContainer.PathSectorStateTableList[pathIndex];
                NativeArray<OverlappingDirection> sectorOverlappingDirectionTable = _pathContainer.SectorOverlappingDirectionTableList[pathIndex];

                //SOURCE SECTOR CALCULATION
                SourceSectorCalculationJob sectorCalcJob = new SourceSectorCalculationJob()
                {
                    FieldColAmount = FlowFieldUtilities.FieldColAmount,
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorMatrixRowAmount = FlowFieldUtilities.SectorMatrixRowAmount,
                    LOSRange = FlowFieldUtilities.LOSRange,
                    SectorSize = FlowFieldUtilities.SectorColAmount * FlowFieldUtilities.TileSize,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    TargetIndex = FlowFieldUtilities.PosTo2D(destinationData.Destination, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition),
                    FieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition,
                    SectorStateTable = sectorStateTable,
                    Sources = flowSources,
                    SectorOverlappingDirectionTable = sectorOverlappingDirectionTable,
                    SectorIndiciesToCalculateIntegration = pathInternalData.SectorIndiciesToCalculateIntegration,
                    SectorIndiciesToCalculateFlow = pathInternalData.SectorIndiciesToCalculateFlow,
                    SectorWithinLOSState = pathInternalData.SectorWithinLOSState,
                };
                JobHandle sourceSectorHandle = sectorCalcJob.Schedule(dependency);
                if (FlowFieldUtilities.DebugMode) { sourceSectorHandle.Complete(); }
                tempHandleArray[i] = sourceSectorHandle;
            }

            return JobHandle.CombineDependencies(tempHandleArray);
        }
    }


}