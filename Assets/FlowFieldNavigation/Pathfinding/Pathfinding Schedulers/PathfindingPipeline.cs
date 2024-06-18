using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    internal class PathfindingPipeline
    {
        FlowFieldNavigationManager _navManager;

        PortalTraversalScheduler _portalTraversalScheduler;
        RequestedSectorCalculationScheduler _requestedSectorCalculationScheduler;
        LOSIntegrationScheduler _losIntegrationScheduler;
        DynamicAreaScheduler _dynamicAreaScheduler;
        FlowCalculationScheduler _flowCalculationScheduler;
        PathfindingDataExposer _pathfindingDataExposer;
        NativeArray<float2> _sources;
        NativeList<PortalTraversalRequest> _porTravRequestedPathList;
        NativeList<FlowRequest> _flowRequestedPathList;
        NativeList<LosRequest> _losRequestedPathList;
        NativeList<int> _goalUpdateRequestedPathList;
        PathfindingPipelineStateHandle _stateHandle;
        internal PathfindingPipeline(FlowFieldNavigationManager navManager)
        {
            _navManager = navManager;
            _losIntegrationScheduler = new LOSIntegrationScheduler(navManager);
            _requestedSectorCalculationScheduler = new RequestedSectorCalculationScheduler(navManager);
            _dynamicAreaScheduler = new DynamicAreaScheduler(navManager);
            _portalTraversalScheduler = new PortalTraversalScheduler(navManager, _requestedSectorCalculationScheduler);
            _flowCalculationScheduler = new FlowCalculationScheduler(navManager, _losIntegrationScheduler);
            _pathfindingDataExposer = new PathfindingDataExposer(_navManager.PathDataContainer);
            _porTravRequestedPathList = new NativeList<PortalTraversalRequest>(Allocator.Persistent);
            _flowRequestedPathList = new NativeList<FlowRequest>(Allocator.Persistent);
            _losRequestedPathList = new NativeList<LosRequest>(Allocator.Persistent);
            _goalUpdateRequestedPathList = new NativeList<int>(Allocator.Persistent);
        }
        internal void Run(NativeArray<float2> sources, NativeList<int> expandedPathIndiciesOutput, NativeList<int> destinationUpdatedPathIndicies)
        {
            _porTravRequestedPathList.Clear();
            _flowRequestedPathList.Clear();
            _losRequestedPathList.Clear();
            _goalUpdateRequestedPathList.Clear();
            _sources = sources;

            //Get scheduling information
            NativeArray<PathRoutineData> pathRoutineDataArray = _navManager.PathDataContainer.PathRoutineDataList.AsArray();
            for (int i = 0; i < pathRoutineDataArray.Length; i++)
            {
                PathRoutineData curPathRoutineData = pathRoutineDataArray[i];
                if (curPathRoutineData.Task == 0 && curPathRoutineData.DestinationState != DynamicDestinationState.Moved) { continue; }
                int flowStart = math.select(curPathRoutineData.PathAdditionSourceStart, curPathRoutineData.FlowRequestSourceStart, curPathRoutineData.FlowRequestSourceCount != 0);
                int flowCount = curPathRoutineData.FlowRequestSourceCount + curPathRoutineData.PathAdditionSourceCount;
                Slice flowReqSlice = new Slice(flowStart, flowCount);
                Slice pathAdditionReqSlice = new Slice(curPathRoutineData.PathAdditionSourceStart, curPathRoutineData.PathAdditionSourceCount);
                bool pathAdditionRequested = (curPathRoutineData.Task & PathTask.PathAdditionRequest) == PathTask.PathAdditionRequest;
                bool flowRequested = (curPathRoutineData.Task & PathTask.FlowRequest) == PathTask.FlowRequest;
                bool destinationMoved = curPathRoutineData.DestinationState == DynamicDestinationState.Moved;

                if (pathAdditionRequested)
                {
                    _porTravRequestedPathList.Add(new PortalTraversalRequest(i, pathAdditionReqSlice));
                    expandedPathIndiciesOutput.Add(i);
                }
                if (flowRequested || pathAdditionRequested)
                {
                    _flowRequestedPathList.Add(new FlowRequest(i, flowReqSlice));
                }
                if(pathAdditionRequested || flowRequested || destinationMoved)
                {
                    _losRequestedPathList.Add(new LosRequest(i, curPathRoutineData.DestinationState));
                }
                if (destinationMoved)
                {
                    _goalUpdateRequestedPathList.Add(i);
                    destinationUpdatedPathIndicies.Add(i);
                }
            }
            //Schedule initial jobs
            JobHandle portalTraversalProcedureHandle = _portalTraversalScheduler.SchedulePortalTraversalFor(_porTravRequestedPathList.AsArray(), _sources);

            JobHandle requestedSectorProcedureHandle = _requestedSectorCalculationScheduler.ScheduleRequestedSectorCalculation(
                _flowRequestedPathList.AsArray(),
                portalTraversalProcedureHandle,
                _sources);

            _stateHandle = new PathfindingPipelineStateHandle()
            {
                Handle = requestedSectorProcedureHandle,
                State = PathfindingPipelineState.Intermediate,
            };
        }
        internal void TryComplete()
        {
            Complete(false);
        }
        internal void ForceComplete(NativeArray<int> destinationUpdatedPathIndicies, NativeArray<int> newPathIndicies, NativeArray<int> expandedPathIndicies)
        {
            Complete(true);
            _pathfindingDataExposer.Expose(_goalUpdateRequestedPathList.AsArray(),
                _porTravRequestedPathList.AsArray(),
                _losIntegrationScheduler._losCalculatedPaths.AsArray(),
                _flowRequestedPathList.AsArray(),
                destinationUpdatedPathIndicies,
                newPathIndicies,
                expandedPathIndicies
                );
            _losIntegrationScheduler._losCalculatedPaths.Clear();
        }

        void Complete(bool forceComplete)
        {
            if (_stateHandle.State == PathfindingPipelineState.Intermediate && (_stateHandle.Handle.IsCompleted || forceComplete))
            {
                _stateHandle.Handle.Complete();

                JobHandle flowProcedureHandle = _flowCalculationScheduler.ScheduleFlow(_flowRequestedPathList.AsArray());
                JobHandle losProcedureHandle = _losIntegrationScheduler.ScheduleLOS(_losRequestedPathList.AsArray(), flowProcedureHandle);
                JobHandle dynamicAreaProcedureHandle = _dynamicAreaScheduler.ScheduleDynamicArea(_goalUpdateRequestedPathList.AsArray());
                JobHandle combinedStepHandle = JobHandle.CombineDependencies(losProcedureHandle, dynamicAreaProcedureHandle);

                _stateHandle.State = PathfindingPipelineState.Final;
                _stateHandle.Handle = combinedStepHandle;
            }
            if(_stateHandle.State == PathfindingPipelineState.Final && forceComplete)
            {
                _stateHandle.Handle.Complete();
            }
        }
    }
}