﻿using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine.Jobs;

namespace FlowFieldNavigation
{

    internal class RoutineScheduler
    {
        internal uint FieldState { get { return _fieldState; } }

        FlowFieldNavigationManager _navigationManager;
        MovementManager _movementManager;
        PathfindingManager _pathfindingManager;

        List<JobHandle> _costEditHandle;
        List<JobHandle> _islandReconfigHandle;

        uint _fieldState;
        internal NativeList<PathRequest> CurrentRequestedPaths;

        NativeList<UnsafeListReadOnly<byte>> _costFieldCosts;
        NativeList<SectorBitArray> EditedSectorBitArray;
        NativeList<CostEdit> NewCostEditRequests;

        NativeList<float3> _agentPositionsForPathfinding;
        NativeList<float3> _agentPositionsForMovement;
        internal RoutineScheduler(FlowFieldNavigationManager navigationManager)
        {
            _navigationManager = navigationManager;
            _movementManager = navigationManager.MovementManager;
            _pathfindingManager = navigationManager.PathfindingManager;
            _costEditHandle = new List<JobHandle>();
            CurrentRequestedPaths = new NativeList<PathRequest>(Allocator.Persistent);
            _islandReconfigHandle = new List<JobHandle>();
            _costFieldCosts = new NativeList<UnsafeListReadOnly<byte>>(Allocator.Persistent);
            EditedSectorBitArray = new NativeList<SectorBitArray>(Allocator.Persistent);
            NewCostEditRequests = new NativeList<CostEdit>(Allocator.Persistent);
            _agentPositionsForMovement = new NativeList<float3>(Allocator.Persistent);
            _agentPositionsForPathfinding = new NativeList<float3>(Allocator.Persistent);
            _fieldState = 0;
        }
        internal void DisposeAll()
        {
            _costEditHandle = null;
            _islandReconfigHandle = null;
            CurrentRequestedPaths.Dispose();
            EditedSectorBitArray.Dispose();
            NewCostEditRequests.Dispose();
            _costFieldCosts.Dispose();

            _movementManager.DisposeAll();
            _pathfindingManager.DisposeAll();
            _movementManager = null;
            _pathfindingManager = null;
        }
        internal void Schedule(NativeList<PathRequest> newPaths, NativeArray<CostEdit>.ReadOnly costEditRequests)
        {
            //COPY OBSTACLE REQUESTS
            ReadOnlyNativeArrayToNativeListCopyJob<CostEdit> obstacleRequestCopy = new ReadOnlyNativeArrayToNativeListCopyJob<CostEdit>()
            {
                Source = costEditRequests,
                Destination = NewCostEditRequests,
            };
            obstacleRequestCopy.Schedule().Complete();

            //REFRESH COST FIELD COSTS
            UnsafeListReadOnly<byte>[] costFielCosts = _navigationManager.GetAllCostFieldCostsAsUnsafeListReadonly();
            _costFieldCosts.Length = costFielCosts.Length;
            for (int i = 0; i < costFielCosts.Length; i++)
            {
                _costFieldCosts[i] = costFielCosts[i];
            }

            //SCHEDULE COST EDITS
            JobHandle costEditHandle = ScheduleCostEditRequests();
            JobHandle islandFieldReconfigHandle = ScheduleIslandFieldReconfig(costEditHandle);

            //GET AGENT POSITIONS
            TransformAccessArray agentTransforms = _navigationManager.AgentDataContainer.AgentTransforms;
            _agentPositionsForMovement.Length = agentTransforms.length;
            _agentPositionsForPathfinding.Length = agentTransforms.length;
            AgentPositionGetJob agentMovementPositionGet = new AgentPositionGetJob()
            {
                MaxXExcluding = FlowFieldUtilities.FieldMaxXExcluding,
                MaxYExcluding = FlowFieldUtilities.FieldMaxYExcluding,
                MinXIncluding = FlowFieldUtilities.FieldMinXIncluding,
                MinYIncluding = FlowFieldUtilities.FieldMinYIncluding,
                PositionOutput = _agentPositionsForMovement.AsArray(),
            };
            JobHandle agentMovementPositionGetHandle = agentMovementPositionGet.Schedule(agentTransforms);
            AgentPositionGetJob agentPathfindingPositionGet = new AgentPositionGetJob()
            {
                MaxXExcluding = FlowFieldUtilities.FieldMaxXExcluding,
                MaxYExcluding = FlowFieldUtilities.FieldMaxYExcluding,
                MinXIncluding = FlowFieldUtilities.FieldMinXIncluding,
                MinYIncluding = FlowFieldUtilities.FieldMinYIncluding,
                PositionOutput = _agentPositionsForPathfinding.AsArray(),
            };
            JobHandle agentPathfindingPositionGetHandle = agentPathfindingPositionGet.Schedule(agentTransforms);

            //COPY REQUESTED TO SCHEDULING SYSTEM
            NativeListCopyJob<PathRequest> copyJob = new NativeListCopyJob<PathRequest>()
            {
                Source = newPaths,
                Destination = CurrentRequestedPaths,
            };
            JobHandle copyHandle = copyJob.Schedule();

            //TRANSFER REQUESTED PATHS TO NEW PATHS
            RequestedToNewPathIndexTransferJob reqToNewTransfer = new RequestedToNewPathIndexTransferJob()
            {
                AgentRequestedPathIndicies = _navigationManager.AgentDataContainer.AgentRequestedPathIndicies.AsArray(),
                AgentNewPathIndicies = _navigationManager.AgentDataContainer.AgentNewPathIndicies.AsArray(),
            };
            JobHandle transferHandle = reqToNewTransfer.Schedule();

            JobHandle.CombineDependencies(transferHandle, copyHandle).Complete();
            JobHandle.CombineDependencies(agentPathfindingPositionGetHandle, agentMovementPositionGetHandle).Complete();

            _pathfindingManager.ShcedulePathRequestEvalutaion(CurrentRequestedPaths, _costFieldCosts.AsArray(), EditedSectorBitArray.AsArray().AsReadOnly(), _agentPositionsForPathfinding.AsArray(), islandFieldReconfigHandle);
            _movementManager.ScheduleRoutine(_costFieldCosts.AsArray(), _agentPositionsForMovement.AsArray(), costEditHandle);
        }
        internal void TryCompletePredecessorJobs()
        {
            //ISLAND REC
            if (_islandReconfigHandle.Count != 0)
            {
                if (_islandReconfigHandle[0].IsCompleted)
                {
                    _islandReconfigHandle[0].Complete();
                    _islandReconfigHandle.RemoveAtSwapBack(0);
                }
                _fieldState++;
            }

            _pathfindingManager.TryComplete();
        }
        internal void ForceCompleteAll(float deltaTime)
        {
            //COST EDIT
            if (_costEditHandle.Count != 0)
            {
                _costEditHandle[0].Complete();
                _costEditHandle.RemoveAtSwapBack(0);
            }
            //ISLAND RECONFIG
            if (_islandReconfigHandle.Count != 0)
            {
                _islandReconfigHandle[0].Complete();
                _islandReconfigHandle.RemoveAtSwapBack(0);
                _fieldState++;
            }

            _movementManager.ForceCompleteRoutine();
            _pathfindingManager.ForceComplete();
            _movementManager.SendRoutineResults(deltaTime);
            _pathfindingManager.TransferNewPathsToCurPaths();

            CurrentRequestedPaths.Clear();
            EditedSectorBitArray.Clear();
            NewCostEditRequests.Clear();
        }
        JobHandle ScheduleCostEditRequests()
        {
            if (NewCostEditRequests.Length == 0) { return new JobHandle(); }

            NativeList<JobHandle> editHandles = new NativeList<JobHandle>(Allocator.Temp);
            for (int i = 0; i <= FlowFieldUtilities.MaxCostFieldOffset; i++)
            {
                CostField costField = _navigationManager.FieldDataContainer.GetCostFieldWithOffset(i);
                FieldGraph fieldGraph = _navigationManager.FieldDataContainer.GetFieldGraphWithOffset(i);

                NativeListCopyJob<CostEdit> newObstaclesTransfer = new NativeListCopyJob<CostEdit>()
                {
                    Source = NewCostEditRequests,
                    Destination = fieldGraph.NewCostEdits,
                };
                JobHandle newObstacleTransferHandle = newObstaclesTransfer.Schedule();

                CostFieldEditJob costEditJob = new CostFieldEditJob()
                {
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorRowAmount = FlowFieldUtilities.SectorRowAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorMatrixRowAmount = FlowFieldUtilities.SectorMatrixRowAmount,
                    FieldColAmount = FlowFieldUtilities.FieldColAmount,
                    FieldRowAmount = FlowFieldUtilities.FieldRowAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    Offset = i,
                    SectorNodes = fieldGraph.SectorNodes,
                    SecToWinPtrs = fieldGraph.SecToWinPtrs,
                    EditedSectorBits = fieldGraph.EditedSectorMarks,
                    EditedSectorIndicies = fieldGraph.EditedSectorList,
                    WinToSecPtrs = fieldGraph.WinToSecPtrs,
                    Costs = costField.Costs,
                    CostStamps = costField.StampCounts,
                    BaseCosts = costField.BaseCosts,
                    EditedWindowIndicies = fieldGraph.EditedWinodwList,
                    EditedWindowMarks = fieldGraph.EditedWindowMarks,
                    IntegratedCosts = fieldGraph.SectorIntegrationField,
                    IslandFields = fieldGraph.IslandFields,
                    Islands = fieldGraph.IslandDataList.AsArray(),
                    NewCostEdits = fieldGraph.NewCostEdits,
                    PorPtrs = fieldGraph.PorToPorPtrs,
                    PortalNodes = fieldGraph.PortalNodes,
                    PortalPerWindow = fieldGraph.PortalPerWindow,
                    WindowNodes = fieldGraph.WindowNodes,
                };
                JobHandle editHandle = costEditJob.Schedule(newObstacleTransferHandle);
                editHandles.Add(editHandle);

                EditedSectorBitArray.Add(costEditJob.EditedSectorBits);
            }
            JobHandle cominedHandle = JobHandle.CombineDependencies(editHandles.AsArray());
            editHandles.Dispose();

            if (FlowFieldUtilities.DebugMode) { cominedHandle.Complete(); }
            _costEditHandle.Add(cominedHandle);
            return cominedHandle;
        }
        JobHandle ScheduleIslandFieldReconfig(JobHandle dependency)
        {
            if (NewCostEditRequests.Length == 0) { return new JobHandle(); }

            NativeArray<JobHandle> handlesToCombine = new NativeArray<JobHandle>(FlowFieldUtilities.MaxCostFieldOffset + 1, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i <= FlowFieldUtilities.MaxCostFieldOffset; i++)
            {
                FieldGraph fieldGraph = _navigationManager.FieldDataContainer.GetFieldGraphWithOffset(i);
                CostField costField = _navigationManager.FieldDataContainer.GetCostFieldWithOffset(i);

                IslandReconfigurationJob islandReconfig = new IslandReconfigurationJob()
                {
                    Offset = i,
                    SectorColAmount = FlowFieldUtilities.SectorColAmount,
                    SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
                    SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
                    SectorNodes = fieldGraph.SectorNodes,
                    SecToWinPtrs = fieldGraph.SecToWinPtrs,
                    EditedSectorIndicies = fieldGraph.EditedSectorList,
                    PortalEdges = fieldGraph.PorToPorPtrs,
                    CostsL = costField.Costs,
                    IslandFields = fieldGraph.IslandFields,
                    Islands = fieldGraph.IslandDataList,
                    PortalNodes = fieldGraph.PortalNodes,
                    WindowNodes = fieldGraph.WindowNodes,
                };
                handlesToCombine[i] = islandReconfig.Schedule(dependency);
            }
            JobHandle combinedHandles = JobHandle.CombineDependencies(handlesToCombine);

            if (FlowFieldUtilities.DebugMode) { combinedHandles.Complete(); }
            _islandReconfigHandle.Add(combinedHandles);
            return combinedHandles;
        }
    }

}