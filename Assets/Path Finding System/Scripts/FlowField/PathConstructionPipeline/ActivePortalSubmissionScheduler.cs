﻿using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

internal class ActivePortalSubmissionScheduler
{
    PathfindingManager _pathfindingManager;
    PathProducer _pathProducer;

    public ActivePortalSubmissionScheduler(PathfindingManager pathfindingManager)
    {
        _pathfindingManager = pathfindingManager;
        _pathProducer = pathfindingManager.PathProducer;
    }

    public HandleWithReqAndPathIndex ScheduleActivePortalSubmission(int pathIndex, int requestIndex)
    {
        Path path = _pathProducer.ProducedPaths[pathIndex];
        FieldGraph pickedFieldGraph = _pathfindingManager.FieldProducer.GetFieldGraphWithOffset(path.Offset);

        //ACTIVE WAVE FRONT SUBMISSION
        ActivePortalSubmitJob submitJob = new ActivePortalSubmitJob()
        {
            SectorColAmount = FlowFieldUtilities.SectorColAmount,
            SectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount,
            SectorMatrixRowAmount = FlowFieldUtilities.SectorMatrixRowAmount,
            SectorRowAmount = FlowFieldUtilities.SectorRowAmount,
            SectorTileAmount = FlowFieldUtilities.SectorTileAmount,
            FieldColAmount = FlowFieldUtilities.FieldColAmount,
            TargetIndex2D = path.TargetIndex,

            PortalEdges = pickedFieldGraph.PorToPorPtrs,
            SectorToPicked = path.SectorToPicked,
            PickedToSector = path.PickedToSector,
            PortalSequence = path.PortalSequence,
            PortalSequenceBorders = path.PortalSequenceBorders,
            WinToSecPtrs = pickedFieldGraph.WinToSecPtrs,
            PortalNodes = pickedFieldGraph.PortalNodes,
            WindowNodes = pickedFieldGraph.WindowNodes,
            ActiveWaveFrontListArray = path.ActiveWaveFrontList,
            NotActivatedPortals = path.NotActivePortalList,
        };
        JobHandle submitHandle = submitJob.Schedule();

        if (FlowFieldUtilities.DebugMode) { submitHandle.Complete(); }

        return new HandleWithReqAndPathIndex(submitHandle, pathIndex, requestIndex);
    }
}
