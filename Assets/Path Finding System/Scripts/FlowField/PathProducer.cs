﻿using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using System.Diagnostics;
using System.IO;
using UnityEditor.PackageManager.Requests;
using UnityEngine.UIElements;

public class PathProducer
{
    public List<Path> ProducedPaths;
    public NativeList<int> ProducedPathSubscribers;
    Stack<int> _removedPathIndicies;
    NativeList<PathData> _pathDataList;
    NativeList<FlowFieldCalculationBufferParent> _flowFieldCalculationBuffers;
    NativeList<int> _losCalculatedPaths;

    FieldProducer _fieldProducer;
    PathPreallocator _preallocator;
    int _columnAmount;
    int _rowAmount;
    float _tileSize;
    int _sectorTileAmount;
    int _sectorMatrixColAmount;
    int _sectorMatrixRowAmount;

    public PathProducer(PathfindingManager pathfindingManager)
    {
        _fieldProducer = pathfindingManager.FieldProducer;
        _columnAmount = pathfindingManager.ColumnAmount;
        _rowAmount = pathfindingManager.RowAmount;
        _tileSize = pathfindingManager.TileSize;
        _sectorTileAmount = pathfindingManager.SectorColAmount;
        _sectorMatrixColAmount = _columnAmount / _sectorTileAmount;
        _sectorMatrixRowAmount = _rowAmount / _sectorTileAmount;
        ProducedPaths = new List<Path>(1);
        _preallocator = new PathPreallocator(_fieldProducer, _sectorTileAmount * _sectorTileAmount, _sectorMatrixColAmount * _sectorMatrixRowAmount);
        _removedPathIndicies = new Stack<int>();
        _pathDataList = new NativeList<PathData>(Allocator.Persistent);
        ProducedPathSubscribers = new NativeList<int>(Allocator.Persistent);
        _flowFieldCalculationBuffers = new NativeList<FlowFieldCalculationBufferParent>(Allocator.Persistent);
        _losCalculatedPaths = new NativeList<int>(Allocator.Persistent);
    }
    public void Update()
    {
        for (int i = 0; i < ProducedPaths.Count; i++)
        {
            Path path = ProducedPaths[i];
            if(path.State == PathState.Removed) { continue; }
            int subsciriber = ProducedPathSubscribers[i];
            if(subsciriber == 0 && path.IsCalculated) { path.State = PathState.ToBeDisposed; }
            if (path.State == PathState.ToBeDisposed && path.IsCalculated)
            {
                path.Dispose();
                path.State = PathState.Removed;
                _removedPathIndicies.Push(i);
                PreallocationPack preallocations = new PreallocationPack()
                {
                    SectorToPicked = path.SectorToPicked,
                    PickedToSector = path.PickedToSector,
                    PortalSequence = path.PortalSequence,
                    PortalSequenceBorders = path.PortalSequenceBorders,
                    PortalTraversalDataArray = path.PortalTraversalDataArray,
                    TargetSectorCosts = path.TargetSectorCosts,
                    FlowFieldLength = path.FlowFieldLength,
                    SourcePortalIndexList = path.SourcePortalIndexList,
                    AStartTraverseIndexList = path.AStartTraverseIndexList,
                    TargetSectorPortalIndexList = path.TargetSectorPortalIndexList,
                    PortalTraversalFastMarchingQueue = path.PortalTraversalFastMarchingQueue,
                    SectorStateTable = path.SectorStateTable,
                };

                _preallocator.SendPreallocationsBack(ref preallocations, path.ActiveWaveFrontList, path.FlowField, path.IntegrationField, path.Offset);
            }
        }
        _preallocator.CheckForDeallocations();
    }
    public int CreatePath(PathRequest request)
    {
        int2 destinationIndex = new int2(Mathf.FloorToInt(request.Destination.x / _tileSize), Mathf.FloorToInt(request.Destination.y / _tileSize));
        PreallocationPack preallocations = _preallocator.GetPreallocations(request.Offset);

        int pathIndex;
        if (_removedPathIndicies.Count != 0) { pathIndex = _removedPathIndicies.Pop(); }
        else { pathIndex = ProducedPaths.Count; }

        Path producedPath = new Path()
        {
            IsCalculated = true,
            Id = pathIndex,
            PickedToSector = preallocations.PickedToSector,
            PortalSequenceBorders = preallocations.PortalSequenceBorders,
            TargetIndex = destinationIndex,
            TargetSectorCosts = preallocations.TargetSectorCosts,
            Destination = request.Destination,
            State = PathState.Clean,
            Offset = request.Offset,
            PortalSequence = preallocations.PortalSequence,
            PortalTraversalDataArray = preallocations.PortalTraversalDataArray,
            SectorToPicked = preallocations.SectorToPicked,
            FlowFieldLength = preallocations.FlowFieldLength,
            SourcePortalIndexList = preallocations.SourcePortalIndexList,
            AStartTraverseIndexList = preallocations.AStartTraverseIndexList,
            TargetSectorPortalIndexList = preallocations.TargetSectorPortalIndexList,
            PortalTraversalFastMarchingQueue = preallocations.PortalTraversalFastMarchingQueue,
            SectorStateTable = preallocations.SectorStateTable,
            PathAdditionSequenceBorderStartIndex = new NativeArray<int>(1, Allocator.Persistent),
            NotActivePortalList = new NativeList<int>(Allocator.Persistent),
            NewPickedSectorStartIndex = new NativeArray<int>(1, Allocator.Persistent),
            SectorFlowStartIndiciesToCalculateIntegration = new NativeList<int>(Allocator.Persistent),
            SectorFlowStartIndiciesToCalculateFlow = new NativeList<int>(Allocator.Persistent),
            SectorWithinLOSState = new NativeArray<SectorsWihinLOSArgument>(1, Allocator.Persistent),
        };

        if (ProducedPaths.Count == pathIndex)
        {
            ProducedPaths.Add(producedPath);
            ProducedPathSubscribers.Add(request.AgentCount);
        }
        else
        {
            ProducedPaths[pathIndex] = producedPath;
            ProducedPathSubscribers[pathIndex] = request.AgentCount;
        }

        return pathIndex;
    }
    public void FinalizePathBuffers(int pathIndex)
    {
        Path path = ProducedPaths[pathIndex];
        NativeArray<int> flowFieldLength = path.FlowFieldLength;
        path.FlowField = _preallocator.GetFlowField(flowFieldLength[0]);
        path.IntegrationField = _preallocator.GetIntegrationField(flowFieldLength[0]);
        path.LOSMap = new UnsafeLOSBitmap(flowFieldLength[0], Allocator.Persistent, NativeArrayOptions.ClearMemory);
        path.ActiveWaveFrontList = _preallocator.GetActiveWaveFrontListPersistent(path.PickedToSector.Length);
    }
    public void ResizeActiveWaveFrontList(int newLength, NativeList<UnsafeList<ActiveWaveFront>> activeWaveFrontList)
    {
        _preallocator.AddToActiveWaveFrontList(newLength, activeWaveFrontList);
    }
    public void GetCurrentPathData(NativeList<PathData> currentPathData)
    {
        currentPathData.Length = ProducedPaths.Count;

        for(int i = 0; i < ProducedPaths.Count; i++)
        {
            Path path = ProducedPaths[i];
            if(path.State == PathState.Removed)
            {
                currentPathData[i] = new PathData()
                {
                    State = PathState.Removed,
                };
                continue;
            }
            currentPathData[i] = new PathData()
            {
                State = path.State,
                Target = path.Destination,
                Task = 0,
                SectorStateTable = path.SectorStateTable,
                SectorToPicked = path.SectorToPicked,
                FlowField = path.FlowField,
            };
        }
    }
}