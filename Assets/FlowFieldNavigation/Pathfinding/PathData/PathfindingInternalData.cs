﻿using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

public struct PathfindingInternalData
{
    public DynamicArea DynamicArea;
    public NativeList<int> PickedSectorList;
    public NativeList<IntegrationTile> IntegrationField;
    public NativeArray<int> FlowFieldLength;
    public NativeQueue<int> PortalTraversalQueue;
    public NativeList<UnsafeList<ActiveWaveFront>> ActivePortalList;
    public NativeList<int> NotActivePortalList;
    public NativeList<int> SectorFlowStartIndiciesToCalculateIntegration;
    public NativeList<int> SectorFlowStartIndiciesToCalculateFlow;
    public NativeArray<SectorsWihinLOSArgument> SectorWithinLOSState;
    public void Dispose()
    {
        NotActivePortalList.Dispose();
        SectorFlowStartIndiciesToCalculateFlow.Dispose();
        SectorFlowStartIndiciesToCalculateIntegration.Dispose();
        SectorWithinLOSState.Dispose();
        DynamicArea.Dispose();
    }
}

public enum PathState : byte
{
    Clean = 0,
    ToBeUpdated = 1,
    ToBeDisposed = 2,
    Removed,
}
public enum SectorsWihinLOSArgument : byte
{
    None = 0,
    RequestedSectorWithinLOS = 1,
    AddedSectorWithinLOS = 2,
};