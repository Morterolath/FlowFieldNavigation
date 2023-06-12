﻿using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

public class Path
{
    public int Offset;
    public PathState State;
    public bool IsCalculated = false;
    public Vector3 Destination;
    public NativeArray<Vector3> Sources;
    public NativeArray<float> PortalDistances;
    public NativeArray<int> ConnectionIndicies;
    public NativeArray<PortalMark> PortalMarks;
    public NativeList<PortalSequence> PortalSequence;
    public NativeList<int> PickedSectors;
    public NativeArray<UnsafeList<IntegrationTile>> IntegrationField;
    public NativeArray<FlowData> FlowField;
    public NativeQueue<int> BlockedWaveFronts;

    public void Dispose()
    {
        BlockedWaveFronts.Dispose();
        Sources.Dispose();
        PortalDistances.Dispose();
        ConnectionIndicies.Dispose();
        PortalMarks.Dispose();
        PortalSequence.Dispose();
        PickedSectors.Dispose();
        for(int i = 0; i < IntegrationField.Length; i++)
        {
            IntegrationField[i].Dispose();
        }
        IntegrationField.Dispose();
        FlowField.Dispose();
    }
    public void SetState(PathState state)
    {
        State = state;
    }
}
public enum PathState : byte
{
    Clean = 0,
    ToBeUpdated = 1,
    ToBeDisposed = 2,
}