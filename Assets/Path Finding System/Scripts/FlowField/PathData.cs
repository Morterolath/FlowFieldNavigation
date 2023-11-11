﻿using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
public struct PathData
{
    public float2 Target;
    public PathTask Task;
    public PathState State;
    public UnsafeList<FlowData> FlowField;
    public UnsafeList<PathSectorState> SectorStateTable;
    public UnsafeList<int> SectorToPicked;
    public int FlowRequestSourceStart;
    public int FlowRequestSourceCount;
}
public enum PathTask : byte
{
    FlowRequest = 1,
}