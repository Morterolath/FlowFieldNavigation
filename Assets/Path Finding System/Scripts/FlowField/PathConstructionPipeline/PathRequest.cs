﻿using Unity.Mathematics;

public struct PathRequest
{
    public float2 Destination;
    public int TargetAgentIndex;
    public int DerivedRequestStartIndex;
    public int DerivedRequestCount;
    public PathType Type;
    public ushort OffsetMask;

    public PathRequest(float2 destination)
    {
        Destination = destination;
        Type = PathType.StaticDestination;
        TargetAgentIndex = 0;
        DerivedRequestCount = 0;
        DerivedRequestStartIndex = 0;
        OffsetMask = 0;
    }

    public PathRequest(int targetAgentIndex)
    {
        TargetAgentIndex = targetAgentIndex;
        Type = PathType.DynamicDestination;
        Destination = 0;
        DerivedRequestCount = 0;
        DerivedRequestStartIndex = 0;
        OffsetMask = 0;
    }
}

public struct OffsetDerivedPathRequest
{
    public float2 Destination;
    public int TargetAgentIndex;
    public int DerivedFialRequestStartIndex;
    public int DerivedFinalRequestCount;
    public int Offset;
    public PathType Type;

    public OffsetDerivedPathRequest(PathRequest initialPathRequest, int offset)
    {
        Destination = initialPathRequest.Destination;
        TargetAgentIndex = initialPathRequest.TargetAgentIndex;
        Offset = offset;
        Type = initialPathRequest.Type;
        DerivedFialRequestStartIndex = 0;
        DerivedFinalRequestCount = 0;
    }

    public bool IsCreated()
    {
        return Type != PathType.None;
    }
}

public struct FinalPathRequest
{
    public float2 Destination;
    public int TargetAgentIndex;
    public int SourcePositionStartIndex;
    public int SourceCount;
    public int Offset;
    public int PathIndex;
    public int SourceIsland;
    public PathType Type;

    public FinalPathRequest(OffsetDerivedPathRequest derivedReq, int sourceIsland)
    {
        Destination = derivedReq.Destination;
        Type = derivedReq.Type;
        Offset = derivedReq.Offset;
        TargetAgentIndex = derivedReq.TargetAgentIndex;
        SourceIsland = sourceIsland;

        SourceCount = 0;
        SourcePositionStartIndex = 0;
        PathIndex = 0;
    }
    public bool IsValid() => SourceCount != 0;
}
public struct PostponedPathRequests
{
    public float2 Destination;
    public int TargetAgentIndex;
    public PathType Type;

    public PostponedPathRequests(FinalPathRequest finalRequest)
    {
        Destination = finalRequest.Destination;
        Type = finalRequest.Type;
        TargetAgentIndex = finalRequest.TargetAgentIndex;
    }
}