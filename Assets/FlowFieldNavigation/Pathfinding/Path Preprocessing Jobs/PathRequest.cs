using Unity.Mathematics;

namespace FlowFieldNavigation
{

    internal struct PathRequest
    {
        internal float2 Destination;
        internal int TargetAgentIndex;
        internal int DerivedRequestStartIndex;
        internal int DerivedRequestCount;
        internal DestinationType Type;
        internal ushort OffsetMask;
        internal int FlockIndex;
        internal bool ReconstructionFlag;
        internal float Range;
        internal PathRequest(float2 destination, float range)
        {
            Destination = destination;
            Type = DestinationType.StaticDestination;
            Range = range;
            TargetAgentIndex = 0;
            DerivedRequestCount = 0;
            DerivedRequestStartIndex = 0;
            OffsetMask = 0;
            FlockIndex = 0;
            ReconstructionFlag = false;
        }

        internal PathRequest(int targetAgentIndex, float range)
        {
            TargetAgentIndex = targetAgentIndex;
            Type = DestinationType.DynamicDestination;
            Range = range;
            Destination = 0;
            DerivedRequestCount = 0;
            DerivedRequestStartIndex = 0;
            OffsetMask = 0;
            FlockIndex = 0;
            ReconstructionFlag = false;
        }
        internal PathRequest(PathRequestRecord requestRecord)
        {
            Destination = requestRecord.Destination;
            Type = requestRecord.Type;
            TargetAgentIndex = requestRecord.TargetAgent;
            Range = requestRecord.Range;
            DerivedRequestCount = 0;
            DerivedRequestStartIndex = 0;
            OffsetMask = 0;
            FlockIndex = 0;
            ReconstructionFlag = false;
        }
    }

    internal struct OffsetDerivedPathRequest
    {
        internal float2 Destination;
        internal int TargetAgentIndex;
        internal int DerivedFialRequestStartIndex;
        internal int DerivedFinalRequestCount;
        internal int Offset;
        internal DestinationType Type;
        internal int FlockIndex;
        internal bool ReconstructionFlag;
        internal float Range;

        internal OffsetDerivedPathRequest(PathRequest initialPathRequest, int offset)
        {
            Destination = initialPathRequest.Destination;
            TargetAgentIndex = initialPathRequest.TargetAgentIndex;
            Offset = offset;
            Type = initialPathRequest.Type;
            DerivedFialRequestStartIndex = 0;
            DerivedFinalRequestCount = 0;
            FlockIndex = initialPathRequest.FlockIndex;
            ReconstructionFlag = initialPathRequest.ReconstructionFlag;
            Range = initialPathRequest.Range;
        }

        internal bool IsCreated()
        {
            return Type != DestinationType.None;
        }
    }

    internal struct FinalPathRequest
    {
        internal float2 Destination;
        internal float2 DesiredDestination;
        internal int TargetAgentIndex;
        internal int SourcePositionStartIndex;
        internal int SourceCount;
        internal int Offset;
        internal int PathIndex;
        internal int SourceIsland;
        internal DestinationType Type;
        internal int FlockIndex;
        internal bool ReconstructionFlag;
        internal float Range;
        internal float DesiredRange;

        internal FinalPathRequest(OffsetDerivedPathRequest derivedReq, int sourceIsland)
        {
            Destination = derivedReq.Destination;
            DesiredDestination = derivedReq.Destination;
            Type = derivedReq.Type;
            Offset = derivedReq.Offset;
            TargetAgentIndex = derivedReq.TargetAgentIndex;
            SourceIsland = sourceIsland;
            FlockIndex = derivedReq.FlockIndex;
            ReconstructionFlag = derivedReq.ReconstructionFlag;
            Range = derivedReq.Range;
            DesiredRange = derivedReq.Range;

            SourceCount = 0;
            SourcePositionStartIndex = 0;
            PathIndex = 0;
        }
        internal bool IsValid() => SourceCount != 0;
    }

}