﻿using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

namespace FlowFieldNavigation
{

    internal struct PreallocationPack
    {
        internal NativeList<ActivePortal> PortalSequence;
        internal NativeList<int> PortalSequenceBorders;
        internal NativeArray<PortalTraversalData> PortalTraversalDataArray;
        internal UnsafeList<DijkstraTile> TargetSectorCosts;
        internal UnsafeList<int> SectorToPicked;
        internal NativeList<int> PickedToSector;
        internal NativeQueue<int> PortalTraversalFastMarchingQueue;
        internal NativeList<int> AStartTraverseIndexList;
        internal NativeList<int> TargetSectorPortalIndexList;
        internal NativeList<int> SourcePortalIndexList;
        internal UnsafeList<PathSectorState> SectorStateTable;
        internal NativeList<int> SectorStartIndexListToCalculateIntegration;
        internal NativeList<int> SectorStartIndexListToCalculateFlow;
        internal NativeList<int> NotActivePortalList;
        internal NativeReference<int> FlowFieldLength;
        internal NativeReference<int> NewPickedSectorStartIndex;
        internal NativeReference<int> PathAdditionSequenceBorderStartIndex;
        internal UnsafeList<FlowData> DynamicAreaFlowFieldCalculationBuffer;
        internal UnsafeList<FlowData> DynamicAreaFlowField;
        internal NativeList<IntegrationTile> DynamicAreaIntegrationField;
        internal UnsafeList<SectorFlowStart> DynamicAreaSectorFlowStartList;
        internal UnsafeList<SectorFlowStart> DynamicAreaSectorFlowStartCalculationList;
        internal NativeReference<SectorsWihinLOSArgument> SectorsWithinLOSState;
        internal SectorBitArray SectorBitArray;
        internal NativeList<int> DijkstraStartIndicies;
    }


}