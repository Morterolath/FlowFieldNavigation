using Unity.Collections;


namespace FlowFieldNavigation
{
    internal struct PathPortalTraversalData
    {
        internal NativeList<PortalTraversalData> GoalDataList;
        internal NativeList<ActivePortal> PortalSequence;
        internal NativeList<Slice> PortalSequenceSlices;
        internal NativeList<int> TargetSectorPortalIndexList;
        internal NativeList<int> SourcePortalIndexList;
        internal NativeReference<int> PathAdditionSequenceSliceStartIndex;
        internal NativeReference<int> NewPickedSectorStartIndex;
        internal NativeList<int> DiskstraStartIndicies;
        internal NativeList<int> NewExploredPortalIndicies;
        internal NativeList<PortalTraversalDataRecord> PortalDataRecords;
        internal NativeList<int> NewPathUpdateSeedIndicies;
    }


}