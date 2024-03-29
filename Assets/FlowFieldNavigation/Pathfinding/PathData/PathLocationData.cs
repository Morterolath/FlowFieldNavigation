﻿using Unity.Collections.LowLevel.Unsafe;


namespace FlowFieldNavigation
{
    internal struct PathLocationData
    {
        internal UnsafeList<int> SectorToPicked;
        internal UnsafeList<SectorFlowStart> DynamicAreaPickedSectorFlowStarts;
    }

}