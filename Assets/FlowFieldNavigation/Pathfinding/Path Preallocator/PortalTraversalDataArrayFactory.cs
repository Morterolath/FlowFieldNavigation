using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Unity.Collections;
using Unity.Jobs;

namespace FlowFieldNavigation
{

    internal class PortalTraversalDataArrayFactory
    {
        List<NativeArray<PortalTraversalData>>[] _preallocationMatrix;
        List<CleaningHandle> _cleaningHandles;
        int[] _portalNodeAmounts;
        internal PortalTraversalDataArrayFactory(FieldGraph[] producedFieldGraphs)
        {
            _preallocationMatrix = new List<NativeArray<PortalTraversalData>>[producedFieldGraphs.Length];
            _portalNodeAmounts = new int[producedFieldGraphs.Length];
            for (int i = 0; i < _portalNodeAmounts.Length; i++)
            {
                _portalNodeAmounts[i] = producedFieldGraphs[i].PortalNodes.Length;
            }
            for (int i = 0; i < _preallocationMatrix.Length; i++)
            {
                _preallocationMatrix[i] = new List<NativeArray<PortalTraversalData>>();
            }
            _cleaningHandles = new List<CleaningHandle>();
        }
        internal void CheckForCleaningHandles()
        {
            for (int i = _cleaningHandles.Count - 1; i >= 0; i--)
            {
                CleaningHandle cleaningHandle = _cleaningHandles[i];
                if (cleaningHandle.handle.IsCompleted)
                {
                    cleaningHandle.handle.Complete();
                    _preallocationMatrix[cleaningHandle.Offset].Add(cleaningHandle.Array);
                    _cleaningHandles.RemoveAtSwapBack(i);
                }
            }
        }

        struct CleaningHandle
        {
            internal NativeArray<PortalTraversalData> Array;
            internal int Offset;
            internal JobHandle handle;
        }
    }


}