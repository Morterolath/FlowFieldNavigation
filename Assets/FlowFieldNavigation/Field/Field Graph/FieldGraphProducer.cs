using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using System.Diagnostics;
using UnityEngine;

namespace FlowFieldNavigation
{

    internal class FieldGraphProducer
    {
        FieldGraph[] _fieldGraphs;

        internal FieldGraphProducer() { }
        internal void ProduceFieldGraphs(CostField[] costFields)
        {
            //ALLOCATE FIELD GRAPHS
            _fieldGraphs = new FieldGraph[costFields.Length];
            for (int i = 0; i < _fieldGraphs.Length; i++)
            {
                _fieldGraphs[i] = new FieldGraph(i);
            }

            //CONFIGURE FIELD GRAPHS
            //field graph config job is weird. It breaks when you want to schedule
            //field graph config job of multiple fields. No unsafe stuff used. Weird.
            NativeList<JobHandle> combinedHandles = new NativeList<JobHandle>(Allocator.Persistent);
            for (int i = 0; i < _fieldGraphs.Length; i++)
            {
                FieldGraphConfigurationJob _fieldGraphConfigJob = _fieldGraphs[i].GetConfigJob(costFields[i].Costs);
                IslandConfigurationJob islandConfigJob = _fieldGraphs[i].GetIslandConfigJob(costFields[i].Costs);
                JobHandle fieldHandle = _fieldGraphConfigJob.Schedule();
                JobHandle islandHandle = islandConfigJob.Schedule(fieldHandle);
                islandHandle.Complete();
                //combinedHandles.Add(islandHandle);
            }
            JobHandle.CompleteAll(combinedHandles.AsArray());
            combinedHandles.Dispose();
        }
        internal NativeArray<UnsafeListReadOnly<PortalNode>> GetAllPortalNodeArrays(Allocator allocator)
        {
            NativeArray<UnsafeListReadOnly<PortalNode>> portalNodes = new NativeArray<UnsafeListReadOnly<PortalNode>>(_fieldGraphs.Length, allocator);
            for(int i = 0; i < _fieldGraphs.Length; i++)
            {
                portalNodes[i] = _fieldGraphs[i].GetPortalNodesAsUnsafeListReadonly();
            }
            return portalNodes;
        }
        internal NativeArray<UnsafeListReadOnly<SectorNode>> GetAllSectorNodeArrays(Allocator allocator)
        {
            NativeArray<UnsafeListReadOnly<SectorNode>> sectorNodes = new NativeArray<UnsafeListReadOnly<SectorNode>>(_fieldGraphs.Length, allocator);
            for (int i = 0; i < _fieldGraphs.Length; i++)
            {
                sectorNodes[i] = _fieldGraphs[i].GetSectorNodesAsUnsafeListReadonly();
            }
            return sectorNodes;

        }
        internal NativeArray<UnsafeListReadOnly<WindowNode>> GetAllWindowNodeArrays(Allocator allocator)
        {
            NativeArray<UnsafeListReadOnly<WindowNode>> windowNodes = new NativeArray<UnsafeListReadOnly<WindowNode>>(_fieldGraphs.Length, allocator);
            for (int i = 0; i < _fieldGraphs.Length; i++)
            {
                windowNodes[i] = _fieldGraphs[i].GetWindowNodesAsUnsafeListReadonly();
            }
            return windowNodes;

        }
        internal NativeArray<UnsafeListReadOnly<int>> GetAllSecToWinPtrArrays(Allocator allocator)
        {
            NativeArray<UnsafeListReadOnly<int>> secToWinPtrs = new NativeArray<UnsafeListReadOnly<int>>(_fieldGraphs.Length, allocator);
            for (int i = 0; i < _fieldGraphs.Length; i++)
            {
                secToWinPtrs[i] = _fieldGraphs[i].GetSecToWinPtrsAsUnsafeListReadonly();
            }
            return secToWinPtrs;
        }
        internal FieldGraph GetFieldGraphWithOffset(int offset)
        {
            return _fieldGraphs[offset];
        }
        internal FieldGraph[] GetAllFieldGraphs()
        {
            FieldGraph[] newarray = new FieldGraph[_fieldGraphs.Length];
            for (int i = 0; i < newarray.Length; i++)
            {
                newarray[i] = _fieldGraphs[i];
            }
            return newarray;
        }
        internal NativeArray<IslandFieldProcessor> GetAllIslandFieldProcessors(Allocator allocator)
        {
            NativeArray<IslandFieldProcessor> islandFieldProcessors = new NativeArray<IslandFieldProcessor>(_fieldGraphs.Length, allocator);
            for (int i = 0; i < _fieldGraphs.Length; i++)
            {
                islandFieldProcessors[i] = _fieldGraphs[i].GetIslandFieldProcessor();
            }
            return islandFieldProcessors;
        }
        internal void DisposeAll()
        {
            for (int i = 0; i < _fieldGraphs.Length; i++)
            {
                _fieldGraphs[i].DisposeAll();
            }
            _fieldGraphs = null;
        }
    }


}