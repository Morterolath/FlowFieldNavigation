using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct CurrentPathReconstructionDeterminationJob : IJob
    {
        [ReadOnly] internal NativeArray<float3> AgentPositions;
        [ReadOnly] internal NativeArray<PathState> PathStateArray;
        [ReadOnly] internal NativeArray<PathDestinationData> PathDestinationDataArray;
        [ReadOnly] internal NativeArray<int> PathFlockIndexArray;
        [ReadOnly] internal NativeArray<int> AgentCurPathIndicies;
        [ReadOnly] internal NativeArray<float> PathDesiredRanges;
        internal NativeArray<int> AgentNewPathIndicies;
        internal NativeArray<PathRoutineData> PathRoutineDataArray;
        internal NativeList<PathRequest> PathRequests;
        internal NativeHashMap<int, int> FlockIndexToPathRequestIndex;
        public void Execute()
        {
            //CHECK IF DYNAMIC PATH TARGETS ARE MOVED
            for (int i = 0; i < PathRoutineDataArray.Length; i++)
            {
                PathState curPathState = PathStateArray[i];
                PathDestinationData curDestinationData = PathDestinationDataArray[i];
                PathRoutineData curRoutineData = PathRoutineDataArray[i];
                int pathFlockIndex = PathFlockIndexArray[i];
                bool shouldNotConsider = curPathState == PathState.Removed;
                bool shouldNotReconstruct = curRoutineData.DestinationState != DynamicDestinationState.OutOfReach && (curRoutineData.Task & PathTask.Reconstruct) != PathTask.Reconstruct;
                if (shouldNotConsider || shouldNotReconstruct) { continue; }
                curRoutineData.PathReconstructionFlag = true;
                PathRoutineDataArray[i] = curRoutineData;

                if (FlockIndexToPathRequestIndex.ContainsKey(pathFlockIndex)) { continue; }
                FlockIndexToPathRequestIndex.Add(pathFlockIndex, PathRequests.Length);
                if (curDestinationData.DestinationType == DestinationType.DynamicDestination)
                {
                    PathRequest reconReq = new PathRequest(curDestinationData.TargetAgentIndex, PathDesiredRanges[i]);
                    float3 targetAgentPos = AgentPositions[reconReq.TargetAgentIndex];
                    float2 targetAgentPos2 = new float2(targetAgentPos.x, targetAgentPos.z);
                    reconReq.Destination = targetAgentPos2;
                    reconReq.FlockIndex = pathFlockIndex;
                    reconReq.ReconstructionFlag = true;
                    PathRequests.Add(reconReq);
                }
                else
                {
                    PathRequest reconReq = new PathRequest(curDestinationData.DesiredDestination, PathDesiredRanges[i]);
                    reconReq.FlockIndex = PathFlockIndexArray[i];
                    reconReq.ReconstructionFlag = true;
                    PathRequests.Add(reconReq);

                }
            }
            //SET NEW PATHS OF AGENTS WHOSE PATHS ARE RECONSTRUCTED
            for (int i = 0; i < AgentCurPathIndicies.Length; i++)
            {
                int curPathIndex = AgentCurPathIndicies[i];
                if (curPathIndex == -1) { continue; }
                int newPathIndex = AgentNewPathIndicies[i];
                if(newPathIndex != -1) { continue; }
                PathRoutineData curRoutineData = PathRoutineDataArray[curPathIndex];
                PathState curPathState = PathStateArray[curPathIndex];
                if (curPathState == PathState.Removed || !curRoutineData.PathReconstructionFlag) { continue; }
                int pahtFlockIndex = PathFlockIndexArray[curPathIndex];
                if (FlockIndexToPathRequestIndex.TryGetValue(pahtFlockIndex, out int pathRequestIndex))
                {
                    AgentNewPathIndicies[i] = pathRequestIndex;
                }
            }
        }
    }


}