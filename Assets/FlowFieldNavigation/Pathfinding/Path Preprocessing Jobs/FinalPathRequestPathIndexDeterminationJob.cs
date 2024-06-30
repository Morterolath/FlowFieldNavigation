using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct FinalPathRequestPathIndexDeterminationJob : IJob
    {
        internal int CurrentPathListLength;
        internal NativeList<int> UnusedPathIndexList;
        internal NativeList<FinalPathRequest> FinalPathRequests;
        internal NativeReference<int> NewPathListLength;
        public void Execute()
        {
            NativeArray<FinalPathRequest> finalPathRequestsAsArray = FinalPathRequests.AsArray();
            NewPathListLength.Value = CurrentPathListLength;
            for(int i = 0; i < finalPathRequestsAsArray.Length; i++)
            {
                FinalPathRequest request = finalPathRequestsAsArray[i];
                if (!request.IsValid()) { continue; }
                if (UnusedPathIndexList.IsEmpty)
                {
                    request.PathIndex = CurrentPathListLength;
                    finalPathRequestsAsArray[i] = request;
                    CurrentPathListLength++;
                    continue;
                }
                request.PathIndex = UnusedPathIndexList[UnusedPathIndexList.Length - 1];
                finalPathRequestsAsArray[i] = request;
                UnusedPathIndexList.RemoveAtSwapBack(UnusedPathIndexList.Length - 1);
            }
            NewPathListLength.Value = CurrentPathListLength;
        }
    }
}
