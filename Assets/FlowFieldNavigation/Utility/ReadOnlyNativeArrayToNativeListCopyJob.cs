﻿using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;

[BurstCompile]
internal struct ReadOnlyNativeArrayToNativeListCopyJob<T> : IJob where T : unmanaged
{
    internal NativeArray<T>.ReadOnly Source;
    internal NativeList<T> Destination;
    public void Execute()
    {
        Destination.Length = Source.Length;
        NativeArray<T> destinationAsArray = Destination;
        for(int i = 0; i < Source.Length; i++)
        {
            destinationAsArray[i] = Source[i];
        }
    }
}
