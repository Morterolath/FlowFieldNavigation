﻿using System;
using System.Numerics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
public struct DynamicArea
{
    public NativeList<IntegrationTile> IntegrationField;
    public UnsafeList<FlowData> FlowFieldCalculationBuffer;

    public void Dispose()
    {
        if (IntegrationField.IsCreated) { IntegrationField.Dispose(); }
        if (FlowFieldCalculationBuffer.IsCreated) { FlowFieldCalculationBuffer.Dispose(); }
    }
}