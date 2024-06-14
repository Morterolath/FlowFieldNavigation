using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Burst;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal struct LineCastJob : IJobParallelFor
    {
        internal float TileSize;
        internal float2 FieldGridStartPos;
        internal int SectorColAmount;
        internal int SectorMatrixColAmount;
        internal int SectorTileAmount;
        [ReadOnly] internal NativeArray<byte> CostField;
        [ReadOnly] internal NativeArray<LineCastData> LinesToCast;
        [WriteOnly] internal NativeArray<bool> ResultFlags;
        public void Execute(int index)
        {
            LineCastData lineCastData = LinesToCast[index];
            lineCastData.MinDistance = math.max(0, lineCastData.MinDistance);
            float2 start = lineCastData.StartPoint;
            float2 end = lineCastData.EndPoint;
            float2 endToStart = start - end;
            float endToStartLength = math.length(endToStart);
            if(endToStartLength <= lineCastData.MinDistance) { ResultFlags[index] = true; return; }
            float2 startToEndNormalized = math.select(endToStart / endToStartLength, 0f, endToStartLength == 0); ;
            end = end + startToEndNormalized * lineCastData.MinDistance;
            ClipLineIfNecessary(ref start, ref end);
            ResultFlags[index] = !LineCast(start, end);
        }

        void ClipLineIfNecessary(ref float2 p1, ref float2 p2)
        {

        }
        bool LineCast(float2 start, float2 end)
        {
            start += math.select(0f, 0.0001f, start.x == end.x);
            float2 leftPoint = math.select(end, start, start.x < end.x);
            float2 rigthPoint = math.select(start, end, start.x < end.x);
            float xMin = leftPoint.x;
            float xMax = rigthPoint.x;
            int2 leftIndex = FlowFieldUtilities.PosTo2D(leftPoint, TileSize, FieldGridStartPos);
            int2 rightIndex = FlowFieldUtilities.PosTo2D(rigthPoint, TileSize, FieldGridStartPos);

            float deltaX = (leftPoint.x - rigthPoint.x);
            float x1 = rigthPoint.x;
            float deltaY = (leftPoint.y - rigthPoint.y);
            float y1 = rigthPoint.y;
            for (int xIndex = leftIndex.x; xIndex <= rightIndex.x; xIndex++)
            {
                float xLeft = FieldGridStartPos.x + xIndex * TileSize;
                float xRight = xLeft + TileSize;
                xLeft = math.max(xLeft, xMin);
                xRight = math.min(xRight, xMax);

                float tLeft = (xLeft - x1) / deltaX;
                float tRight = (xRight - x1) / deltaX;
                float yLeft = y1 + deltaY * tLeft;
                float yRight = y1 + deltaY * tRight;

                int yIndexLeft = (int)math.floor((yLeft - FieldGridStartPos.y) / TileSize);
                int yIndexRight = (int)math.floor((yRight - FieldGridStartPos.y) / TileSize);
                int yIndexMin = math.min(yIndexLeft, yIndexRight);
                int yIndexMax = math.max(yIndexLeft, yIndexRight);

                for (int yIndex = yIndexMin; yIndex <= yIndexMax; yIndex++)
                {
                    int2 indexToPlot = new int2(xIndex, yIndex);
                    LocalIndex1d localToPlot = FlowFieldUtilities.GetLocal1D(indexToPlot, SectorColAmount, SectorMatrixColAmount);
                    if (CostField[localToPlot.sector * SectorTileAmount + localToPlot.index] == byte.MaxValue)
                    {
                        return true;
                    }
                }
            }
            return false;
        }
    }
}
