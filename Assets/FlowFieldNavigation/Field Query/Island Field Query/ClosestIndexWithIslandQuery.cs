using Unity.Mathematics;

namespace FlowFieldNavigation
{
    internal static class ClosestIndexWithIslandQuery
    {
        internal static int2 GetClosestIndexWithIsland(
            float2 initialPoint,
            int desiredIsland,
            float tileSize,
            float2 fieldGridStartPos,
            int sectorTileAmount,
            int sectorColAmount,
            int sectorRowAmount,
            int sectorMatrixColAmount,
            int fieldRowAmount,
            int fieldColAmount,
            IslandFieldProcessor islandFieldProcessors,
            UnsafeListReadOnly<byte> costField)
        {
            int2 foundIndex = GetClosestIndexAtIsland(
                initialPoint,
                desiredIsland,
                tileSize,
                fieldGridStartPos,
                sectorTileAmount,
                sectorColAmount,
                sectorRowAmount,
                sectorMatrixColAmount,
                fieldRowAmount,
                fieldColAmount,
                islandFieldProcessors,
                costField,
                out int2 lastTopLeft, out int2 lastTopRight, out int2 lastBotLeft, out int2 lastBotRight);
             foundIndex = MakeSureExpandedDestinationIsClosest(
                lastBotLeft,
                lastBotRight,
                lastTopLeft,
                lastTopRight,
                foundIndex,
                initialPoint,
                desiredIsland,
                tileSize,
                fieldGridStartPos,
                fieldRowAmount,
                fieldColAmount,
                sectorColAmount,
                sectorMatrixColAmount,
                sectorTileAmount,
                costField,
                islandFieldProcessors);
            return foundIndex;
        }
        static int2 GetClosestIndexAtIsland(
            float2 initialPosition,
            int desiredIsland,
            float tileSize,
            float2 fieldGridStartPos,
            int sectorTileAmount,
            int sectorColAmount,
            int sectorRowAmount,
            int sectorMatrixColAmount,
            int fieldRowAmount,
            int fieldColAmount,
            IslandFieldProcessor islandFieldProcessors,
            UnsafeListReadOnly<byte> costField,
            out int2 lastTopLeft,
            out int2 lastTopRight,
            out int2 lastBotLeft,
            out int2 lastBotRight)
        {

            int2 destinationIndex = FlowFieldUtilities.PosTo2D(initialPosition, tileSize, fieldGridStartPos);
            LocalIndex1d destinationLocal = FlowFieldUtilities.GetLocal1D(destinationIndex, sectorColAmount, sectorMatrixColAmount);
            int destinationLocalIndex = destinationLocal.index;
            int destinationSector = destinationLocal.sector;

            int offset = 1;

            float pickedExtensionIndexDistance = float.MaxValue;
            int pickedExtensionIndexLocalIndex = 0;
            int pickedExtensionIndexSector = 0;


            int2 topLeft = destinationIndex;
            int2 topRight = destinationIndex;
            int2 botLeft = destinationIndex;
            int2 botRight = destinationIndex;
            while (pickedExtensionIndexDistance == float.MaxValue)
            {
                topLeft = destinationIndex + new int2(-offset, offset);
                topRight = destinationIndex + new int2(offset, offset);
                botLeft = destinationIndex + new int2(-offset, -offset);
                botRight = destinationIndex + new int2(offset, -offset);

                bool topOverflow = topLeft.y >= fieldRowAmount;
                bool botOverflow = botLeft.y < 0;
                bool rightOverflow = topRight.x >= fieldColAmount;
                bool leftOverflow = topLeft.x < 0;

                if (topOverflow && botOverflow && rightOverflow && leftOverflow)
                {
                    lastTopLeft = topLeft;
                    lastBotLeft = botLeft;
                    lastTopRight = topRight;
                    lastBotRight = botRight;
                    return FlowFieldUtilities.PosTo2D(initialPosition, tileSize, fieldGridStartPos);
                }

                if (topOverflow)
                {
                    topLeft.y = fieldRowAmount - 1;
                    topRight.y = fieldRowAmount - 1;
                }
                if (botOverflow)
                {
                    botLeft.y = 0;
                    botRight.y = 0;
                }
                if (rightOverflow)
                {
                    botRight.x = fieldColAmount - 1;
                    topRight.x = fieldColAmount - 1;
                }
                if (leftOverflow)
                {
                    topLeft.x = 0;
                    botLeft.x = 0;
                }

                int topLeftSector = FlowFieldUtilities.GetSector1D(topLeft, sectorColAmount, sectorMatrixColAmount);
                int topRightSector = FlowFieldUtilities.GetSector1D(topRight, sectorColAmount, sectorMatrixColAmount);
                int botRightSector = FlowFieldUtilities.GetSector1D(botRight, sectorColAmount, sectorMatrixColAmount);
                int botLeftSector = FlowFieldUtilities.GetSector1D(botLeft, sectorColAmount, sectorMatrixColAmount);
                if (!topOverflow)
                {
                    int rowToCheck = topLeft.y % sectorRowAmount;
                    for (int i = topLeftSector; i <= topRightSector; i++)
                    {
                        int colStart = math.select(0, topLeft.x % sectorColAmount, i == topLeftSector);
                        int colEnd = math.select(9, topRight.x % sectorColAmount, i == topRightSector);
                        ExtensionIndex checkedExtension = CheckSectorRow(i, rowToCheck, colStart, colEnd);
                        if (checkedExtension.IsValid() && checkedExtension.Distance < pickedExtensionIndexDistance)
                        {
                            pickedExtensionIndexDistance = checkedExtension.Distance;
                            pickedExtensionIndexLocalIndex = checkedExtension.LocalIndex;
                            pickedExtensionIndexSector = checkedExtension.SectorIndex;
                        }
                    }
                }
                if (!rightOverflow)
                {
                    int colToCheck = topRight.x % sectorColAmount;
                    for (int i = topRightSector; i >= botRightSector; i -= sectorMatrixColAmount)
                    {
                        int rowStart = math.select(0, botRight.y % sectorRowAmount, i == botRightSector);
                        int rowEnd = math.select(9, topRight.y % sectorRowAmount, i == topRightSector);
                        ExtensionIndex checkedExtension = CheckSectorCol(i, colToCheck, rowStart, rowEnd);
                        if (checkedExtension.IsValid() && checkedExtension.Distance < pickedExtensionIndexDistance)
                        {
                            pickedExtensionIndexDistance = checkedExtension.Distance;
                            pickedExtensionIndexLocalIndex = checkedExtension.LocalIndex;
                            pickedExtensionIndexSector = checkedExtension.SectorIndex;
                        }
                    }
                }
                if (!botOverflow)
                {
                    int rowToCheck = botRight.y % sectorRowAmount;
                    for (int i = botRightSector; i >= botLeftSector; i--)
                    {
                        int colStart = math.select(0, botLeft.x % sectorColAmount, i == botLeftSector);
                        int colEnd = math.select(9, botRight.x % sectorColAmount, i == botRightSector);
                        ExtensionIndex checkedExtension = CheckSectorRow(i, rowToCheck, colStart, colEnd);
                        if (checkedExtension.IsValid() && checkedExtension.Distance < pickedExtensionIndexDistance)
                        {
                            pickedExtensionIndexDistance = checkedExtension.Distance;
                            pickedExtensionIndexLocalIndex = checkedExtension.LocalIndex;
                            pickedExtensionIndexSector = checkedExtension.SectorIndex;
                        }
                    }
                }
                if (!leftOverflow)
                {
                    int colToCheck = topLeft.x % sectorColAmount;
                    for (int i = botLeftSector; i <= topLeftSector; i += sectorMatrixColAmount)
                    {
                        int rowStart = math.select(0, botLeft.y % sectorRowAmount, i == botLeftSector);
                        int rowEnd = math.select(9, topLeft.y % sectorRowAmount, i == topLeftSector);
                        ExtensionIndex checkedExtension = CheckSectorCol(i, colToCheck, rowStart, rowEnd);
                        if (checkedExtension.IsValid() && checkedExtension.Distance < pickedExtensionIndexDistance)
                        {
                            pickedExtensionIndexDistance = checkedExtension.Distance;
                            pickedExtensionIndexLocalIndex = checkedExtension.LocalIndex;
                            pickedExtensionIndexSector = checkedExtension.SectorIndex;
                        }
                    }
                }
                offset++;
            }
            lastTopLeft = topLeft;
            lastBotLeft = botLeft;
            lastTopRight = topRight;
            lastBotRight = botRight;

            int2 outputGeneral2d = FlowFieldUtilities.GetGeneral2d(pickedExtensionIndexLocalIndex, pickedExtensionIndexSector, sectorMatrixColAmount, sectorColAmount);
            return outputGeneral2d;

            ExtensionIndex CheckSectorRow(int sectorToCheck, int rowToCheck, int colToStart, int colToEnd)
            {
                if (islandFieldProcessors.GetIslandIfNotField(sectorToCheck, out int islandOut))
                {
                    if (islandOut != desiredIsland) { return new ExtensionIndex() { Distance = float.MaxValue }; }
                }
                float currentExtensionIndexDistance = float.MaxValue;
                int currentExtensionLocalIndex = 0;
                for (int c = colToStart; c <= colToEnd; c++)
                {
                    int2 local2d = new int2(c, rowToCheck);
                    int local1d = FlowFieldUtilities.To1D(local2d, sectorColAmount);
                    int costFieldIndex = sectorToCheck * sectorTileAmount + local1d;
                    byte cost = costField[costFieldIndex];
                    if (cost == byte.MaxValue) { continue; }
                    int island = islandFieldProcessors.GetIsland(sectorToCheck, local1d);
                    if (island != desiredIsland) { continue; }
                    int2 curGeneralIndex = FlowFieldUtilities.GetGeneral2d(local1d, sectorToCheck, sectorMatrixColAmount, sectorColAmount);
                    float2 curIndexPos = FlowFieldUtilities.IndexToPos(curGeneralIndex, tileSize, fieldGridStartPos);
                    float newExtensionDistance = math.distance(curIndexPos, initialPosition);
                    if (newExtensionDistance < currentExtensionIndexDistance) { currentExtensionIndexDistance = newExtensionDistance; currentExtensionLocalIndex = local1d; }

                }
                return new ExtensionIndex()
                {
                    SectorIndex = sectorToCheck,
                    LocalIndex = currentExtensionLocalIndex,
                    Distance = currentExtensionIndexDistance
                };
            }
            ExtensionIndex CheckSectorCol(int sectorToCheck, int colToCheck, int rowToStart, int rowToEnd)
            {
                if (islandFieldProcessors.GetIslandIfNotField(sectorToCheck, out int islandOut))
                {
                    if (islandOut != desiredIsland) { return new ExtensionIndex() { Distance = float.MaxValue }; }
                }
                float currentExtensionIndexDistance = float.MaxValue;
                int currentExtensionLocalIndex = 0;
                for (int r = rowToStart; r <= rowToEnd; r++)
                {
                    int2 local2d = new int2(colToCheck, r);
                    int local1d = FlowFieldUtilities.To1D(local2d, sectorColAmount);
                    int costFieldIndex = sectorToCheck * sectorTileAmount + local1d;
                    byte cost = costField[costFieldIndex];
                    if (cost == byte.MaxValue) { continue; }
                    int island = islandFieldProcessors.GetIsland(sectorToCheck, local1d);
                    if (island != desiredIsland) { continue; }
                    int2 curGeneralIndex = FlowFieldUtilities.GetGeneral2d(local1d, sectorToCheck, sectorMatrixColAmount, sectorColAmount);
                    float2 curIndexPos = FlowFieldUtilities.IndexToPos(curGeneralIndex, tileSize, fieldGridStartPos);
                    float newExtensionDistance = math.distance(curIndexPos, initialPosition);
                    if (newExtensionDistance < currentExtensionIndexDistance) { currentExtensionIndexDistance = newExtensionDistance; currentExtensionLocalIndex = local1d; }
                }
                return new ExtensionIndex()
                {
                    SectorIndex = sectorToCheck,
                    LocalIndex = currentExtensionLocalIndex,
                    Distance = currentExtensionIndexDistance
                };
            }
        }

        static int2 MakeSureExpandedDestinationIsClosest(
            int2 lastBotLeft,
            int2 lastBotRight,
            int2 lastTopLeft,
            int2 lastTopRight,
            int2 finishedIndex,
            float2 goalPosition,
            int desiredIsland,
            float tileSize,
            float2 fieldGridStartPos,
            int fieldRowAmount,
            int fieldColAmount,
            int sectorColAmount,
            int sectorMatrixColAmount,
            int sectorTileAmount,
            UnsafeListReadOnly<byte> costField,
            IslandFieldProcessor islandFieldProcessor)
        {
            float2 finishedIndexCenter = FlowFieldUtilities.IndexToPos(finishedIndex, tileSize, fieldGridStartPos);
            float curGoalDistSq = math.distancesq(finishedIndexCenter, goalPosition);
            float curGoalDist = math.sqrt(curGoalDistSq);

            int topSearchStartRow = math.min(fieldRowAmount - 1, lastTopLeft.y + 1);
            int topSearchEndRow;
            int topSearchStartCol = lastTopLeft.x;
            int topSearchEndCol = lastTopRight.x;

            int botSearchStartRow = math.max(0, lastBotLeft.y - 1);
            int botSearchEndRow;
            int botSearchStartCol = lastBotLeft.x;
            int botSearchEndCol = lastBotRight.x;

            int rightSearchStartRow = lastBotRight.y;
            int rightSearchEndRow = lastTopRight.y;
            int rightSearchStartCol = math.min(fieldColAmount - 1, lastTopRight.x + 1);
            int rightSearchEndCol;

            int leftSearchStartRow = lastBotLeft.y;
            int leftSearchEndRow = lastTopLeft.y;
            int leftSearchStartCol = math.max(0, lastTopLeft.x - 1);
            int leftSearchEndCol;

            SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, fieldRowAmount, fieldColAmount, tileSize, fieldGridStartPos,
                out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);

            int curTopSearchRow = topSearchStartRow;
            int curBotSearchRow = botSearchStartRow;
            int curRightSearchCol = rightSearchStartCol;
            int curLeftSearchCol = leftSearchStartCol;
            bool topExhausted = false;
            bool botExhausted = false;
            bool rightExhausted = false;
            bool leftExhausted = false;

            bool exhausted = false;
            float2 foundPosition = finishedIndexCenter;
            while (!exhausted)
            {
                topExhausted = topSearchEndRow < curTopSearchRow;
                botExhausted = botSearchEndRow > curBotSearchRow;
                rightExhausted = rightSearchEndCol < curRightSearchCol;
                leftExhausted = leftSearchEndCol > curLeftSearchCol;
                exhausted = topExhausted && botExhausted && rightExhausted && leftExhausted;

                if (!topExhausted)
                {
                    for (int c = topSearchStartCol; c <= topSearchEndCol; c++)
                    {
                        int2 curIndex = new int2(c, curTopSearchRow);
                        if (!IndexSuitable(curIndex, out float2 newPickedPos, out float newGoalDistSq)) { continue; }
                        foundPosition = newPickedPos;
                        curGoalDistSq = newGoalDistSq;
                        curGoalDist = math.sqrt(curGoalDistSq);
                        SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, fieldRowAmount, fieldColAmount, tileSize, fieldGridStartPos, 
                            out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);
                    }
                }
                if (!botExhausted)
                {
                    for (int c = botSearchStartCol; c <= botSearchEndCol; c++)
                    {
                        int2 curIndex = new int2(c, curBotSearchRow);
                        if (!IndexSuitable(curIndex, out float2 newPickedPos, out float newGoalDistSq)) { continue; }
                        foundPosition = newPickedPos;
                        curGoalDistSq = newGoalDistSq;
                        curGoalDist = math.sqrt(curGoalDistSq);
                        SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, fieldRowAmount, fieldColAmount, tileSize, fieldGridStartPos, 
                            out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);
                    }
                }
                if (!rightExhausted)
                {
                    for (int r = rightSearchStartRow; r <= rightSearchEndRow; r++)
                    {
                        int2 curIndex = new int2(curRightSearchCol, r);
                        if (!IndexSuitable(curIndex, out float2 newPickedPos, out float newGoalDistSq)) { continue; }
                        foundPosition = newPickedPos;
                        curGoalDistSq = newGoalDistSq;
                        curGoalDist = math.sqrt(curGoalDistSq);
                        SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, fieldRowAmount, fieldColAmount, tileSize, fieldGridStartPos, 
                            out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);
                    }
                }
                if (!leftExhausted)
                {
                    for (int r = leftSearchStartRow; r <= leftSearchEndRow; r++)
                    {
                        int2 curIndex = new int2(curLeftSearchCol, r);
                        if (!IndexSuitable(curIndex, out float2 newPickedPos, out float newGoalDistSq)) { continue; }
                        foundPosition = newPickedPos;
                        curGoalDistSq = newGoalDistSq;
                        curGoalDist = math.sqrt(curGoalDistSq);
                        SetEndFor_Top_Right_Bot_Left(goalPosition, curGoalDist, fieldRowAmount, fieldColAmount, tileSize, fieldGridStartPos,
                            out topSearchEndRow, out botSearchEndRow, out rightSearchEndCol, out leftSearchEndCol);
                    }
                }
                curTopSearchRow++;
                curBotSearchRow--;
                curRightSearchCol++;
                curLeftSearchCol--;
            }
            return FlowFieldUtilities.PosTo2D(foundPosition, tileSize, fieldGridStartPos);
            bool IndexSuitable(int2 curIndex, out float2 indexPos, out float indexGoalDistSq)
            {
                indexPos = FlowFieldUtilities.IndexToPos(curIndex, tileSize, fieldGridStartPos);
                indexGoalDistSq = math.distancesq(indexPos, goalPosition);
                bool closeEnough = indexGoalDistSq < curGoalDistSq;
                LocalIndex1d curLocalIndex = FlowFieldUtilities.GetLocal1D(curIndex, sectorColAmount, sectorMatrixColAmount);
                int curCostFieldIndex = curLocalIndex.sector * sectorTileAmount + curLocalIndex.index;
                byte curCost = costField[curCostFieldIndex];
                bool walkable = curCost != byte.MaxValue;
                int curIsland = islandFieldProcessor.GetIsland(curLocalIndex.sector, curLocalIndex.index);
                bool onDesiredIsland = curIsland == desiredIsland;
                return onDesiredIsland && closeEnough && walkable;
            }
        }
        static void SetEndFor_Top_Right_Bot_Left(
            float2 goalPosition, 
            float curGoalDist, 
            int fieldRowAmount, 
            int fieldColAmount,
            float tileSize,
            float2 fieldGridStartPos,
            out int topEndRow, out int botEndRow, out int rightEndCol, out int leftEndCol)
        {
            int2 topIndex = FlowFieldUtilities.PosTo2D(new float2(goalPosition.x, goalPosition.y + curGoalDist), tileSize, fieldGridStartPos);
            int2 botIndex = FlowFieldUtilities.PosTo2D(new float2(goalPosition.x, goalPosition.y - curGoalDist), tileSize, fieldGridStartPos);
            int2 rightIndex = FlowFieldUtilities.PosTo2D(new float2(goalPosition.x + curGoalDist, goalPosition.y), tileSize, fieldGridStartPos);
            int2 leftIndex = FlowFieldUtilities.PosTo2D(new float2(goalPosition.x - curGoalDist, goalPosition.y), tileSize, fieldGridStartPos);

            topEndRow = math.min(fieldRowAmount - 1, topIndex.y);
            botEndRow = math.max(0, botIndex.y);
            rightEndCol = math.min(fieldColAmount - 1, rightIndex.x);
            leftEndCol = math.max(0, leftIndex.x);
        }

        private struct ExtensionIndex
        {
            internal int LocalIndex;
            internal int SectorIndex;
            internal float Distance;

            internal bool IsValid()
            {
                return Distance != float.MaxValue;
            }
        }
    }
}
