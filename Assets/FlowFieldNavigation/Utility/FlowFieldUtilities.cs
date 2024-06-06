using Unity.Mathematics;
using Unity.Burst;
using System;
namespace FlowFieldNavigation
{
    internal static class FlowFieldUtilities
    {
        internal static bool DebugMode;
        internal static float TileSize;
        internal static float FieldMinXIncluding;
        internal static float FieldMinYIncluding;
        internal static float FieldMaxXExcluding;
        internal static float FieldMaxYExcluding;
        internal static int SectorColAmount;
        internal static int SectorRowAmount;
        internal static int SectorTileAmount;
        internal static int FieldColAmount;
        internal static int FieldRowAmount;
        internal static int FieldTileAmount;
        internal static int SectorMatrixColAmount;
        internal static int SectorMatrixRowAmount;
        internal static int SectorMatrixTileAmount;
        internal static float BaseAgentSpatialGridSize;
        internal static float BaseTriangleSpatialGridSize;
        internal static float MinAgentSize;
        internal static float MaxAgentSize;
        internal static int LOSRange;
        internal static int MaxCostFieldOffset;
        internal static float2 HeightMeshStartPosition;
        internal static float2 FieldGridStartPosition;

        internal static int To1D(int2 index2, int colAmount)
        {
            return index2.y * colAmount + index2.x;
        }
        internal static int2 To2D(int index, int colAmount)
        {
            return new int2(index % colAmount, index / colAmount);
        }
        internal static int2 PosTo2D(float2 pos, float tileSize, float2 gridStartPos)
        {
            pos -= gridStartPos;
            return new int2((int)math.floor(pos.x / tileSize), (int)math.floor(pos.y / tileSize));
        }
        internal static int2 Clamp(int2 index, int colAmount, int rowAmount)
        {
            index = math.max(index, 0);
            return math.min(index, new int2(colAmount - 1, rowAmount - 1));
        }
        internal static float2 GetGridMaxPosExcluding(int rowAmount, int colAmount, float tileSize, float2 gridStartPos)
        {
            return gridStartPos + new float2(colAmount * tileSize, rowAmount * tileSize);
        }
        internal static float2 LocalIndexToPos(int local1d, int sector1d, int sectorMatrixColAmount, int sectorColAmount, float tileSize, float sectorSize, float2 gridStartPos)
        {
            float2 sectorStartPos = new float2((sector1d % sectorMatrixColAmount) * sectorSize, (sector1d / sectorMatrixColAmount) * sectorSize);
            float2 indexOffset = new float2((local1d % sectorColAmount) * tileSize, (local1d / sectorColAmount) * tileSize);
            float2 tileCenter = new float2(tileSize / 2, tileSize / 2);
            return gridStartPos + sectorStartPos + indexOffset + tileCenter;
        }
        internal static int PosToSector1D(float2 pos, float sectorSize, int sectorMatrixColAmount, float2 gridStartPos)
        {
            pos -= gridStartPos;
            int2 sector2d = new int2((int)math.floor(pos.x / sectorSize), (int)math.floor(pos.y / sectorSize));
            return sector2d.y * sectorMatrixColAmount + sector2d.x;
        }
        internal static float2 IndexToPos(int2 general2d, float tileSize, float2 gridStartPos)
        {
            return gridStartPos + new float2(general2d.x * tileSize + tileSize / 2, general2d.y * tileSize + tileSize / 2);
        }
        internal static float2 IndexToStartPos(int2 general2d, float tileSize, float2 gridStartPos)
        {
            return gridStartPos + new float2(general2d.x * tileSize, general2d.y * tileSize);
        }
        internal static float IndexXToStartPosX(int indexX, float tileSize, float2 gridStartPos)
        {
            return gridStartPos.x + indexX * tileSize;
        }
        internal static float IndexYToStartPosY(int indexY, float tileSize, float2 gridStartPos)
        {
            return gridStartPos.y + indexY * tileSize;
        }
        internal static int2 GetSector2D(int2 index, int sectorColAmount)
        {
            return new int2(index.x / sectorColAmount, index.y / sectorColAmount);
        }
        internal static int GetSector1D(int2 index, int sectorColAmount, int sectorMatrixColAmount)
        {
            int2 sector2d = index / sectorColAmount;
            return sector2d.y * sectorMatrixColAmount + sector2d.x;
        }
        internal static int GetSector1D(int index, int fieldColAmount, int sectorColAmount, int sectorMatrixColAmount)
        {
            int2 sector2d = FlowFieldUtilities.To2D(index, fieldColAmount) / sectorColAmount;
            return sector2d.y * sectorMatrixColAmount + sector2d.x;
        }
        internal static float2 GetSectorStartPos(int sector1d, int sectorMatrixColAmount, int sectorColAmount, float tileSize, float2 fieldGridStartPos)
        {
            int2 sector2d = To2D(sector1d, sectorMatrixColAmount);
            return sector2d * new float2(tileSize, tileSize) * sectorColAmount + fieldGridStartPos;
        }
        internal static int2 GetLocal2D(int2 index, int2 sectorStartIndex)
        {
            return index - sectorStartIndex;
        }
        internal static int GetLocal1D(int2 index, int2 sectorStartIndex, int sectorColAmount)
        {
            int2 local2d = index - sectorStartIndex;
            return local2d.y * sectorColAmount + local2d.x;
        }
        internal static int GetLocal1D(int2 index, int sectorColAmount)
        {
            int2 sectorIndex = new int2(index.x / sectorColAmount, index.y / sectorColAmount);
            int2 sectorStartIndex = new int2(sectorIndex.x * sectorColAmount, sectorIndex.y * sectorColAmount);
            int2 local2d = index - sectorStartIndex;
            return local2d.y * sectorColAmount + local2d.x;
        }
        internal static LocalIndex1d GetLocal1D(int2 general2d, int sectorColAmount, int sectorMatrixColAmount)
        {
            int2 sector2d = general2d / sectorColAmount;
            int2 local2d = general2d - (sector2d * sectorColAmount);
            int sector1d = sector2d.y * sectorMatrixColAmount + sector2d.x;
            return new LocalIndex1d()
            {
                sector = sector1d,
                index = local2d.y * sectorColAmount + local2d.x,
            };
        }
        internal static LocalIndex1d GetLocal1D(int general1d, int fieldColAmount, int sectorColAmount, int sectorMatrixColAmount)
        {
            int2 general2d = new int2(general1d % fieldColAmount, general1d / fieldColAmount);
            int2 sector2d = general2d / sectorColAmount;
            int2 local2d = general2d - (sector2d * sectorColAmount);
            int sector1d = sector2d.y * sectorMatrixColAmount + sector2d.x;
            return new LocalIndex1d()
            {
                sector = sector1d,
                index = local2d.y * sectorColAmount + local2d.x,
            };
        }
        internal static int2 GetSectorStartIndex(int2 sectorIndex, int sectorColAmount)
        {
            return new int2(sectorIndex.x * sectorColAmount, sectorIndex.y * sectorColAmount);
        }
        internal static int GetGeneral1d(int2 local2d, int2 sector2d, int sectorColAmount, int fieldColAmount)
        {
            int2 sectorStart = GetSectorStartIndex(sector2d, sectorColAmount);
            int2 general2d = local2d + sectorStart;
            int general1d = To1D(general2d, fieldColAmount);
            return general1d;
        }
        internal static int2 GetGeneral2d(int2 local2d, int2 sector2d, int sectorColAmount, int fieldColAmount)
        {
            int2 sectorStart = GetSectorStartIndex(sector2d, sectorColAmount);
            int2 general2d = local2d + sectorStart;
            return general2d;
        }
        internal static int2 GetGeneral2d(int local1d, int sector1d, int sectorMatrixColAmount, int sectorColAmount)
        {
            int2 sectorStart2d = new int2(sector1d % sectorMatrixColAmount * sectorColAmount, sector1d / sectorMatrixColAmount * sectorColAmount);
            return sectorStart2d + new int2(local1d % sectorColAmount, local1d / sectorColAmount);
        }
        internal static int2 GetLocal2dInSector(PortalNode portalNode, int sectorIndex, int sectorMatrixColAmount, int sectorColAmount)
        {
            int2 p12d = new int2(portalNode.Portal1.Index.C, portalNode.Portal1.Index.R);
            int2 p22d = new int2(portalNode.Portal2.Index.C, portalNode.Portal2.Index.R);
            int2 sector2d = new int2(sectorIndex % sectorMatrixColAmount, sectorIndex / sectorMatrixColAmount);

            int2 p1Secpr2d = p12d / sectorColAmount;
            int2 p2Secpr2d = p22d / sectorColAmount;

            int2 picked2d = math.select(p22d, p12d, sector2d.Equals(p1Secpr2d));
            int2 sectorStart = new int2(sector2d.x * sectorColAmount, sector2d.y * sectorColAmount);

            return picked2d - sectorStart;
        }
        internal static int GetLocal1dInSector(PortalNode portalNode, int sectorIndex, int sectorMatrixColAmount, int sectorColAmount)
        {
            int2 p12d = new int2(portalNode.Portal1.Index.C, portalNode.Portal1.Index.R);
            int2 p22d = new int2(portalNode.Portal2.Index.C, portalNode.Portal2.Index.R);
            int2 sector2d = new int2(sectorIndex % sectorMatrixColAmount, sectorIndex / sectorMatrixColAmount);//(15,17)

            int2 p1Secpr2d = p12d / sectorColAmount;

            int2 picked2d = math.select(p22d, p12d, sector2d.Equals(p1Secpr2d));//(150,68)
            int2 sectorStart = new int2(sector2d.x * sectorColAmount, sector2d.y * sectorColAmount);//(150,70)
            int2 local2d = picked2d - sectorStart;

            return local2d.y * sectorColAmount + local2d.x;
        }
        internal static int2 GetIndexAtSector(int2 generalIndex1, int2 generalIndex2, int2 sectorIndex, int sectorColAmount)
        {
            int2 sec1 = GetSector2D(generalIndex1, sectorColAmount);
            int2 sec2 = GetSector2D(generalIndex2, sectorColAmount);
            int2 secToReturn = math.select(0,generalIndex1, sectorIndex.Equals(sec1));
            secToReturn = math.select(secToReturn, generalIndex2, sectorIndex.Equals(sec2));
            return secToReturn;
        }
        internal static int GetCommonSector(PortalNode node1, PortalNode node2, int sectorColAmount, int sectorMatrixColAmount)
        {
            int2 n1p1index2d = new int2(node1.Portal1.Index.C, node1.Portal1.Index.R);
            int2 n1p2index2d = new int2(node1.Portal2.Index.C, node1.Portal2.Index.R);
            int2 n2p1index2d = new int2(node2.Portal1.Index.C, node2.Portal1.Index.R);
            int2 n2p2index2d = new int2(node2.Portal2.Index.C, node2.Portal2.Index.R);

            int2 n1p1sector2d = n1p1index2d / sectorColAmount;
            int2 n1p2sector2d = n1p2index2d / sectorColAmount;
            int2 n2p1sector2d = n2p1index2d / sectorColAmount;
            int2 n2p2sector2d = n2p2index2d / sectorColAmount;

            bool isn1p1sectorCommon = n1p1sector2d.Equals(n2p1sector2d) || n1p1sector2d.Equals(n2p2sector2d);
            int n1p1sector1d = n1p1sector2d.y * sectorMatrixColAmount + n1p1sector2d.x;
            int n1p2sector1d = n1p2sector2d.y * sectorMatrixColAmount + n1p2sector2d.x;
            return math.select(n1p2sector1d, n1p1sector1d, isn1p1sectorCommon);
        }
        internal static void GetSectors(PortalNode node1, int sectorColAmount, int sectorMatrixColAmount, out int sector1, out int sector2)
        {
            int2 n1p1index2d = new int2(node1.Portal1.Index.C, node1.Portal1.Index.R);
            int2 n1p2index2d = new int2(node1.Portal2.Index.C, node1.Portal2.Index.R);

            int2 n1p1sector2d = n1p1index2d / sectorColAmount;
            int2 n1p2sector2d = n1p2index2d / sectorColAmount;

            sector1 = n1p1sector2d.y * sectorMatrixColAmount + n1p1sector2d.x;
            sector2 = n1p2sector2d.y * sectorMatrixColAmount + n1p2sector2d.x;
        }
        internal static int RadiusToOffset(float radius, float tileSize)
        {
            float offsetZeroSize = tileSize / 2;
            float radiusWithoutOffsetZeroSize = radius - offsetZeroSize;
            int offset = (int)math.floor(radiusWithoutOffsetZeroSize / tileSize) + 1;
            return math.select(offset, 0, radiusWithoutOffsetZeroSize < 0);
        }
        internal static float GetCostBetween(int sector1, int local1, int sector2, int local2, int sectorColAmount, int sectorMatrixColAmount)
        {
            int2x2 sectors2d = new int2x2()
            {
                c0 = new int2(sector1 % sectorMatrixColAmount, sector1 / sectorMatrixColAmount),
                c1 = new int2(sector2 % sectorMatrixColAmount, sector2 / sectorMatrixColAmount),
            };
            int2x2 locals2d = new int2x2()
            {
                c0 = new int2(local1 % sectorColAmount, local1 / sectorColAmount),
                c1 = new int2(local2 % sectorColAmount, local2 / sectorColAmount),
            };
            int2x2 sectorStartIndicies = sectors2d * sectorColAmount;
            int2 general2d1 = sectorStartIndicies.c0 + locals2d.c0;
            int2 general2d2 = sectorStartIndicies.c1 + locals2d.c1;
            int2 change = math.abs(general2d2 - general2d1);
            int minComponent = math.min(change.x, change.y);
            int maxComponent = math.max(change.x, change.y);
            return minComponent * 1.4f + (maxComponent - minComponent);
        }
        internal static float2 Local1dToPos(int localIndex, int sectorIndex, int sectorColAmount, int sectorMatrixColAmount, int fieldColAmount, float tileSize, float2 gridStartPos)
        {
            int2 local2d = new int2(localIndex % sectorColAmount, localIndex / sectorColAmount);
            int2 sector2d = new int2(sectorIndex % sectorMatrixColAmount, sectorIndex / sectorMatrixColAmount);
            int2 sectorStart = new int2(sector2d.x * sectorColAmount, sector2d.y * sectorColAmount);
            int2 general2d = local2d + sectorStart;
            return gridStartPos + new float2(general2d.x * tileSize + tileSize / 2, general2d.y * tileSize + tileSize / 2);
        }
        internal static BarycentricCoordinates GetBarycentricCoordinatesForEachVectorInTheOrderUVW(float2 a, float2 b, float2 c, float2 p)
        {
            float2 v0 = b - a, v1 = c - a, v2 = p - a;
            float d00 = math.dot(v0, v0);
            float d01 = math.dot(v0, v1);
            float d11 = math.dot(v1, v1);
            float d20 = math.dot(v2, v0);
            float d21 = math.dot(v2, v1);
            float denom = d00 * d11 - d01 * d01;
            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1.0f - v - w;
            return new BarycentricCoordinates()
            {
                v = v,
                u = u,
                w = w,
            };
        }
    }


}
