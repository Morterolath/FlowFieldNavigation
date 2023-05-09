﻿using Unity.Collections;

public struct SectorArray
{
    public NativeArray<SectorNode> Nodes;
    public NativeArray<int> WinPtrs;

    public SectorArray(int totalSectorAmount, int secToWinPtrsAmount)
    {
        Nodes = new NativeArray<SectorNode>(totalSectorAmount, Allocator.Persistent);
        WinPtrs = new NativeArray<int>(secToWinPtrsAmount, Allocator.Persistent);
    }
    public void DisposeNatives()
    {
        Nodes.Dispose();
        WinPtrs.Dispose();
    }
    public void ConfigureSectorNodes(int totalTileAmount, int sectorSize)
    {
        int sectorMatrixSize = totalTileAmount / sectorSize;

        int iterableSecToWinPtr = 0;
        for (int r = 0; r < sectorMatrixSize; r++)
        {
            for (int c = 0; c < sectorMatrixSize; c++)
            {
                int index = r * sectorMatrixSize + c;
                Sector sect = new Sector(new Index2(r * sectorSize, c * sectorSize), sectorSize);
                int secToWinCnt = 4;
                if (sect.IsOnCorner(totalTileAmount))
                {
                    secToWinCnt = 2;
                }
                else if (sect.IsOnEdge(totalTileAmount))
                {
                    secToWinCnt = 3;
                }
                Nodes[index] = new SectorNode(sect, secToWinCnt, iterableSecToWinPtr);
                iterableSecToWinPtr += secToWinCnt;
            }
        }
    }
    public void ConfigureSectorToWindowPoiners(NativeArray<WindowNode> windowNodes)
    {
        int sectorSize = Nodes[0].Sector.Size;
        int secToWinPtrIterable = 0;
        for (int i = 0; i < Nodes.Length; i++)
        {
            Index2 sectorStartIndex = Nodes[i].Sector.StartIndex;
            Index2 topWinIndex = new Index2(sectorStartIndex.R + sectorSize - 1, sectorStartIndex.C);
            Index2 rightWinIndex = new Index2(sectorStartIndex.R, sectorStartIndex.C + sectorSize - 1);
            Index2 botWinIndex = new Index2(sectorStartIndex.R - 1, sectorStartIndex.C);
            Index2 leftWinIndex = new Index2(sectorStartIndex.R, sectorStartIndex.C - 1);
            for (int j = 0; j < windowNodes.Length; j++)
            {
                Window window = windowNodes[j].Window;
                if (window.BottomLeftBoundary == topWinIndex) { WinPtrs[secToWinPtrIterable++] = j; }
                else if (window.BottomLeftBoundary == rightWinIndex) { WinPtrs[secToWinPtrIterable++] = j; }
                else if (window.BottomLeftBoundary == botWinIndex) { WinPtrs[secToWinPtrIterable++] = j; }
                else if (window.BottomLeftBoundary == leftWinIndex) { WinPtrs[secToWinPtrIterable++] = j; }
            }
        }
    }
    public NativeArray<int> GetPortalIndicies(SectorNode sectorNode, NativeArray<WindowNode> windowNodes)
    {
        NativeArray<int> portalIndicies;
        int secToWinCnt = sectorNode.SecToWinCnt;
        int secToWinPtr = sectorNode.SecToWinPtr;

        //determine portal count
        int portalIndexCount = 0;
        for (int i = 0; i < secToWinCnt; i++)
        {
            portalIndexCount += windowNodes[WinPtrs[secToWinPtr + i]].PorCnt;
        }
        portalIndicies = new NativeArray<int>(portalIndexCount, Allocator.Temp);

        //get portals
        int portalIndiciesIterable = 0;
        for (int i = 0; i < secToWinCnt; i++)
        {
            int windowPorPtr = windowNodes[WinPtrs[secToWinPtr + i]].PorPtr;
            int windowPorCnt = windowNodes[WinPtrs[secToWinPtr + i]].PorCnt;
            for (int j = 0; j < windowPorCnt; j++)
            {
                portalIndicies[portalIndiciesIterable++] = windowPorPtr + j;
            }
        }
        return portalIndicies;
    }
}
