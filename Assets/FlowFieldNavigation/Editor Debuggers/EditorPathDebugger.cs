#if (UNITY_EDITOR)

using System;
using System.Linq;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using UnityEngine.Jobs;

namespace FlowFieldNavigation
{
    internal class EditorPathDebugger
    {
        PathDataContainer _pathContainer;
        FlowFieldNavigationManager _navigationManager;
        FieldDataContainer _fieldProducer;
        float _tileSize;
        Color[] _colorArray = new Color[]
        {
            new Color(0,0,0),
            new Color(1,0,0),
            new Color(0,1,0),
            new Color(1,1,0),
            new Color(0,0,1),
            new Color(1,0,1),
            new Color(0,1,1),
            new Color(1,1,1),
        };

        internal EditorPathDebugger(FlowFieldNavigationManager navigationManager)
        {
            _pathContainer = navigationManager.PathDataContainer;
            _navigationManager = navigationManager;
            _fieldProducer = navigationManager.FieldDataContainer;
            _tileSize = FlowFieldUtilities.TileSize;
        }
        internal void DebugIslandSeed(FlowFieldAgent agent)
        {
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }
            int islandSeedAsIndex1d = _navigationManager.PathDataContainer.PathIslandSeedsAsFieldIndicies[pathIndex];
            int2 islandSeedAsIndex2d = FlowFieldUtilities.To2D(islandSeedAsIndex1d, FlowFieldUtilities.FieldColAmount);
            float2 islandSeedAsPos = FlowFieldUtilities.IndexToPos(islandSeedAsIndex2d, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
            float3 pos3 = new float3(islandSeedAsPos.x, _navigationManager.HeightMeshImmediateQueryManager.GetHeightBurst(islandSeedAsPos), islandSeedAsPos.y);
            Gizmos.color = Color.cyan;
            Gizmos.DrawCube(pos3, new Vector3(0.3f, 0.3f, 0.3f));
        }
        internal void DebugPathUpdateSeeds(FlowFieldAgent agent)
        {
            int pathIndex = agent.GetPathIndex();
            if(pathIndex == -1) { return; }
            NativeArray<PathUpdateSeed> seeds = _navigationManager.PathUpdateSeedContainer.UpdateSeeds.AsArray();
            Gizmos.color = Color.white;
            for(int i = 0; i < seeds.Length; i++)
            {
                PathUpdateSeed seed = seeds[i];
                if(seed.PathIndex !=  pathIndex) { continue; }

                int2 goal2d = FlowFieldUtilities.To2D(seed.TileIndex, FlowFieldUtilities.FieldColAmount);
                float2 goalpos2d = FlowFieldUtilities.IndexToPos(goal2d, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
                float3 goalpos3d = new float3(goalpos2d.x, 0f, goalpos2d.y);
                Gizmos.DrawCube(goalpos3d, Vector3.one / 2);
            }
        }
        internal void DebugOverlappingSectors(FlowFieldAgent agent)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }
            Gizmos.color = Color.white;

            int sectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount;
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            float tileSize = FlowFieldUtilities.TileSize;
            float2 fieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition;
            NativeArray<OverlappingDirection> sectorOverlappingDirectionTable = _pathContainer.SectorOverlappingDirectionTableList[pathIndex];
            for(int i = 0; i < sectorOverlappingDirectionTable.Length; i++)
            {
                OverlappingDirection overlapping = sectorOverlappingDirectionTable[i];
                Vector2 botLeftCorner = FlowFieldUtilities.GetSectorStartPos(i, sectorMatrixColAmount, sectorColAmount, tileSize, fieldGridStartPos);
                Vector3 botLeftCorner3d = new Vector3(botLeftCorner.x, 0.2f, botLeftCorner.y);
                Vector3 p1 = Vector3.zero;
                Vector3 p2 = Vector3.zero;
                Vector3 p3 = Vector3.zero;
                if((overlapping & OverlappingDirection.N) == OverlappingDirection.N)
                {
                    p1 = botLeftCorner3d + new Vector3(0, 0, tileSize * sectorColAmount);
                    p2 = botLeftCorner3d + new Vector3(tileSize * sectorColAmount, 0, tileSize * sectorColAmount);
                    p3 = (p1 + p2) / 2 + new Vector3(0, 0, 1f);
                    Gizmos.DrawLine(p1, p2);
                    Gizmos.DrawLine(p2, p3);
                    Gizmos.DrawLine(p3, p1);
                }
                if ((overlapping & OverlappingDirection.E) == OverlappingDirection.E)
                {
                    p1 = botLeftCorner3d + new Vector3(tileSize * sectorColAmount, 0, tileSize * sectorColAmount);
                    p2 = botLeftCorner3d + new Vector3(tileSize * sectorColAmount, 0, 0);
                    p3 = (p1 + p2) / 2 + new Vector3(1, 0, 0f);
                    Gizmos.DrawLine(p1, p2);
                    Gizmos.DrawLine(p2, p3);
                    Gizmos.DrawLine(p3, p1);
                }
                if ((overlapping & OverlappingDirection.S) == OverlappingDirection.S)
                {
                    p1 = botLeftCorner3d;
                    p2 = botLeftCorner3d + new Vector3(tileSize * sectorColAmount, 0, 0);
                    p3 = (p1 + p2) / 2 + new Vector3(0, 0, -1f);
                    Gizmos.DrawLine(p1, p2);
                    Gizmos.DrawLine(p2, p3);
                    Gizmos.DrawLine(p3, p1);
                }
                if ((overlapping & OverlappingDirection.W) == OverlappingDirection.W)
                {
                    p1 = botLeftCorner3d + new Vector3(0, 0, tileSize * sectorColAmount);
                    p2 = botLeftCorner3d;
                    p3 = (p1 + p2) / 2 + new Vector3(-1, 0, 0f);
                    Gizmos.DrawLine(p1, p2);
                    Gizmos.DrawLine(p2, p3);
                    Gizmos.DrawLine(p3, p1);
                }

            }
        }
        internal void DebugDynamicAreaIntegration(FlowFieldAgent agent, NativeArray<float> tileCenterHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathfindingInternalData internalData = _pathContainer.PathfindingInternalDataList[pathIndex];
            NativeArray<SectorFlowStart> pickedSectorFlowStarts = internalData.DynamicArea.SectorFlowStartCalculationBuffer.AsArray();
            NativeArray<IntegrationTile> integrationField = internalData.DynamicArea.IntegrationField.AsArray();
            int sectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount;
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            float tileSize = FlowFieldUtilities.TileSize;
            float sectorSize = tileSize * sectorColAmount;
            float2 fieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition;
            for (int i = 0; i < pickedSectorFlowStarts.Length; i++)
            {
                int pickedSectorFlowStart = pickedSectorFlowStarts[i].FlowStartIndex;
                int pickedSectorIndex = pickedSectorFlowStarts[i].SectorIndex;

                for (int j = pickedSectorFlowStart; j < pickedSectorFlowStart + FlowFieldUtilities.SectorTileAmount; j++)
                {
                    int local1d = (j - 1) % FlowFieldUtilities.SectorTileAmount;
                    float2 debugPos = FlowFieldUtilities.LocalIndexToPos(local1d, pickedSectorIndex, sectorMatrixColAmount, sectorColAmount, tileSize, sectorSize, fieldGridStartPos);
                    int2 local2d = FlowFieldUtilities.To2D(local1d, sectorColAmount);
                    int2 sector2d = FlowFieldUtilities.To2D(pickedSectorIndex, sectorMatrixColAmount);
                    int general1d = FlowFieldUtilities.GetGeneral1d(local2d, sector2d, sectorColAmount, sectorMatrixColAmount);
                    Vector3 debugPos3 = new Vector3(debugPos.x, tileCenterHeights[general1d], debugPos.y);
                    float cost = integrationField[j].Cost;
                    if (cost != float.MaxValue)
                    {
                        Handles.Label(debugPos3, cost.ToString());
                    }
                }
            }
        }
        internal void DebugDynamicAreaFlow(FlowFieldAgent agent, NativeArray<float> tileCenterHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            float yOffset = 0.2f;
            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            PathfindingInternalData internalData = _navigationManager.PathDataContainer.PathfindingInternalDataList[pathIndex];
            NativeArray<SectorFlowStart> pickedSectorFlowStarts = internalData.DynamicArea.SectorFlowStartCalculationBuffer.AsArray();
            NativeArray<FlowData> flowField = internalData.DynamicArea.FlowFieldCalculationBuffer.AsArray();
            Gizmos.color = Color.blue;
            int sectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount;
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            float tileSize = FlowFieldUtilities.TileSize;
            float sectorSize = tileSize * sectorColAmount;
            float2 fieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition;
            for (int i = 0; i < pickedSectorFlowStarts.Length; i++)
            {
                int pickedSectorFlowStart = pickedSectorFlowStarts[i].FlowStartIndex;
                int pickedSectorIndex = pickedSectorFlowStarts[i].SectorIndex;

                for (int j = pickedSectorFlowStart; j < pickedSectorFlowStart + FlowFieldUtilities.SectorTileAmount; j++)
                {
                    int local1d = (j - 1) % FlowFieldUtilities.SectorTileAmount;
                    float2 debugPos = FlowFieldUtilities.LocalIndexToPos(local1d, pickedSectorIndex, sectorMatrixColAmount, sectorColAmount, tileSize, sectorSize, fieldGridStartPos);
                    int2 local2d = FlowFieldUtilities.To2D(local1d, sectorColAmount);
                    int2 sector2d = FlowFieldUtilities.To2D(pickedSectorIndex, sectorMatrixColAmount);
                    int general1d = FlowFieldUtilities.GetGeneral1d(local2d, sector2d, sectorColAmount, sectorMatrixColAmount);
                    Vector3 debugPos3 = new Vector3(debugPos.x, tileCenterHeights[general1d], debugPos.y);
                    if (j >= flowField.Length) { continue; }
                    FlowData flow = flowField[j];
                    if (!flow.IsValid()) { continue; }
                    DrawSquare(debugPos3, 0.2f);
                    DrawFlow(j, debugPos3, destinationData.Destination);
                }
            }

            void DrawSquare(Vector3 pos, float size)
            {
                Vector3 botLeft = new Vector3(pos.x - size / 2, pos.y, pos.z - size / 2);
                Vector3 botRight = new Vector3(pos.x + size / 2, pos.y, pos.z - size / 2);
                Vector3 topLeft = new Vector3(pos.x - size / 2, pos.y, pos.z + size / 2);
                Vector3 topRight = new Vector3(pos.x + size / 2, pos.y, pos.z + size / 2);

                Gizmos.DrawLine(topRight, botRight);
                Gizmos.DrawLine(botRight, botLeft);
                Gizmos.DrawLine(botLeft, topLeft);
                Gizmos.DrawLine(topLeft, topRight);
            }
            void DrawFlow(int flowIndex, Vector3 pos, float2 destination)
            {
                float2 flowDir = flowField[flowIndex].GetFlow(_tileSize);
                flowDir = math.normalizesafe(flowDir) * 0.4f;
                Vector3 targetPos = pos + new Vector3(flowDir.x, 0f, flowDir.y);
                Gizmos.DrawLine(pos, targetPos);
            }
        }
        internal void DebugPortalTraversalMarks(FlowFieldAgent agent, NativeArray<float> portalHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }


            PathPortalTraversalData portalTraversalData = _pathContainer.PathPortalTraversalDataList[pathIndex];
            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            float tileSize = FlowFieldUtilities.TileSize;
            FieldGraph fg = _fieldProducer.GetFieldGraphWithOffset(destinationData.Offset);
            NativeArray<PortalNode> portalNodes = fg.PortalNodes;
            NativeArray<PortalTraversalDataRecord> porTravDataRecod = portalTraversalData.PortalDataRecords.AsArray();

            for (int i = 0; i < porTravDataRecod.Length; i++)
            {
                PortalTraversalDataRecord record = porTravDataRecod[i];
                PortalNode node = portalNodes[record.PortalIndex];
                Vector3 portalPos = node.GetPosition(tileSize, FlowFieldUtilities.FieldGridStartPosition);
                portalPos.y = portalHeights[record.PortalIndex];
                Vector3 labelPos = portalPos + new Vector3(0, 3, 0);
                if ((record.Mark & PortalTraversalMark.Explored) == PortalTraversalMark.Explored)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawSphere(portalPos, 0.25f);
                }
                if ((record.Mark & PortalTraversalMark.DijkstraPicked) == PortalTraversalMark.DijkstraPicked)
                {
                    Gizmos.color = Color.white;
                    Gizmos.DrawSphere(portalPos + new Vector3(-0.35f, 0, 0), 0.25f);
                }
                if ((record.Mark & PortalTraversalMark.DijkstraTraversable) == PortalTraversalMark.DijkstraTraversable)
                {
                    Gizmos.color = Color.black;
                    Gizmos.DrawSphere(portalPos + new Vector3(0.35f, 0, 0), 0.25f);
                }
                if ((record.Mark & PortalTraversalMark.GoalNeighbour) == PortalTraversalMark.GoalNeighbour)
                {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawSphere(portalPos + new Vector3(0,0,0.35f), 0.25f);
                }
                Handles.Label(labelPos, record.PortalIndex + " : " + record.DistanceFromTarget.ToString());
            }
        }
        internal void DebugPortalSequence(FlowFieldAgent agent, NativeArray<float> portalHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            float tileSize = FlowFieldUtilities.TileSize;
            int fieldColAmount = FlowFieldUtilities.FieldColAmount;
            float2 fieldGridStartPos = FlowFieldUtilities.FieldGridStartPosition;
            PathPortalTraversalData portalTraversalData = _pathContainer.PathPortalTraversalDataList[pathIndex];
            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            FieldGraph fg = _fieldProducer.GetFieldGraphWithOffset(destinationData.Offset);
            NativeList<ActivePortal> porSeq = portalTraversalData.PortalSequence;
            NativeList<Slice> portSeqSlices = portalTraversalData.PortalSequenceSlices;

            for(int i = 0; i < portSeqSlices.Length; i++)
            {
                Slice slice = portSeqSlices[i];
                int seqStart = slice.Index;
                int seqEnd = slice.Index + slice.Count;
                Gizmos.color = _colorArray[i % _colorArray.Length];
                for(int j = seqStart; j < seqEnd - 1; j++)
                {
                    ActivePortal curActivePortal = porSeq[j];
                    ActivePortal nextActivePortal = porSeq[j + 1];

                    int2 curActivePortalFieldIndex1 = FlowFieldUtilities.To2D(curActivePortal.FieldIndex1, fieldColAmount);
                    int2 curActivePortalFieldIndex2 = FlowFieldUtilities.To2D(curActivePortal.FieldIndex2, fieldColAmount);
                    int2 nextActivePortalFieldIndex1 = FlowFieldUtilities.To2D(nextActivePortal.FieldIndex1, fieldColAmount);
                    int2 nextActivePortalFieldIndex2 = FlowFieldUtilities.To2D(nextActivePortal.FieldIndex2, fieldColAmount);
                    Vector2 curActivePortalPos1 = FlowFieldUtilities.IndexToPos(curActivePortalFieldIndex1, tileSize, fieldGridStartPos);
                    Vector2 curActivePortalPos2 = FlowFieldUtilities.IndexToPos(curActivePortalFieldIndex2, tileSize, fieldGridStartPos);
                    Vector2 nextActivePortalPos1 = FlowFieldUtilities.IndexToPos(nextActivePortalFieldIndex1, tileSize, fieldGridStartPos);
                    Vector2 nextActivePortalPos2 = FlowFieldUtilities.IndexToPos(nextActivePortalFieldIndex2, tileSize, fieldGridStartPos);
                    Vector2 curAvtivePortalAvgPos = (curActivePortalPos1 + curActivePortalPos2) / 2;
                    Vector2 nextAvtivePortalAvgPos = (nextActivePortalPos1 + nextActivePortalPos2) / 2;
                    Vector3 firstPorPos = new Vector3(curAvtivePortalAvgPos.x, 1f, curAvtivePortalAvgPos.y);
                    Vector3 secondPorPos = new Vector3(nextAvtivePortalAvgPos.x, 1f, nextAvtivePortalAvgPos.y);
                    Gizmos.DrawLine(firstPorPos, secondPorPos);
                }
            }
        }
        internal void DebugActiveWaveFronts(FlowFieldAgent agent)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathfindingInternalData internalData = _pathContainer.PathfindingInternalDataList[pathIndex];
            NativeArray<int> keys = internalData.SectorToWaveFrontsMap.GetKeyArray(Allocator.Temp);
            Gizmos.color = Color.red;
            for (int i = 0; i <keys.Length; i++)
            {
                int key = keys[i];
                NativeParallelMultiHashMap<int, ActiveWaveFront>.Enumerator enumerator = internalData.SectorToWaveFrontsMap.GetValuesForKey(key);
                while (enumerator.MoveNext())
                {
                    ActiveWaveFront front = enumerator.Current;
                    int2 general2d = FlowFieldUtilities.GetGeneral2d(front.LocalIndex, key, FlowFieldUtilities.SectorMatrixColAmount, FlowFieldUtilities.SectorColAmount);
                    Vector2 pos = FlowFieldUtilities.IndexToPos(general2d, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
                    Vector3 pos3 = new Vector3(pos.x, 0f, pos.y);
                    Gizmos.DrawCube(pos3, new Vector3(0.5f, 0.5f, 0.5f));
                }
            }

        }
        internal void DebugPickedSectors(FlowFieldAgent agent, NativeArray<float> sectorCornerHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            NativeArray<int> sectorFlowStartTable = _navigationManager.PathDataContainer.SectorToFlowStartTables[pathIndex];
            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            SectorBitArray sectorBitArray = _navigationManager.PathDataContainer.PathSectorBitArrays[pathIndex];
            Gizmos.color = Color.black;
            FieldGraph fg = _fieldProducer.GetFieldGraphWithOffset(destinationData.Offset);
            NativeArray<SectorNode> sectorNodes = fg.SectorNodes;

            for (int i = 0; i < sectorFlowStartTable.Length; i++)
            {
                if (sectorFlowStartTable[i] == 0 && sectorBitArray.HasBit(i)) { UnityEngine.Debug.Log("woooo"); }
                if (sectorFlowStartTable[i] != 0 && !sectorBitArray.HasBit(i)) { UnityEngine.Debug.Log("woooo"); }
            }

            for (int i = 0; i < sectorFlowStartTable.Length; i++)
            {
                if (sectorFlowStartTable[i] == 0) { continue; }
                int2 sector2d = FlowFieldUtilities.To2D(i, FlowFieldUtilities.SectorMatrixColAmount);
                float sectorSize = FlowFieldUtilities.SectorColAmount * FlowFieldUtilities.TileSize;
                float2 sectorPos = FlowFieldUtilities.IndexToPos(sector2d, sectorSize, FlowFieldUtilities.FieldGridStartPosition);
                float2 botLeft2 = sectorPos + new float2(-sectorSize / 2, -sectorSize / 2);
                float2 topLeft2 = sectorPos + new float2(-sectorSize / 2, sectorSize / 2);
                float2 botRight2 = sectorPos + new float2(sectorSize / 2, -sectorSize / 2);
                float2 topRight2 = sectorPos + new float2(sectorSize / 2, sectorSize / 2);
                float3 botLeft3 = new float3(botLeft2.x, sectorCornerHeights[i * 4], botLeft2.y);
                float3 topLeft3 = new float3(topLeft2.x, sectorCornerHeights[i * 4 + 1], topLeft2.y);
                float3 topRight3 = new float3(topRight2.x, sectorCornerHeights[i * 4 + 2], topRight2.y);
                float3 botRight3 = new float3(botRight2.x, sectorCornerHeights[i * 4 + 3], botRight2.y);
                Gizmos.DrawLine(botLeft3, topLeft3);
                Gizmos.DrawLine(topLeft3, topRight3);
                Gizmos.DrawLine(topRight3, botRight3);
                Gizmos.DrawLine(botRight3, botLeft3);
            }
        }
        internal void DebugIntegrationField(FlowFieldAgent agent, NativeArray<float> tileCenterHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathfindingInternalData internalData = _pathContainer.PathfindingInternalDataList[pathIndex];

            NativeArray<int> sectorFlowStartTable = _navigationManager.PathDataContainer.SectorToFlowStartTables[pathIndex];
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            int sectorTileAmount = sectorColAmount * sectorColAmount;
            NativeArray<IntegrationTile> integrationField = internalData.IntegrationField.AsArray();
            for (int i = 0; i < sectorFlowStartTable.Length; i++)
            {
                if (sectorFlowStartTable[i] == 0) { continue; }
                int pickedStartIndex = sectorFlowStartTable[i];
                for (int j = pickedStartIndex; j < pickedStartIndex + sectorTileAmount; j++)
                {
                    int local1d = (j - 1) % sectorTileAmount;
                    int2 general2d = FlowFieldUtilities.GetGeneral2d(local1d, i, FlowFieldUtilities.SectorMatrixColAmount, sectorColAmount);
                    float2 debugPos2 = FlowFieldUtilities.IndexToPos(general2d, _tileSize, FlowFieldUtilities.FieldGridStartPosition);
                    int general1d = FlowFieldUtilities.To1D(general2d, FlowFieldUtilities.FieldColAmount);
                    Vector3 debugPos3 = new Vector3(debugPos2.x, tileCenterHeights[general1d], debugPos2.y);
                    float cost = integrationField[j].Cost;
                    if (cost != float.MaxValue)
                    {
                        Handles.Label(debugPos3, cost.ToString());
                    }
                }

            }
        }
        internal void LOSBlockDebug(FlowFieldAgent agent)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathfindingInternalData internalData = _pathContainer.PathfindingInternalDataList[pathIndex];
            NativeArray<int> sectorFlowStartTable = _navigationManager.PathDataContainer.SectorToFlowStartTables[pathIndex];
            NativeArray<IntegrationTile> integrationField = internalData.IntegrationField.AsArray();
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            int sectorTileAmount = sectorColAmount * sectorColAmount;
            for (int i = 0; i < sectorFlowStartTable.Length; i++)
            {
                if (sectorFlowStartTable[i] == 0) { continue; }
                int pickedStartIndex = sectorFlowStartTable[i];
                for (int j = pickedStartIndex; j < pickedStartIndex + sectorTileAmount; j++)
                {
                    if ((integrationField[j].Mark & IntegrationMark.LOSBlock) == IntegrationMark.LOSBlock)
                    {
                        Gizmos.color = Color.white;
                        int local1d = (j - 1) % sectorTileAmount;
                        int2 general2d = FlowFieldUtilities.GetGeneral2d(local1d, i, FlowFieldUtilities.SectorMatrixColAmount, sectorColAmount);
                        float2 debugPos2 = FlowFieldUtilities.IndexToPos(general2d, _tileSize, FlowFieldUtilities.FieldGridStartPosition);
                        float3 debugPos = new float3(debugPos2.x, 0.02f, debugPos2.y);
                        Gizmos.DrawCube(debugPos, new Vector3(0.3f, 0.3f, 0.3f));
                    }
                    else if ((integrationField[j].Mark & IntegrationMark.LOSC) == IntegrationMark.LOSC)
                    {
                        Gizmos.color = Color.black;
                        int local1d = (j - 1) % sectorTileAmount;
                        int2 general2d = FlowFieldUtilities.GetGeneral2d(local1d, i, FlowFieldUtilities.SectorMatrixColAmount, sectorColAmount);
                        float2 debugPos2 = FlowFieldUtilities.IndexToPos(general2d, _tileSize, FlowFieldUtilities.FieldGridStartPosition);
                        float3 debugPos = new float3(debugPos2.x, 0.02f, debugPos2.y);
                        Gizmos.DrawCube(debugPos, new Vector3(0.3f, 0.3f, 0.3f));
                    }
                }
            }
        }
        internal void DebugFlowField(FlowFieldAgent agent, NativeArray<float> tileCenterHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            float2 destination = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex].Destination;
            NativeArray<SectorFlowStart> dynamicAreaFlowStarts = _navigationManager.PathDataContainer.PathfindingInternalDataList[pathIndex].DynamicArea.SectorFlowStartCalculationBuffer.AsArray();
            NativeArray<FlowData> exposedFlowData = _pathContainer.ExposedFlowData.AsArray();
            NativeArray<bool> exposedLosData = _pathContainer.ExposedLosData.AsArray();
            PathSectorToFlowStartMapper newFlowStartMap = _pathContainer.SectorFlowStartMap;
            NativeArray<ulong> keys = newFlowStartMap.Map.GetKeyArray(Allocator.Temp);
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            int sectorTileAmount = sectorColAmount * sectorColAmount;

            for(int i = 0; i < keys.Length; i++)
            {
                ulong key = keys[i];
                PathSectorToFlowStartMapper.KeyToPathSector(key, out int curPathIndex, out int sectorIndex);
                if(curPathIndex != pathIndex) { continue; }
                newFlowStartMap.TryGet(curPathIndex, sectorIndex, out int newSectorFlowStart);
                for (int j = newSectorFlowStart; j < newSectorFlowStart + sectorTileAmount; j++)
                {
                    int local1d = j % sectorTileAmount;
                    int2 general2d = FlowFieldUtilities.GetGeneral2d(local1d, sectorIndex, FlowFieldUtilities.SectorMatrixColAmount, sectorColAmount);
                    float2 debugPos2 = FlowFieldUtilities.IndexToPos(general2d, _tileSize, FlowFieldUtilities.FieldGridStartPosition);
                    int general1d = FlowFieldUtilities.To1D(general2d, FlowFieldUtilities.FieldColAmount);
                    float3 debugPos = new float3(debugPos2.x, tileCenterHeights[general1d], debugPos2.y);
                    if (HasLOS(newSectorFlowStart, local1d))
                    {
                        Gizmos.color = Color.white;
                        DrawSquare(debugPos, 0.2f);
                        DrawLOS(debugPos, destination);
                    }
                    else
                    {
                        Gizmos.color = Color.black;
                        FlowData flowData = GetFlow(newSectorFlowStart, local1d);
                        if (flowData.IsValid())
                        {
                            DrawSquare(debugPos, 0.2f);
                            DrawFlow(flowData, debugPos);
                        }
                    }
                }
            }
            bool HasLOS(int sectorLosStart, int localIndex)
            {
                return exposedLosData[sectorLosStart + localIndex];
            }

            FlowData GetFlow(int sectorFlowStart, int localIndex)
            {
                return exposedFlowData[sectorFlowStart + localIndex];
            }

            void DrawSquare(Vector3 pos, float size)
            {
                Vector3 botLeft = new Vector3(pos.x - size / 2, pos.y, pos.z - size / 2);
                Vector3 botRight = new Vector3(pos.x + size / 2, pos.y, pos.z - size / 2);
                Vector3 topLeft = new Vector3(pos.x - size / 2, pos.y, pos.z + size / 2);
                Vector3 topRight = new Vector3(pos.x + size / 2, pos.y, pos.z + size / 2);

                Gizmos.DrawLine(topRight, botRight);
                Gizmos.DrawLine(botRight, botLeft);
                Gizmos.DrawLine(botLeft, topLeft);
                Gizmos.DrawLine(topLeft, topRight);
            }
            void DrawLOS(Vector3 pos, float2 destination)
            {
                float3 destination3 = new float3(destination.x, pos.y, destination.y);
                float3 dirToDes = destination3 - (float3)pos;
                dirToDes = math.normalizesafe(dirToDes) * 0.4f;
                Vector3 target = pos + (Vector3)dirToDes;
                Gizmos.DrawLine(pos, target);
            }
            void DrawFlow(FlowData flowData, Vector3 pos)
            {
                float2 flowDir = flowData.GetFlow(_tileSize);
                flowDir = math.normalizesafe(flowDir) * 0.4f;
                Vector3 targetPos = pos + new Vector3(flowDir.x, 0f, flowDir.y);
                Gizmos.DrawLine(pos, targetPos);
            }
        }

        internal void DebugAgentsSubscribedToPath(int pathIndex)
        {
            if(pathIndex >= _navigationManager.PathDataContainer.ExposedPathStateList.Length) { return; }
            NativeArray<int> agentCurPathIndicies = _navigationManager.AgentDataContainer.AgentCurPathIndicies.AsArray();
            TransformAccessArray agentTransforms = _navigationManager.AgentDataContainer.AgentTransforms;
            for(int i = 0; i< agentCurPathIndicies.Length; i++)
            {
                if (agentCurPathIndicies[i] == pathIndex)
                {
                    Gizmos.DrawCube(agentTransforms[i].position, new Vector3(0.5f, 0.5f, 0.5f));
                }
            }
        }
    }

}

#endif