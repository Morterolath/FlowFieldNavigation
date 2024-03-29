﻿#if (UNITY_EDITOR)

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

        internal EditorPathDebugger(FlowFieldNavigationManager navigationManager)
        {
            _pathContainer = navigationManager.PathDataContainer;
            _navigationManager = navigationManager;
            _fieldProducer = navigationManager.FieldDataContainer;
            _tileSize = FlowFieldUtilities.TileSize;
        }
        internal void DebugDynamicAreaIntegration(FlowFieldAgent agent, NativeArray<float> tileCenterHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathfindingInternalData internalData = _pathContainer.PathfindingInternalDataList[pathIndex];
            PathLocationData locationData = _navigationManager.PathDataContainer.PathLocationDataList[pathIndex];
            UnsafeList<SectorFlowStart> pickedSectorFlowStarts = locationData.DynamicAreaPickedSectorFlowStarts;
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
            PathLocationData locationData = _navigationManager.PathDataContainer.PathLocationDataList[pathIndex];
            PathFlowData flowData = _navigationManager.PathDataContainer.PathFlowDataList[pathIndex];
            UnsafeList<SectorFlowStart> pickedSectorFlowStarts = locationData.DynamicAreaPickedSectorFlowStarts;
            UnsafeList<FlowData> flowField = flowData.DynamicAreaFlowField;
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
            NativeArray<PortalTraversalData> portalTraversalDataArray = portalTraversalData.PortalTraversalDataArray;

            for (int i = 0; i < portalNodes.Length; i++)
            {
                PortalTraversalData travData = portalTraversalDataArray[i];
                PortalNode node = portalNodes[i];
                if (travData.HasMark(PortalTraversalMark.DijkstraPicked))
                {
                    Gizmos.color = Color.white;
                    Vector3 portalPos = node.GetPosition(tileSize, FlowFieldUtilities.FieldGridStartPosition);
                    portalPos.y = portalHeights[i];
                    Vector3 labelPos = portalPos + new Vector3(0, 3, 0);
                    Handles.Label(labelPos, "d: " + travData.DistanceFromTarget.ToString());
                    Gizmos.DrawSphere(portalPos, 0.25f);
                }
                else if (travData.HasMark(PortalTraversalMark.Reduced))
                {
                    Gizmos.color = Color.black;
                    Vector3 portalPos = node.GetPosition(tileSize, FlowFieldUtilities.FieldGridStartPosition);
                    portalPos.y = portalHeights[i];
                    Vector3 labelPos = portalPos + new Vector3(0, 3, 0);
                    Handles.Label(labelPos, "d: " + travData.DistanceFromTarget.ToString());
                    Gizmos.DrawSphere(portalPos, 0.25f);
                }
            }
        }
        internal void DebugTargetNeighbourPortals(FlowFieldAgent agent, NativeArray<float> portalHeights)
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
            NativeArray<PortalTraversalData> portalTraversalDataArray = portalTraversalData.PortalTraversalDataArray;

            for (int i = 0; i < portalNodes.Length; i++)
            {
                PortalTraversalData travData = portalTraversalDataArray[i];
                PortalNode node = portalNodes[i];
                if (travData.HasMark(PortalTraversalMark.TargetNeighbour))
                {
                    Gizmos.color = Color.magenta;
                    Vector3 portalPos = node.GetPosition(tileSize, FlowFieldUtilities.FieldGridStartPosition);
                    portalPos.y = portalHeights[i];
                    Vector3 labelPos = portalPos + new Vector3(0, 3, 0);
                    Handles.Label(labelPos, "d: " + travData.DistanceFromTarget.ToString());
                    Gizmos.DrawSphere(portalPos, 0.25f);
                }
            }
        }
        internal void DebugPortalSequence(FlowFieldAgent agent, NativeArray<float> portalHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathPortalTraversalData portalTraversalData = _pathContainer.PathPortalTraversalDataList[pathIndex];
            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            FieldGraph fg = _fieldProducer.GetFieldGraphWithOffset(destinationData.Offset);
            NativeArray<PortalNode> portalNodes = fg.PortalNodes;
            NativeList<ActivePortal> porSeq = portalTraversalData.PortalSequence;
            NativeList<int> portSeqBorders = portalTraversalData.PortalSequenceBorders;
            Gizmos.color = Color.white;
            if (porSeq.Length == 0) { return; }

            for (int i = 0; i < portSeqBorders.Length - 1; i++)
            {
                int start = portSeqBorders[i];
                int curNodeIndex = start;
                int nextNodeIndex = porSeq[start].NextIndex;
                while (nextNodeIndex != -1)
                {
                    int curPortalIndex = porSeq[curNodeIndex].Index;
                    int nextPortalIndex = porSeq[nextNodeIndex].Index;

                    //DRAW
                    PortalNode firstportalNode = portalNodes[curPortalIndex];
                    PortalNode secondportalNode = portalNodes[nextPortalIndex];
                    if (firstportalNode.Portal1.Index.R == 0) { continue; }
                    Vector3 secondPorPos = secondportalNode.GetPosition(_tileSize, FlowFieldUtilities.FieldGridStartPosition) + new Vector3(0, portalHeights[nextPortalIndex], 0);
                    Vector3 firstPorPos = firstportalNode.GetPosition(_tileSize, FlowFieldUtilities.FieldGridStartPosition) + new Vector3(0, portalHeights[curNodeIndex], 0);

                    Gizmos.DrawLine(firstPorPos, secondPorPos);
                    //Gizmos.DrawLine(secondPorPos, secondPorPos + rightArrow3);
                    //Gizmos.DrawLine(secondPorPos, secondPorPos + leftArrow3);

                    curNodeIndex = nextNodeIndex;
                    nextNodeIndex = porSeq[nextNodeIndex].NextIndex;
                }
            }
        }
        internal void DebugPickedSectors(FlowFieldAgent agent, NativeArray<float> sectorCornerHeights)
        {
            if (_pathContainer == null) { return; }
            if (_pathContainer.PathfindingInternalDataList.Count == 0) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathLocationData locationData = _navigationManager.PathDataContainer.PathLocationDataList[pathIndex];
            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            SectorBitArray sectorBitArray = _navigationManager.PathDataContainer.PathSectorBitArrays[pathIndex];
            Gizmos.color = Color.black;
            FieldGraph fg = _fieldProducer.GetFieldGraphWithOffset(destinationData.Offset);
            NativeArray<SectorNode> sectorNodes = fg.SectorNodes;
            UnsafeList<int> sectorMarks = locationData.SectorToPicked;

            for (int i = 0; i < sectorMarks.Length; i++)
            {
                if (sectorMarks[i] == 0 && sectorBitArray.HasBit(i)) { UnityEngine.Debug.Log("woooo"); }
                if (sectorMarks[i] != 0 && !sectorBitArray.HasBit(i)) { UnityEngine.Debug.Log("woooo"); }
            }

            for (int i = 0; i < sectorMarks.Length; i++)
            {
                if (sectorMarks[i] == 0) { continue; }
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

            PathLocationData locationData = _navigationManager.PathDataContainer.PathLocationDataList[pathIndex];
            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            int sectorTileAmount = sectorColAmount * sectorColAmount;
            int sectorMatrixColAmount = FlowFieldUtilities.SectorMatrixColAmount;
            UnsafeList<int> sectorMarks = locationData.SectorToPicked;
            NativeArray<IntegrationTile> integrationField = internalData.IntegrationField.AsArray();
            for (int i = 0; i < sectorMarks.Length; i++)
            {
                if (sectorMarks[i] == 0) { continue; }
                int pickedStartIndex = sectorMarks[i];
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

            PathLocationData locationData = _navigationManager.PathDataContainer.PathLocationDataList[pathIndex];
            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            UnsafeList<int> sectorMarks = locationData.SectorToPicked;
            NativeArray<IntegrationTile> integrationField = internalData.IntegrationField.AsArray();
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            int sectorTileAmount = sectorColAmount * sectorColAmount;
            for (int i = 0; i < sectorMarks.Length; i++)
            {
                if (sectorMarks[i] == 0) { continue; }
                int pickedStartIndex = sectorMarks[i];
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

            PathfindingInternalData internalData = _pathContainer.PathfindingInternalDataList[pathIndex];

            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[pathIndex];
            PathLocationData locationData = _navigationManager.PathDataContainer.PathLocationDataList[pathIndex];
            PathFlowData pathFlowData = _navigationManager.PathDataContainer.PathFlowDataList[pathIndex];
            float yOffset = 0.2f;
            UnsafeList<int> sectorMarks = locationData.SectorToPicked;
            UnsafeList<FlowData> flowField = pathFlowData.FlowField;
            UnsafeLOSBitmap losmap = pathFlowData.LOSMap;
            UnsafeList<SectorFlowStart> dynamicAreaFlowStarts = locationData.DynamicAreaPickedSectorFlowStarts;
            UnsafeList<FlowData> dynamicAreaFlowField = internalData.DynamicArea.FlowFieldCalculationBuffer;
            int sectorColAmount = FlowFieldUtilities.SectorColAmount;
            int sectorTileAmount = sectorColAmount * sectorColAmount;
            for (int i = 0; i < sectorMarks.Length; i++)
            {
                if (sectorMarks[i] == 0) { continue; }
                int pickedStartIndex = sectorMarks[i];
                for (int j = pickedStartIndex; j < pickedStartIndex + sectorTileAmount; j++)
                {
                    int local1d = (j - 1) % sectorTileAmount;
                    int2 general2d = FlowFieldUtilities.GetGeneral2d(local1d, i, FlowFieldUtilities.SectorMatrixColAmount, sectorColAmount);
                    float2 debugPos2 = FlowFieldUtilities.IndexToPos(general2d, _tileSize, FlowFieldUtilities.FieldGridStartPosition);
                    int general1d = FlowFieldUtilities.To1D(general2d, FlowFieldUtilities.FieldColAmount);
                    float3 debugPos = new float3(debugPos2.x, tileCenterHeights[general1d], debugPos2.y);
                    if (j >= flowField.Length) { continue; }
                    if (HasLOS(i, local1d))
                    {
                        Gizmos.color = Color.white;
                        DrawSquare(debugPos, 0.2f);
                        DrawLOS(debugPos, destinationData.Destination);
                    }
                    else if (HasDynamicFlow(i, local1d))
                    {
                        Gizmos.color = Color.blue;
                        DrawSquare(debugPos, 0.2f);
                        DrawFlow(GetDynamicFlow(i, local1d), debugPos);
                    }
                    else
                    {
                        Gizmos.color = Color.black;
                        FlowData flowData = GetFlow(i, local1d);
                        if (flowData.IsValid())
                        {
                            DrawSquare(debugPos, 0.2f);
                            DrawFlow(flowData, debugPos);
                        }
                    }
                }
            }
            bool HasLOS(int sectorIndex, int localIndex)
            {
                return losmap.IsLOS(sectorMarks[sectorIndex] + localIndex);
            }
            bool HasDynamicFlow(int sectorIndex, int localIndex)
            {
                int sectorFlowStart = 0;
                for (int i = 0; i < dynamicAreaFlowStarts.Length; i++)
                {
                    SectorFlowStart flowStart = dynamicAreaFlowStarts[i];
                    sectorFlowStart = math.select(sectorFlowStart, flowStart.FlowStartIndex, flowStart.SectorIndex == sectorIndex);
                }
                if (sectorFlowStart == 0) { return false; }

                return dynamicAreaFlowField[sectorFlowStart + localIndex].IsValid();
            }

            FlowData GetFlow(int sectorIndex, int localIndex)
            {
                return flowField[sectorMarks[sectorIndex] + localIndex];
            }

            FlowData GetDynamicFlow(int sectorIndex, int localIndex)
            {
                int sectorFlowStart = 0;
                for (int i = 0; i < dynamicAreaFlowStarts.Length; i++)
                {
                    SectorFlowStart flowStart = dynamicAreaFlowStarts[i];
                    sectorFlowStart = math.select(sectorFlowStart, flowStart.FlowStartIndex, flowStart.SectorIndex == sectorIndex);
                }
                return dynamicAreaFlowField[sectorFlowStart + localIndex];
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

        internal void DebugDestination(FlowFieldAgent agent)
        {
            if (agent == null) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[_navigationManager.Interface.GetPathIndex(agent)];
            NativeArray<float3> heightMeshVerticies = _navigationManager.FieldDataContainer.HeightMeshGenerator.Verticies.AsArray();
            TriangleSpatialHashGrid spatialHashGrid = _navigationManager.FieldDataContainer.HeightMeshGenerator.GetTriangleSpatialHashGrid();
            Vector2 destination = destinationData.Destination;
            Vector3 destination3 = new Vector3(destination.x, GetHeight(destination, spatialHashGrid, heightMeshVerticies), destination.y);
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(destination3, 0.3f);



            float GetHeight(float2 pos, TriangleSpatialHashGrid triangleSpatialHashGrid, NativeArray<float3> heightMeshVerts)
            {
                float curHeight = float.MinValue;
                for (int i = 0; i < triangleSpatialHashGrid.GetGridCount(); i++)
                {
                    bool succesfull = triangleSpatialHashGrid.TryGetIterator(pos, i, out TriangleSpatialHashGridIterator triangleGridIterator);
                    if (!succesfull) { return 0; }
                    while (triangleGridIterator.HasNext())
                    {
                        NativeSlice<int> triangles = triangleGridIterator.GetNextRow();
                        for (int j = 0; j < triangles.Length; j += 3)
                        {
                            int v1Index = triangles[j];
                            int v2Index = triangles[j + 1];
                            int v3Index = triangles[j + 2];
                            float3 v13d = heightMeshVerts[v1Index];
                            float3 v23d = heightMeshVerts[v2Index];
                            float3 v33d = heightMeshVerts[v3Index];
                            float2 v1 = new float2(v13d.x, v13d.z);
                            float2 v2 = new float2(v23d.x, v23d.z);
                            float2 v3 = new float2(v33d.x, v33d.z);

                            BarycentricCoordinates barCords = GetBarycentricCoordinatesForEachVectorInTheOrderUVW(v1, v2, v3, pos);
                            if (barCords.u < 0 || barCords.w < 0 || barCords.v < 0) { continue; }
                            float newHeight = v13d.y * barCords.u + v23d.y * barCords.v + v33d.y * barCords.w;
                            curHeight = math.select(curHeight, newHeight, newHeight > curHeight);
                        }
                    }
                }
                return math.select(curHeight, 0, curHeight == float.MinValue);
            }
            BarycentricCoordinates GetBarycentricCoordinatesForEachVectorInTheOrderUVW(float2 a, float2 b, float2 c, float2 p)
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