﻿using Unity.Collections;
using UnityEditor;
using UnityEngine;

public class PathDebugger
{
    PathProducer _pathProducer;
    PathfindingManager _pathfindingManager;
    CostFieldProducer _costFieldProducer;
    float _tileSize;

    public PathDebugger(PathfindingManager pathfindingManager)
    {
        _pathProducer = pathfindingManager.PathProducer;
        _pathfindingManager = pathfindingManager;
        _costFieldProducer = pathfindingManager.CostFieldProducer;
        _tileSize = pathfindingManager.TileSize;
    }

    public void DebugBFS()
    {
        Gizmos.color = Color.black;

        Path producedPath = _pathProducer.ProducedPath;
        float tileSize = _pathfindingManager.TileSize;
        FieldGraph fg = _costFieldProducer.GetCostFieldWithOffset(producedPath.Offset).FieldGraph;
        NativeArray<PortalNode> portalNodes = fg.PortalNodes;
        NativeArray<int> connections = producedPath.ConnectionIndicies;
        NativeArray<float> distances = producedPath.PortalDistances;
        for (int i = 0; i < portalNodes.Length; i++)
        {
            PortalNode startNode = portalNodes[i];
            if (startNode.Portal1.Index.R == 0) { continue; }
            PortalNode endNode = portalNodes[connections[i]];
            Vector3 start = startNode.GetPosition(tileSize) + new Vector3(0, 0.05f, 0);
            Vector3 end = endNode.GetPosition(tileSize) + new Vector3(0, 0.05f, 0);
            float distance = Vector3.Distance(start, end);
            end = Vector3.MoveTowards(start, end, distance - 0.3f);
            float cost = distances[i];
            Gizmos.DrawLine(start, end);
            Handles.Label(start + new Vector3(0, 0, 0.5f), cost.ToString());
        }
    }
    public void DebugPortalSequence()
    {
        Gizmos.color = Color.black;

        Path producedPath = _pathProducer.ProducedPath;
        float tileSize = _pathfindingManager.TileSize;
        FieldGraph fg = _costFieldProducer.GetCostFieldWithOffset(producedPath.Offset).FieldGraph;
        NativeArray<PortalNode> portalNodes = fg.PortalNodes;
        NativeList<PortalSequence> porSeq = producedPath.PortalSequence;
        for (int i = 0; i < porSeq.Length; i++)
        {
            PortalNode portalNode = portalNodes[porSeq[i].PortalPtr];
            Gizmos.DrawSphere(portalNode.GetPosition(_tileSize), 0.5f);
        }
    }
    public void DebugPickedSectors()
    {

        Path producedPath = _pathProducer.ProducedPath;
        float yOffset = 0.3f;
        Gizmos.color = Color.black;
        float tileSize = _pathfindingManager.TileSize;
        FieldGraph fg = _costFieldProducer.GetCostFieldWithOffset(producedPath.Offset).FieldGraph;
        NativeArray<SectorNode> sectorNodes = fg.SectorNodes;
        NativeArray<int> pickedSectorNodes = producedPath.PickedSectors;
        for (int i = 0; i < pickedSectorNodes.Length; i++)
        {
            Index2 index = sectorNodes[pickedSectorNodes[i]].Sector.StartIndex;
            int sectorSize = sectorNodes[pickedSectorNodes[i]].Sector.Size;
            Vector3 botLeft = new Vector3(index.C * tileSize, yOffset, index.R * tileSize);
            Vector3 botRight = new Vector3((index.C + sectorSize) * tileSize, yOffset, index.R * tileSize);
            Vector3 topLeft = new Vector3(index.C * tileSize, yOffset, (index.R + sectorSize) * tileSize);
            Vector3 topRight = new Vector3((index.C + sectorSize) * tileSize, yOffset, (index.R + sectorSize) * tileSize);
            Gizmos.DrawLine(botLeft, topLeft);
            Gizmos.DrawLine(topLeft, topRight);
            Gizmos.DrawLine(topRight, botRight);
            Gizmos.DrawLine(botRight, botLeft);
        }
    }
    public void DebugIntegrationField(NativeArray<Vector3> tilePositions)
    {
        Path producedPath = _pathProducer.ProducedPath;
        float yOffset = 0.2f;
        NativeArray<IntegrationTile> integrationField = producedPath.IntegrationField;
        for (int i = 0; i < integrationField.Length; i++)
        {
            DebugCosts(i);
        }
        void DebugCosts(int index)
        {
            float cost = integrationField[index].Cost;
            if (cost == float.MaxValue)
            {
                return;
            }
            Handles.Label(tilePositions[index], cost.ToString());
        }
        void DebugMarks(int index)
        {
            IntegrationMark mark = integrationField[index].Mark;
            if (mark == IntegrationMark.None)
            {
                Handles.Label(tilePositions[index], "N");
            }
        }
    }
    public void DebugFlowField(NativeArray<Vector3> tilePositions)
    {
        Path producedPath = _pathProducer.ProducedPath;
        float yOffset = 0.1f;
        Gizmos.color = Color.black;
        float tileSize = _tileSize;
        NativeArray<FlowData> flowfield = producedPath.FlowField;
        for (int i = 0; i < flowfield.Length; i++)
        {
            if (flowfield[i] == FlowData.None) { continue; }
            DrawSquare(tilePositions[i], 0.125f);
            DrawFlow(flowfield[i], tilePositions[i]);
        }

        void DrawSquare(Vector3 pos, float size)
        {
            Vector3 botLeft = new Vector3(pos.x - size / 2, yOffset, pos.z - size / 2);
            Vector3 botRight = new Vector3(pos.x + size / 2, yOffset, pos.z - size / 2);
            Vector3 topLeft = new Vector3(pos.x - size / 2, yOffset, pos.z + size / 2);
            Vector3 topRight = new Vector3(pos.x + size / 2, yOffset, pos.z + size / 2);

            Gizmos.DrawLine(topRight, botRight);
            Gizmos.DrawLine(botRight, botLeft);
            Gizmos.DrawLine(botLeft, topLeft);
            Gizmos.DrawLine(topLeft, topRight);
        }
        void DrawFlow(FlowData flow, Vector3 pos)
        {
            pos = pos + new Vector3(0, yOffset, 0);

            if (flow == FlowData.N)
            {
                Vector3 dir = pos + new Vector3(0, yOffset, 0.4f);
                Gizmos.DrawLine(pos, dir);
            }
            else if (flow == FlowData.NE)
            {
                Vector3 dir = pos + new Vector3(0.4f, yOffset, 0.4f);
                Gizmos.DrawLine(pos, dir);
            }
            else if (flow == FlowData.E)
            {
                Vector3 dir = pos + new Vector3(0.4f, yOffset, 0);
                Gizmos.DrawLine(pos, dir);
            }
            else if (flow == FlowData.SE)
            {
                Vector3 dir = pos + new Vector3(0.4f, yOffset, -0.4f);
                Gizmos.DrawLine(pos, dir);
            }
            else if (flow == FlowData.S)
            {
                Vector3 dir = pos + new Vector3(0, yOffset, -0.4f);
                Gizmos.DrawLine(pos, dir);
            }
            else if (flow == FlowData.SW)
            {
                Vector3 dir = pos + new Vector3(-0.4f, yOffset, -0.4f);
                Gizmos.DrawLine(pos, dir);
            }
            else if (flow == FlowData.W)
            {
                Vector3 dir = pos + new Vector3(-0.4f, yOffset, 0);
                Gizmos.DrawLine(pos, dir);
            }
            else if (flow == FlowData.NW)
            {
                Vector3 dir = pos + new Vector3(-0.4f, yOffset, 0.4f);
                Gizmos.DrawLine(pos, dir);
            }
        }
    }
}