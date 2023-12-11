﻿using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using System.Diagnostics;
using static UnityEngine.GraphicsBuffer;

public class PathfindingManager : MonoBehaviour
{
    [SerializeField] TerrainGenerator _terrainGenerator;
    [SerializeField] int _maxCostfieldOffset;
    [SerializeField] public float AgentUpdateFrequency;
    [SerializeField] public float BaseSpatialGridSize;
    [SerializeField] public int LineOfSightRange;

    [HideInInspector] public float TileSize;
    [HideInInspector] public int RowAmount;
    [HideInInspector] public int ColumnAmount;
    [HideInInspector] public byte SectorColAmount = 10;
    [HideInInspector] public int SectorMatrixColAmount;
    [HideInInspector] public int SectorMatrixRowAmount;

    public FieldProducer FieldProducer;
    public PathContainer PathContainer;
    public AgentDataContainer AgentDataContainer;

    float _lastAgentUpdateTime = 0;
    PathfindingUpdateRoutine _pathfindingRoutineUpdater;
    AgentUpdater _agentUpdater;

    private void Awake()
    {
        AgentDataContainer = new AgentDataContainer(this);
    }
    private void Start()
    {

        //!!!ORDER IS IMPORTANT!!!
        TileSize = _terrainGenerator.TileSize;
        RowAmount = _terrainGenerator.RowAmount;
        ColumnAmount = _terrainGenerator.ColumnAmount;
        SectorMatrixColAmount = ColumnAmount / SectorColAmount;
        SectorMatrixRowAmount = RowAmount / SectorColAmount;
        SetFlowFieldUtilities();


        FieldProducer = new FieldProducer(_terrainGenerator.WalkabilityData, SectorColAmount);
        FieldProducer.CreateField(_maxCostfieldOffset, SectorColAmount, SectorMatrixColAmount, SectorMatrixRowAmount, RowAmount, ColumnAmount, TileSize);

        PathContainer = new PathContainer(this);
        _pathfindingRoutineUpdater = new PathfindingUpdateRoutine(this, PathContainer);
        _agentUpdater = new AgentUpdater(AgentDataContainer);

        
    }
    private void Update()
    {
        _agentUpdater.OnUpdate();
        _pathfindingRoutineUpdater.IntermediateLateUpdate();

        float curTime = Time.realtimeSinceStartup;
        float timePassed = curTime - _lastAgentUpdateTime;
        if (timePassed >= AgentUpdateFrequency)
        {
            _lastAgentUpdateTime = curTime;
            _pathfindingRoutineUpdater.RoutineUpdate(timePassed);
        }
    }
    private void FixedUpdate()
    {
        //_pathfindingRoutineUpdater.RoutineUpdate(Time.fixedDeltaTime);

    }
    private void LateUpdate()
    {
        _pathfindingRoutineUpdater.IntermediateLateUpdate();
    }
    void SetFlowFieldUtilities()
    {
        FlowFieldUtilities.DebugMode = false;
        FlowFieldUtilities.SectorMatrixTileAmount = SectorMatrixColAmount * SectorMatrixRowAmount;
        FlowFieldUtilities.SectorMatrixRowAmount = SectorMatrixRowAmount;
        FlowFieldUtilities.SectorMatrixColAmount = SectorMatrixColAmount;
        FlowFieldUtilities.SectorColAmount = SectorColAmount;
        FlowFieldUtilities.SectorRowAmount = SectorColAmount;
        FlowFieldUtilities.SectorTileAmount = SectorColAmount * SectorColAmount;
        FlowFieldUtilities.TileSize = TileSize;
        FlowFieldUtilities.FieldColAmount = ColumnAmount;
        FlowFieldUtilities.FieldRowAmount = RowAmount;
        FlowFieldUtilities.FieldTileAmount = ColumnAmount * RowAmount;
        FlowFieldUtilities.BaseSpatialGridSize = BaseSpatialGridSize;
        FlowFieldUtilities.MinAgentSize = 0;
        FlowFieldUtilities.MaxAgentSize = (_maxCostfieldOffset * TileSize * 2) + TileSize;
        FlowFieldUtilities.LOSRange = LineOfSightRange;
    }
    public void SetDestination(List<FlowFieldAgent> agents, Vector3 target)
    {
        if (agents.Count == 0) { UnityEngine.Debug.Log("Agent list passed is empty"); return; }
        _pathfindingRoutineUpdater.RequestPath(agents, target);
    }
    public void SetDestination(List<FlowFieldAgent> agents, FlowFieldAgent targetAgent)
    {
        if (agents.Count == 0) { UnityEngine.Debug.Log("Agent list passed is empty"); return; }
        _pathfindingRoutineUpdater.RequestPath(agents, targetAgent);
    }
    public NativeArray<UnsafeList<HashTile>> GetSpatialHashGridArray()
    {
        return _pathfindingRoutineUpdater.GetRoutineScheduler().GetRoutineDataProducer().HashGridArray;
    }
    public NativeArray<int> GetNormalToHashed()
    {
        return _pathfindingRoutineUpdater.GetRoutineScheduler().GetRoutineDataProducer().NormalToHashed;
    }
    public NativeArray<AgentMovementData> GetAgentMovementData()
    {
        return _pathfindingRoutineUpdater.GetRoutineScheduler().GetRoutineDataProducer().AgentMovementDataList;
    }
    public void EditCost(int2 startingPoint, int2 endPoint, byte newCost)
    {
        _pathfindingRoutineUpdater.RequestCostEdit(startingPoint, endPoint, newCost);
    }
    public void RequestSubscription(FlowFieldAgent agent)
    {
        _pathfindingRoutineUpdater.RequestAgentAddition(agent);
    }
    public void UnSubscribe(FlowFieldAgent agent)
    {
        AgentDataContainer.UnSubscribe(agent);
    }
    public Path GetPath(int agentIndex)
    {
        int curPathIndex = AgentDataContainer.AgentCurPathIndicies[agentIndex];
        if(curPathIndex == -1) { return null; }
        return PathContainer.ProducedPaths[curPathIndex];
    }
    public int GetPathIndex(int agentIndex)
    {
        return AgentDataContainer.AgentCurPathIndicies[agentIndex];
    }
    public List<FlowFieldAgent> GetAllAgents()
    {
        return AgentDataContainer.Agents;
    }
    public int GetAgentCount()
    {
        return AgentDataContainer.Agents.Count;
    }
}
