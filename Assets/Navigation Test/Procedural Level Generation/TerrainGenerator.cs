using System.Collections.Generic;
using System.Collections.ObjectModel;
using Unity.Collections;
using UnityEngine;
using System.Diagnostics;
using FlowFieldNavigation;
public class TerrainGenerator : MonoBehaviour
{
    [SerializeField] MeshCollider _fieldMeshCollider;
    [SerializeField] MeshFilter _fieldMeshFilter;
    [SerializeField] FlowFieldNavigationManager _navigationManager;
    [Header("Random Generator")]
    [SerializeField][Range(1,100)] float _unwalkabilityResolution;
    [SerializeField][Range(1,200)] float _heightNoiseResolution;
    public float MaxHeight;
    [SerializeField] SimulationState _simulationState;
    [SerializeField] Material _obstacleMat;
    [SerializeField] Material _fieldMat;
    [HideInInspector] public WalkabilityData WalkabilityData;

    public float TileSize;
    public int RowAmount;
    public int ColumnAmount;
    public bool RandomHeights;
    public bool DisableObstacles;
    public bool DisableTerrainGeneration;

    ObstacleGenerator obsGenerator;

    private void Start()
    {
        if (!DisableTerrainGeneration)
        {
            WalkabilityData = new WalkabilityData(TileSize, RowAmount, ColumnAmount, _unwalkabilityResolution, _simulationState);

            NativeArray<float> vertexHeights = GenerateMesh();
            if (!DisableObstacles)
            {
                obsGenerator = new ObstacleGenerator(this, WalkabilityData, _obstacleMat);
                obsGenerator.CreateMesh(vertexHeights);
            }
            FlowFieldStaticObstacle[] obstacleBehaviors = FindObjectsByType<FlowFieldStaticObstacle>(FindObjectsSortMode.None);
            FlowFieldSurface[] flowFieldSurfaces = FindObjectsByType<FlowFieldSurface>(FindObjectsSortMode.None);

            SimulationStartParametersStandard simParam = new SimulationStartParametersStandard(flowFieldSurfaces, obstacleBehaviors, 0.5f, 0.25f, 5f, 0.1f, 1f, 0.1f, 30);
            _navigationManager.Interface.StartSimulation(simParam);

            for (int i = 0; i < obstacleBehaviors.Length; i++)
            {
                FlowFieldStaticObstacle obstacle = obstacleBehaviors[i];
                if (!obstacle.CanBeDisposed) { continue; }
                Destroy(obstacle.gameObject);
            }
        }

    }
    public NativeArray<float> GenerateMesh()
    {
        int vertColAmount = ColumnAmount + 1;
        int vertRowAmount = RowAmount + 1;
        NativeArray<float> vertexHeights = new NativeArray<float>(vertColAmount * vertRowAmount, Allocator.Temp);
        int vertexHeightIndex = 0;
        for(int i = 0; i < vertRowAmount; i++)
        {
            for(int j = 0; j < vertColAmount; j++)
            {
                float height = Mathf.PerlinNoise(j / _heightNoiseResolution, i / _heightNoiseResolution) * MaxHeight;
                vertexHeights[vertexHeightIndex] = RandomHeights ? height : 0;
                vertexHeightIndex++;
            }
        }

        int basePartitionSize = 128;
        int partitionColAmount = vertColAmount / 128;
        partitionColAmount = vertColAmount % 128 != 0 ? partitionColAmount + 1 : partitionColAmount;
        int partitionRowAmount = vertRowAmount / 128;
        partitionRowAmount = vertRowAmount % 128 != 0 ? partitionRowAmount + 1 : partitionRowAmount;

        for(int i = 0; i < partitionRowAmount; i++)
        {
            int partitionVertRowAmount = basePartitionSize;
            int startVertexRow = i * (basePartitionSize - 1);
            if (i + 1 == partitionRowAmount) { partitionVertRowAmount = vertRowAmount - startVertexRow; }
            for(int j = 0; j < partitionColAmount; j++)
            {
                int partitionVertColAmount = basePartitionSize;
                int startVertexCol = j * (basePartitionSize - 1);
                if (j + 1 == partitionColAmount) { partitionVertColAmount = vertColAmount - startVertexCol; }
                Vector3 partitionStartPos = gameObject.transform.position + new Vector3(startVertexCol * TileSize, 0f, startVertexRow * TileSize);
                GeneratePartition(partitionStartPos, partitionVertColAmount, partitionVertRowAmount, vertexHeights, startVertexRow * vertColAmount + startVertexCol);
            }
        }
        return vertexHeights;
    }

    void GeneratePartition(Vector3 startPos, int vertColAmount, int vertRowAmount, NativeArray<float> heights, int vertStartIndex)
    {
        GameObject partitionObject = new GameObject("Partition");
        partitionObject.layer = 3;
        partitionObject.transform.position = startPos + new Vector3((vertColAmount -1)* TileSize / 2, 0f, (vertRowAmount -1) * TileSize / 2);
        partitionObject.transform.parent = gameObject.transform;
        //partitionObject.transform.eulerAngles = new Vector3(0, 45, 0);
        partitionObject.AddComponent<MeshFilter>();
        partitionObject.AddComponent<MeshCollider>();
        partitionObject.AddComponent<MeshRenderer>();
        partitionObject.AddComponent<FlowFieldSurface>();
        FlowFieldSurface navSurface = partitionObject.GetComponent<FlowFieldSurface>();
        MeshFilter meshFilter = partitionObject.GetComponent<MeshFilter>();
        MeshCollider meshCollider = partitionObject.GetComponent<MeshCollider>();
        MeshRenderer meshRenderer = partitionObject.GetComponent<MeshRenderer>();
        meshRenderer.material = _fieldMat;


        //CONFIGURE FIELD MESH
        Mesh partitionMesh = new Mesh();
        partitionMesh.name = "Field Mesh";
        meshFilter.mesh = partitionMesh;


        Vector3 vertexBaseTranslation = new Vector3(-(vertColAmount - 1) * TileSize / 2, 0f, -(vertRowAmount - 1) * TileSize / 2);
        //Set verts and trigs
        Vector3[] verticies = new Vector3[vertColAmount * vertRowAmount];
        int vertexIndex = 0;
        for (int i = 0; i < vertRowAmount; i++)
        {
            for (int j = 0; j < vertColAmount; j++)
            {
                int heightIndex = vertStartIndex + j + i * (ColumnAmount + 1);
                verticies[vertexIndex] = new Vector3(j * TileSize, heights[heightIndex], i * TileSize) + vertexBaseTranslation;
                vertexIndex++;
            }
        }

        int[] triangles = new int[(vertColAmount -1) * (vertRowAmount - 1) * 3 * 2];
        int trigIndex = 0;
        for (int i = 0; i < vertRowAmount - 1; i++)
        {
            for (int j = 0; j < vertColAmount - 1; j++)
            {
                int index1 = j + i * vertColAmount;
                int index2 = index1 + vertColAmount;
                int index3 = index1 + 1;
                triangles[trigIndex] = index1;
                triangles[trigIndex + 1] = index2;
                triangles[trigIndex + 2] = index3;
                trigIndex += 3;
            }
        }
        for (int i = 0; i < vertRowAmount - 1; i++)
        {
            for (int j = 1; j < vertColAmount; j++)
            {
                int index1 = j + i * vertColAmount;
                int index2 = index1 + vertColAmount - 1;
                int index3 = index2 + 1;
                triangles[trigIndex] = index1;
                triangles[trigIndex + 1] = index2;
                triangles[trigIndex + 2] = index3;
                trigIndex += 3;
            }
        }

        //UPDATE FIELD MESH
        partitionMesh.Clear();
        partitionMesh.vertices = verticies;
        partitionMesh.triangles = triangles;
        partitionMesh.RecalculateNormals();

        meshCollider.sharedMesh = partitionMesh;
    }
}
public enum SimulationState : byte
{
    PerlinNoise,
    FullWalkable
}