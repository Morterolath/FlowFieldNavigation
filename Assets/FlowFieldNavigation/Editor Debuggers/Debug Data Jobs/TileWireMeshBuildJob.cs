using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace FlowFieldNavigation
{

    [BurstCompile]
    internal struct TileWireMeshBuildJob : IJob
    {
        internal float TileSize;
        internal int2 StartFieldIndex;
        internal int MeshRowCount;
        internal int MeshColCount;

        internal float2 MeshStartPos;
        [ReadOnly] internal TriangleSpatialHashGrid TriangleSpatialHashGrid;
        [ReadOnly] internal NativeArray<float3> HeightMeshVerts;
        [WriteOnly] internal NativeArray<int> Trigs;
        internal NativeArray<float3> Verts;
        public void Execute()
        {
            int vertIndex = 0;
            for (int r = 0; r < MeshRowCount; r++)
            {
                for (int c = 0; c < MeshColCount; c++)
                {
                    float2 v2d = MeshStartPos + new float2(c * TileSize, r * TileSize);
                    float3 v3d = new float3(v2d.x, 0f, v2d.y);
                    Verts[vertIndex++] = v3d;
                }
            }

            HeightMeshImmediateQueryManager.SetHeightsAsY(Verts, TriangleSpatialHashGrid, HeightMeshVerts);

            int trigIndex = 0;
            for (int r = 0; r < MeshRowCount - 1; r++)
            {
                for (int c = 0; c < MeshColCount - 1; c++)
                {
                    int vCur = r * MeshColCount + c;
                    int vUp = vCur + MeshColCount;
                    int vRight = vCur + 1;
                    Trigs[trigIndex++] = vCur;
                    Trigs[trigIndex++] = vUp;
                    Trigs[trigIndex++] = vCur;
                    Trigs[trigIndex++] = vRight;
                }
            }
        }
    }


}