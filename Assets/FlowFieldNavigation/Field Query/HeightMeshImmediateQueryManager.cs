using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    [BurstCompile]
    internal class HeightMeshImmediateQueryManager
    {
        FlowFieldNavigationManager _navManager;

        internal HeightMeshImmediateQueryManager(FlowFieldNavigationManager navManager)
        {
            _navManager = navManager;
        }

        internal float GetHeight(in float2 point)
        {
            TriangleSpatialHashGrid triangleSpatialHashGrid = _navManager.FieldDataContainer.HeightMeshGenerator.GetTriangleSpatialHashGrid();
            NativeArray<float3> heightMeshVerticies = _navManager.FieldDataContainer.HeightMeshGenerator.Verticies.AsArray();
            return GetHeightBurst(in point, in triangleSpatialHashGrid, in heightMeshVerticies);
        }
        [BurstCompile]
        internal static float GetHeightBurst(in float2 point, in TriangleSpatialHashGrid triangleSpatialHashGrid, in NativeArray<float3> heightMeshVerticies)
        {
            float curHeight = float.MinValue;
            for (int i = 0; i < triangleSpatialHashGrid.GetGridCount(); i++)
            {
                bool succesfull = triangleSpatialHashGrid.TryGetIterator(point, i, out TriangleSpatialHashGridIterator triangleGridIterator);
                if (!succesfull) { return 0; }
                while (triangleGridIterator.HasNext())
                {
                    NativeSlice<int> triangles = triangleGridIterator.GetNextRow();
                    for (int j = 0; j < triangles.Length; j += 3)
                    {
                        int v1Index = triangles[j];
                        int v2Index = triangles[j + 1];
                        int v3Index = triangles[j + 2];
                        float3 v13d = heightMeshVerticies[v1Index];
                        float3 v23d = heightMeshVerticies[v2Index];
                        float3 v33d = heightMeshVerticies[v3Index];
                        float2 v1 = new float2(v13d.x, v13d.z);
                        float2 v2 = new float2(v23d.x, v23d.z);
                        float2 v3 = new float2(v33d.x, v33d.z);

                        BarycentricCoordinates barCords = FlowFieldUtilities.GetBarycentricCoordinatesForEachVectorInTheOrderUVW(v1, v2, v3, point);
                        if (barCords.u < 0 || barCords.w < 0 || barCords.v < 0) { continue; }
                        float newHeight = v13d.y * barCords.u + v23d.y * barCords.v + v33d.y * barCords.w;
                        curHeight = math.select(curHeight, newHeight, newHeight > curHeight);
                    }
                }
            }
            return math.select(curHeight, 0, curHeight == float.MinValue);
        }
        [BurstCompile]
        internal static void SetHeightsAsY(ref NativeArray<float3> points, in TriangleSpatialHashGrid TriangleSpatialHashGrid, in NativeArray<float3> HeightMeshVerts)
        {
            for(int pointIndexToSet = 0; pointIndexToSet < points.Length; pointIndexToSet++)
            {
                float3 point3 = points[pointIndexToSet];
                float2 point2 = new float2(point3.x, point3.z);
                float curHeight = float.MinValue;
                for (int i = 0; i < TriangleSpatialHashGrid.GetGridCount(); i++)
                {
                    bool succesfull = TriangleSpatialHashGrid.TryGetIterator(point2, i, out TriangleSpatialHashGridIterator triangleGridIterator);
                    if (!succesfull) { break; }
                    while (triangleGridIterator.HasNext())
                    {
                        NativeSlice<int> triangles = triangleGridIterator.GetNextRow();
                        for (int j = 0; j < triangles.Length; j += 3)
                        {
                            int v1Index = triangles[j];
                            int v2Index = triangles[j + 1];
                            int v3Index = triangles[j + 2];
                            float3 v13d = HeightMeshVerts[v1Index];
                            float3 v23d = HeightMeshVerts[v2Index];
                            float3 v33d = HeightMeshVerts[v3Index];
                            float2 v1 = new float2(v13d.x, v13d.z);
                            float2 v2 = new float2(v23d.x, v23d.z);
                            float2 v3 = new float2(v33d.x, v33d.z);

                            BarycentricCoordinates barCords = FlowFieldUtilities.GetBarycentricCoordinatesForEachVectorInTheOrderUVW(v1, v2, v3, point2);
                            if (barCords.u < 0 || barCords.w < 0 || barCords.v < 0) { continue; }
                            float newHeight = v13d.y * barCords.u + v23d.y * barCords.v + v33d.y * barCords.w;
                            curHeight = math.select(curHeight, newHeight, newHeight > curHeight);
                        }
                    }
                }
                point3.y = math.select(curHeight, 0, curHeight == float.MinValue);
                points[pointIndexToSet] = point3;
            }
        }
    }
}