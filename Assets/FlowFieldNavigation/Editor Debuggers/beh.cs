using UnityEngine;
using Unity.Jobs;
using Unity.Collections;
using Unity.Burst;
using Unity.Mathematics;
using Unity.Collections.LowLevel.Unsafe;
using System.Diagnostics;
namespace FlowFieldNavigation
{
    internal class beh : MonoBehaviour
    {
        public float2 center;
        public float radius;

        private void OnDrawGizmos()
        {
            if(Input.GetMouseButton(0))
            {
                Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                if(Physics.Raycast(ray, out RaycastHit hitinfo))
                {
                    center = new float2(hitinfo.point.x, hitinfo.point.z);
                }
            }

            if (center.Equals(0)) { return; }
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(new Vector3(center.x, 0f, center.y), 3f);
            Gizmos.DrawWireSphere(new Vector3(center.x, 0f, center.y), radius);
            Color color = Color.blue;
            color.a = 0.3f;
            Gizmos.color = color;
            GridCircleBorderOverlapper overlapper = 
                new GridCircleBorderOverlapper(
                    center, 
                    radius,
                    FlowFieldUtilities.SectorColAmount,
                    FlowFieldUtilities.SectorMatrixColAmount,
                    FlowFieldUtilities.TileSize,
                    FlowFieldUtilities.FieldGridStartPosition);
            overlapper.Start();
            while(overlapper.TryGetCurrent(out int2 currentIndex))
            {
                overlapper.MoveNext();
                bool nOverflow = currentIndex.y >= FlowFieldUtilities.SectorMatrixRowAmount;
                bool eOverflow = currentIndex.x >= FlowFieldUtilities.SectorMatrixColAmount;
                bool sOverflow = currentIndex.y < 0;
                bool wOverflow = currentIndex.x < 0;

                if(nOverflow || eOverflow || sOverflow || wOverflow) { continue; }

                float2 sectorStartPos = currentIndex * new float2(FlowFieldUtilities.TileSize) * FlowFieldUtilities.SectorColAmount + FlowFieldUtilities.FieldGridStartPosition;
                float2 sectorEndPos = sectorStartPos + (FlowFieldUtilities.TileSize * FlowFieldUtilities.SectorColAmount);
                float2 center = (sectorStartPos + sectorEndPos) / 2;
                float3 center3 = new float3(center.x, 0.1f, center.y);
                Gizmos.DrawCube(center3, new Vector3(FlowFieldUtilities.SectorColAmount * FlowFieldUtilities.TileSize, 1f, FlowFieldUtilities.SectorColAmount * FlowFieldUtilities.TileSize));
            }
        }
    }
}
