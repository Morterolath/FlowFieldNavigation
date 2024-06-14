
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using System.Diagnostics;
using System.Collections.Generic;
namespace FlowFieldNavigation
{
	internal class EditorPathGoalDebugger
	{
		FlowFieldNavigationManager _navigationManager;
		GoalIndexDebugMeshBuilder _debugGoalMeshBuilder;
        GoalIndexDebugMeshBuilder _debugDesiredGoalMeshBuilder;

		internal EditorPathGoalDebugger(FlowFieldNavigationManager navMan, GoalIndexDebugMeshBuilder goalIndexDebugMeshBuilder, GoalIndexDebugMeshBuilder desiredGoalIndexDebugMeshBuilder)
		{
			_navigationManager = navMan;
			_debugGoalMeshBuilder = goalIndexDebugMeshBuilder;
            _debugDesiredGoalMeshBuilder = desiredGoalIndexDebugMeshBuilder;
		}

		internal void DebugGoal(FlowFieldAgent agent)
		{
			if (agent == null) { return; }
			int pathIndex = agent.GetPathIndex();
			if (pathIndex == -1) { return; }

			PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[_navigationManager.Interface.GetPathIndex(agent)];
			NativeArray<float> pathRanges = _navigationManager.PathDataContainer.PathRanges.AsArray();

			//Debug destination point
			Vector2 destination = destinationData.Destination;
			Vector3 destination3 = new Vector3(destination.x, _navigationManager.HeightMeshImmediateQueryManager.GetHeightBurst(destination), destination.y);
			Gizmos.color = Color.blue;
			Gizmos.DrawSphere(destination3, 0.3f);

			//Debug range
			float range = pathRanges[pathIndex];
            Gizmos.DrawWireSphere(destination3, range);

			//Debug indicies
			Gizmos.color = Color.red;
            _debugGoalMeshBuilder.GetDebugMeshes(destination, range, pathIndex, out List<Mesh> debugMeshes, out List<Mesh> borderMeshes);
			for(int i = 0; i < debugMeshes.Count; i++)
			{
				Gizmos.DrawWireMesh(debugMeshes[i], new Vector3(0,0.1f,0));
			}
			Gizmos.color = Color.blue;
            for (int i = 0; i < borderMeshes.Count; i++)
            {
                Gizmos.DrawWireMesh(borderMeshes[i], new Vector3(0, 0.1f, 0));
            }
        }
        internal void DebugDesiredGoal(FlowFieldAgent agent, bool goalAlreadyBeingDebugged)
        {
            if (agent == null) { return; }
            int pathIndex = agent.GetPathIndex();
            if (pathIndex == -1) { return; }

            PathDestinationData destinationData = _navigationManager.PathDataContainer.PathDestinationDataList[_navigationManager.Interface.GetPathIndex(agent)];
            NativeArray<float> pathDesiredRanges = _navigationManager.PathDataContainer.PathDesiredRanges.AsArray();

            //Check if desired goal and goal same
            int2 goalIndex = FlowFieldUtilities.PosTo2D(destinationData.Destination, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
            int2 desiredGoalIndex = FlowFieldUtilities.PosTo2D(destinationData.DesiredDestination, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
            if (goalAlreadyBeingDebugged && desiredGoalIndex.Equals(goalIndex)) { return; }

            //Debug destination point
            Vector2 desiredGoal = destinationData.DesiredDestination;
            Vector3 desiredGoal3 = new Vector3(desiredGoal.x, _navigationManager.HeightMeshImmediateQueryManager.GetHeightBurst(desiredGoal), desiredGoal.y);
            Gizmos.color = Color.cyan;
            Gizmos.DrawSphere(desiredGoal3, 0.3f);

            //Debug range
            float desiredRange = pathDesiredRanges[pathIndex];
            Gizmos.DrawWireSphere(desiredGoal3, desiredRange);

            //Debug indicies
            Gizmos.color = Color.magenta;
            _debugDesiredGoalMeshBuilder.GetDebugMeshes(desiredGoal, desiredRange, pathIndex, out List<Mesh> debugMeshes, out List<Mesh> borderMeshes);
            for (int i = 0; i < debugMeshes.Count; i++)
            {
                Gizmos.DrawWireMesh(debugMeshes[i], new Vector3(0, 0.1f, 0));
            }
            Gizmos.color = Color.black;
            for (int i = 0; i < borderMeshes.Count; i++)
            {
                Gizmos.DrawWireMesh(borderMeshes[i], new Vector3(0, 0.1f, 0));
            }
        }
    }
}