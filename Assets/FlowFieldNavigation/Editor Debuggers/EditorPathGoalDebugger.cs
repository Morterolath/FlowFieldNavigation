
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
		GoalIndexDebugMeshBuilder _debugMeshBuilder;

		internal EditorPathGoalDebugger(FlowFieldNavigationManager navMan, GoalIndexDebugMeshBuilder goalIndexDebugMeshBuilder)
		{
			_navigationManager = navMan;
			_debugMeshBuilder = goalIndexDebugMeshBuilder;
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
			Vector3 destination3 = new Vector3(destination.x, _navigationManager.FieldImmediateQueryManager.GetHeight(destination), destination.y);
			Gizmos.color = Color.blue;
			Gizmos.DrawSphere(destination3, 0.3f);

			//Debug range
			float range = pathRanges[pathIndex];
            Gizmos.DrawWireSphere(destination3, range);

			//Debug indicies
			Gizmos.color = Color.red;
            List<Mesh> debugMeshes = _debugMeshBuilder.GetDebugMeshes(destination, range, pathIndex);
			for(int i = 0; i < debugMeshes.Count; i++)
			{
				Gizmos.DrawWireMesh(debugMeshes[i]);
			}
        }
	}
}