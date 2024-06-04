using System;

namespace FlowFieldNavigation
{
    [Flags]
    internal enum PortalTraversalMark : short
    {
        AStarTraversed = 1,
        AStarExtracted = 2,
        AStarPicked = 4,
        DijkstraTraversed = 8,
        DijkstraPicked = 16,
        DijstraExtracted = 32,
        GoalNeighbour = 64,
        Explored = 128,
        DijkstraTraversable = 256,
    }

}

