﻿namespace FlowFieldNavigation
{
    internal struct AgentDataReferance
    {
        uint _index;
        internal AgentDataReferance(int agentDataIndexIndex)
        {
            _index = (uint)(agentDataIndexIndex + 1);
        }
        internal bool TryGetIndex(out int index)
        {
            bool succesfull = _index != 0;
            index = (int)_index - 1;
            return succesfull;
        }
        internal int GetIndexNonchecked()
        {
            return (int)_index - 1;
        }
        internal bool IsInstantiated()
        {
            return _index != 0;
        }
    }
}
