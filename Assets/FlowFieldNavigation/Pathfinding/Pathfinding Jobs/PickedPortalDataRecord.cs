using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace FlowFieldNavigation
{
    internal struct PickedPortalDataRecord
    {
        internal int PortalIndex;
        internal int OriginIndex;
        internal int PortalCountToGoal;
        internal float GCost;
    }
}
