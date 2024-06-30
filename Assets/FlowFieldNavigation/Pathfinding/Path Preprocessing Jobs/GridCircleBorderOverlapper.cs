using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Mathematics;

namespace FlowFieldNavigation
{
    internal struct GridCircleBorderOverlapper
    {
        const int BIGGER_THAN_MIN_Y = 0b_1000;
        const int BUGGER_THAN_MIN_X = 0b_0100;
        const int SMALLER_THAN_MAX_Y = 0b_0010;
        const int SMALLER_THAN_MAX_X = 0b_0001;
        const int AREA_TOPLEFT = BIGGER_THAN_MIN_Y | SMALLER_THAN_MAX_X;
        const int AREA_TOPMID = BIGGER_THAN_MIN_Y | BUGGER_THAN_MIN_X | SMALLER_THAN_MAX_X;
        const int AREA_TOPRIGHT = BIGGER_THAN_MIN_Y | BUGGER_THAN_MIN_X;
        const int AREA_MIDLEFT = BIGGER_THAN_MIN_Y | SMALLER_THAN_MAX_Y | SMALLER_THAN_MAX_X;
        const int AREA_MIDRIGHT = BIGGER_THAN_MIN_Y | BUGGER_THAN_MIN_X | SMALLER_THAN_MAX_Y;
        const int AREA_BOTLEFT = SMALLER_THAN_MAX_Y | SMALLER_THAN_MAX_X;
        const int AREA_BOTMID = BUGGER_THAN_MIN_X | SMALLER_THAN_MAX_Y | SMALLER_THAN_MAX_X;
        const int AREA_BOTRIGHT = BUGGER_THAN_MIN_X | SMALLER_THAN_MAX_Y;
        const byte STAG_TOPRIGHT = 1;
        const byte STAGE_BOTRIGHT = 2;
        const byte STAGE_BOTLEFT = 3;
        const byte STAGE_TOPLEFT = 4;
        internal float2 GoalPosition;
        internal float GoalRange;

        internal NativeArray<int> BorderSectors()
        {
            NativeList<int> borderSectors = new NativeList<int>(Allocator.Temp);
            float2 topPosition = GoalPosition + new float2(0, GoalRange);
            int2 topIndex = FlowFieldUtilities.PosTo2D(topPosition, FlowFieldUtilities.TileSize, FlowFieldUtilities.FieldGridStartPosition);
            int2 topSector2d = FlowFieldUtilities.GetSector2D(topIndex, FlowFieldUtilities.SectorColAmount);

            int2 curSector2d = topSector2d;
            byte stage = STAG_TOPRIGHT;
            do
            {
                bool nOverflow = curSector2d.y >= FlowFieldUtilities.SectorMatrixRowAmount;
                bool eOverflow = curSector2d.x >= FlowFieldUtilities.SectorMatrixColAmount;
                bool sOverflow = curSector2d.y < 0;
                bool wOverflow = curSector2d.x < 0;
                if (!(nOverflow || eOverflow || sOverflow || wOverflow))
                {
                    borderSectors.Add(FlowFieldUtilities.To1D(curSector2d, FlowFieldUtilities.SectorMatrixColAmount));
                }

                int2 pickedSector = curSector2d;
                bool succesfull = false;
                while (!succesfull)
                {
                    succesfull = RunWithStage(curSector2d, stage, out pickedSector);
                    if (!succesfull)
                    {
                        stage++;
                    }
                }
                curSector2d = pickedSector;

            }
            while (!curSector2d.Equals(topSector2d));
            return borderSectors.AsArray();
        }
        bool RunWithStage(int2 curSector, byte stage, out int2 pickedSector)
        {
            pickedSector = -1;
            int2 n1 = curSector;
            int2 n2 = curSector;
            int2 n3 = curSector;

            switch (stage)
            {
                case STAG_TOPRIGHT:
                    n1 += new int2(1, 0);
                    n2 += new int2(0, -1);
                    n3 += new int2(1, -1);
                    break;
                case STAGE_BOTRIGHT:
                    n1 += new int2(0, -1);
                    n2 += new int2(-1, 0);
                    n3 += new int2(-1, -1);
                    break;
                case STAGE_BOTLEFT:
                    n1 += new int2(-1, 0);
                    n2 += new int2(0, 1);
                    n3 += new int2(-1, 1);
                    break;
                case STAGE_TOPLEFT:
                    n1 += new int2(0, 1);
                    n2 += new int2(1, 0);
                    n3 += new int2(1, 1);
                    break;
            }
            if (IsBorderSector(n1))
            {
                pickedSector = n1;
                return true;
            }
            if (IsBorderSector(n2))
            {
                pickedSector = n2;
                return true;
            }
            if (IsBorderSector(n3))
            {
                pickedSector = n3;
                return true;
            }
            return false;
        }
        bool IsBorderSector(int2 sector2d)
        {
            float sectorSize = FlowFieldUtilities.SectorColAmount * FlowFieldUtilities.TileSize;
            float2 sectorMin = sector2d * math.float2(FlowFieldUtilities.TileSize) * FlowFieldUtilities.SectorColAmount + FlowFieldUtilities.FieldGridStartPosition;
            float2 sectorMax = sectorMin + sectorSize;
            bool2 goalSmallerThanSectorMaxPos = GoalPosition <= sectorMax;
            bool2 goalBiggerThanSectorMinPos = GoalPosition >= sectorMin;
            int resultBits = math.select(0, SMALLER_THAN_MAX_Y, goalSmallerThanSectorMaxPos.y);
            resultBits |= math.select(0, BIGGER_THAN_MIN_Y, goalBiggerThanSectorMinPos.y);
            resultBits |= math.select(0, SMALLER_THAN_MAX_X, goalSmallerThanSectorMaxPos.x);
            resultBits |= math.select(0, BUGGER_THAN_MIN_X, goalBiggerThanSectorMinPos.x);

            float2 sectorBotLeft = sectorMin;
            float2 sectorTopLeft = sectorMin;
            sectorTopLeft.y += sectorSize;
            float2 sectorTopRight = sectorMax;
            float2 sectorBotRight = sectorMax;
            sectorBotRight.y -= sectorSize;
            switch (resultBits)
            {
                case AREA_TOPLEFT:
                    return math.distance(sectorTopLeft, GoalPosition) <= GoalRange && math.distance(sectorBotRight, GoalPosition) > GoalRange;
                case AREA_TOPMID:
                    return (GoalPosition.y - sectorMax.y) <= GoalRange && math.max(math.distance(sectorBotRight, GoalPosition), math.distance(sectorBotLeft, GoalPosition)) > GoalRange;
                case AREA_TOPRIGHT:
                    return math.distance(sectorTopRight, GoalPosition) <= GoalRange && math.distance(sectorBotLeft, GoalPosition) > GoalRange;
                case AREA_MIDLEFT:
                    return (sectorMin.x - GoalPosition.x) <= GoalRange && math.max(math.distance(sectorTopRight, GoalPosition), math.distance(sectorBotRight, GoalPosition)) > GoalRange;
                case AREA_MIDRIGHT:
                    return (GoalPosition.x - sectorMax.x) <= GoalRange && math.max(math.distance(sectorTopLeft, GoalPosition), math.distance(sectorBotLeft, GoalPosition)) > GoalRange;
                case AREA_BOTLEFT:
                    return math.distance(sectorBotLeft, GoalPosition) <= GoalRange && math.distance(sectorTopRight, GoalPosition) > GoalRange;
                case AREA_BOTMID:
                    return (sectorMin.y - GoalPosition.y) <= GoalRange && math.max(math.distance(sectorTopLeft, GoalPosition), math.distance(sectorTopRight, GoalPosition)) > GoalRange;
                case AREA_BOTRIGHT:
                    return math.distance(sectorBotRight, GoalPosition) <= GoalRange && math.distance(sectorTopLeft, GoalPosition) > GoalRange;
                default:
                    return true;
            }
        }
    }
}
