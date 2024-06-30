using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Mathematics;
using UnityEditor.SceneManagement;

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
        const byte STAGE_FINISHED = 0;
        const byte STAGE_TOPRIGHT = 1;
        const byte STAGE_BOTRIGHT = 2;
        const byte STAGE_BOTLEFT = 3;
        const byte STAGE_TOPLEFT = 4;

        int _sectorColAmount;
        float _tileSize;
        float2 _fieldGridStartPosition;

        float2 _circleCenter;
        float _circleRadius;
        int2 _startIndex;
        int2 _previousIndex;
        int2 _currentIndex;
        byte _lookupStage;
        internal GridCircleBorderOverlapper(float2 circleCenter, float circleRadius, int sectorColAmount, float tileSize, float2 fieldGridStartPos)
        {
            _sectorColAmount = sectorColAmount;
            _tileSize = tileSize;
            _fieldGridStartPosition = fieldGridStartPos;
            _circleCenter = circleCenter;
            _circleRadius = circleRadius;
            _currentIndex = 0;
            _previousIndex = 0;
            _startIndex = 0;
            _lookupStage = STAGE_FINISHED;
        }
        internal void Start()
        {
            float2 topPosition = _circleCenter + new float2(0, _circleRadius);
            int2 topIndex = FlowFieldUtilities.PosTo2D(topPosition, _tileSize * _sectorColAmount, _fieldGridStartPosition);
            _currentIndex = topIndex;
            _startIndex = topIndex;
            _lookupStage = STAGE_TOPRIGHT;
        }
        internal bool TryGetCurrent(out int2 currentIndex)
        {
            currentIndex = _currentIndex;
            return _lookupStage != STAGE_FINISHED;
        }
        internal void MoveNext()
        {
            //Is finished
            if (_lookupStage == STAGE_FINISHED)
            {
                return;
            }

            int2 newIndex = _currentIndex;
            bool succesfull = false;
            while (!succesfull)
            {
                succesfull = RunWithStage(_currentIndex, _lookupStage, out newIndex);
                _lookupStage = (byte) math.select((_lookupStage + 1), _lookupStage, succesfull);
            }
            _previousIndex = _currentIndex;
            _currentIndex = newIndex;
            _lookupStage = (byte) math.select((int) _lookupStage, STAGE_FINISHED, _currentIndex.Equals(_startIndex));
        }
        bool RunWithStage(int2 curSector, byte stage, out int2 pickedSector)
        {
            pickedSector = -1;
            int2 n1 = curSector;
            int2 n2 = curSector;
            int2 n3 = curSector;

            switch (stage)
            {
                case STAGE_TOPRIGHT:
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
                default:
                    pickedSector = _startIndex;
                    return true;
            }
            if (IsBorderSector(n1) && !n1.Equals(_previousIndex))
            {
                pickedSector = n1;
                return true;
            }
            if (IsBorderSector(n2) && !n2.Equals(_previousIndex))
            {
                pickedSector = n2;
                return true;
            }
            if (IsBorderSector(n3) && !n3.Equals(_previousIndex))
            {
                pickedSector = n3;
                return true;
            }
            return false;
        }
        bool IsBorderSector(int2 sector2d)
        {
            float sectorSize = _sectorColAmount * _tileSize;
            float2 sectorMin = sector2d * math.float2(_tileSize) * _sectorColAmount + _fieldGridStartPosition;
            float2 sectorMax = sectorMin + sectorSize;
            bool2 goalSmallerThanSectorMaxPos = _circleCenter <= sectorMax;
            bool2 goalBiggerThanSectorMinPos = _circleCenter >= sectorMin;
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
                    return math.distance(sectorTopLeft, _circleCenter) <= _circleRadius && math.distance(sectorBotRight, _circleCenter) > _circleRadius;
                case AREA_TOPMID:
                    return (_circleCenter.y - sectorMax.y) <= _circleRadius && math.max(math.distance(sectorBotRight, _circleCenter), math.distance(sectorBotLeft, _circleCenter)) > _circleRadius;
                case AREA_TOPRIGHT:
                    return math.distance(sectorTopRight, _circleCenter) <= _circleRadius && math.distance(sectorBotLeft, _circleCenter) > _circleRadius;
                case AREA_MIDLEFT:
                    return (sectorMin.x - _circleCenter.x) <= _circleRadius && math.max(math.distance(sectorTopRight, _circleCenter), math.distance(sectorBotRight, _circleCenter)) > _circleRadius;
                case AREA_MIDRIGHT:
                    return (_circleCenter.x - sectorMax.x) <= _circleRadius && math.max(math.distance(sectorTopLeft, _circleCenter), math.distance(sectorBotLeft, _circleCenter)) > _circleRadius;
                case AREA_BOTLEFT:
                    return math.distance(sectorBotLeft, _circleCenter) <= _circleRadius && math.distance(sectorTopRight, _circleCenter) > _circleRadius;
                case AREA_BOTMID:
                    return (sectorMin.y - _circleCenter.y) <= _circleRadius && math.max(math.distance(sectorTopLeft, _circleCenter), math.distance(sectorTopRight, _circleCenter)) > _circleRadius;
                case AREA_BOTRIGHT:
                    return math.distance(sectorBotRight, _circleCenter) <= _circleRadius && math.distance(sectorTopLeft, _circleCenter) > _circleRadius;
                default:
                    return true;
            }
        }
    }
}
