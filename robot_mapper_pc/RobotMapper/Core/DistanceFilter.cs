using System;

namespace RobotMapper.Core;

internal sealed class DistanceFilter
{
    private readonly int _minMm;
    private readonly int _maxMm;
    private readonly int _maxJumpMm;
    private readonly int _invalidStreakForZero;

    private int _lastGoodMm;
    private int _invalidStreak;

    private int _v0;
    private int _v1;
    private int _v2;
    private int _validCount;

    public DistanceFilter(int minMm = 25, int maxMm = 6000, int maxJumpMm = 1200, int invalidStreakForZero = 3)
    {
        _minMm = minMm;
        _maxMm = maxMm;
        _maxJumpMm = maxJumpMm;
        _invalidStreakForZero = invalidStreakForZero;
    }

    public int Filtrer(int rawMm)
    {
        if (!EstValide(rawMm))
            return GérerInvalide();

        if (_lastGoodMm > 0 && Math.Abs(rawMm - _lastGoodMm) > _maxJumpMm)
            return GérerInvalide();

        _invalidStreak = 0;
        _lastGoodMm = rawMm;

        // Petit lissage robuste: médiane des 3 dernières valeurs valides.
        if (_validCount == 0)
        {
            _v0 = rawMm;
            _validCount = 1;
            return rawMm;
        }

        if (_validCount == 1)
        {
            _v1 = rawMm;
            _validCount = 2;
            return rawMm;
        }

        if (_validCount == 2)
        {
            _v2 = rawMm;
            _validCount = 3;
            return Median3(_v0, _v1, _v2);
        }

        _v0 = _v1;
        _v1 = _v2;
        _v2 = rawMm;
        return Median3(_v0, _v1, _v2);
    }

    private bool EstValide(int mm) => mm >= _minMm && mm <= _maxMm;

    private int GérerInvalide()
    {
        _invalidStreak++;

        // Ignore les glitches isolés (0 / valeurs hors bornes) : on conserve la dernière valeur ok.
        if (_lastGoodMm > 0 && _invalidStreak < _invalidStreakForZero)
            return _lastGoodMm;

        return 0;
    }

    private static int Median3(int a, int b, int c)
    {
        if (a > b) (a, b) = (b, a);
        if (b > c) (b, c) = (c, b);
        if (a > b) (a, b) = (b, a);
        return b;
    }
}
