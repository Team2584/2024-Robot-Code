#ifndef MATH_H
#define MATH_H

/**
 *  The Signum Function returns the sign of a double
 *  EX: 1 if num is positive, -1 if negative, 0 if zero
 */
inline int sgn(double num)
{
    if (num > 0)
        return 1;
    else if (num < 0)
        return -1;
    else
        return 0;
}

#endif // MATH_H