#ifndef MATH_H // Ensures that this header file is only compiled once
#define MATH_H

/**
 *  The Signum Function returns the sign of a double
 *  Ex: 1 if num is positive, -1 if negative, 0 if zero
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