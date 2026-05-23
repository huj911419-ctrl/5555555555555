#include "Function.h"

int16 Limit_int16(int16 a, int16 b, int16 c)
{
    return (int16)CLAMP(a, b, c);
}
