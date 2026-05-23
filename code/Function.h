#ifndef CODE_FUNCTION_H_
#define CODE_FUNCTION_H_

#include "zf_common_headfile.h"

#define CLAMP(a, b, c) ((b) < (a) ? (a) : ((b) > (c) ? (c) : (b)))

int16 Limit_int16(int16 a, int16 b, int16 c);

#endif /* CODE_FUNCTION_H_ */
