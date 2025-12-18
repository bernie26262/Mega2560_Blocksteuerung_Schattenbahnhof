#include "safety_error.h"

static SafetyErrorInfo g_error = { SAFETY_ERR_NONE, 0 };

void safetyErrorSet(SafetyErrorType type, uint8_t index)
{
    g_error.type  = type;
    g_error.index = index;
}

void safetyErrorClear()
{
    g_error.type  = SAFETY_ERR_NONE;
    g_error.index = 0;
}

const SafetyErrorInfo& safetyErrorGet()
{
    return g_error;
}

bool safetyErrorActive()
{
    return g_error.type != SAFETY_ERR_NONE;
}
