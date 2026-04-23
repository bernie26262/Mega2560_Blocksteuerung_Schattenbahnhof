#include "safety_error.h"

static SafetyErrorInfo g_error = { ERR_CAUSE_NONE, 0, ERR_DETAIL_NONE };

void safetyErrorSet(ErrorCause cause, uint8_t index, uint8_t detailCode)
{
    g_error.cause = cause;
    g_error.index = index;
    g_error.detailCode = detailCode;
}

void safetyErrorClear()
{
    g_error.cause = ERR_CAUSE_NONE;
    g_error.index = 0;
    g_error.detailCode = ERR_DETAIL_NONE;
}

const SafetyErrorInfo& safetyErrorGet()
{
    return g_error;
}

bool safetyErrorActive()
{
    return g_error.cause != ERR_CAUSE_NONE;
}