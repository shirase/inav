#include "debug.h"

static char debugData[10000];

void addDebugData(char* str)
{
    snprintf(debugData, sizeof(debugData), "%s%s", debugData, str);
}

const char* getDebugData(void)
{
    return debugData;
}