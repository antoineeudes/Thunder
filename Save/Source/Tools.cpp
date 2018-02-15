#include "Tools.h"

int bpmToUS(float bpm)
{
    return int(1000000*60/bpm);
}
