// system-specific Timer class

#include "Timer.h"

#ifdef _WIN32
const __int64 Timer::MACINE_FREQUENCY = Timer::getClockFrequency();
#endif

// end of Timer.cpp
