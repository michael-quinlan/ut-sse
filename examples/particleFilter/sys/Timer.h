#pragma once

// system-specific Timer class

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
namespace Windows {
	#include <windows.h>
};

class Timer {
private:
	// static variables
	static const __int64 MACINE_FREQUENCY;

private:
	// instance variables
	__int64 startTime;
	__int64 duration;

public:
	Timer() {}

	static __int64 getClockFrequency() {
		Windows::LARGE_INTEGER f;
		Windows::QueryPerformanceFrequency(&f);
		return f.QuadPart;
	}

	void start() {
		Windows::LARGE_INTEGER c;
		Windows::QueryPerformanceCounter(&c);
		startTime = c.QuadPart;
	}

	void stop() {
		Windows::LARGE_INTEGER c;
		Windows::QueryPerformanceCounter(&c);
		duration = c.QuadPart - startTime;
	}

	double getElapsedSeconds() {
		return (double)duration / (double)Timer::MACINE_FREQUENCY;
	}
};

#else

#include <sys/time.h>

class Timer {
private:
	// instance variables
	timeval startTime;
	timeval duration;

public:
	Timer() {}

	void start() {
		gettimeofday(&startTime, NULL);
	}

	void stop() {
		timeval endTime;
		gettimeofday(&endTime, NULL);
		duration.tv_sec = endTime.tv_sec - startTime.tv_sec;
		duration.tv_usec = endTime.tv_usec - startTime.tv_usec;
	}

	double getElapsedSeconds() {
		return duration.tv_sec + duration.tv_usec * 1.0e-6;
	}
};

#endif

// end of Timer.h
