#pragma once

// low-level math routines, all for scalar math

#include <math.h>

#include "sys/crossplatform.h"
#include "sys/debug.h"


#undef M_PI
static const float M_PI = 3.141592653589f;

#pragma warning(push)
#pragma warning(disable: 39)		// division by zero
	static const float INF = 1.0f / 0.0f;
	static const float NEGINF = -INF;
#pragma warning(pop)

#undef isnan
static forceinline bool isnan(float f) {
	return f != f;
}

static forceinline bool finite(float f) {
	return f < INF && f > NEGINF;
}

// a bool is either 0 (false) or 1 (true)
static forceinline int toInt(bool b) {
	return (int)b;
}

static forceinline bool isEven(int i) {
	return i % 2 == 0;
}

static forceinline bool isOdd(int i) {
	return i % 2 != 0;
}

static forceinline bool isMultipleOf(int i, int mbase) {
	return i % mbase == 0;
}

#undef min
static forceinline int min(int a, int b) {
	return (a < b) ? a : b;
}

static forceinline float min(float a, float b) {
	return (a < b) ? a : b;
}

#undef max
static forceinline int max(int a, int b) {
	return (a > b) ? a : b;
}

static forceinline float max(float a, float b) {
	return (a > b) ? a : b;
}

// clamp to [lo, hi]
static forceinline int clamp(int val, int lo, int hi) {
	assert(lo <= hi);
	return max(lo, min(val, hi));
}

// inclusive bounds [lo, hi]
static forceinline bool inbounds(float val, float lo, float hi) {
	return (val >= lo && val <= hi);
}

// returns the absolute value of the difference between a and b,
// if both inputs are INF or both inputs are NEGINF, returns 0.0f
static forceinline float absDiff(float a, float b) {
	if (a == INF && b == INF) {
		return 0.0f;
	}

	if (a == NEGINF && b == NEGINF) {
		return 0.0f;
	}

	return fabsf(a - b);
}

// returns relative error
static forceinline float relErr(float approx, float baseline) {
	if (approx == 0.0f && baseline == 0.0f) {
		return 0.0f;
	}

	return absDiff(approx, baseline) / fabsf(baseline);
}


// end of sysMath.h
