// tests SSE math functions against scalar versions from math.h

#include <stdio.h>

#include "sys/common.h"
#include "sys/Timer.h"

#include "sse/sseMath.h"


typedef sse4Floats (*ONE_ARG_FUNC)(sse4Floats x);
typedef sse4Floats (*TWO_ARG_FUNC)(sse4Floats x, sse4Floats y);

// the 4 positive normals closest to zero, in unsigned integer form
static const unsigned int initPos[] = {
	0x00800000,
	0x00800001,
	0x00800002,
	0x00800003
};

// the 4 negative normals closest to zero, in unsigned integer form
static const unsigned int initNeg[] = {
	0x80800000,
	0x80800001,
	0x80800002,
	0x80800003
};

// computes the number of floats that are needed to go from the smallest positive
// normal up to but not including the bound, the interval is [min_pos_normal, bound),
// the input must be positive
static noinline
unsigned int numNormalsUpTo(float bound) {
	assert(bound > 0.0f);

	unsigned int hi = *(unsigned int *)&bound;
	unsigned int lo = initPos[0];

	assert(lo <= hi);
	return hi - lo;
}


//--- LOCAL WRAPPERS ---//

// the following functions must all be non-static

forceinline sse4Floats abs_loc(sse4Floats x) {
	return abs(x);
}

forceinline sse4Floats abs_ref_loc(sse4Floats x) {
	return abs_ref(x);
}

forceinline sse4Floats exp_loc(sse4Floats x) {
	return exp(x);
}

forceinline sse4Floats exp_ref_loc(sse4Floats x) {
	return exp_ref(x);
}

forceinline sse4Floats sin_loc(sse4Floats x) {
	return sin(x);
}

forceinline sse4Floats sin_ref_loc(sse4Floats x) {
	return sin_ref(x);
}

forceinline sse4Floats cos_loc(sse4Floats x) {
	return cos(x);
}

forceinline sse4Floats cos_ref_loc(sse4Floats x) {
	return cos_ref(x);
}

forceinline sse4Floats atan_loc(sse4Floats x) {
	return atan(x);
}

forceinline sse4Floats atan_ref_loc(sse4Floats x) {
	return atan_ref(x);
}

forceinline sse4Floats atan2_loc(sse4Floats y, sse4Floats x) {
	return atan2(y, x);
}

forceinline sse4Floats atan2_ref_loc(sse4Floats y, sse4Floats x) {
	return atan2_ref(y, x);
}

forceinline sse4Floats old_atan2_loc(sse4Floats y, sse4Floats x) {
	return old_atan2(y, x);
}


//--- ONE ARGUMENT FUNCTIONS ---//

// time how long it takes to iterate through all floats in the operating range,
// the operating range consists of all floats that are not denormals
//
// func - the function to test, must be known at compile-time
// boundNeg - tests from 0 down to this negative value exclusive, nearby values may also not be tested
// boundPos - tests from 0 up to this positive value exclusive, nearby values may also not be tested
// testExt - if true, tests the extraordinary values +0, -0, INF, and NEGINF
template <ONE_ARG_FUNC func>
static noinline
void time_func(float boundNeg, float boundPos, bool testExt) {
	assert(boundPos >= 0.0f && boundNeg <= 0.0f);

	Timer t;
	t.start();

	sse4Floats resAccum = sse4Floats::zeros();
	for (unsigned int sign = 0; sign < 2; sign++) {	// iterate through both positive and negative

		float bound = (sign == 0) ? boundPos : boundNeg;
		if (bound == 0.0f) {		// no normalized floats to test
			continue;
		}

		// each iteration is 4-wide, so scale down the number of iterations by 4
		unsigned int n = numNormalsUpTo(fabsf(bound)) / SSE_WIDTH;

		sse4Ints intCurr = (sign == 0) ? sse4Ints(initPos[0], initPos[1], initPos[2], initPos[3])
									   : sse4Ints(initNeg[0], initNeg[1], initNeg[2], initNeg[3]);
		sse4Ints intInc = sse4Ints(0x4, 0x4, 0x4, 0x4);

		// test all normals (except 0)
		for (unsigned int i = 0; i < n; i++) {
			sse4Floats curr = sse4Floats(intCurr.data);

			// invoke the function
			resAccum += func(curr);

			intCurr += intInc;
		}
	}

	if (testExt) {
		// test both zeros and both infinities
		sse4Floats curr = sse4Floats(0.0f, -0.0f, INF, NEGINF);

		// invoke the function
		resAccum += func(curr);
	}

	t.stop();
	printf("accum results: "); resAccum.println();
	printf("time: %f sec\n", t.getElapsedSeconds());
}

// compares component-wise the elements in val (experimental) with
// the components in ref (reference), updates the running state
//
// val - IN 4-wide list of experimental values
// ref - IN 4-wide list of reference values
// x - IN the input to the function which generated val and ref, i.e. val = f(x)
// maxRelErr - IN/OUT updated if relative error between exp / ref pairs exceeds current value
// totalRelErr - IN/OUT accumulates the relative error across all 4 exp / ref pairs
// worstX - OUT if a maxRelErr is updated, this gets updated with the corresponding x value
static
void check_precision_val(sse4Floats val, sse4Floats ref, sse4Floats x,
						 float &maxRelErr, float &totalRelErr, float &worstX)
{
	for (int i = 0; i < SSE_WIDTH; i++) {
		float relError = relErr(val[i], ref[i]);

		if (isnan(relError)) {
			printf("index: %d\n", i);
			printf("x  : "); x.println();
			printf("val: "); val.println();
			printf("ref: "); ref.println();
			continue;
		}

		totalRelErr += relError;

		if (relError > maxRelErr) {
			maxRelErr = relError;
			worstX = x[i];
		}
	}
}

// measure the maximum and average error for all floats in the operating range,
// the operating range consists of all floats that are not denormals
//
// func - the function to test, must be known at compile-time
// func_ref - the reference function to compare func to, must be known at compile-time
// boundNeg - tests from 0 down to this negative value exclusive, nearby values may also not be tested
// boundPos - tests from 0 up to this positive value exclusive, nearby values may also not be tested
// testExt - if true, tests the extraordinary values +0, -0, INF, and NEGINF
template <ONE_ARG_FUNC func, ONE_ARG_FUNC func_ref>
static noinline
void check_precision_func(float boundNeg, float boundPos, bool testExt) {
	assert(boundPos >= 0.0f && boundNeg <= 0.0f);

	Timer t;
	t.start();

	float maxRelErr   = 0.0f;
	float totalRelErr = 0.0f;
	float worstX      = 0.0f;

	unsigned int numTested = 0;
	for (unsigned int sign = 0; sign < 2; sign++) {	// iterate through both positive and negative

		float bound = (sign == 0) ? boundPos : boundNeg;
		if (bound == 0.0f) {		// no normalized floats to test
			continue;
		}

		// each iteration is 4-wide, so scale down the number of iterations by 4
		unsigned int n = numNormalsUpTo(fabsf(bound)) / SSE_WIDTH;

		sse4Ints intCurr = (sign == 0) ? sse4Ints(initPos[0], initPos[1], initPos[2], initPos[3])
									   : sse4Ints(initNeg[0], initNeg[1], initNeg[2], initNeg[3]);
		sse4Ints intInc = sse4Ints(0x4, 0x4, 0x4, 0x4);

		// test all normals (except 0)
		for (unsigned int i = 0; i < n; i++) {
			sse4Floats curr = sse4Floats(intCurr.data);

			// invoke the functions
			sse4Floats val = func(curr);
			sse4Floats ref = func_ref(curr);
			check_precision_val(val, ref, curr, maxRelErr, totalRelErr, worstX);
			numTested += SSE_WIDTH;

			intCurr += intInc;
		}
	}

	if (testExt) {
		// test both zeros and both infinities
		sse4Floats curr = sse4Floats(0.0f, -0.0f, INF, NEGINF);

		// invoke the functions
		sse4Floats val = func(curr);
		sse4Floats ref = func_ref(curr);
		check_precision_val(val, ref, curr, maxRelErr, totalRelErr, worstX);
		numTested += SSE_WIDTH;
	}

	printf("maxRelErr: %f%% at x coord %f (0x%08x)\n",
			maxRelErr * 100.0f, worstX, *(int *)&worstX);
	printf("avgRelErr: %f%%\n", (totalRelErr / numTested)*100.0f);

	t.stop();
	printf("time: %f sec\n\n", t.getElapsedSeconds());
}

// performs timing and accuracy tests in the operating range,
// the operating range consists of all floats that are not denormals
//
// func - the function to test, must be known at compile-time
// func_ref - the reference function to compare func to, must be known at compile-time
// label - identifies the function
// boundNeg - tests from 0 down to this negative value exclusive, nearby values may also not be tested
// boundPos - tests from 0 up to this positive value exclusive, nearby values may also not be tested
// testExt - if true, tests the extraordinary values +0, -0, INF, and NEGINF
template <ONE_ARG_FUNC func, ONE_ARG_FUNC func_ref>
static noinline
void compareFuncs(const char *label, float boundNeg, float boundPos, bool testExt) {
	printf("=================================================\n");
	printf("testing %s on (%f, %f)\n", label, boundNeg, boundPos);
	printf("=================================================\n");

	printf("\nreference func:\n");
	time_func<func_ref>(boundNeg, boundPos, testExt);

	printf("\nfunc:\n");
	time_func<func>    (boundNeg, boundPos, testExt);

	printf("\nprecision check:\n");
	check_precision_func<func, func_ref>(boundNeg, boundPos, testExt);
}

void compareAbs() {
	const float BOUND_NEG = NEGINF;
	const float BOUND_POS = INF;

	compareFuncs<abs_loc, abs_ref_loc>("abs", BOUND_NEG, BOUND_POS, true);
}

void compareExp() {
	const float BOUND_NEG = -80.0f;
	const float BOUND_POS =  80.0f;

	compareFuncs<exp_loc, exp_ref_loc>("exp", BOUND_NEG, BOUND_POS, true);
}

void compareSin() {
	const float BOUND_NEG = -100.0f;
	const float BOUND_POS =  100.0f;

	compareFuncs<sin_loc, sin_ref_loc>("sin", BOUND_NEG, BOUND_POS, false);
}

void compareCos() {
	const float BOUND_NEG = -100.0f;
	const float BOUND_POS =  100.0f;

	compareFuncs<cos_loc, cos_ref_loc>("cos", BOUND_NEG, BOUND_POS, false);
}

void compareAtan() {
	const float BOUND_NEG = NEGINF;
	const float BOUND_POS = INF;

	compareFuncs<atan_loc, atan_ref_loc>("atan", BOUND_NEG, BOUND_POS, true);
}


//--- TWO ARGUMENT FUNCTIONS ---//

// time how long it takes to iterate through all floats on the unit circle,
// all pairs of normals on the unit circle are tested
//
// func - the function to test, must be known at compile-time
// testExt - if true, tests all combinations of the extraordinary values +0, -0, INF, and NEGINF
template <TWO_ARG_FUNC func>
static noinline
void time_func_uc(bool testExt) {
	Timer t;
	t.start();

	sse4Floats const_one = sse4Floats::expand(1.0f);
	sse4Floats resAccum = sse4Floats::zeros();
	for (unsigned int sign = 0; sign < 2; sign++) {	// iterate through both positive and negative

		float bound = (sign == 0) ? 1.0f : -1.0f;

		// each iteration is 4-wide, so scale down the number of iterations by 4
		unsigned int n = numNormalsUpTo(fabsf(bound)) / SSE_WIDTH;

		sse4Ints intCurr = (sign == 0) ? sse4Ints(initPos[0], initPos[1], initPos[2], initPos[3])
									   : sse4Ints(initNeg[0], initNeg[1], initNeg[2], initNeg[3]);
		sse4Ints intInc = sse4Ints(0x4, 0x4, 0x4, 0x4);

		// test all normals in range, except 0 and 1
		for (unsigned int i = 0; i < n; i++) {
			// generate the two points that are on the unit circle at the current x coord
			sse4Floats x_vec = sse4Floats(intCurr.data);
			sse4Floats y_vec0 = sqrt(const_one - x_vec*x_vec);		// positive y
			sse4Floats y_vec1 = -y_vec0;							// negative y

			// invoke the function on positive y
			resAccum += func(y_vec0, x_vec);

			// invoke the function on negative y
			resAccum += func(y_vec1, x_vec);

			intCurr += intInc;
		}
	}

	// test both zeros and both ones
	for (unsigned int i = 0; i < 2; i++) {
		// NOTE: +0 and -0 must remain in separate 4-wides due to a compiler bug
		sse4Floats arg_a = sse4Floats(0.0f,  0.0f,  1.0f, -1.0f);
		sse4Floats arg_b = sse4Floats(1.0f, -1.0f, -0.0f, -0.0f);

		sse4Floats x_vec = (i == 0) ? arg_a : arg_b;
		sse4Floats y_vec = (i == 0) ? arg_b : arg_a;

		// invoke the function
		resAccum += func(y_vec, x_vec);
	}

	if (testExt) {
		// test both zeros and both infinities
		for (unsigned int i = 0; i < 2; i++) {
			// NOTE: +0 and -0 must remain in separate 4-wides due to a compiler bug
			sse4Floats arg_a = sse4Floats(0.0f, 0.0f,    INF,   NEGINF);
			sse4Floats arg_b = sse4Floats(INF,  NEGINF, -0.0f, -0.0f  );

			sse4Floats x_vec = (i == 0) ? arg_a : arg_b;
			sse4Floats y_vec = (i == 0) ? arg_b : arg_a;

			// invoke the function
			resAccum += func(y_vec, x_vec);
		}
	}

	t.stop();
	printf("accum results: "); resAccum.println();
	printf("time: %f sec\n", t.getElapsedSeconds());
}

// compares component-wise the elements in val (experimental) with
// the components in ref (reference), updates the running state
//
// val - IN 4-wide list of experimental values
// ref - IN 4-wide list of reference values
// x - IN the x input to the function which generated val and ref
// y - IN the y input to the function which generated val and ref
// maxRelErr - IN/OUT updated if relative error between exp / ref pairs exceeds current value
// totalRelErr - IN/OUT accumulates the relative error across all 4 exp / ref pairs
// worstX - OUT if a maxRelErr is updated, this gets updated with the corresponding x value
// worstY - OUT if a maxRelErr is updated, this gets updated with the corresponding y value
static
void check_precision_val(sse4Floats val, sse4Floats ref, sse4Floats x, sse4Floats y,
						 float &maxRelErr, float &totalRelErr, float &worstX, float &worstY)
{
	for (int i = 0; i < SSE_WIDTH; i++) {
		float relError = relErr(val[i], ref[i]);

		if (isnan(relError)) {
			printf("index: %d\n", i);
			printf("x  : "); x.println();
			printf("y  : "); y.println();
			printf("val: "); val.println();
			printf("ref: "); ref.println();
			continue;
		}

		totalRelErr += relError;

		if (relError > maxRelErr) {
			maxRelErr = relError;
			worstX = x[i];
			worstY = y[i];
		}
	}
}

// measure the maximum and average error for all floats on the unit circle,
// all pairs of normals on the unit circle are tested
//
// func - the function to test, must be known at compile-time
// func_ref - the reference function to compare func to, must be known at compile-time
// testExt - if true, tests all combinations of the extraordinary values +0, -0, INF, and NEGINF
template <TWO_ARG_FUNC func, TWO_ARG_FUNC func_ref>
static noinline
void check_precision_func_uc(bool testExt) {
	Timer t;
	t.start();

	float maxRelErr   = 0.0f;
	float totalRelErr = 0.0f;
	float worstX      = 0.0f;
	float worstY      = 0.0f;

	sse4Floats const_one = sse4Floats::expand(1.0f);
	unsigned int numTested = 0;
	for (unsigned int sign = 0; sign < 2; sign++) {	// iterate through both positive and negative

		float bound = (sign == 0) ? 1.0f : -1.0f;

		// each iteration is 4-wide, so scale down the number of iterations by 4
		unsigned int n = numNormalsUpTo(fabsf(bound)) / SSE_WIDTH;

		sse4Ints intCurr = (sign == 0) ? sse4Ints(initPos[0], initPos[1], initPos[2], initPos[3])
									   : sse4Ints(initNeg[0], initNeg[1], initNeg[2], initNeg[3]);
		sse4Ints intInc = sse4Ints(0x4, 0x4, 0x4, 0x4);

		// test all normals in range, except 0 and 1
		for (unsigned int i = 0; i < n; i++) {
			// generate the two points that are on the unit circle at the current x coord
			sse4Floats x_vec = sse4Floats(intCurr.data);
			sse4Floats y_vec0 = sqrt(const_one - x_vec*x_vec);		// positive y
			sse4Floats y_vec1 = -y_vec0;							// negative y

			// invoke the functions on positive y
			sse4Floats val0 = func(y_vec0, x_vec);
			sse4Floats ref0 = func_ref(y_vec0, x_vec);
			check_precision_val(val0, ref0, x_vec, y_vec0, maxRelErr, totalRelErr, worstX, worstY);
			numTested += SSE_WIDTH;

			// invoke the functions on negative y
			sse4Floats val1 = func(y_vec1, x_vec);
			sse4Floats ref1 = func_ref(y_vec1, x_vec);
			check_precision_val(val1, ref1, x_vec, y_vec1, maxRelErr, totalRelErr, worstX, worstY);
			numTested += SSE_WIDTH;

			intCurr += intInc;
		}
	}

	// test both zeros and both ones
	for (unsigned int i = 0; i < 2; i++) {
		// NOTE: +0 and -0 must remain in separate 4-wides due to a compiler bug
		sse4Floats arg_a = sse4Floats(0.0f,  0.0f,  1.0f, -1.0f);
		sse4Floats arg_b = sse4Floats(1.0f, -1.0f, -0.0f, -0.0f);

		sse4Floats x_vec = (i == 0) ? arg_a : arg_b;
		sse4Floats y_vec = (i == 0) ? arg_b : arg_a;

		// invoke the functions
		sse4Floats val = func(y_vec, x_vec);
		sse4Floats ref = func_ref(y_vec, x_vec);
		check_precision_val(val, ref, x_vec, y_vec, maxRelErr, totalRelErr, worstX, worstY);
		numTested += SSE_WIDTH;
	}

	if (testExt) {
		// test both zeros and both infinities
		for (unsigned int i = 0; i < 2; i++) {
			// NOTE: +0 and -0 must remain in separate 4-wides due to a compiler bug
			sse4Floats arg_a = sse4Floats(0.0f, 0.0f,    INF,   NEGINF);
			sse4Floats arg_b = sse4Floats(INF,  NEGINF, -0.0f, -0.0f  );

			sse4Floats x_vec = (i == 0) ? arg_a : arg_b;
			sse4Floats y_vec = (i == 0) ? arg_b : arg_a;

			// invoke the functions
			sse4Floats val = func(y_vec, x_vec);
			sse4Floats ref = func_ref(y_vec, x_vec);
			check_precision_val(val, ref, x_vec, y_vec, maxRelErr, totalRelErr, worstX, worstY);
			numTested += SSE_WIDTH;
		}
	}

	printf("maxRelErr: %f%% at x, y coord %f, %f (0x%08x, 0x%08x)\n",
			maxRelErr * 100.0f, worstX, worstY, *(int *)&worstX, *(int *)&worstY);
	printf("avgRelErr: %f%%\n", (totalRelErr / numTested)*100.0f);

	t.stop();
	printf("time: %f sec\n\n", t.getElapsedSeconds());
}

// performs timing and accuracy tests on the unit circle,
// all pairs of normals on the unit circle are tested
//
// func - the function to test, must be known at compile-time
// func_ref - the reference function to compare func to, must be known at compile-time
// label - identifies the function
// testExt - if true, tests all combinations of the extraordinary values +0, -0, INF, and NEGINF
template <TWO_ARG_FUNC func, TWO_ARG_FUNC func_ref>
static noinline
void compareFuncsUnitCircle(const char *label, bool testExt) {
	printf("=================================================\n");
	printf("testing %s on unit circle\n", label);
	printf("=================================================\n");

	printf("\nreference func:\n");
	time_func_uc<func_ref>(testExt);

	printf("\nfunc:\n");
	time_func_uc<func>    (testExt);

	printf("\nprecision check:\n");
	check_precision_func_uc<func, func_ref>(testExt);
}

void compareAtan2() {
	compareFuncsUnitCircle<atan2_loc, atan2_ref_loc>("atan2", true);
}

void compareOldAtan2() {
	// old atan2 doesn't handle extraordinary values
	compareFuncsUnitCircle<old_atan2_loc, atan2_ref_loc>("old atan2", false);
}


// end of Comparison.cpp
