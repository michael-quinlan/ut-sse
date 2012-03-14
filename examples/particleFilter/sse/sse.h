#pragma once

// main header file for the SSE library

// SSE1
#include <xmmintrin.h>
// SSE2
#include <emmintrin.h>

#include "sse/sseUtil.h"
#include "sse/sseMask.h"
#include "sse/sse4Floats.h"
#include "sse/sse4Ints.h"


class SSE {
private:
	// re-definition of constants that happen to be in
	// the SSE3 header file, note the extra underscore
	static const unsigned int __MM_DENORMALS_ZERO_MASK = 0x00000040;
	static const unsigned int __MM_DENORMALS_ZERO_ON   = 0x00000040;
	static const unsigned int __MM_DENORMALS_ZERO_OFF  = 0x00000000;

	// re-implementation of built-in macros which happens
	// to be in the SSE3 header file, note the extra underscore
	static void __MM_SET_DENORMALS_ZERO_MODE(unsigned int daz_mode) {
		return _mm_setcsr((_mm_getcsr() & ~__MM_DENORMALS_ZERO_MASK) | daz_mode);
	}

	static unsigned int __MM_GET_DENORMALS_ZERO_MODE() {
		return _mm_getcsr() & __MM_DENORMALS_ZERO_MASK;
	}

public:
	static void init() {
		// set the control register
		// 1) request default rounding mode for floats
		// 2) denormal outputs are flushed to zero
		// 3) denormal inputs are treated as zero
		static const unsigned int RD_MODE = _MM_ROUND_NEAREST;
		static const unsigned int FZ_MODE = _MM_FLUSH_ZERO_ON;
		static const unsigned int DAZ_MODE = __MM_DENORMALS_ZERO_ON;

		_MM_SET_ROUNDING_MODE(RD_MODE);
		_MM_SET_FLUSH_ZERO_MODE(FZ_MODE);
		__MM_SET_DENORMALS_ZERO_MODE(DAZ_MODE);

		// check that the requested modes have been set
		unsigned int rd_mode  = _MM_GET_ROUNDING_MODE();
		unsigned int fz_mode  = _MM_GET_FLUSH_ZERO_MODE();
		unsigned int daz_mode = __MM_GET_DENORMALS_ZERO_MODE();

		bool rd_match  = (rd_mode  == RD_MODE);
		bool fz_match  = (fz_mode  == FZ_MODE);
		bool daz_match = (daz_mode == DAZ_MODE);

		if (!rd_match || !fz_match || !daz_match) {
			printf("could not initialize the SSE unit\n");

			if (!rd_match) {
				printf("  rd_mode: 0x%08x, could not be set to 0x%08x\n",
						rd_mode, RD_MODE);
			}
			if (!fz_match) {
				printf("  fz_mode: 0x%08x, could not be set to 0x%08x\n",
						fz_mode, FZ_MODE);
			}
			if (!daz_match) {
				printf("  daz_mode: 0x%08x, could not be set to 0x%08x\n",
						daz_mode, DAZ_MODE);
			}

			exit(0);
		}
	}
};

// end of sse.h
