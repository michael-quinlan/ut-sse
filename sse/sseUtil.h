#pragma once

// low-level SSE functionality

#include <xmmintrin.h>
#include <emmintrin.h>

#include "sys/common.h"


// four 32-bit elements per SSE primitive
static const int SSE_WIDTH = 4;


#pragma warning(push)
#pragma warning(disable: 1684)	// conversion from pointer to same-sized integral type (potential portability problem)

static forceinline bool is_align16(void *p) {
	size_t i = (size_t)p;
	return i % 16 == 0;
}

#pragma warning(pop)

// reinterpret the bits of val as 4 floats, all bits are unchanged
static forceinline __m128 reint(__m128i val) {
	return _mm_castsi128_ps(val);
}

// reinterpret the bits of val as 4 ints, all bits are unchanged
static forceinline __m128i reint(__m128 val) {
	return _mm_castps_si128(val);
}

namespace sseImpl {
	// perform the shuffle on data, the element at i0 in data will
	// appear as element0 in the return value, the element at i1 in
	// data will appear as element1 in the return value, etc.,
	// duplicate indices in [i0, i3] are allowed
	template <int i0, int i1, int i2, int i3>
	forceinline __m128 shuffle(__m128 data) {
		assert(i0 >= 0 && i0 < SSE_WIDTH);
		assert(i1 >= 0 && i1 < SSE_WIDTH);
		assert(i2 >= 0 && i2 < SSE_WIDTH);
		assert(i3 >= 0 && i3 < SSE_WIDTH);
		return _mm_shuffle_ps(data, data, _MM_SHUFFLE(i3, i2, i1, i0));
	}

	// wherever the mask is set, selects the entry in arg_true,
	// wherever the mask is not set, selects the entry in arg_false
	static forceinline
	__m128  blend4(__m128 mask, __m128  arg_true, __m128  arg_false) {
		return _mm_or_ps(_mm_and_ps(mask, arg_true),
						 _mm_andnot_ps(mask, arg_false));
	}

	// wherever the mask is set, selects the entry in arg_true,
	// wherever the mask is not set, selects the entry in arg_false
	static forceinline
	__m128i blend4(__m128 mask, __m128i arg_true, __m128i arg_false) {
		__m128i imask = reint(mask);
		return _mm_or_si128(_mm_and_si128(imask, arg_true),
							_mm_andnot_si128(imask, arg_false));
	}
}

// end of sseUtil.h
