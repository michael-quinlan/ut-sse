#pragma once

// wrapper for four 32-bit floats

#include "sys/common.h"

#include "sse/sseUtil.h"
#include "sse/sseMask.h"


class sse4Floats {
public:
	__m128 data;		// public to allow outside tinkering, as necessary

	forceinline sse4Floats() {}

	forceinline sse4Floats(__m128 input)
		: data(input) {}

	forceinline sse4Floats(__m128i input)
		: data(reint(input)) {}

	forceinline sse4Floats(float f0, float f1, float f2, float f3)
		: data(_mm_set_ps(f3, f2, f1, f0)) {}	// order is reversed

	forceinline sse4Floats(float *fp) {
		assert(is_align16(fp));
		data = _mm_load_ps(fp);
	}

	forceinline float operator [](int index) const {
		assert(index >= 0 && index < SSE_WIDTH);
		return ((float *)&data)[index];
	}

	//--- STATIC GENERATORS ---//
	static forceinline sse4Floats zeros() {
		return sse4Floats(_mm_setzero_ps());
	}

	static forceinline sse4Floats expand(float f) {
		return sse4Floats(_mm_set1_ps(f));
	}

	//--- ARITHMETIC ---//
	forceinline sse4Floats operator +(const sse4Floats &rhs) const {
		return _mm_add_ps(data, rhs.data);
	}

	forceinline sse4Floats operator -(const sse4Floats &rhs) const {
		return _mm_sub_ps(data, rhs.data);
	}

	forceinline sse4Floats operator *(const sse4Floats &rhs) const {
		return _mm_mul_ps(data, rhs.data);
	}

	forceinline sse4Floats operator /(const sse4Floats &rhs) const {
		return _mm_div_ps(data, rhs.data);
	}

	forceinline sse4Floats operator -() const {
		return _mm_sub_ps(sse4Floats::zeros().data, data);
	}

	//--- BITWISE ---//
	forceinline sse4Floats operator &(const sse4Floats &rhs) const {
		return _mm_and_ps(data, rhs.data);
	}

	forceinline sse4Floats operator |(const sse4Floats &rhs) const {
		return _mm_or_ps(data, rhs.data);
	}

	forceinline sse4Floats operator ^(const sse4Floats &rhs) const {
		return _mm_xor_ps(data, rhs.data);
	}

	forceinline sse4Floats operator ~() const {
		return operator ^(sseMask::on().data);
	}

	//--- ASSIGNMENT ---//
	forceinline sse4Floats &operator +=(const sse4Floats &rhs) {
		operator =(operator +(rhs)); return *this;
	}

	forceinline sse4Floats &operator -=(const sse4Floats &rhs) {
		operator =(operator -(rhs)); return *this;
	}

	forceinline sse4Floats &operator *=(const sse4Floats &rhs) {
		operator =(operator *(rhs)); return *this;
	}

	forceinline sse4Floats &operator /=(const sse4Floats &rhs) {
		operator =(operator /(rhs)); return *this;
	}

	forceinline sse4Floats &operator &=(const sse4Floats &rhs) {
		operator =(operator &(rhs)); return *this;
	}

	forceinline sse4Floats &operator |=(const sse4Floats &rhs) {
		operator =(operator |(rhs)); return *this;
	}

	forceinline sse4Floats &operator ^=(const sse4Floats &rhs) {
		operator =(operator ^(rhs)); return *this;
	}

	//--- COMPARISON ---//
	forceinline sseMask operator ==(const sse4Floats &rhs) const {
		return _mm_cmpeq_ps(data, rhs.data);
	}

	forceinline sseMask operator !=(const sse4Floats &rhs) const {
		return _mm_cmpneq_ps(data, rhs.data);
	}

	forceinline sseMask operator <(const sse4Floats &rhs) const {
		return _mm_cmplt_ps(data, rhs.data);
	}

	forceinline sseMask operator <=(const sse4Floats &rhs) const {
		return _mm_cmple_ps(data, rhs.data);
	}

	forceinline sseMask operator >(const sse4Floats &rhs) const {
		return _mm_cmpgt_ps(data, rhs.data);
	}

	forceinline sseMask operator >=(const sse4Floats &rhs) const {
		return _mm_cmpge_ps(data, rhs.data);
	}

	//--- SHUFFLE ---//
	template <int i0, int i1, int i2, int i3>
	forceinline sse4Floats shuffle() const {
		return sseImpl::shuffle<i0, i1, i2, i3>(data);
	}

	//--- REDUCTION ---//

	// adds the 4 components into a single float
	forceinline float reduce_add() const {
		sse4Floats temp1 = operator +(shuffle<1, 0, 3, 2>());
		sse4Floats temp2 = temp1 + temp1.shuffle<2, 3, 0, 1>();
		return temp2[0];
	}

	// multiplies the 4 components into a single float
	forceinline float reduce_mult() const {
		sse4Floats temp1 = operator *(shuffle<1, 0, 3, 2>());
		sse4Floats temp2 = temp1 * temp1.shuffle<2, 3, 0, 1>();
		return temp2[0];
	}

	//--- PRINT ---//
	void print() const {
		printf("(% f, % f, % f, % f)", operator [](0), operator [](1),
									   operator [](2), operator [](3));
	}

	void println() const {
		print(); printf("\n");
	}

	void hex_print() const {
		int *ip = (int *)&data;
		printf("(0x%08x, 0x%08x, 0x%08x, 0x%08x)",
				ip[0], ip[1], ip[2], ip[3]);
	}

	void hex_println() const {
		hex_print(); printf("\n");
	}
};

//--- STORE ---//
static forceinline
void store4(float *dst, const sse4Floats &src) {
	assert(is_align16(dst));
	_mm_store_ps(dst, src.data);
}

//--- BLEND ---//
static forceinline
sse4Floats blend4(const sseMask &mask,
				  const sse4Floats &arg_true,
				  const sse4Floats &arg_false)
{
	return sseImpl::blend4(mask.data, arg_true.data, arg_false.data);
}

//--- MIN and MAX ---//
static forceinline
sse4Floats min4(const sse4Floats &a, const sse4Floats &b) {
	return _mm_min_ps(a.data, b.data);
}

static forceinline
sse4Floats max4(const sse4Floats &a, const sse4Floats &b) {
	return _mm_max_ps(a.data, b.data);
}

//--- COMPARISON ---//
static forceinline
sseMask nanMask(const sse4Floats &input) {
	return input != input;
}

// inclusive range test on [lo, hi]
static forceinline
sseMask inRangeMask(const sse4Floats &input,
					const sse4Floats &lo,
					const sse4Floats &hi)
{
	return (input >= lo) & (input <= hi);
}

// exclusive range test on (lo, hi)
static forceinline
sseMask exRangeMask(const sse4Floats &input,
					const sse4Floats &lo,
					const sse4Floats &hi)
{
	return (input > lo) & (input < hi);
}

// end of sse4Floats.h
