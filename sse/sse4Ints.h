#pragma once

// wrapper for four 32-bit ints

#include "sys/common.h"

#include "sse/sseUtil.h"
#include "sse/sseMask.h"


class sse4Ints {
public:
	__m128i data;		// public to allow outside tinkering, as necessary

	forceinline sse4Ints() {}

	forceinline sse4Ints(__m128i input)
		: data(input) {}

	forceinline sse4Ints(__m128 input)
		: data(reint(input)) {}

	forceinline sse4Ints(int i0, int i1, int i2, int i3)
		: data(_mm_set_epi32(i3, i2, i1, i0)) {}	// order is reversed

	forceinline sse4Ints(int *ip) {
		assert(is_align16(ip));
		data = _mm_load_si128((__m128i *)ip);
	}

	forceinline int operator [](int index) const {
		assert(index >= 0 && index < SSE_WIDTH);
		return ((int *)&data)[index];
	}

	//--- STATIC GENERATORS ---//
	static forceinline sse4Ints zeros() {
		return sse4Ints(_mm_setzero_si128());
	}

	static forceinline sse4Ints expand(int i) {
		return sse4Ints(_mm_set1_epi32(i));
	}

	//--- CONVERT ---//
	static forceinline sse4Ints cast(const sseMask &rhs) {
		return rhs.data;
	}

	//--- ARITHMETIC ---//
	forceinline sse4Ints operator +(const sse4Ints &rhs) const {
		return _mm_add_epi32(data, rhs.data);
	}

	forceinline sse4Ints operator -(const sse4Ints &rhs) const {
		return _mm_sub_epi32(data, rhs.data);
	}

	forceinline sse4Ints operator -() const {
		return _mm_sub_epi32(sse4Ints::zeros().data, data);
	}

	//--- BITWISE ---//
	forceinline sse4Ints operator &(const sse4Ints &rhs) const {
		return _mm_and_si128(data, rhs.data);
	}

	forceinline sse4Ints operator |(const sse4Ints &rhs) const {
		return _mm_or_si128(data, rhs.data);
	}

	forceinline sse4Ints operator ^(const sse4Ints &rhs) const {
		return _mm_xor_si128(data, rhs.data);
	}

	forceinline sse4Ints operator ~() const {
		return operator ^(sseMask::on().data);
	}

	//--- SHIFTING ---//
	forceinline sse4Ints operator <<(int i) const {
		return _mm_slli_epi32(data, i);
	}

	forceinline sse4Ints operator >>(int i) const {
		return _mm_srli_epi32(data, i);
	}

	//--- ASSIGNMENT ---//
	forceinline sse4Ints &operator +=(const sse4Ints &rhs) {
		operator =(operator +(rhs)); return *this;
	}

	forceinline sse4Ints &operator -=(const sse4Ints &rhs) {
		operator =(operator -(rhs)); return *this;
	}

	forceinline sse4Ints &operator &=(const sse4Ints &rhs) {
		operator =(operator &(rhs)); return *this;
	}

	forceinline sse4Ints &operator |=(const sse4Ints &rhs) {
		operator =(operator |(rhs)); return *this;
	}

	forceinline sse4Ints &operator ^=(const sse4Ints &rhs) {
		operator =(operator ^(rhs)); return *this;
	}

	forceinline sse4Ints &operator <<=(int i) {
		operator =(operator <<(i)); return *this;
	}

	forceinline sse4Ints &operator >>=(int i) {
		operator =(operator >>(i)); return *this;
	}

	//--- COMPARISON ---//
	forceinline sseMask operator ==(const sse4Ints &rhs) const {
		return _mm_cmpeq_epi32(data, rhs.data);
	}

	forceinline sseMask operator !=(const sse4Ints &rhs) const {
		return ~(operator ==(rhs));
	}

	forceinline sseMask operator <(const sse4Ints &rhs) const {
		return _mm_cmplt_epi32(data, rhs.data);
	}

	forceinline sseMask operator <=(const sse4Ints &rhs) const {
		return ~(operator >(rhs));
	}

	forceinline sseMask operator >(const sse4Ints &rhs) const {
		return _mm_cmpgt_epi32(data, rhs.data);
	}

	forceinline sseMask operator >=(const sse4Ints &rhs) const {
		return ~(operator <(rhs));
	}

	//--- SHUFFLE ---//
	template <int i0, int i1, int i2, int i3>
	forceinline sse4Ints shuffle() const {
		return sseImpl::shuffle<i0, i1, i2, i3>(reint(data));
	}

	//--- REDUCTION ---//

	// adds the 4 components into a single int
	forceinline int reduce_add() const {
		sse4Ints temp1 = operator +(shuffle<1, 0, 3, 2>());
		sse4Ints temp2 = temp1 + temp1.shuffle<2, 3, 0, 1>();
		return temp2[0];
	}

	//--- PRINT ---//
	void print() const {
		printf("(%d, %d, %d, %d)", operator [](0), operator [](1),
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
void store4(int *dst, const sse4Ints &src) {
	assert(is_align16(dst));
	_mm_store_si128((__m128i *)dst, src.data);
}

//--- BLEND ---//
static forceinline
sse4Ints blend4(const sseMask &mask,
				const sse4Ints &arg_true,
				const sse4Ints &arg_false)
{
	return sseImpl::blend4(mask.data, arg_true.data, arg_false.data);
}

//--- MIN and MAX ---//
static forceinline
sse4Ints min4(const sse4Ints &a, const sse4Ints &b) {
	return blend4(a < b, a, b);		// [<] is faster than [<=]
}

static forceinline
sse4Ints max4(const sse4Ints &a, const sse4Ints &b) {
	return blend4(a > b, a, b);		// [>] is faster than [>=]
}

// end of sse4Ints.h
