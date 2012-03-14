#pragma once

// wrapper for four 32-bit masks

#include "sys/common.h"

#include "sse/sseUtil.h"


class sseMask {
private:
	static const int ELT_OFF = 0x00000000;	// mask element off
	static const int ELT_ON  = 0xffffffff;	// mask element on

	static forceinline int getElt(bool b) {
		return b ? ELT_ON : ELT_OFF;
	}

	static forceinline char toChar(bool b) {
		return b ? 'T' : 'F';
	}

	static const int BITS_PER_MASK_ELT = 32;
	static const int BITS_PER_SI128_SHIFT = 8;
	static const int SI128_SHIFTS_PER_MASK_ELEMENT =
			BITS_PER_MASK_ELT / BITS_PER_SI128_SHIFT;

public:
	__m128 data;		// public to allow outside tinkering, as necessary

	forceinline sseMask() {}

	forceinline sseMask(__m128 input)
		: data(input) {}

	forceinline sseMask(__m128i input)
		: data(reint(input)) {}

	forceinline sseMask(bool b0, bool b1, bool b2, bool b3)
		: data(reint(_mm_set_epi32(getElt(b3),
								   getElt(b2),
								   getElt(b1),
								   getElt(b0)))) {}		// order is reversed

	forceinline bool operator [](int index) const {
		assert(index >= 0 && index < SSE_WIDTH);
		return ((int *)&data)[index] == ELT_ON;
	}

	static forceinline sseMask off() {
		return _mm_setzero_ps();
	}

	static forceinline sseMask on() {
		return off() == off();
	}

	//--- BITWISE ---//
	forceinline sseMask operator &(const sseMask &rhs) const {
		return _mm_and_ps(data, rhs.data);
	}

	forceinline sseMask operator |(const sseMask &rhs) const {
		return _mm_or_ps(data, rhs.data);
	}

	forceinline sseMask operator ^(const sseMask &rhs) const {
		return _mm_xor_ps(data, rhs.data);
	}

	forceinline sseMask operator ~() const {
		return operator ^(sseMask::on());
	}

	//--- SHIFTING ---//
	// shifts by units of 32-bits
	forceinline sseMask operator <<(int index) const {
		assert(index >= 0 && index < SSE_WIDTH);
		return _mm_slli_si128(reint(data), index * SI128_SHIFTS_PER_MASK_ELEMENT);
	}

	// shifts by units of 32-bits
	forceinline sseMask operator >>(int index) const {
		assert(index >= 0 && index < SSE_WIDTH);
		return _mm_srli_si128(reint(data), index * SI128_SHIFTS_PER_MASK_ELEMENT);
	}

	//--- ASSIGNMENT ---//
	forceinline sseMask &operator &=(const sseMask &rhs) {
		operator =(operator &(rhs)); return *this;
	}

	forceinline sseMask &operator |=(const sseMask &rhs) {
		operator =(operator |(rhs)); return *this;
	}

	forceinline sseMask &operator ^=(const sseMask &rhs) {
		operator =(operator ^(rhs)); return *this;
	}

	forceinline sseMask &operator <<=(int index) {
		operator =(operator <<(index)); return *this;
	}

	forceinline sseMask &operator >>=(int index) {
		operator =(operator >>(index)); return *this;
	}

	//--- COMPARISON ---//
	forceinline sseMask operator ==(const sseMask &rhs) const {
		return _mm_cmpeq_epi32(reint(data), reint(rhs.data));
	}

	forceinline sseMask operator !=(const sseMask &rhs) const {
		return ~(operator ==(rhs));
	}

	//--- SHUFFLE ---//
	template <int i0, int i1, int i2, int i3>
	forceinline sseMask shuffle() const {
		return sseImpl::shuffle<i0, i1, i2, i3>(data);
	}

	//--- PRINT ---//
	void print() const {
		printf("(%c, %c, %c, %c)", toChar(operator [](0)), toChar(operator [](1)),
								   toChar(operator [](2)), toChar(operator [](3)));
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

static forceinline
bool all(const sseMask &mask) {
	return _mm_movemask_ps(mask.data) == 0xf;
}

static forceinline
bool none(const sseMask &mask) {
	return _mm_movemask_ps(mask.data) == 0x0;
}

static forceinline
bool any(const sseMask &mask) {
	return _mm_movemask_ps(mask.data) != 0x0;
}

// end of sseMask.h
