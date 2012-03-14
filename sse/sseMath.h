#pragma once

// SSE version of functions in math.h, and some miscellaneous helper functions

#include <math.h>

#include "sys/common.h"
#include "sys/sysMath.h"

#include "sse/sse.h"


// 4-wide cast from float to int, i.e. truncation
static forceinline
sse4Ints cast_f2i(sse4Floats input) {
	return sse4Ints(_mm_cvtps_epi32(input.data));
}


// 4-wide cast from int to float, i.e. promotion
static forceinline
sse4Floats cast_i2f(sse4Ints input) {
	return sse4Floats(_mm_cvtepi32_ps(input.data));
}


// 4-wide reinterpretation of bits from float to int
static forceinline
sse4Ints reint_f2i(sse4Floats input) {
	return sse4Ints(input.data);
}


// 4-wide reinterpretation of bits from int to float
static forceinline
sse4Floats reint_i2f(sse4Ints input) {
	return sse4Floats(input.data);
}


#undef isnan
static forceinline
bool isnan(sse4Floats input) {
	return any(nanMask(input));
}


// inclusive bounds [lo, hi]
static forceinline
bool inbounds(sse4Floats val, float lo, float hi) {
	return all( inRangeMask(val, sse4Floats::expand(lo),
								 sse4Floats::expand(hi)) );
}


// returns which elements have their sign bit set
static forceinline
sseMask sign_bit_mask(sse4Floats input) {
	sse4Ints int_input = sse4Ints(input.data);

	// shift until only the sign bit remains
	return (int_input >> 31) != sse4Ints::zeros();
}


// returns a mask of which elements are negative,
// for this function -0, NEGINF, and NaN with the sign
// bit set are considered to be negative
static forceinline
sseMask is_neg_special(sse4Floats input) {
	return sign_bit_mask(input);
}


static forceinline
sse4Floats approx_rcp(sse4Floats input) {
	return _mm_rcp_ps(input.data);
}


// approximate reciprocal and one iteration of Newton-Raphson,
// does not work if input is zero
static forceinline
sse4Floats nr_rcp(sse4Floats input) {
	assert( none( input == sse4Floats::zeros() ) );

	sse4Floats r = approx_rcp(input);
	return r + r - input*r*r;
}


// division using an approximate reciprocal
static forceinline
sse4Floats approx_div(sse4Floats numer, sse4Floats denom) {
	return  numer * approx_rcp(denom);
}


// division using an approximate reciprocal and one iteration of Newton-Raphson,
// does not work if numer is non-zero and denom is zero
static forceinline
sse4Floats nr_div(sse4Floats numer, sse4Floats denom ) {
	assert( none( (numer != sse4Floats::zeros()) & (denom == sse4Floats::zeros()) ) );

	sse4Floats r = approx_rcp(denom);
	sse4Floats nr = numer*r;
	sse4Floats drnr = denom*r*nr;
	return nr + nr - drnr;
}


static forceinline
sse4Floats sqrt(sse4Floats input) {
	return _mm_sqrt_ps(input.data);
}


//--- ABS ---//

// fast version
static forceinline
sse4Floats abs(sse4Floats x) {
	sse4Floats no_sign_bit = reint_i2f(sse4Ints::expand(0x7fffffff));
	return x & no_sign_bit;		// clear the sign bit
}


// reference version
static forceinline
sse4Floats abs_ref(sse4Floats x) {
	return sse4Floats(fabsf(x[0]),
					  fabsf(x[1]),
					  fabsf(x[2]),
					  fabsf(x[3]));
}


//--- ATAN ---//

// domain: [0, 1]
// range:  [0, PI/4]
static forceinline
sse4Floats __atan_rd(sse4Floats x) {
	// either the value is a NaN or it's in the range [0, 1]
	assert( all( nanMask(x) |
				 inRangeMask(x, sse4Floats::zeros(),
								reint_i2f(sse4Ints::expand(0x3f800000))) ) );

	// using Euler's version of the atan series expansion, which converges quickly
	sse4Floats c1 = reint_i2f(sse4Ints::expand(0x3f800000));	//              1.0f
	sse4Floats c2 = reint_i2f(sse4Ints::expand(0x3f2aaaab));	//    2.0f /    3.0f
	sse4Floats c3 = reint_i2f(sse4Ints::expand(0x3f088889));	//    8.0f /   15.0f
	sse4Floats c4 = reint_i2f(sse4Ints::expand(0x3eea0ea1));	//   16.0f /   35.0f
	sse4Floats c5 = reint_i2f(sse4Ints::expand(0x3ed00d01));	//  128.0f /  315.0f
	sse4Floats c6 = reint_i2f(sse4Ints::expand(0x3ebd2318));	//  256.0f /  693.0f
	sse4Floats c7 = reint_i2f(sse4Ints::expand(0x3eae968c));	// 1024.0f / 3003.0f

	sse4Floats q = approx_div(x, (x*x + c1));

	sse4Floats z   = x * q;
	sse4Floats z_2 = z * z;
	sse4Floats z_3 = z * z_2;
	sse4Floats s   = c1 + c2*z + c3*z_2 + z_3*(c5*z + c4 + c6*z_2 + c7*z_3);
	sse4Floats rval = q * s;

	// fix up values that generate 0 but should just be x,
	// below this cutoff x and atan(x) are identical
	sse4Floats thr = reint_i2f(sse4Ints::expand(0x39b89ba3));	// 0.000352f

	return blend4(x < thr, x, rval);
}


// fast version
static forceinline
sse4Floats atan(sse4Floats x) {
	sse4Floats one = reint_i2f(sse4Ints::expand(0x3f800000));	// 1.0f

	// use the following identities:
	// 1) atan(x) = PI/2 - atan(1/x)
	// 2) atan(x) = -atan(-x)
	// ...so that all input is transformed into the range [0, 1]

	// take absolute value
	sseMask neg_x = x < sse4Floats::zeros();
	sse4Floats sign_conv = blend4(neg_x, -one, one);
	sse4Floats abs_x = sign_conv * x;

	// invert all values that are greater than one
	sseMask inv_mask = (abs_x > one);
	sse4Floats inv_abs_x = approx_rcp(abs_x);
	sse4Floats x_ror = blend4(inv_mask, inv_abs_x, abs_x);

	// call the helper on the in-range values
	sse4Floats atan_rd = __atan_rd(x_ror);

	// fix signs based on the signs of the input
	sse4Floats signs_fixed = sign_conv * atan_rd;

	// correct the output range for all inverted input by
	// either subtracting from PI/2 or -PI/2, depending on the
	// sign of signs_fixed (which matches the neg_x mask)
	sse4Floats half_pi = reint_i2f(sse4Ints::expand(0x3fc90fdb));	// 1.570796f
	sse4Floats base = blend4(neg_x, -half_pi, half_pi);
	sse4Floats range_fixed = blend4(inv_mask, base-signs_fixed, signs_fixed);

	return range_fixed;
}


// reference version
static forceinline
sse4Floats atan_ref(sse4Floats x) {
	return sse4Floats(atan(x[0]),
					  atan(x[1]),
					  atan(x[2]),
					  atan(x[3]));
}


//--- ATAN2 ---//

// fast version
//
// NOTE: does not handle any of the following inputs:
// (+0, +0), (+0, -0), (-0, +0), (-0, -0)
static forceinline
sse4Floats atan2(sse4Floats y, sse4Floats x) {
	sse4Floats pi = reint_i2f(sse4Ints::expand(0x40490fdb));	// 3.141593f

	// compute the atan
	sse4Floats raw_atan = atan(approx_div(y, x));

	// treat -0 as though it were negative
	sseMask neg_x = is_neg_special(x);
	sseMask neg_y = is_neg_special(y);

	// fix up quadrants 2 and 3 based on the sign of the input

	// move from quadrant 4 to 2 by adding PI
	sseMask in_quad2 = neg_x & ~neg_y;
	sse4Floats quad2_fixed = blend4(in_quad2, raw_atan + pi, raw_atan);

	// move from quadrant 1 to 3 by subtracting PI
	sseMask in_quad3 = neg_x &  neg_y;
	sse4Floats quad23_fixed = blend4(in_quad3, raw_atan - pi, quad2_fixed);

	return quad23_fixed;
}


// reference version
static forceinline
sse4Floats atan2_ref(sse4Floats y, sse4Floats x) {
	return sse4Floats(atan2(y[0], x[0]),
					  atan2(y[1], x[1]),
					  atan2(y[2], x[2]),
					  atan2(y[3], x[3]));
}


//--- OLD ATAN2 ---//

static forceinline
sse4Floats old__atan2_helper2(sse4Floats z, sse4Floats d) {
	sse4Floats c3 = reint_i2f(sse4Ints::expand(0xbeaaa9e3));
	sse4Floats c5 = reint_i2f(sse4Ints::expand(0x3e4c7fcd));
	sse4Floats c7 = reint_i2f(sse4Ints::expand(0xbe0d6825));
	sse4Floats c9 = reint_i2f(sse4Ints::expand(0x3da0ce39));
	sse4Floats scale = reint_i2f(sse4Ints::expand(0x40800000));

	sse4Floats x = nr_div(z, d + sqrt(d*d + z*z));
	sse4Floats x2 = x*x;
	sse4Floats xs = x*scale;
	sse4Floats x3s = xs*x2;
	return xs + x3s*c3 + x3s*x2*(x2*c7 + c5 + x2*x2*c9);
}


static forceinline
sse4Floats old__atan2_helper1(sse4Floats y, sse4Floats x) {
		return old__atan2_helper2(y, x + sqrt(x*x + y*y));
}


static forceinline
sse4Floats old_atan2(sse4Floats y, sse4Floats x) {
	sse4Floats raw = old__atan2_helper1(y, x);

	// patch up the negative x-axis if we're on it
	sseMask mask = (y == sse4Floats::zeros()) & (x < sse4Floats::zeros());
	sse4Floats pi = reint_i2f(sse4Ints::expand(0x40490fdb));	// 3.141593

	return blend4(mask, pi, raw);
}


//--- EXP ---//

// computes 2^x, input is in integer format, output is in float format
// domain: [-126, 127]
// range:  [2^-126, 2^127]
static forceinline
sse4Floats __exp_exponent(sse4Ints x) {
	sse4Floats c1 = reint_i2f(sse4Ints::expand(0x3f800000));	// 1.0f
	sse4Ints   as_int = (x << 23) + sse4Ints(c1.data);
	return sse4Floats(as_int.data);
}


// computes e^x
// domain: [0.0, log_2(e)],
// range:  [1.0, 2.0)
static forceinline
sse4Floats __exp_mantissa(sse4Floats x) {
	sse4Floats c1 = reint_i2f(sse4Ints::expand(0x3f800000));	// 1.0f
	sse4Floats c2 = reint_i2f(sse4Ints::expand(0x3f000000));	// 0.5f
	sse4Floats c3 = reint_i2f(sse4Ints::expand(0x3e2aaa1d));	// 0.166665f
	sse4Floats c5 = reint_i2f(sse4Ints::expand(0x3d093a89));	// 0.033503f
	sse4Floats c6 = reint_i2f(sse4Ints::expand(0x3bb71b61));	// 0.005588f

	sse4Floats x2 = x*x;
	sse4Floats x2_2 = x2*c2;
	return c1 + x + x2_2 + c3*x*x2 + x2_2*x2_2*(c3 + c5*x + c6*x2);
}


// handles everything in the reduced domain (0xc2aeac51, 0x42b0c0a6) which is
// approximately (-87.3, 88.4), if the input is not in this range
// the results are undefined
static forceinline
sse4Floats __exp_rd(sse4Floats x) {
	sse4Floats log_2e = reint_i2f(sse4Ints::expand(0x3fb8aa3b));	// 1.442695f
	sse4Floats log_e2 = reint_i2f(sse4Ints::expand(0x3f317218));	// 0.693147f

	sse4Ints   pre_e = cast_f2i(log_2e*x);			// generates exponent
	sse4Floats pre_m = x - log_e2*cast_i2f(pre_e);	// generates mantissa

	return __exp_exponent(pre_e) * __exp_mantissa(pre_m);

}

// fast version
//
// NOTE: the output of this function produces infinity at a lower value of x
// than the reference version, at x = 88.376266 rather than x = 88.722839
static forceinline
sse4Floats exp(sse4Floats x) {
	sse4Floats min_thr = reint_i2f(sse4Ints::expand(0xc2aeac51));	// -87.336555f
	sse4Floats max_thr = reint_i2f(sse4Ints::expand(0x42b0c0a6));	//  88.376266f

	sse4Floats clamp0 = max4(min_thr, x);
	sse4Floats clamp1 = min4(max_thr, clamp0);

	return __exp_rd(clamp1);
}


// reference version
static forceinline
sse4Floats exp_ref(sse4Floats x) {
	return sse4Floats(exp(x[0]),
					  exp(x[1]),
					  exp(x[2]),
					  exp(x[3]));
}


//--- SIN ---//

// domain: [ -PI,  PI]
// range:  [-1.0, 1.0]
static forceinline
sse4Floats __sin_ror(sse4Floats x) {
	// either the value is a NaN or it's in the range [-PI, PI]
	assert( all( nanMask(x) |
				 inRangeMask(x, reint_i2f(sse4Ints::expand(0xc0490fdb)),
								reint_i2f(sse4Ints::expand(0x40490fdb))) ) );

	sse4Floats c3 = reint_i2f(sse4Ints::expand(0xbe2aaaab));	// -0.166667f
	sse4Floats c5 = reint_i2f(sse4Ints::expand(0x3c0887e6));	//  0.008333f
	sse4Floats c7 = reint_i2f(sse4Ints::expand(0xb94fc635));	// -0.000198f
	sse4Floats c9 = reint_i2f(sse4Ints::expand(0x362f5e1d));	//  0.000003f

	sse4Floats x2 = x*x;
	sse4Floats x3 = x*x2;
	sse4Floats rval = x + x3*c3 + x3*x2*(x2*c7 + c5 + x2*x2*c9);

	// fix up values that generate 0 but should just be x,
	// below this absolute value cutoff x and sin(x) are identical
	sse4Floats thr = reint_i2f(sse4Ints::expand(0x39e89769));	// 0.000444f
	return blend4(abs(x) < thr, x, rval);
}


// fast version
static forceinline
sse4Floats sin(sse4Floats x) {
	sse4Floats pi = reint_i2f(sse4Ints::expand(0x40490fdb));		//        3.141593f
	sse4Floats inv_pi = reint_i2f(sse4Ints::expand(0x3ea2f983));	// 1.0f / 3.141593f

	// figure out how many multiples of pi are in x
	sse4Ints ipart  = cast_f2i(inv_pi*x);

	// if ipart is odd, set the sign bit to make x_ror negative
	sse4Floats x_ror = reint_i2f(ipart << 31) ^ (x - cast_i2f(ipart)*pi);

	return __sin_ror(x_ror);
}


// reference version
static forceinline
sse4Floats sin_ref(sse4Floats x) {
	return sse4Floats(sin(x[0]),
					  sin(x[1]),
					  sin(x[2]),
					  sin(x[3]));
}


//--- COS ---//

// fast version
static forceinline
sse4Floats cos(sse4Floats x) {
	sse4Floats half_pi = reint_i2f(sse4Ints::expand(0x3fc90fdb));	// 1.570796f
	return sin(x + half_pi);
}


// reference version
static forceinline
sse4Floats cos_ref(sse4Floats x) {
	return sse4Floats(cos(x[0]),
					  cos(x[1]),
					  cos(x[2]),
					  cos(x[3]));
}


// end of sseMath.h
