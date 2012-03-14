#pragma once

// routines for manipulating angles

#include "sys/common.h"

#include "sse/sseMath.h"

#include "Geometry.h"


// 4 floats is the same as 4 angles in radians
typedef sse4Floats AngRad4;

// given two angles representing orientations, returns the minimum angle
// needed to rotate from one angle to another, the return value
// is on the range [0, PI]
static forceinline AngRad absMinAngleDiff(AngRad a, AngRad b) {
	assert( inbounds(a, -M_PI, M_PI) );
	assert( inbounds(b, -M_PI, M_PI) );

	AngRad d = fabsf(a - b);

	// the final answer should be on [0, PI], so if d is
	// greater than PI, then that means we went the wrong way
	// going from a to b, to go the other way, we rotate using
	// the angle 2PI - d
	return (d <= M_PI) ? d : (M_PI+M_PI) - d;
}


// 4-wide version,
// given two angles representing orientations, returns the minimum angle
// needed to rotate from one angle to another, the return value
// is on the range [0, PI]
static forceinline AngRad4 absMinAngleDiff(AngRad4 a, AngRad4 b) {
	assert( inbounds(a, -M_PI, M_PI) );
	assert( inbounds(b, -M_PI, M_PI) );

	AngRad4 pi = AngRad4::expand(M_PI);

	AngRad4 d = abs(a - b);

	// the final answer should be on [0, PI], so if d is
	// greater than PI, then that means we went the wrong way
	// going from a to b, to go the other way, we rotate using
	// the angle 2PI - d
	return blend4(d <= pi, d, pi + pi - d);
}


// 4-wide version,
// all output angles are in the range [-PI, PI]
static forceinline AngRad4 normalizeAngle(AngRad4 ang) {
	AngRad4 pi = AngRad4::expand(M_PI);
	AngRad4 two_pi = pi + pi;

	while (true) {
		sseMask over = ang > pi;
		if (none(over)) {
			break;
		}

		ang = blend4(over, ang - two_pi, ang);
	}

	AngRad4 neg_pi = -pi;

	while (true) {
		sseMask under = ang < neg_pi;
		if (none(under)) {
			break;
		}

		ang = blend4(under, ang + two_pi, ang);
	}

	return ang;
}


// 1-wide version of normalizing angles with a reduced domain,
// all input angles must be on the interval [-2PI, 2PI],
// all output angles are in the range [-PI, PI]
static forceinline AngRad normalizeAngleRD(AngRad ang) {
	assert( inbounds(ang, -2.0f*M_PI, 2.0f*M_PI) );
	AngRad two_pi = M_PI + M_PI;

	return (ang > M_PI) ? ang - two_pi
						: (ang < -M_PI) ? ang + two_pi
										: ang;
}


// 4-wide version of normalizing angles with a reduced domain,
// all input angles must be on the interval [-2PI, 2PI],
// all output angles are in the range [-PI, PI]
static forceinline AngRad4 normalizeAngleRD(AngRad4 ang) {
	assert( inbounds(ang, -2.0f*M_PI, 2.0f*M_PI) );

	AngRad4 pi = AngRad4::expand(M_PI);
	AngRad4 two_pi = pi + pi;

	AngRad4 temp1 = blend4(ang >  pi, ang - two_pi, ang);
	AngRad4 temp2 = blend4(ang < -pi, ang + two_pi, temp1);

	return temp2;
}


// end of Angle.h
