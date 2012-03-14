#pragma once

// basic point, SSE version

#include "sys/common.h"

#include "sse/sseMath.h"

#include "Geometry.h"
#include "Angle.h"


class Point2D_4Wide {
public:
	sse4Floats x;
	sse4Floats y;

	forceinline Point2D_4Wide() {}

	forceinline Point2D_4Wide(sse4Floats in_x,  sse4Floats in_y)
		: x(in_x), y(in_y) {}

	forceinline Point2D_4Wide(Point2D in0, Point2D in1,
								Point2D in2, Point2D in3)
		: x(in0.x, in1.x, in2.x, in3.x), y(in0.y, in1.y, in2.y, in3.y) {}

	// converts a 1-wide into a 4-wide via expansion
	static forceinline Point2D_4Wide expand(Point2D in) {
		return Point2D_4Wide(sse4Floats::expand(in.x), sse4Floats::expand(in.y));
	}

	// extract an element from the 4-wide
	forceinline Point2D operator[] (int index) const {
		return Point2D(x[index], y[index]);
	}

	//--- ARITHMETIC ---//
	forceinline Point2D_4Wide operator+ (Point2D_4Wide rhs) const {
		return Point2D_4Wide(x + rhs.x, y + rhs.y);
	}

	forceinline Point2D_4Wide operator- (Point2D_4Wide rhs) const {
		return Point2D_4Wide(x - rhs.x, y - rhs.y);
	}

	forceinline Point2D_4Wide operator* (Point2D_4Wide rhs) const {
		return Point2D_4Wide(x * rhs.x, y * rhs.y);
	}

	forceinline Point2D_4Wide operator/ (Point2D_4Wide rhs) const {
		return Point2D_4Wide(x / rhs.x, y / rhs.y);
	}

	forceinline Point2D_4Wide operator* (sse4Floats scale) const {
		return Point2D_4Wide(x * scale, y * scale);
	}

	//--- ASSIGNMENT ---//
	forceinline Point2D_4Wide &operator+= (const Point2D_4Wide &rhs) {
		operator=(operator+(rhs)); return *this;
	}

	//--- REDUCTION ---//
	forceinline Point2D reduce_add() const {
		return Point2D(x.reduce_add(), y.reduce_add());
	}

	//--- MISC ---//

	// the distance from this point to the given position
	forceinline sse4Floats getDistanceTo(Point2D_4Wide p) const {
		sse4Floats dx = p.x - x;
		sse4Floats dy = p.y - y;
		return sqrt(dx*dx + dy*dy);
	}

	// the bearing from this point to the given position assuming that
	// this point is at the given orientation
	//
	// all elements in the orientation should be on [-PI, PI]
	forceinline AngRad4 getBearingTo(Point2D_4Wide p,
									   AngRad4 o) const
	{
		assert(inbounds(o, -M_PI, M_PI));

		AngRad4 theta = atan2(p.y - y, p.x - x);
		return normalizeAngleRD(theta - o);
	}
};


// treat 2D points and vectors similarly
typedef Point2D_4Wide Vector2D_4Wide;


// constructor for a Vector2D_4Wide in polar coordinates
static forceinline
Vector2D_4Wide Vector2D_4Wide_Polar(sse4Floats mag,
									AngRad4    ang)
{
	return Vector2D_4Wide(cos(ang), sin(ang)) * mag;
}

// end of Point2D_4Wide.h
