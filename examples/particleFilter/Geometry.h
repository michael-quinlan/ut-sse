#pragma once

// basic geometry objects, scalar version

#include <math.h>

#include "sys/common.h"
#include "sys/sysMath.h"


typedef float AngRad;


class Point2D {
public:
	float x; /*!< x-coordinate of this position */
	float y; /*!< y-coordinate of this position */


	//--- STATIC CLASS METHODS ---//

	static forceinline Point2D fromPolar(float mag, AngRad ang) {
		// cos(phi) = x/r <=> x = r*cos(phi)
		// sin(phi) = y/r <=> y = r*sin(phi)
		return Point2D(mag * cosf(ang), mag * sinf(ang));
	}


	//--- CONSTRUCTORS ---//

	forceinline Point2D()
		: x(0.0f), y(0.0f) {}

	forceinline Point2D(float in_x, float in_y)
		: x(in_x), y(in_y) {}


	//--- ARITHMETIC OPERATORS ---//

	forceinline Point2D operator -(const Point2D &rhs) const {
		return Point2D(x - rhs.x, y - rhs.y);
	}

	forceinline Point2D operator *(const Point2D &rhs) const {
		return Point2D(x * rhs.x, y * rhs.y);
	}

	forceinline Point2D operator *(float d) const {
		return Point2D(x * d, y * d);
	}

	forceinline Point2D &operator += (const Point2D &rhs) {
		x += rhs.x;
		y += rhs.y;
		return *this;
	}

	//--- VECTOR OPERATIONS ---//

	forceinline float getMagnitude() const {
		return sqrtf(x*x + y*y);
	}

	forceinline AngRad getDirection() const {
		return atan2f(y, x);
	}

	forceinline float getDistanceTo(const Point2D &rhs) const {
		return (*this - rhs).getMagnitude();
	}

	//--- I/O ---//
	void println() const {
		printf("(%f, %f)\n", x, y);
	}
};


// component-wise sqrt
static forceinline
Point2D sqrt(const Point2D &p) {
	return Point2D(sqrtf(p.x), sqrtf(p.y));
}


typedef Point2D Vector2D;


class Rectangle {
private:
	Point2D lo;		// bottom left corner
	Point2D hi;		// top right corner

public:
	forceinline Rectangle() {}

	forceinline Rectangle(Point2D p1, Point2D p2) {
		lo = Point2D(min(p1.x, p2.x), min(p1.y, p2.y));
		hi = Point2D(max(p1.x, p2.x), max(p1.y, p2.y));
	}

	forceinline Point2D getBottomLeft() const {
		return lo;
	}

	forceinline Point2D getTopRight() const {
		return hi;
	}

	forceinline float getWidth() const {
		return hi.x - lo.x;
	}

	forceinline float getHeight() const {
		return hi.y - lo.y;
	}
};
