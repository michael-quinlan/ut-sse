#pragma once

// basic particle, scalar version

#include <math.h>

#include "sys/common.h"
#include "sys/rand.h"

#include "Geometry.h"
#include "Angle.h"


class Particle {
public:
	Point2D pos;
	AngRad  ang;

	forceinline void placeRandomly(const Rectangle &bounds) {
		Point2D bl = bounds.getBottomLeft();

		// generate a random point within the boundary
		pos = Point2D(getRand(bl.x, bounds.getWidth()),
					  getRand(bl.y, bounds.getHeight()));
		ang = getRand(-M_PI, 2.0 * M_PI);		// [-PI, PI]
	}

	// returns the distance of this particle to the point
	forceinline float getDistanceTo(const Point2D &point) const {
		return pos.getDistanceTo(point);
	}

	// returns the bearing of this particle to the point
	forceinline AngRad getBearingTo(const Point2D &point) const {
		float dx = point.x - pos.x;
		float dy = point.y - pos.y;

		AngRad theta = atan2f(dy, dx);
		return normalizeAngleRD(theta - ang);
	}
};

// end of Particle.h
