#pragma once

// basic particle, SSE version

#include "sys/common.h"

#include "sse/sse4Floats.h"

#include "Particle.h"
#include "Point2D_4Wide.h"

class Particle_4Wide {
public:
	Point2D_4Wide pos;		// 4-wide
	AngRad4       ang;		// 4-wide

	forceinline Particle_4Wide() {}

	// angles must be in the range [-PI, PI]
	forceinline Particle_4Wide(Point2D_4Wide in_pos, AngRad4 in_ang)
		: pos(in_pos), ang(in_ang)
	{
		assert(inbounds(ang, -M_PI, M_PI));
	}

	forceinline Particle_4Wide(Particle in0, Particle in1,
								 Particle in2, Particle in3)
		: pos(in0.pos, in1.pos, in2.pos, in3.pos),
		  ang(in0.ang, in1.ang, in2.ang, in3.ang)
	{
		assert(inbounds(ang, -M_PI, M_PI));
	}

	// converts a 1-wide into a 4-wide via expansion
	static forceinline Particle_4Wide expand(Particle in) {
		return Particle_4Wide(Point2D_4Wide::expand(in.pos),
							  sse4Floats::expand(in.ang));
	}

	// returns the distance of this particle to the point
	forceinline sse4Floats getDistanceTo(Point2D_4Wide p) const {
		return pos.getDistanceTo(p);
	}

	// returns the bearing of this particle to the point
	forceinline sse4Floats getBearingTo(Point2D_4Wide p) const
	{
		return pos.getBearingTo(p, ang);
	}
};

// end of Particle_4Wide.h
