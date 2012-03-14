#pragma once

// interface to the particle filter

#include "sys/common.h"

#include "sse/sse.h"

#include "Particle.h"
#include "Particle_4Wide.h"


// operating modes for the particle filter
enum PfMode {
	PF_SCALAR,	// scalar-based particle filter
	PF_SSE		// SSE-based particle filter
};


// distance, bearing, and id
class Observation {
public:
	float d;		// distance
	AngRad b;		// bearing
	int id;			// reference object ID

	forceinline Observation(float in_d, AngRad in_b, int in_id)
		: d(in_d), b(in_b), id(in_id) {}
};

// the observations to use, in the form of base and size
class ObservationWindow {
private:
	int base;		// observation ID to start at
	int size;		// number of observations to use
	int total;		// total number of observations

public:
	forceinline ObservationWindow() {}

	forceinline ObservationWindow(int in_base, int in_size, int in_total)
		: base(in_base), size(in_size), total(in_total) {}

	forceinline int getBase() const {
		return base;
	}

	forceinline int getSize() const {
		return size;
	}

	forceinline int getTotal() const {
		return total;
	}

	// go to the previous observation
	forceinline void prev() {
		base = clamp(base - 1, 0, total - size);
	}

	// go to the next observation
	forceinline void next() {
		base = clamp(base + 1, 0, total - size);
	}

	// increase the size of the observation window
	forceinline void grow() {
		size = clamp(size + 1, 1, total);
		// reclamp using the new window size
		base = clamp(base, 0, total - size);

	}

	// decrease the size of the observation window
	forceinline void shrink() {
		size = clamp(size - 1, 1, total);
		// reclamp using the new window size
		base = clamp(base, 0, total - size);
	}
};

// estimated robot pose and error
class RobotPose {
public:
	Point2D pos_mn;		// mean position
	AngRad  ang_mn;		// mean angle

	Point2D pos_sd;		// standard deviation in x and standard deviation in y
	AngRad  ang_sd;		// standard deviation in angle

	forceinline RobotPose() {}

	forceinline RobotPose(Point2D in_pos_mn, AngRad in_ang_mn,
							Point2D in_pos_sd, AngRad in_ang_sd)
		: pos_mn(in_pos_mn), ang_mn(in_ang_mn),
		  pos_sd(in_pos_sd), ang_sd(in_ang_sd) {}

	forceinline RobotPose operator -(const RobotPose &rhs) const {
		return RobotPose(pos_mn - rhs.pos_mn, ang_mn - rhs.ang_mn,
						 pos_sd - rhs.pos_sd, ang_sd - rhs.ang_sd);
	}

	void println() {
		printf("position: (%f, %f), angle: %f\n",
				pos_mn.x, pos_mn.y, ang_mn);
		printf("std dev position: (%f, %f), std dev angle: %f\n",
				pos_sd.x, pos_sd.y, ang_sd);
	}
};

// a pair of exponents, one for distance similarity and one for bearing similarity
class ProbabilityExponents {
public:
	float distanceExp;
	float bearingExp;

	forceinline ProbabilityExponents() {}

	forceinline ProbabilityExponents(float in_distanceExp, float in_bearingExp)
		: distanceExp(in_distanceExp), bearingExp(in_bearingExp) {}

	forceinline ProbabilityExponents operator +
							(const ProbabilityExponents &rhs) const
	{
		return ProbabilityExponents(distanceExp + rhs.distanceExp,
									bearingExp  + rhs.bearingExp);
	}

	forceinline ProbabilityExponents &operator +=
							(const ProbabilityExponents &rhs)
	{
		operator =(operator +(rhs)); return *this;
	}

	void println() const {
		printf("d: % f, b: % f\n", distanceExp, bearingExp);
	}
};


// different helper methods for extracting items from ProbabilityExponents

static forceinline
float getDistanceExponent(const ProbabilityExponents &pe) {
	return pe.distanceExp;
}

static forceinline
float getBearingExponent(const ProbabilityExponents &pe) {
	return pe.bearingExp;
}

static forceinline
float getDistancePlusBearingExponent(const ProbabilityExponents &pe) {
	return pe.distanceExp + pe.bearingExp;
}


class ProbabilityExponents_4Wide {
public:
	sse4Floats distanceExp;
	sse4Floats bearingExp;

	forceinline ProbabilityExponents_4Wide() {}

	forceinline ProbabilityExponents_4Wide(sse4Floats in_distanceExp,
											 sse4Floats in_bearingExp)
		: distanceExp(in_distanceExp), bearingExp(in_bearingExp) {}

	// extract an element from the 4-wide
	forceinline ProbabilityExponents operator [](int index) const {
		return ProbabilityExponents(distanceExp[index], bearingExp[index]);
	}

	forceinline ProbabilityExponents_4Wide operator +
							(const ProbabilityExponents_4Wide &rhs) const
	{
		return ProbabilityExponents_4Wide(distanceExp + rhs.distanceExp,
										  bearingExp  + rhs.bearingExp);
	}

	forceinline ProbabilityExponents_4Wide &operator +=
							(const ProbabilityExponents_4Wide &rhs)
	{
		operator =(operator +(rhs)); return *this;
	}

	void println() const {
		operator [](0).println();
		operator [](1).println();
		operator [](2).println();
		operator [](3).println();
	}
};


// different helper methods for extracting items from ProbabilityExponents4Wide

static forceinline
sse4Floats getDistanceExponent(const ProbabilityExponents_4Wide &pe) {
	return pe.distanceExp;
}

static forceinline
sse4Floats getBearingExponent(const ProbabilityExponents_4Wide&pe) {
	return pe.bearingExp;
}

static forceinline
sse4Floats getDistancePlusBearingExponent(const ProbabilityExponents_4Wide &pe) {
	return pe.distanceExp + pe.bearingExp;
}


// all scalar particles and associated data
class ParticleArray {
public:
	Particle *p;
	ProbabilityExponents *e;
	int n;

	forceinline ParticleArray() {}

	forceinline ParticleArray(Particle *in_p,
								ProbabilityExponents *in_e,
								int in_n)
		: p(in_p), e(in_e), n(in_n) {}
};

// all SSE particles and associated data
class ParticleArray_4Wide {
public:
	Particle_4Wide *p;
	ProbabilityExponents_4Wide *e;
	int n;		// number of 4-wides

	forceinline ParticleArray_4Wide() {}

	forceinline ParticleArray_4Wide(Particle_4Wide *in_p,
									  ProbabilityExponents_4Wide *in_e,
									  int in_n)
		: p(in_p), e(in_e), n(in_n) {}
};


//--- EXTERNAL INTERFACE---//

void seedParticleGen(unsigned int rand_seed);

void initAllParticles();


void loadObservationData(const char *filename);

Observation *getObservations();

ObservationWindow *getObservationWindow();


Rectangle getField();

Rectangle getGrass();

RobotPose getActualPose();

const Point2D *getReferenceObjects();

int getNumReferenceObjects();


void togglePfMode();

PfMode getPfMode();

const char *getPfModeString();

RobotPose runPf();

float getLastPfFps();

ParticleArray getParticles();

ParticleArray_4Wide getParticles_4Wide();

void comparePfResults();


// end of pf.h
