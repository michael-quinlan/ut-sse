// particle filter, scalar and SSE versions

#include <stdio.h>
#include <math.h>

#include <iostream>
#include <fstream>
using namespace std;

#include "sys/common.h"
#include "sys/mem.h"
#include "sys/rand.h"
#include "sys/Timer.h"

#include "sse/sseMath.h"

#include "Particle.h"
#include "Particle_4Wide.h"

#include "pf.h"


//--- CONSTANTS ---//

static const float INV_M_PI = 1.0f / M_PI;

static const int NUM_SCALAR_PARTICLES = 16384;
static const int NUM_SSE_PARTICLES = NUM_SCALAR_PARTICLES / SSE_WIDTH;

// coordinate system for the field
//
//                +---------+
//                |         |
//                |         |
// y              |         |
//                |    O    |    O - origin at center of field
// ^              |         |
// |              |         |
// |              |         |
//    -----> x    +---------+

// in-bounds area of the field
static const float FIELD_X = 6000.0f;	// in millimeters
static const float FIELD_Y = 4000.0f;	// in millimeters
static const Rectangle FIELD = Rectangle( Point2D( -FIELD_X * 0.5f, -FIELD_Y * 0.5f ),
										  Point2D(  FIELD_X * 0.5f,  FIELD_Y * 0.5f ) );

// area where the robots are assumed to be, slightly larger than the field
static const float GRASS_X = 6800.0f;	// in millimeters
static const float GRASS_Y = 4400.0f;	// in millimeters
static const Rectangle GRASS = Rectangle( Point2D( -GRASS_X * 0.5f, -GRASS_Y * 0.5f ),
										  Point2D(  GRASS_X * 0.5f,  GRASS_Y * 0.5f ) );

// the robot position (known from observation generator)
static const Point2D ROBOT_POS  = Point2D(1500.0f, 1500.0f);
static const AngRad ROBOT_ANGLE = 0.0f;

// the location of the reference objects (known from observation generator)
static const Point2D REF_OBJ_POS_ARR[] = {
	Point2D(2600.0f,   700.0f),		// object 1
	Point2D(2600.0f,  -700.0f),		// object 2
	Point2D(2000.0f, -1000.0f)		// object 3
};
static const int NUM_REF_OBJS = sizeof(REF_OBJ_POS_ARR) / sizeof(Point2D);

// standard deviations for the observations (known from observation generator)
static const float DIST_SIGMA = 0.2f;
static const float BEAR_SIGMA = 0.05f;

// precomputed versions of 1/(sigma*sigma),
// used as coefficients when computing the exponents to the exponential function
static const float DIST_EXP_COEFF = 1.0f / (DIST_SIGMA * DIST_SIGMA);
static const float BEAR_EXP_COEFF = 1.0f / (BEAR_SIGMA * BEAR_SIGMA);

static const float IDEAL_FPS = 30.0f;	// frames per second

const char *PF_MODE_STRINGS[] = {
	"scalar",
	"sse"
};


//--- GLOBALS ---//

// the observations
static Observation *obsData = NULL;		// observation data
static ObservationWindow obsWindow;

// the particles
static Particle       scalarParticles[NUM_SCALAR_PARTICLES];
static Particle_4Wide sseParticles   [NUM_SSE_PARTICLES];

// probabilities calculated for each particle
static ProbabilityExponents       scalarProb[NUM_SCALAR_PARTICLES];
static ProbabilityExponents_4Wide sseProb   [NUM_SSE_PARTICLES];

static PfMode pfMode = PF_SSE;		// default particle filter mode
static float pfFps = 0.0f;			// last invocation's frames per second


//--- BOUNDS CHECK ---//

static
void checkRefObjIdRange(int ref_obj_i) {
	if (ref_obj_i < 0 || ref_obj_i >= NUM_REF_OBJS) {
		printf("num reference objects: %d, bad index: %d\n",
			   NUM_REF_OBJS, ref_obj_i);
		exit(0);
	}
}


//--- SETUP ---//

static
void initScalarParticles() {
	for (int i = 0; i < NUM_SCALAR_PARTICLES; i++){
		scalarParticles[i].placeRandomly(GRASS);
	}
}

// creates the sse particles from the scalar particles
static
void initSseParticles() {
	for (int i = 0; i < NUM_SSE_PARTICLES; i++) {
		int j = 4 * i;		// index into the non-sse array

		Particle &p0 = scalarParticles[j + 0];
		Particle &p1 = scalarParticles[j + 1];
		Particle &p2 = scalarParticles[j + 2];
		Particle &p3 = scalarParticles[j + 3];

		sseParticles[i] = Particle_4Wide(p0, p1, p2, p3);
	}
}

static
void clearScalarProbabilities() {
	for (int p = 0; p < NUM_SCALAR_PARTICLES; p++) {
		scalarProb[p] = ProbabilityExponents(0.0f, 0.0f);
	}
}

static
void clearSseProbabilities() {
	for (int p = 0; p < NUM_SSE_PARTICLES; p++) {
		sseProb[p] = ProbabilityExponents_4Wide(sse4Floats::zeros(),
												sse4Floats::zeros());
	}
}


//--- DISTANCE PROBABILITY ---//

// Gets the exponent of the similarity measure based on seen and expected distances to
// two objects.
static forceinline
float getDistanceSimExponent(float expectedDist, float observedDist, float coeffDist) {
	// normalize by max(expected, observed) to account for the fact that greater
	// deviation is expected when the distance is greater
	float d = fabsf(expectedDist - observedDist) / max(expectedDist, observedDist);
	assert(inbounds(d, 0.0f, 1.0f));
	return -coeffDist * d * d;
}

// Gets the similarity measure based on seen and expected distances to
// two objects.
static forceinline
float getDistanceSim(float expectedDist, float observedDist, float coeffDist) {
	return exp(getDistanceSimExponent(expectedDist, observedDist, coeffDist));
}

// Gets the exponent of the similarity measure based on seen and expected distances to
// two objects.
static forceinline
sse4Floats getDistanceSimExponent(sse4Floats expectedDist,
								  sse4Floats observedDist,
								  sse4Floats coeffDist)
{
	// normalize by max(expected, observed) to account for the fact that greater
	// deviation is expected when the distance is greater
	sse4Floats d = abs(expectedDist - observedDist) / max4(expectedDist, observedDist);
	assert(inbounds(d, 0.0f, 1.0f));
	return -coeffDist * d * d;
}

// Gets the exponent of the similarity measure based on seen and expected distances to
// two objects.
static forceinline
sse4Floats getDistanceSim(sse4Floats expectedDist,
						  sse4Floats observedDist,
						  sse4Floats coeffDist)
{
	return exp(getDistanceSimExponent(expectedDist, observedDist, coeffDist));
}


//--- BEARING PROBABILITY ---//

// Gets the exponent of the similarity measure based on seen and expected angles of
// the landmarks.
static forceinline
float getBearingSimExponent(AngRad expectedAng,
							AngRad observedAng,
							float coeffAng)
{
	// normalize by PI since the absolue min angle diff is on [0, PI]
	float d = absMinAngleDiff(expectedAng, observedAng) * INV_M_PI;
	assert(inbounds(d, 0.0f, 1.0f));
	return -coeffAng * d * d;
}

// Gets the similarity measure based on seen and expected angles of
// the landmarks.
static forceinline
float getBearingSim(AngRad expectedAng,
					AngRad observedAng,
					float coeffAng)
{
	return exp(getBearingSimExponent(expectedAng, observedAng, coeffAng));
}

// Gets the exponent of the similarity measure based on seen and expected angles of
// the landmarks.
static forceinline
sse4Floats getBearingSimExponent(AngRad4 expectedAng,
								 AngRad4 observedAng,
								 AngRad4 coeffAng)
{
	// normalize by PI since the absolue min angle diff is on [0, PI]
	AngRad4 d = absMinAngleDiff(expectedAng, observedAng)
				* sse4Floats::expand(INV_M_PI);
	assert(inbounds(d, 0.0f, 1.0f));
	return -coeffAng * d * d;
}

// Gets the similarity measure based on seen and expected angles of
// the landmarks.
static forceinline
sse4Floats getBearingSim(AngRad4 expectedAng,
						 AngRad4 observedAng,
						 AngRad4 coeffAng)
{
	return exp(getBearingSimExponent(expectedAng, observedAng, coeffAng));
}


//--- POSE ESTIMATION ---//

// Computes the weighted mean of the robot pose (location and bearing) and
// the weighted standard deviation of the robot pose.  This
// version computes the values using a standard two-pass algorithm
// which computes the mean in the first pass and the standard deviation
// in the second pass.
//
// Only scalar operations are used.
static noinline
RobotPose scalarEstimatePose() {
	Point2D  pos_accum = Point2D(0.0f, 0.0f);
	Vector2D ori_accum = Vector2D(0.0f, 0.0f);
	float    w_accum   = 0.0f;		// weight accumulator

	// compute weighted mean
	for (int i = 0; i < NUM_SCALAR_PARTICLES; i++) {
		Point2D &pos = scalarParticles[i].pos;
		AngRad  &ang = scalarParticles[i].ang;
		float    w   = expf(getDistancePlusBearingExponent(scalarProb[i]));

		pos_accum += pos * w;
		ori_accum += Point2D::fromPolar(w, ang);
		w_accum   += w;
	}
	assert(w_accum != 0.0f);
	assert(ori_accum.getMagnitude() != 0.0f);

	float inv_total_w = 1.0f / w_accum;

	Point2D pos_mn = pos_accum * inv_total_w;
	AngRad  ang_mn = ori_accum.getDirection();

	Point2D pd2_accum = Point2D(0.0f, 0.0f);
	AngRad  ad2_accum = 0.0f;

	// compute weighted standard deviation
	for (int i = 0; i < NUM_SCALAR_PARTICLES; i++) {
		Point2D &pos = scalarParticles[i].pos;
		AngRad  &ang = scalarParticles[i].ang;
		float    w   = expf(getDistancePlusBearingExponent(scalarProb[i]));

		Point2D pd = pos - pos_mn;
		pd2_accum += pd * pd * w;

		AngRad  ad = absMinAngleDiff(ang, ang_mn);
		ad2_accum += ad * ad * w;
	}

	Point2D pos_var = pd2_accum * inv_total_w;
	AngRad  ang_var = ad2_accum * inv_total_w;

	Point2D pos_sd = sqrt(pos_var);
	AngRad  ang_sd = sqrt(ang_var);

	return RobotPose(pos_mn, ang_mn, pos_sd, ang_sd);
}

// Computes the weighted mean of the robot pose (location and bearing) and
// the weighted standard deviation of the robot pose.  This
// version computes the values using a standard two-pass algorithm
// which computes the mean in the first pass and the standard deviation
// in the second pass.
//
// SSE operations are used whenever possible.
static noinline
RobotPose sseEstimatePose() {
	Point2D_4Wide  pos_accum4 = Point2D_4Wide (sse4Floats::zeros(),
											   sse4Floats::zeros());
	Vector2D_4Wide ori_accum4 = Vector2D_4Wide(sse4Floats::zeros(),
											   sse4Floats::zeros());
	sse4Floats       w_accum4 = sse4Floats::zeros();

	// compute weighted mean
	for (int i = 0; i < NUM_SSE_PARTICLES; i++) {
		Point2D_4Wide &pos4 = sseParticles[i].pos;
		AngRad4       &ang4 = sseParticles[i].ang;
		sse4Floats     w4   = exp(getDistancePlusBearingExponent(sseProb[i]));

		pos_accum4 += pos4 * w4;
		ori_accum4 += Vector2D_4Wide_Polar(w4, ang4);
		w_accum4   += w4;
	}
	Point2D  pos_accum = pos_accum4.reduce_add();
	Vector2D ori_accum = ori_accum4.reduce_add();
	float      w_accum = w_accum4.reduce_add();
	assert(w_accum != 0.0f);
	assert(ori_accum.getMagnitude() != 0.0f);

	float inv_total_w = 1.0f / w_accum;

	Point2D pos_mn = pos_accum * inv_total_w;
	AngRad  ang_mn = ori_accum.getDirection();

	Point2D_4Wide pos_mn4 = Point2D_4Wide::expand(pos_mn);
	AngRad4       ang_mn4 = AngRad4::expand(ang_mn);

	Point2D_4Wide pd2_accum4 = Point2D_4Wide(sse4Floats::zeros(),
											 sse4Floats::zeros());
	AngRad4       ad2_accum4 = AngRad4::zeros();

	// compute weighted standard deviation
	for (int i = 0; i < NUM_SSE_PARTICLES; i++) {
		Point2D_4Wide &pos4 = sseParticles[i].pos;
		AngRad4       &ang4 = sseParticles[i].ang;
		sse4Floats     w4   = exp(getDistancePlusBearingExponent(sseProb[i]));

		Point2D_4Wide pd4 = pos4 - pos_mn4;
		pd2_accum4       += pd4 * pd4 * w4;

		AngRad4       ad4 = absMinAngleDiff(ang4, ang_mn4);
		ad2_accum4       += ad4 * ad4 * w4;
	}
	Point2D pd2_accum = pd2_accum4.reduce_add();
	AngRad  ad2_accum = ad2_accum4.reduce_add();

	Point2D pos_var = pd2_accum * inv_total_w;
	AngRad  ang_var = ad2_accum * inv_total_w;

	Point2D pos_sd = sqrt(pos_var);
	AngRad  ang_sd = sqrt(ang_var);

	return RobotPose(pos_mn, ang_mn, pos_sd, ang_sd);
}


//--- PARTICLE FILTER ---//

// scalar version of the particle filter
static noinline
RobotPose scalarPf() {
	clearScalarProbabilities();

	int ob = obsWindow.getBase();
	int on = obsWindow.getSize();

	for (int oi = 0; oi < on ; oi++) {
		Observation &obs = obsData[ob + oi];

		// the observed distance and bearing to the landmark
		float observedDistance = obs.d;
		AngRad observedBearing = obs.b;

		// location of the reference object
		Point2D refObjPos = REF_OBJ_POS_ARR[obs.id];

		for (int p = 0; p < NUM_SCALAR_PARTICLES; p++) {
			Particle &part = scalarParticles[p];

			// if we were at the current particle, this is the expected
			// distance and expected bearing to the landmark's known location
			float expectedDistance = part.getDistanceTo(refObjPos);
			AngRad expectedBearing = part.getBearingTo(refObjPos);

			float distanceExp = getDistanceSimExponent(expectedDistance,
													   observedDistance,
													   DIST_EXP_COEFF);

			float bearingExp = getBearingSimExponent(expectedBearing,
													 observedBearing,
													 BEAR_EXP_COEFF);

			scalarProb[p] += ProbabilityExponents(distanceExp, bearingExp);
		}
	}

	return scalarEstimatePose();
}

// SSE version of the particle filter
static noinline
RobotPose ssePf() {
	clearSseProbabilities();

	sse4Floats distExpCoeff = sse4Floats::expand(DIST_EXP_COEFF);
	sse4Floats bearExpCoeff = sse4Floats::expand(BEAR_EXP_COEFF);

	int ob = obsWindow.getBase();
	int on = obsWindow.getSize();

	for (int oi = 0; oi < on; oi++) {
		Observation &obs = obsData[ob + oi];

		// the observed distance and bearing to the landmark
		sse4Floats observedDistance = sse4Floats::expand(obs.d);
		AngRad4    observedBearing  = AngRad4::expand(obs.b);

		// location of the reference object
		Point2D_4Wide refObjPos = Point2D_4Wide::expand(REF_OBJ_POS_ARR[obs.id]);

		for (int p = 0; p < NUM_SSE_PARTICLES; p++) {
			Particle_4Wide &part = sseParticles[p];

			// if we were at the current particle, this is the expected
			// distance and expected bearing to the landmark's known location
			sse4Floats expectedDistance = part.getDistanceTo(refObjPos);
			AngRad4    expectedBearing  = part.getBearingTo(refObjPos);

			sse4Floats distanceExp = getDistanceSimExponent(expectedDistance,
															observedDistance,
															distExpCoeff);

			sse4Floats bearingExp = getBearingSimExponent(expectedBearing,
														  observedBearing,
														  bearExpCoeff);

			sseProb[p] += ProbabilityExponents_4Wide(distanceExp, bearingExp);
		}
	}

	return sseEstimatePose();
}


//--- EXTERNAL INTERFACE ---//

void seedParticleGen(unsigned int rand_seed) {
	seedRand(rand_seed);
}

void initAllParticles() {
	initScalarParticles();		// must be called first
	initSseParticles();			// must be called second
}

// loads the observation data from the given file and initializes
// the observation data array and observation data array size
void loadObservationData(const char *filename) {
	int n = 0;
	int i = 0;
	int id;
	char c;
	float d;
	float b;

	ifstream fp_in(filename, ifstream::in);
	if (!fp_in.is_open()) {
		printf("could not open \"%s\"\n", filename);
		exit(0);
	}

	// observation format:
	// each line is in the following form
	// id,x,y

	// first pass: find out how many observations there are
	while (fp_in >> id) {
		fp_in >> c;		// chomp comma
		fp_in >> d;
		fp_in >> c;		// chomp comma
		fp_in >> b;
		n++;
	}
	obsData = (Observation *)malloc16(sizeof(Observation) * n);
	obsWindow = ObservationWindow(0, 1, n);

	// second pass: instantiate each observation
	fp_in.clear();		// must appear before seekg
	fp_in.seekg(0);		// must appear after clear
	while (fp_in >> id) {
		fp_in >> c;		// chomp comma
		fp_in >> d;
		fp_in >> c;		// chomp comma
		fp_in >> b;

		checkRefObjIdRange(id);
		obsData[i++] = Observation(d, b, id);
	}
	fp_in.close();

	printf("number of observations loaded: %d\n\n", i);
}


Observation *getObservations() {
	return obsData;
}

ObservationWindow *getObservationWindow() {
	return &obsWindow;
}

Rectangle getField() {
	return FIELD;
}

Rectangle getGrass() {
	return GRASS;
}

RobotPose getActualPose() {
	return RobotPose(ROBOT_POS, ROBOT_ANGLE, Point2D(0.0f, 0.0f), 0.0f);
}

const Point2D *getReferenceObjects() {
	return REF_OBJ_POS_ARR;
}

int getNumReferenceObjects() {
	return NUM_REF_OBJS;
}

void togglePfMode() {
	pfMode = (pfMode == PF_SSE) ? PF_SCALAR : PF_SSE;
}

PfMode getPfMode() {
	return pfMode;
}

const char *getPfModeString() {
	assert(pfMode == PF_SCALAR || pfMode == PF_SSE);
	return PF_MODE_STRINGS[pfMode];
}

RobotPose runPf() {
	const char *mode;
	RobotPose pose;

	Timer t;
	t.start();

	if (pfMode == PF_SSE) {
		mode = "SSE   ";
		pose = ssePf();
	} else {
		mode = "scalar";
		pose = scalarPf();
	}

	t.stop();
	float sec = t.getElapsedSeconds();
	pfFps = 1.0f / sec;		// saved to global state

	if (pfFps < IDEAL_FPS) {
		printf("\n%s inner loop exceeded time threshold: %f sec\n\n", mode, sec);
	} else {
		printf("%s inner loop: %f sec\n", mode, sec);
	}

	return pose;
}

float getLastPfFps() {
	return pfFps;
}

ParticleArray getParticles() {
	return ParticleArray(scalarParticles, scalarProb, NUM_SCALAR_PARTICLES);
}

ParticleArray_4Wide getParticles_4Wide() {
	return ParticleArray_4Wide(sseParticles, sseProb, NUM_SSE_PARTICLES);
}

// runs both versions of the particle filter and compares their results
void comparePfResults() {
	int numObs = obsWindow.getTotal();
	obsWindow = ObservationWindow(0, min(5, numObs/2), numObs);

	if (getPfMode() == PF_SSE) {
		togglePfMode();
	}

	RobotPose scalarPose = runPf();
	togglePfMode();
	RobotPose ssePose = runPf();

	float totalDistDiff = 0.0f;
	float totalBearDiff = 0.0f;
	float maxDistDiff = 0.0f;
	float maxBearDiff = 0.0f;
	int numNans = 0;

	// compare exponents of the similarity measures
	int k = 0;		// index into standardArr
	for (int i = 0; i < NUM_SSE_PARTICLES; i++) {		// index of the 4-wide

		ProbabilityExponents_4Wide sseArrElt = sseProb[i];

		for (int j = 0; j < SSE_WIDTH; j++)  {			// index into the current 4-wide
			ProbabilityExponents a = scalarProb[k++];
			ProbabilityExponents b = sseArrElt[j];

			float distDiff = absDiff(a.distanceExp, b.distanceExp);
			float bearDiff = absDiff(a.bearingExp, b.bearingExp);

			if (isnan(distDiff + bearDiff)) {
				numNans++;
				continue;
			}

			totalDistDiff += distDiff;
			totalBearDiff += bearDiff;

			maxDistDiff = max(maxDistDiff, distDiff);
			maxBearDiff = max(maxBearDiff, bearDiff);
		}
	}

	printf("\n");
	printf("scalar vs sse comparison\n");
	printf("------------------------\n");

	int numOk = k - numNans;
	printf("per-particle similarity diff (in log-space):\n");
	printf("maxDistDiff: %f, avgDistDiff: %f\n", maxDistDiff, totalDistDiff / numOk);
	printf("maxBearDiff: %f, avgBearDiff: %f\n", maxBearDiff, totalBearDiff / numOk);
	printf("\n");

	if (numNans != 0) {
		printf("implementation is really borked, found %d NaNs!!!\n\n", numNans);
	}

	printf("scalar pose:\n");
	scalarPose.println();
	printf("\n");

	printf("SSE pose:\n");
	ssePose.println();
	printf("\n");

	printf("diff pose:\n");
	(ssePose - scalarPose).println();
	printf("\n");
}


// pf.cpp
