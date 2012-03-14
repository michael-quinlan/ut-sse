#pragma once

// random number generation

#include <stdlib.h>

#include "sys/crossplatform.h"

// seed the random number generator, uses srand()
static forceinline void seedRand(unsigned int i) {
	srand(i);
}

// generates a random float from the interval [base, base + span),
// used rand()
static forceinline float getRand(float base, float span) {
	float nf = rand() / ((float)RAND_MAX + 1.0f);
	return base + span * nf;
}

// end of rand.h
