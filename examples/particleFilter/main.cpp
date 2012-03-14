// program entry point

#include <stdio.h>

#include "sse/sse.h"

#include "pf.h"
#include "Comparison.h"
#include "Draw.h"


static const char *OBS_FILENAME = "sim_obs.csv";


int main(int argc, char **argv) {
	SSE::init();

	seedParticleGen(1);		// use a fixed random seed under normal runs

	initAllParticles();

	loadObservationData(OBS_FILENAME);

#if 0
	// test for accuracy in console mode
	comparePfResults();
#elif 0
	// compare SSE implementations of complex math functions
	compareAbs();
	compareExp();
	compareSin();
	compareCos();
	compareAtan();
	compareAtan2();
//	compareOldAtan2();
#else
	// use the graphical viewer
	initWindow(argc, argv);
	enterDrawLoop();		// does not return
#endif

	return 0;
}


// main.cpp
