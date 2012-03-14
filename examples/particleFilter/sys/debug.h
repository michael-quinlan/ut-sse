#pragma once

// debugging support

#include <stdio.h>
#include <stdlib.h>

#include "sys/crossplatform.h"


//--- COMPILE-TIME OPTIONS ---//

// set _FORCE_ASSERTS to turn on asserts regardless of the build mode
#define _FORCE_ASSERTS        0


//--- MACROS ---//

#define _SIMP_STRING2(arg) #arg
#define _SIMP_STRING(arg) _SIMP_STRING2(arg)
#define _FILE_LINE __FILE__"("_SIMP_STRING(__LINE__)")"

#ifdef _DEBUG
	#define __ASSERTS_ENABLED 1
#elif _FORCE_ASSERTS
	#define __ASSERTS_ENABLED 1
#else
	#define __ASSERTS_ENABLED 0
#endif

#if __ASSERTS_ENABLED
	#define assert(exp) do { if (!(exp)) { fprintf(stderr, "assert failed: %s in %s\n", #exp, _FILE_LINE); __debugbreak(); exit(1); } } while(false)
#else
	#define assert(exp)
#endif


//--- FUNCTIONS ---//

static forceinline void error(const char *msg) {
	printf("%s\n", msg);
	__debugbreak();
	exit(1);
}

static forceinline void dieIf(bool b, const char *msg) {
	if (b) {
		error(msg);
	}
}

// end of debug.h
