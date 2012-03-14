#pragma once

// memory routines

#include <malloc.h>
#include <memory.h>

#include "sys/crossplatform.h"


static forceinline void *malloc16(size_t size) {
#ifdef _WIN32
	return _aligned_malloc(size, 16);
#else
	return memalign(16, size);
#endif
}


static forceinline void free16(void *p) {
#ifdef _WIN32
	_aligned_free(p);
#else
	free(p);
#endif
}


// end of mem.h
