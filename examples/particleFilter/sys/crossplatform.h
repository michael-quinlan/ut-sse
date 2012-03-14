#pragma once

// enables compilation across multiple platforms

#ifdef _WIN32
	// definitions for MSVS and ICC on Windows
	#define forceinline __forceinline
	#define noinline __declspec(noinline)
#else
	// definitions for GCC
	#define forceinline __attribute__((always_inline))
	#define noinline __attribute__((noinline))
	#define __debugbreak()
#endif

// end of crossplatform.h
