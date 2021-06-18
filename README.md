Both the SSE wrapper library and the Particle Filter example are described in the [paper](http://www.cs.utexas.edu/~pstone/Papers/bib2html-links/IROS09-djeu.pdf) :

```
Improving Particle Filter Performance Using SSE Instructions, 
Peter Djeu, Michael Quinlan, and Peter Stone. 
In Proceedings of the 2009 IEEE International Conference on Intelligent Robots and Systems (IROS 2009), 
October 2009. 
```

See README.txt in each directory for usage information.

### SSE Wrapper Library ###
This is designed to be a lightweight SSE wrapper library. Primarily it was designed to provide SSE operators that are useful in robotics, such as sin,cos,atan,atan2,exp,abs etc. The SSE operations achieve up to a 4x speed up over the standard C++ operators.

| Function | Scalar (sec) | SSE (sec) | Speedup | 
|-------------|:-----------------|:--------------:|:------------:| 
| exp | 19.501226 | 4.967288 | 3.9259 | 
| atan | 38.086196 | 20.303419 | 1.8769 | 
| atan2 | 65.351570 | 28.582232 | 2.2644 |

SSE Math Results (operations used in the paper): 
 - exp tests all floats between (-80.0, 80.0). 
 - atan tests all floats between (-INF , INF).
 - atan2 tests all pairs of floats on the unit circle.

### SSE Particle Filter example ###
This example illustrate how a substantial run-time gain can be achieved by taking advantage of the extended instruction sets found in modern processors, in particular the SSE1 and SSE2 instruction sets. An SSE version of Monte Carlo Localization is demonstrated and it results in an impressive 9x speedup over the standard scalar implementation.

| n | Scalar (sec) | SSE (sec) | Speedup | Error (mm) | Error (rad) | 
|--------|:-----------------|:--------------:|:------------:|:---------------:|:----------------:| 
| 4 | 0.000023 | 0.000003 | 7.7 | N/A | N/A | 
| 16 | 0.000028 | 0.000003 | 9.3 | N/A | N/A | 
| 64 | 0.000047 | 0.000006 | 7.8 | 1590.0 | 0.513 | 
| 256 | 0.000116 | 0.000015 | 7.7 | 820.0 | 0.307 | 
| 1024 | 0.000403 | 0.000053 | 7.6 | 359.6 | 0.155 | |
4096 | 0.001607 | 0.00020 | 7.7 | 171.0 | 0.0742 | 
| 16384 | 0.006647 | 0.000831 | 8.0 | 109.0 | 0.0447 | 
| 65536 | 0.031178 | 0.003308 | 9.4 | 108.0 | 0.0446 | 
| 262144 | 0.125763 | 0.013664 | 9.2 | 110.0 | 0.0450 | |
1048576 | 0.460547 | 0.057089 | 8.1 | 109.0 | 0.0446 |

Scaling results over n particles: Error is the average error over 100 random configurations of particles.

#### Prerequisites ####

*Linux*

g++. Tested with g++ ver. 4.3.3.

OpenGL and glut must be installed. On Ubuntu, get the "freeglut3-dev" package.

*Windows*

Microsoft Visual Studio 2005 or 2008
