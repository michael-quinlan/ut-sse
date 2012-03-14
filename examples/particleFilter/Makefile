EXE = particle_filter
SRCS = Comparison.cpp Draw.cpp main.cpp pf.cpp sys/Timer.cpp
OBJS = $(SRCS:.cpp=.o)
HDRS = sys/Timer.h sys/common.h sys/crossplatform.h sys/debug.h \
       sys/mem.h sys/rand.h sys/sysMath.h sse/sse.h sse/sse4Floats.h \
       sse/sse4Ints.h sse/sseMask.h sse/sseMath.h sse/sseUtil.h \
       Angle.h Comparison.h Draw.h Geometry.h Particle.h \
       Particle_4Wide.h Point2D_4Wide.h pf.h
CC = g++
CFLAGS = -msse2 -O3 -I.
LFLAGS = -lglut

$(EXE): $(OBJS)
	$(CC) $(LFLAGS) -o $@ $(OBJS)

$(OBJS) : $(HDRS)

%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJS) $(EXE)

