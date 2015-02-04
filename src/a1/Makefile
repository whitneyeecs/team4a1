include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) -O3 -DNDEBUG
CXXFLAGS = $(CXXFLAGS_STD) -O3 -DNDEBUG
LDFLAGS = $(LDFLAGS_STD) 

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_A1 = $(LIB_PATH)/liba1.a
LIB_A1_OBJS = StateEstimator.o VirtualOdometry.o LaserCorrector.o \
	Mapper.o ParticleFilter.o

ALL = $(LIB_A1)

all: $(ALL)

$(LIB_A1): $(LIB_A1_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
