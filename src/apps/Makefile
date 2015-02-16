include ../common.mk

# flags for building the gtk library
CFLAGS = $(CFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_A1) \
	$(CFLAGS_GSL) \
	-O2

LDFLAGS = $(LDFLAGS_STD) \
	$(LDFLAGS_VX_GTK) \
	$(LDFLAGS_IMAGESOURCE) \
	$(LDFLAGS_MATH) \
	$(LDFLAGS_COMMON) \
	$(LDFLAGS_LCMTYPES) \
	$(LDFLAGS_MAPPING) \
	$(LDFLAGS_A1) \
	$(LDFLAGS_GSL)

CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_IMAGESOURCE) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_A1) \
	$(CFLAGS_GSL) \
	-O2

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_EECS467 = $(LIB_PATH)/libeecs467.a
LIBEECS467_OBJS = eecs467_util.o

BIN_EECS467_GUI_EXAMPLE = $(BIN_PATH)/eecs467_gui_example
BIN_EECS467_REXARM_EXAMPLE = $(BIN_PATH)/eecs467_rexarm_example
BIN_EECS467_MAEBOT_EXAMPLE = $(BIN_PATH)/eecs467_maebot_example
BIN_EECS467_MAEBOT_VIDEO_TELOP = $(BIN_PATH)/eecs467_maebot_video_telop
BIN_EECS467_MAP_GUI = $(BIN_PATH)/eecs467_map_gui
BIN_EECS467_MAPPER = $(BIN_PATH)/eecs467_mapper
BIN_EECS_PARTICLE_GUI = $(BIN_PATH)/eecs_particle_gui
BIN_EECS467_SLAM = $(BIN_PATH)/eecs467_slam
BIN_EECS467_SLAM_MANUAL = $(BIN_PATH)/eecs467_slam_manual

ALL = $(LIB_EECS467) $(BIN_EECS467_GUI_EXAMPLE) $(BIN_EECS467_REXARM_EXAMPLE) \
	  $(BIN_EECS467_MAEBOT_EXAMPLE) $(BIN_EECS467_MAEBOT_VIDEO_TELOP) \
	  $(BIN_EECS467_MAP_GUI) $(BIN_EECS467_MAPPER) \
	  $(BIN_EECS_PARTICLE_GUI) \
	  $(BIN_EECS467_SLAM) \
	  $(BIN_EECS467_SLAM_MANUAL)

all: $(ALL)

$(LIB_EECS467): $(LIBEECS467_OBJS) $(LIBDEPS)
	@echo "\t$@"
	@ar rc $@ $(LIBEECS467_OBJS)

$(BIN_EECS467_GUI_EXAMPLE): gui_example.o $(LIB_EECS467) $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS) $(LIB_EECS467)

$(BIN_EECS467_REXARM_EXAMPLE): rexarm_example.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_EXAMPLE): maebot_example.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAEBOT_VIDEO_TELOP): maebot_video_teleop.o $(LIBDEPS)
	@echo "\t$@"
	@$(CC) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAP_GUI): eecs467_map_gui.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS_PARTICLE_GUI): eecs_particle_gui.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_MAPPER): eecs467_mapper.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_SLAM): eecs467_slam.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)

$(BIN_EECS467_SLAM_MANUAL): eecs467_slam_manual.o $(LIBDEPS)
	@echo "\t$@"
	@$(CXX) -o $@ $^ $(LDFLAGS)	

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
