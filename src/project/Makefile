include ../common.mk

# flags for building the gtk library
CXXFLAGS = $(CXXFLAGS_STD) \
	$(CFLAGS_MATH) \
	$(CFLAGS_COMMON) \
	$(CFLAGS_LCMTYPES) \
	$(CFLAGS_LCM) \
	$(CFLAGS_GTK) \
	$(CFLAGS_VX_GTK) \
	$(CFLAGS_GSL) \
	-O3

LDFLAGS = $(LDFLAGS_STD) \
      $(LDFLAGS_MATH) \
      $(LDFLAGS_COMMON) \
      $(LDFLAGS_LCMTYPES) \
      $(LDFLAGS_LCM) \
      $(LDFLAGS_GSL) \
      $(LDFLAGS_MAPPING) \
	  -lasound

LIBDEPS = $(call libdeps, $(LDFLAGS))

LIB_MAPPING = $(LIB_PATH)/libmapping.a
LIBMAPPING_OBJS = occupancy_grid.o

BIN_PROJECT = $(BIN_PATH)/project

ALL = $(LIB_MAPPING) $(BIN_PROJECT)

all: $(ALL)

$(BIN_PROJECT): play_audio.o slam_main.o moving_laser_scan.o mapping.o slam.o action_model.o particle_filter.o sensor_model.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

$(LIB_MAPPING): $(LIBMAPPING_OBJS)
	@echo "    $@"
	@ar rc $@ $^

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
