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

BIN_AUDIO = $(BIN_PATH)/audio

ALL = $(LIB_MAPPING) $(BIN_PROJECT)

all: $(ALL)

$(BIN_AUDIO): play_audio.o $(LIBDEPS)
	@echo "    $@"
	@$(CXX) -o $@ $^ $(LDFLAGS) $(CXXFLAGS)

clean:
	@rm -f *.o *~ *.a
	@rm -f $(ALL)
