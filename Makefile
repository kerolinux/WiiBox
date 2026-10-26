#---------------------------------------------------------------------------------
# WiiBox - Wii Robotics & Physics Sandbox
# Requires: devkitPPC, libogc, GRRLIB (installed in portlibs), Box2D 2.4.x source
# Place Box2D under ./box2d  (so ./box2d/include/box2d/box2d.h and ./box2d/src/... exist)
#---------------------------------------------------------------------------------
.SUFFIXES:

ifeq ($(strip $(DEVKITPPC)),)
$(error "Please set DEVKITPPC in your environment. export DEVKITPPC=<path to>devkitPPC")
endif

include $(DEVKITPPC)/wii_rules

TARGET   := WiiBox
BUILD    := build
SOURCES  := source box2d/src/collision box2d/src/common box2d/src/dynamics box2d/src/rope
INCLUDES := source

CFLAGS   = -g -O3 -Wall $(MACHDEP) $(INCLUDE)
CXXFLAGS = $(CFLAGS) -std=gnu++17 -fno-exceptions -fno-rtti
LDFLAGS  = -g $(MACHDEP) -Wl,-Map,$(notdir $@).map

LIBS     := -lgrrlib -lfreetype -lbz2 -ljpeg -lpng -lz -lfat -lwiiuse -lbte -logc -lm
LIBDIRS  := $(PORTLIBS)

#---------------------------------------------------------------------------------
ifneq ($(BUILD),$(notdir $(CURDIR)))

export OUTPUT  := $(CURDIR)/$(TARGET)
export VPATH   := $(foreach dir,$(SOURCES),$(CURDIR)/$(dir))
export DEPSDIR := $(CURDIR)/$(BUILD)

CFILES   := $(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))
CPPFILES := $(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.cpp)))

export LD     := $(CXX)
export OFILES := $(CPPFILES:.cpp=.o) $(CFILES:.c=.o)

export INCLUDE := $(foreach dir,$(INCLUDES),-iquote $(CURDIR)/$(dir)) \
                  -I$(CURDIR)/box2d/include \
                  $(foreach dir,$(LIBDIRS),-I$(dir)/include) \
                  -I$(CURDIR)/$(BUILD) -I$(LIBOGC_INC)

export LIBPATHS := -L$(LIBOGC_LIB) $(foreach dir,$(LIBDIRS),-L$(dir)/lib)

.PHONY: $(BUILD) clean run dist

$(BUILD):
	@[ -d $@ ] || mkdir -p $@
	@$(MAKE) --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile

clean:
	@echo clean ...
	@rm -fr $(BUILD) $(OUTPUT).elf $(OUTPUT).dol

run:
	wiiload $(OUTPUT).dol

dist: $(BUILD)
	@mkdir -p apps/WiiBox
	@cp $(OUTPUT).dol apps/WiiBox/boot.dol
	@echo "ready: copy the apps/WiiBox folder to sd:/apps/ (or usb:/apps/)"

#---------------------------------------------------------------------------------
else

DEPENDS := $(OFILES:.o=.d)

$(OUTPUT).dol: $(OUTPUT).elf
$(OUTPUT).elf: $(OFILES)

-include $(DEPENDS)

endif
