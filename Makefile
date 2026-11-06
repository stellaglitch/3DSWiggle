#---------------------------------------------------------------------------------
# 3DSWiggle - 3DS homebrew (devkitARM + libctru). Produces 3DSWiggle.3dsx
#---------------------------------------------------------------------------------
.SUFFIXES:

ifeq ($(strip $(DEVKITARM)),)
$(error "Please set DEVKITARM in your environment. export DEVKITARM=<path to>devkitARM")
endif

TOPDIR ?= $(CURDIR)
include $(DEVKITARM)/3ds_rules

TARGET          :=  3DSWiggle
BUILD           :=  build
SOURCES         :=  source
DATA            :=  data
INCLUDES        :=  source

APP_TITLE       :=  3DSWiggle
APP_DESCRIPTION :=  Wigglegram camera (GIF)
APP_AUTHOR      :=  stellaglitch

#---------------------------------------------------------------------------------
ARCH     :=  -march=armv6k -mtune=mpcore -mfloat-abi=hard -mtp=soft

CFLAGS   :=  -g -Wall -O3 -mword-relocations -ffunction-sections $(ARCH)
CFLAGS   +=  $(INCLUDE) -D__3DS__
CXXFLAGS :=  $(CFLAGS) -fno-rtti -fno-exceptions -std=gnu++11
ASFLAGS  :=  -g $(ARCH)
LDFLAGS   =  -specs=3dsx.specs -g $(ARCH) -Wl,-Map,$(notdir $*.map)

LIBS     :=  -lctru -lm
LIBDIRS  :=  $(CTRULIB)

#---------------------------------------------------------------------------------
ifneq ($(BUILD),$(notdir $(CURDIR)))
#---------------------------------------------------------------------------------

export OUTPUT   :=  $(CURDIR)/$(TARGET)
export TOPDIR   :=  $(CURDIR)
export VPATH    :=  $(foreach dir,$(SOURCES),$(CURDIR)/$(dir)) \
                    $(foreach dir,$(DATA),$(CURDIR)/$(dir))
export DEPSDIR  :=  $(CURDIR)/$(BUILD)

CFILES          :=  $(foreach dir,$(SOURCES),$(notdir $(wildcard $(dir)/*.c)))

export LD       :=  $(CC)
export OFILES   :=  $(CFILES:.c=.o)
export INCLUDE  :=  $(foreach dir,$(INCLUDES),-I$(CURDIR)/$(dir)) \
                    $(foreach dir,$(LIBDIRS),-I$(dir)/include) \
                    -I$(CURDIR)/$(BUILD)
export LIBPATHS :=  $(foreach dir,$(LIBDIRS),-L$(dir)/lib)

ifneq (,$(wildcard $(TOPDIR)/assets/icon.png))
    export APP_ICON := $(TOPDIR)/assets/icon.png
else
    export APP_ICON := $(CTRULIB)/default_icon.png
endif

export _3DSXFLAGS += --smdh=$(CURDIR)/$(TARGET).smdh

# CIA packaging tools (built by tools/build_cia_tools.sh into tools/bin, or on PATH)
MAKEROM    ?= $(firstword $(wildcard $(CURDIR)/tools/bin/makerom) makerom)
BANNERTOOL ?= $(firstword $(wildcard $(CURDIR)/tools/bin/bannertool) bannertool)

.PHONY: all cia clean

all: $(BUILD)
	@$(MAKE) --no-print-directory -C $(BUILD) -f $(CURDIR)/Makefile

$(BUILD):
	@mkdir -p $@

# Installable title: 3DSWiggle.cia (install with FBI or another CIA installer)
cia: all $(BUILD)/banner.bnr
	@$(MAKEROM) -f cia -o $(TARGET).cia -target t -exefslogo \
		-elf $(TARGET).elf -rsf app.rsf -icon $(TARGET).smdh -banner $(BUILD)/banner.bnr
	@echo built ... $(TARGET).cia

$(BUILD)/banner.bnr: assets/banner.png assets/audio.wav | $(BUILD)
	@$(BANNERTOOL) makebanner -i assets/banner.png -a assets/audio.wav -o $@

clean:
	@echo clean ...
	@rm -fr $(BUILD) $(TARGET).3dsx $(TARGET).smdh $(TARGET).elf $(TARGET).cia

#---------------------------------------------------------------------------------
else
#---------------------------------------------------------------------------------

DEPENDS := $(OFILES:.o=.d)

$(OUTPUT).3dsx : $(OUTPUT).elf $(OUTPUT).smdh
$(OUTPUT).elf  : $(OFILES)

-include $(DEPENDS)

#---------------------------------------------------------------------------------
endif
#---------------------------------------------------------------------------------
