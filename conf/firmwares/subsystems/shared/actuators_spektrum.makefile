
# Actuator drivers for spektrum output

ACTUATORS_SPEKTRUM_DEV ?= UART4
ACTUATORS_SPEKTRUM_DEV_UPPER =$(shell echo $(ACTUATORS_SPEKTRUM_DEV) | tr a-z A-Z)
ACTUATORS_SPEKTRUM_DEV_LOWER =$(shell echo $(ACTUATORS_SPEKTRUM_DEV) | tr A-Z a-z)

ACTUATORS_SPEKTRUM_CFLAGS  = -DACTUATORS_SPEKTRUM_DEV=$(ACTUATORS_SPEKTRUM_DEV_LOWER) -DUSE_$(ACTUATORS_SPEKTRUM_DEV_UPPER)
ACTUATORS_SPEKTRUM_CFLAGS  += -D$(ACTUATORS_SPEKTRUM_DEV_UPPER)_BAUD=115200 -DUSE_$(ACTUATORS_SPEKTRUM_DEV_UPPER)_RX=FALSE

ifdef ACTUATORS_SPEKTRUM_DEV2
ACTUATORS_SPEKTRUM_DEV2_UPPER =$(shell echo $(ACTUATORS_SPEKTRUM_DEV2) | tr a-z A-Z)
ACTUATORS_SPEKTRUM_DEV2_LOWER =$(shell echo $(ACTUATORS_SPEKTRUM_DEV2) | tr A-Z a-z)
ACTUATORS_SPEKTRUM_CFLAGS  += -DACTUATORS_SPEKTRUM_DEV2=$(ACTUATORS_SPEKTRUM_DEV2_LOWER) -DUSE_$(ACTUATORS_SPEKTRUM_DEV2_UPPER)
ACTUATORS_SPEKTRUM_CFLAGS  += -D$(ACTUATORS_SPEKTRUM_DEV2_UPPER)_BAUD=115200 -DUSE_$(ACTUATORS_SPEKTRUM_DEV2_UPPER)_RX=FALSE
endif

$(TARGET).CFLAGS += -DACTUATORS $(ACTUATORS_SPEKTRUM_CFLAGS)
$(TARGET).srcs   += subsystems/actuators/actuators_spektrum.c
