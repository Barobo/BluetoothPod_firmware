##############################################################################
#CONFIG = Debug
CONFIG = Release

##############################################################################
.PHONY: all directory clean size

STACK_PATH = LwMesh_1_0_0
APP_PATH = .

LFUSE = 0xFF

CC = avr-gcc
OBJCOPY = avr-objcopy
SIZE = avr-size

CFLAGS += -W -Wall --std=gnu99 -Os -fno-zero-initialized-in-bss
CFLAGS += -fdata-sections -ffunction-sections -fpack-struct -fshort-enums
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mmcu=atmega328
CFLAGS += -MD -MP -MT $(CONFIG)/$(*F).o -MF $(CONFIG)/$(@F).d

ifeq ($(CONFIG), Debug)
  CFLAGS += -g
endif

RFTESTCFLAGS += -W -Wall --std=gnu99 -Os -fno-zero-initialized-in-bss
RFTESTCFLAGS += -fdata-sections -ffunction-sections -fpack-struct -fshort-enums
RFTESTCFLAGS += -funsigned-char -funsigned-bitfields
RFTESTCFLAGS += -mmcu=atmega328
RFTESTCFLAGS += -MD -MP -MT $(CONFIG)/$(*F).o 

ifeq ($(CONFIG), Debug)
  RFTESTCFLAGS += -g
endif

LDFLAGS += -Wl,--gc-sections #-Wl,-section-start=.ZigbeeFrameSection=0x180
#LDFLAGS += -Wl,-section-start=.ZigbeeFrameSection=0x180
#LDFLAGS += -Wl,-section-start=.init1=0x200
LDFLAGS += -mmcu=atmega328

LIBS       = -lm 


INCLUDES += \
  -I$(APP_PATH) 

SRCS += \
  main.c \
  twi.c \
  serial.c \
  timer.c

DEFINES += \
  -DPHY_ATtiny2313 \
  -DHAL_ATtiny2313 \
  -DPLATFORM_RCB128RFA1 \
  -DF_CPU=8000000

CFLAGS += $(INCLUDES) $(DEFINES)
RFTESTCFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(CONFIG)/, $(notdir %/$(subst .c,.o, $(SRCS))))

all: directory $(CONFIG)/main.elf $(CONFIG)/main.hex $(CONFIG)/main.bin size

$(CONFIG)/main.elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(CONFIG)/main.hex: $(CONFIG)/main.elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex -R .eeprom $^ $@

$(CONFIG)/main.bin: $(CONFIG)/main.elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary -R .eeprom $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@mkdir -p $(CONFIG)

size: $(CONFIG)/main.elf
	@echo size:
	@$(SIZE) -t $^

ctags:
	ctags $(SRCS)  *.h \
  $(STACK_PATH)/hal/atmega128rfa1/inc/* \
  $(STACK_PATH)/phy/atmega128rfa1/inc/* \
  $(STACK_PATH)/nwk/inc/* \
  $(STACK_PATH)/sys/inc/* *.c

loadflash0:
	avrdude -c avrisp2 -P /dev/ttyACM0 -p m328p -e -U fl:w:$(CONFIG)/main.hex -U lfuse:w:0xff:m

loadflash1:
	avrdude -c avrisp2 -P /dev/ttyACM1 -p m328p -e -U fl:w:$(CONFIG)/main.hex -U lfuse:w:0xff:m

loadflash2:
	avrdude -c avrisp2 -P /dev/ttyACM2 -p m328p -e -U fl:w:$(CONFIG)/main.hex -U lfuse:w:0xff:m

loadflash3:
	avrdude -c avrisp2 -P /dev/ttyACM3 -p m328p -e -U fl:w:$(CONFIG)/main.hex -U lfuse:w:0xff:m
#avrdude -c avrispv2 -P /dev/ttyACM0 -p m128rfa1 -e -U lfuse:w:$(LFUSE):m -b 19200 

rftest0:
	$(CC) $(RFTESTCFLAGS) -DDEVICE=0 rftest.c -c -o rftest.o
	$(CC) $(LDFLAGS) rftest.o -o rftest.elf
	$(OBJCOPY) -O ihex -R .eeprom rftest.elf rftest.hex

rftest1:
	$(CC) $(RFTESTCFLAGS) -DDEVICE=1 rftest.c -c -o rftest.o
	$(CC) $(LDFLAGS) rftest.o -o rftest.elf
	$(OBJCOPY) -O ihex -R .eeprom rftest.elf rftest.hex

loadrftest0:
	avrdude -c arduino -P /dev/ttyACM0 -p m128rfa1 -e -U fl:w:rftest.hex -b 57600
loadrftest1:
	avrdude -c arduino -P /dev/ttyACM1 -p m128rfa1 -e -U fl:w:rftest.hex -b 57600
loadrftest2:
	avrdude -c arduino -P /dev/ttyACM2 -p m128rfa1 -e -U fl:w:rftest.hex -b 57600

clean:
	@echo clean
	@-rm -rf $(CONFIG)
	@rm -rf rftest.o rftest.elf rftest.hex

-include $(wildcard $(CONFIG)/*.d)
