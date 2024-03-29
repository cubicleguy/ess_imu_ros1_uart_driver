CC=g++

# pass BUILD_FOR as string to $(CC)
DEFINES= -D$(MODEL) -DBUILD_FOR=\"$(MODEL)\"
INCLUDES= .
CFLAGS= -I$(INCLUDES) $(DEFINES) -Wall
LIBS=
DEPS= hcl.h hcl_gpio.h sensor_epsonCommon.h main_helper.h
DEPS_UART= hcl_uart.h

OBJ= main_helper.o hcl_gpio.o hcl_linux.o hcl_uart.o sensor_epsonCommon.o sensor_epsonUart.o

# If no MODEL macro is defined when calling make it defaults to G366PDG0
MODEL ?= G366PDG0

####### Adding MODEL Specific Files
ifeq ($(MODEL), G320PDG0)
	OBJ+= sensor_epsonG320.o
	DEPS+= sensor_epsonG320.h

else ifeq ($(MODEL), G330PDG0)
	OBJ+= sensor_epsonG330_G366.o
	DEPS+= sensor_epsonG330PDG0.h

else ifeq ($(MODEL), G354PDH0)
	OBJ+= sensor_epsonG354.o
	DEPS+= sensor_epsonG354.h

else ifeq ($(MODEL), G364PDC0)
	OBJ+= sensor_epsonG364.o
	DEPS+= sensor_epsonG364PDC0.h

else ifeq ($(MODEL), G364PDCA)
	OBJ+= sensor_epsonG364.o
	DEPS+= sensor_epsonG364PDCA.h

else ifeq ($(MODEL), G365PDC1)
	OBJ+= sensor_epsonG365.o
	DEPS+= sensor_epsonG365PDC1.h

else ifeq ($(MODEL), G365PDF1)
	OBJ+= sensor_epsonG365.o
	DEPS+= sensor_epsonG365PDF1.h

else ifeq ($(MODEL), G366PDG0)
	OBJ+= sensor_epsonG330_G366.o
	DEPS+= sensor_epsonG366PDG0.h

else ifeq ($(MODEL), G370PDF1)
	OBJ+= sensor_epsonG370.o
	DEPS+= sensor_epsonG370PDF1.h

else ifeq ($(MODEL), G370PDS0)
	OBJ+= sensor_epsonG370.o
	DEPS+= sensor_epsonG370PDS0.h

else ifeq ($(MODEL), G370PDG0)
	OBJ+= sensor_epsonG370.o
	DEPS+= sensor_epsonG370PDG0.h

else ifeq ($(MODEL), G370PDT0)
	OBJ+= sensor_epsonG370.o
	DEPS+= sensor_epsonG370PDT0.h

else ifeq ($(MODEL), V340PDD0)
	OBJ+= sensor_epsonV340.o
	DEPS+= sensor_epsonV340.h
endif

all: screen csvlogger regdump

%.o: %.c $(DEPS) $(DEPS_UART)
	$(CC) -c -o $@ $< $(CFLAGS) $(LIBS)

screen: $(OBJ) main_screen.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

csvlogger: $(OBJ) main_csvlogger.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

regdump: $(OBJ) main_regdump.o
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean all tar help

clean:
	rm -f $(OBJ) *~ core *~ *.o
	rm -f csvlogger screen regdump

tar:
	tar cvzf archive.tar.gz *.c *.h readme.txt Makefile

help:
	@echo "supported make commands are:"
	@echo "\tmake clean"
	@echo "\tmake <targets> MODEL=<model>\n"
	@echo "valid <targets> are: all csvlogger screen or regdump\n"
	@echo "valid <models> are:"
	@echo "\tG354PDH0 G364PDC0 G364PDCA G320PDG0 V340PDD0"
	@echo "\tG365PDC1 G365PDF1 G370PDF1 G370PDS0"
	@echo "\tG330PDG0 G366PDG0(default) G370PDG0 G370PDT0\n"
	@echo "example:\n\tmake csvlogger MODEL=G364PDC0"
	@echo "\tmake screen (defaults to MODEL=G366PDG0)"
