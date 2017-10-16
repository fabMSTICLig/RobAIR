.PHONY: clean distclean

EXE = robairsim
OBJS = main.o gui.o arduino.o servo.o ws2812.o md49.o

CC = gcc

SIMAVR_OUT = $(shell pwd)/simavr-out
SIMAVR_LIB = $(SIMAVR_OUT)/lib/libsimavr.so
SIMAVR_PARTS_LIB = $(SIMAVR_OUT)/lib/libsimavrparts.so
SIMAVR_LIBS = .simavr-intermediate
.INTERMEDIATE: $(SIMAVR_LIBS)

CFLAGS = -I$(SIMAVR_OUT)/include -I$(SIMAVR_OUT)/include/simavr
LDFLAGS = -L$(SIMAVR_OUT)/lib -lsimavr -lsimavrparts \
		  -lpthread -lelf -lm -lGL -lglut -lutil -lSDL2 -lSDL2_gfx -lrt

ifdef DEBUG
	CFLAGS += -g -O0
	LDFLAGS += -g -O0
else
	CFLAGS += -O2
	LDFLAGS += -O2
endif

$(EXE): $(OBJS)
	$(CC) $^ $(LDFLAGS) -o $@

$(OBJS): $(SIMAVR_LIB) $(SIMAVR_PARTS_LIB)

$(SIMAVR_LIB) $(SIMAVR_PARTS_LIB): $(SIMAVR_LIBS)

$(SIMAVR_LIBS):
	make -C simavr/examples/parts
	DESTDIR=$(SIMAVR_OUT) make -C simavr -e install

clean:
	rm -f $(OBJS)

distclean: clean
	rm -f $(EXE)
	rm -rf $(SIMAVR_OUT)
	make -C simavr clean
