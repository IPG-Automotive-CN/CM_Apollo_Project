DEBUG ?= 0

CFLAGS = -Wall

ifeq ($(DEBUG), 0)
    CFLAGS    += -O3
else ifeq ($(DEBUG), 1)
	CFLAGS    += -g
endif

CC = gcc
DEF_CFLAGS += -DLINUX -D_GNU_SOURCE

CFLAGS += $(DEF_CFLAGS)

ifeq ($(OSTYPE), msys)
EXT =	.exe
LIBS =	-lws2_32
endif

rsds-client$(EXT):	rsds-client.c rsds-client.h
	$(CC) -o $@ $(CFLAGS) rsds-client.c $(LIBS)

clean:
	rm -f *.o rsds-client

