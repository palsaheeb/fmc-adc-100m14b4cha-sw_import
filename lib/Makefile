# This is not a kbuild Makefile. It is a plain Makefile so it can be copied

# If it exists includes Makefile.specific. In this Makefile, you should put
# specific Makefile code that you want to run before this. For example,
# build a particular environment.
-include Makefile.specific

# include parent_common.mk for buildsystem's defines
REPO_PARENT ?= ../..
-include $(REPO_PARENT)/parent_common.mk

ZIO ?= ../zio
ZIO_ABS ?= $(abspath $(ZIO) )

GIT_VERSION := $(shell git describe --dirty --long --tags)
ZIO_GIT_VERSION := $(shell cd $(ZIO_ABS); git describe --dirty --long --tags)

LIB = libfmcadc.a
LOBJ := route.o
LOBJ += init.o
LOBJ += boards.o
LOBJ += config-zio.o
LOBJ += buffer-zio.o
LOBJ += lib.o
LOBJ += fmc-adc-100m14b4cha.o
CFLAGS = -Wall -ggdb -O2 -fPIC -I../kernel -I$(ZIO_ABS)/include $(EXTRACFLAGS)
CFLAGS += -DGIT_VERSION="\"$(GIT_VERSION)\""
CFLAGS += -DZIO_GIT_VERSION="\"$(ZIO_GIT_VERSION)\""
LDFLAGS = -L. -lfmcadc

CC ?= $(CROSS_COMPILE)gcc

modules all: $(LIB)


%: %.c $(LIB)
	$(CC) $(CFLAGS) $*.c $(LDFLAGS) -o $@

$(LIB): $(LOBJ)
	$(AR) r $@ $^

clean:
	rm -f $(LIB) .depend *.o *~

.depend: Makefile $(wildcard *.c *.h ../*.h)
	$(CC) $(CFLAGS) -M $(LOBJ:.o=.c) -o $@

install modules_install:

-include .depend

