# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT
CURDIR:=$(shell /bin/pwd)
REPO_PARENT ?= $(CURDIR)/../..
-include $(REPO_PARENT)/parent_common.mk

LINUX ?= /lib/modules/$(shell uname -r)/build
FMC_BUS ?= ../fmc-bus
ZIO ?= ../zio
SPEC_SW ?= ../spec-sw
SVEC_SW ?= ../svec-sw

# FMC_BUS_ABS, ZIO_ABS, SPEC_SW_ABS and SVEC_SW_ABS has to be absolut path,
# due to beeing passed to the Kbuild
FMC_BUS_ABS ?= $(abspath $(FMC_BUS) )
ZIO_ABS ?= $(abspath $(ZIO) )
SPEC_SW_ABS ?= $(abspath $(SPEC_SW) )
SVEC_SW_ABS ?= $(abspath $(SVEC_SW) )

GIT_VERSION = $(shell git describe --always --dirty --long --tags)
export GIT_VERSION

FMC_BUS_VERSION ?= $(shell cd $(FMC_BUS_ABS); git describe --always --dirty --long --tags)
ZIO_VERSION     ?= $(shell cd $(ZIO_ABS);     git describe --always --dirty --long --tags)
SPEC_SW_VERSION ?= $(shell cd $(SPEC_SW_ABS); git describe --always --dirty --long --tags)
SVEC_SW_VERSION ?= $(shell cd $(SVEC_SW_ABS); git describe --always --dirty --long --tags)

export FMC_BUS_VERSION
export ZIO_VERSION
export SPEC_SW_VERSION
export SVEC_SW_VERSION

all modules:
	$(MAKE) -C $(LINUX) M=$(CURDIR) FMC_BUS_ABS=$(FMC_BUS_ABS) \
		ZIO_ABS=$(ZIO_ABS) SPEC_SW_ABS=$(SPEC_SW_ABS) \
		SVEC_SW_ABS=$(SVEC_SW_ABS) modules

install modules_install: modules
	$(MAKE) -C $(LINUX) M=$(CURDIR) modules_install

# be able to run the "clean" rule even if $(LINUX) is not valid
clean:
	rm -rf *.o *~  .*.cmd *.ko *.mod.c .tmp_versions Module.symvers \
		Module.markers modules.order
