# include parent_common.mk for buildsystem's defines
# use absolute path for REPO_PARENT
CURDIR:=$(shell /bin/pwd)
REPO_PARENT ?= $(CURDIR)/..
-include $(REPO_PARENT)/parent_common.mk

all: kernel tools lib libtools

FMC_BUS ?= fmc-bus
ZIO = zio
SPEC_SW ?= spec-sw
SVEC_SW ?= svec-sw

# Use the absolute path so it can be used by submodule
# FMC_BUS_ABS, ZIO_ABS, SPEC_SW_ABS and SVEC_SW_ABS has to be absolut path,
# due to beeing passed to the Kbuild
FMC_BUS_ABS ?= $(abspath $(FMC_BUS) )
ZIO_ABS ?= $(abspath $(ZIO) )
SPEC_SW_ABS ?= $(abspath $(SPEC_SW) )
SVEC_SW_ABS ?= $(abspath $(SVEC_SW) )

export FMC_BUS_ABS
export ZIO_ABS
export SPEC_SW_ABS
export SVEC_SW_ABS

DIRS = $(FMC_BUS_ABS) $(ZIO_ABS) $(SPEC_SW_ABS) kernel tools lib libtools

$(SPEC_SW_ABS): $(FMC_BUS_ABS)
kernel: $(FMC_BUS_ABS) $(ZIO_ABS) $(SPEC_SW_ABS)
lib: $(ZIO_ABS)
tools libtools: lib
# we take only headers from svec-sw, no need to compile
kernel: svec-sw-init_repo

.PHONY: all clean modules install modules_install $(DIRS)
.PHONY: gitmodules prereq_install prereq_install_warn

install modules_install: prereq_install_warn

all clean modules install modules_install: $(DIRS)

clean: TARGET = clean
modules: TARGET = modules
install: TARGET = install
modules_install: TARGET = modules_install


$(DIRS):
	$(MAKE) -C $@ $(TARGET)


SUBMOD = $(FMC_BUS_ABS) $(ZIO_ABS) $(SPEC_SW_ABS)

prereq_install_warn:
	@test -f .prereq_installed || \
		echo -e "\n\n\tWARNING: Consider \"make prereq_install\"\n"

prereq_install:
	for d in $(SUBMOD); do $(MAKE) -C $$d modules_install || exit 1; done
	touch .prereq_installed

$(FMC_BUS_ABS): fmc-bus-init_repo
$(ZIO_ABS): zio-init_repo
$(SPEC_SW_ABS): spec-sw-init_repo

# init submodule if missing
fmc-bus-init_repo:
	@test -d $(FMC_BUS_ABS)/doc || ( echo "Checking out submodule $(FMC_BUS_ABS)" && git submodule update --init $(FMC_BUS_ABS) )

# init submodule if missing
zio-init_repo:
	@test -d $(ZIO_ABS)/doc || ( echo "Checking out submodule $(ZIO_ABS)" && git submodule update --init $(ZIO_ABS) )

# init submodule if missing
spec-sw-init_repo:
	@test -d $(SPEC_SW_ABS)/doc || ( echo "Checking out submodule $(SPEC_SW_ABS)" && git submodule update --init $(SPEC_SW_ABS) )

# init submodule if missing
svec-sw-init_repo:
	@test -d $(SVEC_SW_ABS)/doc || ( echo "Checking out submodule $(SVEC_SW_ABS)" && git submodule update --init $(SVEC_SW_ABS) )
