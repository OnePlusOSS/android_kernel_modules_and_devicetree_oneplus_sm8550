$(warning explorer makefile should only work on QCOM platform)

#For explorer on QCOM platform
#Makefile for use with Android's kernel/build system
M=$(PWD)
EXPLORER_ROOT=$(KERNEL_SRC)/$(M)

KBUILD_OPTIONS := EXPLORER_ROOT=$(EXPLORER_ROOT)

all: modules

%:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $@ $(KBUILD_OPTIONS)

modules_install:
	$(MAKE) M=$(M) -C $(KERNEL_SRC) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) clean
