ifeq ($(CONFIG_OPLUS_SYSTEM_KERNEL_QCOM),y)
$(warning explorer uses kbuild for QCOM)

##########################################################################
#For explorer on QCOM platform

ccflags-y := -Werror
ccflags-y += -DSLT_ENABLE
ccflags-y += -DOPLUS_EXPLORER_PLATFORM_QCOM
ifneq ($(CONFIG_BOARD),RASPBERRYPI)
    ccflags-y += -DQCOM_AON
endif

ifeq ($(CONFIG_BOARD),RASPBERRYPI)
    ccflags-y += -DZEKU_EXPLORER_PLATFORM_RPI
endif

ifeq ($(CONFIG_ARCH_KALAMA), y)
    ccflags-y += -DOPLUS_EXPLORER_NO_PROJECT
endif

obj-m += explorer.o
explorer-y := explorer/main.o \
    explorer/spi.o \
    explorer/ipc.o \
    explorer/isp.o \
    explorer/irq.o \
    explorer/sdio_pi.o \
    explorer/power.o \
    explorer/ap_boot.o \
    explorer/exception.o \
    explorer/rtt_debug.o \
    slt/slt.o \
    slt/case/sdio_test_case.o \
    slt/slt_loader.o

ifneq ($(CONFIG_BOARD),RASPBERRYPI)
     explorer-y += aon/aon_sensor_common.o
     explorer-y += aon/qcom/qcom_aon_sensor_dev.o
     explorer-y += aon/qcom/qcom_aon_sensor_io.o
     explorer-y += aon/qcom/qcom_aon_soc_util.o
endif

BOARD_VENDOR_KERNEL_MODULES += $(KERNEL_MODULES_OUT)/explorer.ko

#End for QCOM
##########################################################################

else
$(warning explorer uses kbuild for MTK)

##########################################################################
#For explorer on MTK platform

# Necessary Check
ifneq ($(KERNEL_OUT),)
    ccflags-y += -imacros $(KERNEL_OUT)/include/generated/autoconf.h
endif

# Force build fail on modpost warning
KBUILD_MODPOST_FAIL_ON_WARNINGS := y

ccflags-y := -Werror
ccflags-y += -DSLT_ENABLE
ccflags-y += -I$(srctree)/drivers/misc/mediatek/clkbuf/v1/inc \
             -I$(srctree)/drivers/misc/mediatek/clkbuf/v1/src

ifneq ($(CONFIG_BOARD),RASPBERRYPI)
    ccflags-y += -DMTK_AON
endif

MODULE_NAME := explorer
obj-m += $(MODULE_NAME).o
$(warning $(MODULE_NAME) is kernel module)

explorer-y := explorer/main.o \
    explorer/spi.o \
    explorer/ipc.o \
    explorer/isp.o \
    explorer/irq.o \
    explorer/sdio_pi.o \
    explorer/power.o \
    explorer/ap_boot.o \
    explorer/exception.o \
    explorer/rtt_debug.o \
    slt/slt.o \
    slt/case/sdio_test_case.o \
    slt/slt_loader.o

ifneq ($(CONFIG_BOARD),RASPBERRYPI)
     explorer-y += aon/aon_sensor_common.o
     explorer-y += aon/mtk/mtk_aon_sensor_dev.o
     explorer-y += aon/mtk/mtk_aon_sensor_io.o
     explorer-y += aon/mtk/mtk_aon_soc_util.o
endif

#End for MTK
##########################################################################

endif
