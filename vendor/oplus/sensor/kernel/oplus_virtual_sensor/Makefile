ifeq ($(subst ",,$(CONFIG_MTK_SENSOR_ARCHITECTURE)),1.0)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/hwmon/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/sensorHub/inc_v1
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(CONFIG_MTK_PLATFORM)
subdir-ccflags-y += -D _OPLUS_SENSOR_HUB_VI
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/oplus_virtual_sensor/platform/sensor_cmd
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/oplus_virtual_sensor/platform/sensor_device
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensors-1.0/oplus_virtual_sensor/platform/sensor_event
else
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/mtk_nanohub
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/mtk_nanohub/nanohub
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/core
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/$(CONFIG_MTK_PLATFORM)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/include
ifeq ($(CONFIG_MTK_TINYSYS_SCP_RV_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/rv
else ifeq ($(CONFIG_MTK_TINYSYS_SCP_CM4_SUPPORT),y)
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/scp/cm4
endif
subdir-ccflags-y += -D _OPLUS_SENSOR_HUB_V2
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/oplus_virtual_sensor/platform/sensor_cmd
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/oplus_virtual_sensor/platform/sensor_device
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/sensor/2.0/oplus_virtual_sensor/platform/sensor_event
endif
obj-y += virtual_sensor.o
obj-y += platform/
obj-y += sensors/
