#ifdef CONFIG_OPLUS_SECURE_GUARD
ifeq ($(TARGET_BUILD_VARIANT),userdebug)
ccflags-y += -DWHITE_LIST_SUPPORT
endif
obj-$(CONFIG_OPLUS_ROOT_CHECK)	 +=  oplus_root_check.o
obj-$(CONFIG_OPLUS_EXECVE_BLOCK)	 +=  oplus_exec_block.o
obj-$(CONFIG_OPLUS_MOUNT_BLOCK)	 +=  oplus_mount_block.o
obj-$(CONFIG_OPLUS_SECURE_GUARD)	 +=  oplus_guard_general.o
obj-$(CONFIG_OPLUS_SECURE_GUARD)	 +=  oplus_hook_syscall.o
#endif /* CONFIG_OPLUS_SECURE_GUARD */
