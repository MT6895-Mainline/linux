# SPDX-License-Identifier: GPL-2.0-only
ccflags-y += -I$(src)			# needed for trace events

# Android common kernels build the binder core as the composite
# "binder_linux" module so GKI vendor modules can depend on it; keep
# that layout here as well.
binder_linux-y := binder.o binder_alloc.o binder_netlink.o
binder_linux-$(CONFIG_ANDROID_BINDERFS)	+= binderfs.o
obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder_linux.o
obj-$(CONFIG_ANDROID_BINDER_ALLOC_KUNIT_TEST)	+= tests/
obj-$(CONFIG_ANDROID_BINDER_IPC_RUST)	+= binder/
