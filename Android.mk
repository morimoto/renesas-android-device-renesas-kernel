# Copyright (C) 2012 The CyanogenMod Project
# Copyright (C) 2016 GloballLogic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Device tree ID <board id><SiP><revision>
# Board ID:
# 0x00 Salvator-X
# 0x02 StarterKit Pro
# 0x04 Salvator-XS
# 0x05 Salvator-MS
# 0x0A Salvator-M
# 0x0B StarterKit Premier


# Android makefile to build kernel as a part of Android Build
ifeq ($(TARGET_PREBUILT_KERNEL),)

LOCAL_PATH := $(call my-dir)

KERNEL_SRC := $(TARGET_KERNEL_SOURCE)
KERNEL_DEFCONFIG := $(TARGET_KERNEL_CONFIG)

KERNEL_OUT := $(ANDROID_PRODUCT_OUT)/obj/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
KERNEL_MODULES_OUT := $(ANDROID_PRODUCT_OUT)/obj/KERNEL_MODULES

KERNEL_TARGET_BINARY := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/Image.lz4

ifeq ($(TARGET_KERNEL_MODULES_OUT),)
$(warning "TARGET_KERNEL_MODULES_OUT is not set, default path '$(KERNEL_MODULES_OUT)' used")
endif

ifeq ($(TARGET_USES_HIGHEST_DPI),true)
DTB_FOOTER := -uhd
endif

define strip-modules
    mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.order`; \
    if [ "$$mdpath" != "" ];then \
        mpath=`dirname $$mdpath`; \
        ko=`find $$mpath/kernel -type f -name *.ko`; \
        for i in $$ko; do $(ANDROID_TOOLCHAIN)/aarch64-linux-android-strip --strip-unneeded $$i; \
        done; \
    fi
endef

ifeq ($(TARGET_ARCH),arm64)
    ifneq ($(TARGET_KERNEL_CUSTOM_TOOLCHAIN),)
        ifeq ($(HOST_OS),darwin)
            ARM_CROSS_COMPILE:=CROSS_COMPILE="$(ccache) $(ANDROID_BUILD_TOP)/prebuilts/gcc/darwin-x86/aarch64/$(TARGET_KERNEL_CUSTOM_TOOLCHAIN)/bin/aarch64-linux-android-"
        else
            ARM_CROSS_COMPILE:=CROSS_COMPILE="$(ccache) $(ANDROID_BUILD_TOP)/prebuilts/gcc/linux-x86/aarch64/$(TARGET_KERNEL_CUSTOM_TOOLCHAIN)/bin/aarch64-linux-android-"
        endif
    else
        ARM_CROSS_COMPILE:=CROSS_COMPILE="$(ccache) $(ANDROID_TOOLCHAIN)/aarch64-linux-android-"
    endif
endif

ifeq ($(TARGET_KERNEL_EXT_MODULES),)
    TARGET_KERNEL_EXT_MODULES := no-external-modules
endif

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_MODULES_OUT):
	mkdir -p $(KERNEL_MODULES_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(TARGET_ARCH) $(ARM_CROSS_COMPILE) VARIANT_DEFCONFIG=$(VARIANT_DEFCONFIG) SELINUX_DEFCONFIG=$(SELINUX_DEFCONFIG) $(KCFLAGS) $(KERNEL_DEFCONFIG)

$(KERNEL_TARGET_BINARY): $(KERNEL_OUT) $(KERNEL_CONFIG) $(KERNEL_MODULES_OUT)
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(TARGET_ARCH) $(ARM_CROSS_COMPILE) $(KCFLAGS) Image.lz4 dtbs

TARGET_KERNEL_MODULES: $(KERNEL_TARGET_BINARY)
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT) ARCH=$(TARGET_ARCH) $(ARM_CROSS_COMPILE) modules
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT) ARCH=$(TARGET_ARCH) $(ARM_CROSS_COMPILE) modules_install

$(TARGET_KERNEL_EXT_MODULES) : TARGET_KERNEL_MODULES

# Below workaround for kernel modules dependencies in modules.mk:
# > $(BOARD_VENDOR_KERNEL_MODULES) : $(ANDROID_PRODUCT_OUT)/kernel
# In original code the path was hardcoded. But PRODUCT_OUT variable not
# available in modules.mk, so we must make fake target to resolve that deps.
ifneq ($(PRODUCT_OUT),$(ANDROID_PRODUCT_OUT))
$(ANDROID_PRODUCT_OUT)/kernel: $(PRODUCT_OUT)/kernel
endif

$(PRODUCT_OUT)/kernel: $(TARGET_KERNEL_EXT_MODULES) mkdtimg
	cp -v $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/Image.lz4 $(PRODUCT_OUT)/kernel
	mkdtimg create $(PRODUCT_OUT)/dtb.img --page_size=4096 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7795-es1-salvator-x$(DTB_FOOTER).dtb --id=0x00779510 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7795-es1-salvator-x$(DTB_FOOTER).dtb --id=0x00779511 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7795-salvator-x.dtb --id=0x00779520 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7796-salvator-x.dtb --id=0x00779610 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7796-salvator-x.dtb --id=0x00779611 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7796-salvator-x.dtb --id=0x00779620 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7795-salvator-xs.dtb --id=0x04779520 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7796-salvator-xs.dtb --id=0x04779610 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7796-salvator-xs.dtb --id=0x04779611 \
		$(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas/r8a7796-salvator-xs.dtb --id=0x04779620

endif # TARGET_PREBUILT_KERNEL
