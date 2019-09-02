# Copyright (C) 2012 The CyanogenMod Project
# Copyright (C) 2018 GloballLogic
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

# Include only for Renesas ones.
ifneq (,$(filter $(TARGET_PRODUCT), salvator ulcb kingfisher))

XARGS       := /usr/bin/xargs
NPROC       := /usr/bin/nproc
MKDTIMG     := $(abspath ./prebuilts/misc/linux-x86/libufdt/mkdtimg)

# Android makefile to build kernel as a part of Android Build
ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_OUT                  := $(PRODUCT_OUT)/obj/KERNEL_OBJ
KERNEL_OUT_ABS              := $(abspath $(KERNEL_OUT))
KERNEL_CONFIG               := $(KERNEL_OUT)/.config
KERNEL_IMAGE_BINARY         := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/Image.lz4
KERNEL_DTB_BLOBS            := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas
KERNEL_DTBO_BLOBS           := $(KERNEL_DTB_BLOBS)/overlays

KERNEL_COMPILE_FLAGS        := HOSTCC=$(ANDROID_CLANG_TOOLCHAIN) HOSTCFLAGS="-fuse-ld=lld" HOSTLDFLAGS=-fuse-ld=lld ARCH=$(TARGET_ARCH)
KERNEL_COMPILE_FLAGS        += CC=$(ANDROID_CLANG_TOOLCHAIN) CLANG_TRIPLE=$(BSP_GCC_CROSS_COMPILE) CROSS_COMPILE=$(BSP_GCC_CROSS_COMPILE)
KERNEL_COMPILE_FLAGS        += -j `$(NPROC)`

KERNEL_MODULES              := $(KERNEL_OUT)/modules.order
KERNEL_MODULES_OUT          := $(PRODUCT_OUT)/obj/KERNEL_MODULES
KERNEL_MODULES_OUT_ABS      := $(abspath $(KERNEL_MODULES_OUT))

BOARD_PREBUILT_DTBOIMAGE    := $(KERNEL_OUT)/dtbo.img
BOARD_PREBUILT_DTBIMAGE     := $(KERNEL_OUT)/dtb.img

ifeq ($(TARGET_USES_HIGHEST_DPI),true)
DTB_FOOTER := -uhd
endif

ifeq ($(TARGET_PRODUCT),salvator)
DTB_BLOBS := \
	$(KERNEL_DTB_BLOBS)/r8a7795-salvator-x-android.dtb --id=0x00779520 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-x-android.dtb --id=0x00779610 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-x-android.dtb --id=0x00779611 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-x-android.dtb --id=0x00779613 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-x-android.dtb --id=0x00779620 \
	$(KERNEL_DTB_BLOBS)/r8a7795-salvator-xs-android.dtb --id=0x04779520 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-xs-android.dtb --id=0x04779610 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-xs-android.dtb --id=0x04779611 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-xs-android.dtb --id=0x04779613 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-xs-android.dtb --id=0x04779620 \
	$(KERNEL_DTB_BLOBS)/r8a7796-salvator-xs-2x4g-android.dtb --id=0x04779630 \
	$(KERNEL_DTB_BLOBS)/r8a77965-salvator-xs-android.dtb --id=0x04779650
ifeq ($(H3_OPTION),4GB2x2)
DTB_BLOBS += \
	$(KERNEL_DTB_BLOBS)/r8a7795-salvator-xs-2x2g-android.dtb --id=0x04779530
else
DTB_BLOBS += \
	$(KERNEL_DTB_BLOBS)/r8a7795-salvator-xs-android.dtb --id=0x04779530
endif

# Custom fields is used for identification in u-boot
# custom0 must describe for which board this overlay:
# (ASCII sumbols in hex)
# 0x73616c76 - 'salv': salvator
# 0x736b6b66 - 'skkf': SK+KF
# 0x72636172 - 'rcar': for all boards
#
# 0x34783267 - '4x2g'
# 0x76330000 - 'v3'
# 0x76320000 - 'v2'
# 0x61647370 - 'adsp'
# 0x32300000 - '20'
# 0x33300000 - '30'
DTBO_BLOBS += \
	$(KERNEL_DTBO_BLOBS)/r8a7795-salvator-4x2g-overlay.dtb --id=0x04779530 --custom0=0x73616c76 --custom1=0x34783267 --custom2=0x33300000 \
	$(KERNEL_DTBO_BLOBS)/r8a7795-salvator-4x2g-overlay.dtb --id=0x04779520 --custom0=0x73616c76 --custom1=0x34783267 --custom2=0x32300000 \
	$(KERNEL_DTBO_BLOBS)/r8a7795v3-salvator-overlay.dtb    --id=0x04779530 --custom0=0x73616c76 --custom1=0x76330000 \
	$(KERNEL_DTBO_BLOBS)/r8a7795v2-salvator-overlay.dtb    --id=0x04779520 --custom0=0x73616c76 --custom1=0x76320000 \
	$(KERNEL_DTBO_BLOBS)/salvator-adsp-overlay.dtb         --id=0x00779000 --custom0=0x73616c76 --custom1=0x61647370
endif

ifeq ($(TARGET_PRODUCT),ulcb)
DTB_BLOBS := \
    $(KERNEL_DTB_BLOBS)/r8a7795-ulcb-android.dtb --id=0x0b779520
endif

ifeq ($(TARGET_PRODUCT),kingfisher)
DTB_BLOBS := $(KERNEL_DTB_BLOBS)/r8a7795-ulcb-kf-android.dtb --id=0x0b779520 \
	$(KERNEL_DTB_BLOBS)/r8a7795-ulcb-kf-android.dtb --id=0x0b779530
DTBO_BLOBS += \
	$(KERNEL_DTBO_BLOBS)/r8a7795-h3ulcb-4x2g-overlay.dtb --id=0x0b779530 --custom0=0x736b6b66 --custom1=0x34783267 --custom2=0x33300000 \
	$(KERNEL_DTBO_BLOBS)/r8a7795-h3ulcb-4x2g-overlay.dtb --id=0x0b779520 --custom0=0x736b6b66 --custom1=0x34783267 --custom2=0x32300000 \
	$(KERNEL_DTBO_BLOBS)/r8a7795v3-h3ulcb-kf-overlay.dtb --id=0x0b779530 --custom0=0x736b6b66 --custom1=0x76330000 \
	$(KERNEL_DTBO_BLOBS)/r8a7795v2-h3ulcb-kf-overlay.dtb --id=0x0b779520 --custom0=0x736b6b66 --custom1=0x76320000 \
	$(KERNEL_DTBO_BLOBS)/ulcb-kf-adsp-overlay.dtb        --id=0x00779000 --custom0=0x736b6b66 --custom1=0x61647370
endif

# 0x6c766473 - 'lvds'
# 0x54583331 - 'TX31'
# 0x41413130 - 'AA10'
# 0x41413132 - 'AA12'
DTBO_BLOBS += \
	$(KERNEL_DTBO_BLOBS)/lvds-TX31D200VM0BAA-overlay.dtb --id=0x00779000 --custom0=0x72636172 --custom1=0x6c766473 --custom2=0x54583331 \
	$(KERNEL_DTBO_BLOBS)/lvds-AA104XD12-overlay.dtb      --id=0x00779000 --custom0=0x72636172 --custom1=0x6c766473 --custom2=0x41413130 \
	$(KERNEL_DTBO_BLOBS)/lvds-AA121TD01-overlay.dtb      --id=0x00779000 --custom0=0x72636172 --custom1=0x6c766473 --custom2=0x41413132

# Include only for Renesas ones.
ifeq ($(DTB_BLOBS),)
ifneq (,$(filter $(TARGET_PRODUCT), salvator ulcb kingfisher))
$(error "DTB_BLOBS is not set for target product $(TARGET_PRODUCT)")
endif
endif

ifeq ($(KERNEL_EXT_MODULES),)
    KERNEL_EXT_MODULES := no-external-modules
endif

$(KERNEL_OUT):
	mkdir -p $@

$(KERNEL_MODULES_OUT):
	mkdir -p $@

$(KERNEL_CONFIG) : $(KERNEL_OUT)
	$(ANDROID_MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(KERNEL_OUT_ABS) $(KERNEL_COMPILE_FLAGS) $(TARGET_KERNEL_CONFIG)

$(KERNEL_IMAGE_BINARY): $(KERNEL_CONFIG) $(DTC)
	$(ANDROID_MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(KERNEL_OUT_ABS) $(KERNEL_COMPILE_FLAGS) Image.lz4 dtbs

$(KERNEL_MODULES): $(KERNEL_IMAGE_BINARY) $(KERNEL_MODULES_OUT)
	@rm -rf $(KERNEL_MODULES_OUT_ABS)/lib/modules
	$(ANDROID_MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(KERNEL_OUT_ABS) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT_ABS) $(KERNEL_COMPILE_FLAGS) modules
	$(ANDROID_MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(KERNEL_OUT_ABS) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT_ABS) $(KERNEL_COMPILE_FLAGS) modules_install
	find $(KERNEL_MODULES_OUT_ABS) -mindepth 2 -type f -name '*.ko' | grep "$(shell head -1 $(KERNEL_OUT_ABS)/include/config/kernel.release)" | $(XARGS) -I{} mv {} $(KERNEL_MODULES_OUT_ABS)/

$(PRODUCT_OUT)/kernel: $(KERNEL_IMAGE_BINARY) $(PRODUCT_OUT)/dtb.img
	cp -v $< $@

$(KERNEL_EXT_MODULES): $(KERNEL_MODULES)
$(BOARD_VENDOR_KERNEL_MODULES): $(KERNEL_EXT_MODULES)

$(BOARD_PREBUILT_DTBIMAGE): $(KERNEL_IMAGE_BINARY) $(AVBTOOL)
	$(MKDTIMG) create $(BOARD_PREBUILT_DTBIMAGE) --page_size=4096 $(DTB_BLOBS)
	cp -v $(BOARD_PREBUILT_DTBIMAGE) $(PRODUCT_OUT)/dtb.img

$(BOARD_PREBUILT_DTBOIMAGE): $(BOARD_PREBUILT_DTBIMAGE)
	$(MKDTIMG) create $(BOARD_PREBUILT_DTBOIMAGE) --page_size=4096 $(DTBO_BLOBS)

$(PRODUCT_OUT)/dtb.img: $(BOARD_PREBUILT_DTBIMAGE)
$(PRODUCT_OUT)/dtbo.img: $(BOARD_PREBUILT_DTBOIMAGE)

endif # TARGET_PREBUILT_KERNEL
endif # TARGET_PRODUCT salvator ulcb kingfisher
