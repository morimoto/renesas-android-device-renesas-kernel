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
MAKE        := /usr/bin/make
MKDIR       := /bin/mkdir
CP          := /bin/cp

# Android makefile to build kernel as a part of Android Build
ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_DEFCONFIG            := $(TARGET_KERNEL_CONFIG)
KERNEL_SRC                  := $(TARGET_KERNEL_SOURCE)
KERNEL_OUT                  := $(abspath $(PRODUCT_OUT)/obj/KERNEL_OBJ)
KERNEL_MODULES_OUT          := $(abspath $(PRODUCT_OUT)/obj/KERNEL_MODULES)
KERNEL_CONFIG               := $(KERNEL_OUT)/.config
KERNEL_DTS_DIR              := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas
KERNEL_TARGET_BINARY        := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/Image.lz4
KERNEL_COMPILE_FLAGS        := HOSTCC=$(ANDROID_CLANG_TOOLCHAIN) CC=$(ANDROID_CLANG_TOOLCHAIN) CLANG_TRIPLE=$(BSP_GCC_CROSS_COMPILE) CROSS_COMPILE=$(BSP_GCC_CROSS_COMPILE) -j64

BOARD_PREBUILT_DTBOIMAGE    := $(KERNEL_OUT)/dtbo.img
BOARD_PREBUILT_DTBIMAGE     := $(KERNEL_OUT)/dtb.img

ifeq ($(TARGET_KERNEL_MODULES_OUT),)
$(warning "TARGET_KERNEL_MODULES_OUT is not set, default path '$(KERNEL_MODULES_OUT)' used")
endif

ifeq ($(TARGET_USES_HIGHEST_DPI),true)
DTB_FOOTER := -uhd
endif

ifeq ($(TARGET_PRODUCT),salvator)
DTB_BLOBS := \
	$(KERNEL_DTS_DIR)/android-h3-salvator-x.dtb --id=0x00779520 \
	$(KERNEL_DTS_DIR)/android-m3-salvator-x.dtb --id=0x00779610 \
	$(KERNEL_DTS_DIR)/android-m3-salvator-x.dtb --id=0x00779611 \
	$(KERNEL_DTS_DIR)/android-m3-salvator-x.dtb --id=0x00779620 \
	$(KERNEL_DTS_DIR)/android-h3-salvator-xs.dtb --id=0x04779520 \
	$(KERNEL_DTS_DIR)/android-m3-salvator-xs.dtb --id=0x04779610 \
	$(KERNEL_DTS_DIR)/android-m3-salvator-xs.dtb --id=0x04779611 \
	$(KERNEL_DTS_DIR)/android-m3-salvator-xs.dtb --id=0x04779620 \
	$(KERNEL_DTS_DIR)/android-m3n-salvator-xs.dtb --id=0x04779650
ifeq ($(H3_OPTION),4GB2x2)
DTB_BLOBS += \
	$(KERNEL_DTS_DIR)/android-h3-salvator-xs-2x2g.dtb --id=0x04779530
else
DTB_BLOBS += \
	$(KERNEL_DTS_DIR)/android-h3-salvator-xs.dtb --id=0x04779530
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
# 0x70617274 - 'part'
# 0x32300000 - '20'
# 0x33300000 - '30'
DTBO_BLOBS += \
	$(KERNEL_DTS_DIR)/r8a7795-salvator-4x2g-overlay.dtb --id=0x04779530 --custom0=0x73616c76 --custom1=0x34783267 --custom2=0x33300000 \
	$(KERNEL_DTS_DIR)/r8a7795-salvator-4x2g-overlay.dtb --id=0x04779520 --custom0=0x73616c76 --custom1=0x34783267 --custom2=0x32300000 \
	$(KERNEL_DTS_DIR)/r8a7795v3-salvator-overlay.dtb    --id=0x04779530 --custom0=0x73616c76 --custom1=0x76330000 \
	$(KERNEL_DTS_DIR)/r8a7795v2-salvator-overlay.dtb    --id=0x04779520 --custom0=0x73616c76 --custom1=0x76320000 \
	$(KERNEL_DTS_DIR)/salvator-adsp-overlay.dtb         --id=0x00779000 --custom0=0x73616c76 --custom1=0x61647370
endif

ifeq ($(TARGET_PRODUCT),ulcb)
DTB_BLOBS := \
    $(KERNEL_DTS_DIR)/android-h3ulcb.dtb --id=0x0b779520
endif

ifeq ($(TARGET_PRODUCT),kingfisher)
DTB_BLOBS := $(KERNEL_DTS_DIR)/android-h3-kingfisher.dtb --id=0x0b779520 $(KERNEL_DTS_DIR)/android-h3-kingfisher.dtb --id=0x0b779530

DTBO_BLOBS += \
	$(KERNEL_DTS_DIR)/r8a7795-h3ulcb-4x2g-overlay.dtb    --id=0x0b779530 --custom0=0x736b6b66 --custom1=0x34783267 --custom2=0x33300000 \
	$(KERNEL_DTS_DIR)/r8a7795-h3ulcb-4x2g-overlay.dtb    --id=0x0b779520 --custom0=0x736b6b66 --custom1=0x34783267 --custom2=0x32300000 \
	$(KERNEL_DTS_DIR)/r8a7795v3-h3ulcb-kf-overlay.dtb    --id=0x0b779530 --custom0=0x736b6b66 --custom1=0x76330000 \
	$(KERNEL_DTS_DIR)/r8a7795v2-h3ulcb-kf-overlay.dtb    --id=0x0b779520 --custom0=0x736b6b66 --custom1=0x76320000 \
	$(KERNEL_DTS_DIR)/ulcb-kf-adsp-overlay.dtb           --id=0x00779000 --custom0=0x736b6b66 --custom1=0x61647370
endif

# 0x6c766473 - 'lvds'
# 0x54583331 - 'TX31'
# 0x41413130 - 'AA10'
# 0x41413132 - 'AA12'
DTBO_BLOBS += \
	$(KERNEL_DTS_DIR)/partitions-overlay.dtb              --id=0x00779000 --custom0=0x72636172 --custom1=0x70617274 \
	$(KERNEL_DTS_DIR)/lvds-TX31D200VM0BAA-overlay.dtb     --id=0x00779000 --custom0=0x72636172 --custom1=0x6c766473 --custom2=0x54583331 \
	$(KERNEL_DTS_DIR)/lvds-AA104XD12-overlay.dtb          --id=0x00779000 --custom0=0x72636172 --custom1=0x6c766473 --custom2=0x41413130 \
	$(KERNEL_DTS_DIR)/lvds-AA121TD01-overlay.dtb          --id=0x00779000 --custom0=0x72636172 --custom1=0x6c766473 --custom2=0x41413132

# Include only for Renesas ones.
ifeq ($(DTB_BLOBS),)
ifneq (,$(filter $(TARGET_PRODUCT), salvator ulcb kingfisher))
$(error "DTB_BLOBS is not set for target product $(TARGET_PRODUCT)")
endif
endif

ifeq ($(TARGET_KERNEL_EXT_MODULES),)
    TARGET_KERNEL_EXT_MODULES := no-external-modules
endif

.PHONY: kernel-out-dir
kernel-out-dir:
	$(MKDIR) -p $(KERNEL_OUT)
	$(MKDIR) -p $(KERNEL_MODULES_OUT)

.PHONY: kernel-config
kernel-config: kernel-out-dir
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(TARGET_ARCH) $(KERNEL_COMPILE_FLAGS) $(KERNEL_DEFCONFIG)

.PHONY: kernel-binary
kernel-binary: kernel-out-dir kernel-config $(DTC) $(AVBTOOL) mkdtimg
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(TARGET_ARCH) $(KERNEL_COMPILE_FLAGS) $(KCFLAGS) Image.lz4 dtbs

.PHONY: kernel-installed-binary
kernel-installed-binary: kernel-binary
	$(CP) -v $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/Image.lz4 $(PRODUCT_OUT)/kernel
	mkdtimg create $(BOARD_PREBUILT_DTBOIMAGE) --page_size=4096 $(DTBO_BLOBS)
	mkdtimg create $(BOARD_PREBUILT_DTBIMAGE) --page_size=4096 $(DTB_BLOBS)
ifeq ($(BOARD_AVB_ENABLE),true)
	$(AVBTOOL) add_hash_footer \
	--image $(BOARD_PREBUILT_DTBIMAGE) \
	--partition_size $(BOARD_DTBIMAGE_PARTITION_SIZE) \
	--partition_name dtb $(INTERNAL_AVB_SIGNING_ARGS) \
	$(BOARD_AVB_DTB_ADD_HASH_FOOTER_ARGS)
endif # BOARD_AVB_ENABLE
	# $(BOARD_PREBUILT_DTBOIMAGE) will be copied by build system
	$(CP) -v $(BOARD_PREBUILT_DTBIMAGE) $(PRODUCT_OUT)/dtb.img

.PHONY: kernel-modules
kernel-modules: kernel-binary
	# clearing old kernel modules:
	find $(KERNEL_MODULES_OUT) -maxdepth 1 -type f -name '*.ko' | $(XARGS) -I{} echo {}
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT) ARCH=$(TARGET_ARCH) $(KCFLAGS) $(KERNEL_COMPILE_FLAGS) modules
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT) ARCH=$(TARGET_ARCH) $(KCFLAGS) $(KERNEL_COMPILE_FLAGS) modules_install
	find $(KERNEL_MODULES_OUT) -mindepth 2 -type f -name '*.ko' | grep "$(shell head -1 $(KERNEL_OUT)/include/config/kernel.release)" | $(XARGS) -I{} $(CP) {} $(KERNEL_MODULES_OUT)/

$(TARGET_KERNEL_EXT_MODULES) : kernel-modules

#$(PRODUCT_OUT)/boot.img: kernel-installed-binary
.PHONY: kernel-ext-modules
kernel-ext-modules: $(TARGET_KERNEL_EXT_MODULES)

endif # TARGET_PREBUILT_KERNEL
endif # TARGET_PRODUCT salvator ulcb kingfisher
