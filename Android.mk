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

# Android makefile to build kernel as a part of Android Build
ifeq ($(TARGET_PREBUILT_KERNEL),)

KERNEL_DEFCONFIG     := $(TARGET_KERNEL_CONFIG)
KERNEL_SRC           := $(TARGET_KERNEL_SOURCE)
KERNEL_OUT           := $(abspath $(PRODUCT_OUT)/obj/KERNEL_OBJ)
KERNEL_MODULES_OUT   := $(abspath $(PRODUCT_OUT)/obj/KERNEL_MODULES)
KERNEL_CONFIG        := $(KERNEL_OUT)/.config
KERNEL_DTS_DIR       := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/dts/renesas
KERNEL_TARGET_BINARY := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/Image.lz4
KERNEL_CROSS_COMPILE := $(BSP_CROSS_COMPILE)
DTB_IMG_OUT          := $(PRODUCT_OUT)/dtb.img

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

DTBO_BLOBS += $(KERNEL_DTS_DIR)/r8a7795-salvator-4x2g-overlay.dtb --id=0x04779530 \
	$(KERNEL_DTS_DIR)/r8a7795-salvator-4x2g-overlay.dtb --id=0x04779520 \
	$(KERNEL_DTS_DIR)/r8a7795v3-salvator-overlay.dtb --id=0x04779530 \
	$(KERNEL_DTS_DIR)/r8a7795v2-salvator-overlay.dtb --id=0x04779520 \
	$(KERNEL_DTS_DIR)/salvator-adsp-overlay.dtb --id=0x00779000 \
	$(KERNEL_DTS_DIR)/partitions-overlay.dtb --id=0x00779000
endif

ifeq ($(TARGET_PRODUCT),ulcb)
DTB_BLOBS := \
    $(KERNEL_DTS_DIR)/android-h3ulcb.dtb --id=0x0b779520
endif

ifeq ($(TARGET_PRODUCT),kingfisher)
DTB_BLOBS := $(KERNEL_DTS_DIR)/android-h3-kingfisher.dtb --id=0x0b779520 \
	$(KERNEL_DTS_DIR)/android-h3-kingfisher.dtb --id=0x0b779530
DTBO_BLOBS := $(KERNEL_DTS_DIR)/r8a7795-h3ulcb-4x2g-overlay.dtb --id=0x0b779530 \
	$(KERNEL_DTS_DIR)/r8a7795-h3ulcb-4x2g-overlay.dtb --id=0x0b779520 \
	$(KERNEL_DTS_DIR)/r8a7795v3-h3ulcb-kf-overlay.dtb --id=0x0b779530 \
	$(KERNEL_DTS_DIR)/r8a7795v2-h3ulcb-kf-overlay.dtb --id=0x0b779520 \
	$(KERNEL_DTS_DIR)/ulcb-kf-adsp-overlay.dtb --id=0x00779000 \
	$(KERNEL_DTS_DIR)/partitions-overlay.dtb --id=0x00779000
endif

# Include only for Renesas ones.
ifeq ($(DTB_BLOBS),)
ifneq (,$(filter $(TARGET_PRODUCT), salvator ulcb kingfisher))
$(error "DTB_BLOBS is not set for target product $(TARGET_PRODUCT)")
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
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(TARGET_ARCH) $(KERNEL_DEFCONFIG)

$(KERNEL_TARGET_BINARY): $(KERNEL_OUT) $(KERNEL_CONFIG) $(KERNEL_MODULES_OUT) $(DTC)
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(TARGET_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(KCFLAGS) Image.lz4 dtbs

TARGET_KERNEL_MODULES: $(KERNEL_TARGET_BINARY)
	# clearing old kernel modules:
	find $(KERNEL_MODULES_OUT) -maxdepth 1 -type f -name '*.ko' | xargs -I{} rm -f {}
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT) ARCH=$(TARGET_ARCH) $(KCFLAGS) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) modules
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) INSTALL_MOD_PATH=$(KERNEL_MODULES_OUT) ARCH=$(TARGET_ARCH) $(KCFLAGS) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) modules_install
	# copying kernel modules with correct vermagic:
	find $(KERNEL_MODULES_OUT) -mindepth 2 -type f -name '*.ko' | grep "$(shell head -1 $(KERNEL_OUT)/include/config/kernel.release)" | xargs -I{} cp {} $(KERNEL_MODULES_OUT)

$(TARGET_KERNEL_EXT_MODULES) : TARGET_KERNEL_MODULES

DTBO_OUT := $(TARGET_OUT_INTERMEDIATES)/DTBO
BOARD_PREBUILT_DTBOIMAGE := $(DTBO_OUT)/dtbo.img

$(DTBO_OUT):
	$(hide) mkdir -p $(DTBO_OUT)

$(BOARD_PREBUILT_DTBOIMAGE): $(TARGET_KERNEL_EXT_MODULES) $(DTBO_OUT) mkdtimg
	mkdtimg create $(BOARD_PREBUILT_DTBOIMAGE) --page_size=4096 $(DTBO_BLOBS)
	# Add padding to image for avb singing. Avbtool needs file more then 64 bytes.
ifeq ($(BOARD_AVB_ENABLE),true)
ifeq ($(DTBO_BLOBS),)
	$(hide) dd if=/dev/zero bs=1 count=64 >> $(BOARD_PREBUILT_DTBOIMAGE)
endif # DTBO_BLOBS
endif # BOARD_AVB_ENABLE
	cp -v $(BOARD_PREBUILT_DTBOIMAGE) $(PRODUCT_OUT)/dtbo.img

$(DTB_IMG_OUT): $(TARGET_KERNEL_EXT_MODULES) mkdtimg
	mkdtimg create $(PRODUCT_OUT)/dtb.img --page_size=4096 $(DTB_BLOBS)
ifeq ($(BOARD_AVB_ENABLE),true)
	$(hide) $(AVBTOOL) add_hash_footer \
	--image $(PRODUCT_OUT)/dtb.img \
	--partition_size $(BOARD_DTBIMAGE_PARTITION_SIZE) \
	--partition_name dtb $(INTERNAL_AVB_SIGNING_ARGS) \
	$(BOARD_AVB_DTB_ADD_HASH_FOOTER_ARGS)
endif # BOARD_AVB_ENABLE

$(PRODUCT_OUT)/kernel: $(BOARD_PREBUILT_DTBOIMAGE) $(DTB_IMG_OUT)
	cp -v $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/Image.lz4 $(PRODUCT_OUT)/kernel

endif # TARGET_PREBUILT_KERNEL
endif # TARGET_PRODUCT salvator ulcb kingfisher
