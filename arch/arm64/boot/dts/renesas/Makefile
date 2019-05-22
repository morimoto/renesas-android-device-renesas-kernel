# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_ARCH_R8A7795) +=  r8a7795-salvator-x-android.dtb
dtb-$(CONFIG_ARCH_R8A7795) +=  r8a7795-salvator-xs-2x2g-android.dtb
dtb-$(CONFIG_ARCH_R8A7795) +=  r8a7795-salvator-xs-android.dtb
dtb-$(CONFIG_ARCH_R8A7795) +=  r8a7795-ulcb-4x2g-android.dtb
dtb-$(CONFIG_ARCH_R8A7795) +=  r8a7795-ulcb-android.dtb
dtb-$(CONFIG_ARCH_R8A7795) +=  r8a7795-ulcb-kf-4x2g-android.dtb
dtb-$(CONFIG_ARCH_R8A7795) +=  r8a7795-ulcb-kf-android.dtb
dtb-$(CONFIG_ARCH_R8A7796) +=  r8a7796-salvator-x-android.dtb
dtb-$(CONFIG_ARCH_R8A7796) +=  r8a7796-salvator-xs-android.dtb
dtb-$(CONFIG_ARCH_R8A77965) += r8a77965-salvator-xs-android.dtb

dts-dirs	:= overlays
subdir-y	:= $(dts-dirs)

DTC_FLAGS	+= -@ -p 1024
always		:= $(dtb-y)
clean-files	:= *.dtb
