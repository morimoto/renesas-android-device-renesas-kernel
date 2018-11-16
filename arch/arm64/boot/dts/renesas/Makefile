# SPDX-License-Identifier: GPL-2.0
dtb-$(CONFIG_ARCH_R8A7795) += android-h3-salvator-x.dtb
dtb-$(CONFIG_ARCH_R8A7795) += android-h3-salvator-xs.dtb
dtb-$(CONFIG_ARCH_R8A7795) += android-h3-salvator-xs-2x2g.dtb
dtb-$(CONFIG_ARCH_R8A7795) += android-h3ulcb.dtb
dtb-$(CONFIG_ARCH_R8A7795) += android-h3ulcb-4x2g.dtb
dtb-$(CONFIG_ARCH_R8A7795) += android-h3-kingfisher.dtb
dtb-$(CONFIG_ARCH_R8A7795) += android-h3-kingfisher-4x2g.dtb
dtb-$(CONFIG_ARCH_R8A7796) += android-m3-salvator-x.dtb
dtb-$(CONFIG_ARCH_R8A7796) += android-m3-salvator-xs.dtb
dtb-$(CONFIG_ARCH_R8A77965) += android-m3n-salvator-xs.dtb

dtb-$(CONFIG_ARCH_R8A7795) += r8a7795-salvator-4x2g-overlay.dtb
dtb-$(CONFIG_ARCH_R8A7795) += r8a7795-h3ulcb-4x2g-overlay.dtb
dtb-$(CONFIG_ARCH_R8A7795) += r8a7795v2-h3ulcb-kf-overlay.dtb
dtb-$(CONFIG_ARCH_R8A7795) += r8a7795v2-salvator-overlay.dtb
dtb-$(CONFIG_ARCH_R8A7795) += r8a7795v3-h3ulcb-kf-overlay.dtb
dtb-$(CONFIG_ARCH_R8A7795) += r8a7795v3-salvator-overlay.dtb

ifeq ($(ENABLE_ADSP),true)
dtb-y += ulcb-kf-adsp-overlay.dtb
dtb-y += salvator-adsp-overlay.dtb
endif

DTC_FLAGS	+= -@
always		:= $(dtb-y)
clean-files	:= *.dtb
