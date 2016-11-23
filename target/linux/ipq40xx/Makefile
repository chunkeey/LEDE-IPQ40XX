# Copyright (c) 2013 The Linux Foundation. All rights reserved.
#
include $(TOPDIR)/rules.mk

ARCH:=arm
BOARD:=ipq40xx
BOARDNAME:=Qualcomm Atheros IPQ40xx
FEATURES:=nand fpu ramdisk squashfs
CPU_TYPE:=cortex-a7
CPU_SUBTYPE:=neon-vfpv4
MAINTAINER:=Christian Lamparter <chunkeey@gmail.com>

KERNEL_PATCHVER:=4.8

KERNELNAME:=Image dtbs

include $(INCLUDE_DIR)/target.mk
DEFAULT_PACKAGES += \
	kmod-leds-gpio kmod-gpio-button-hotplug swconfig kmod-crypto-hw-qce \
	kmod-usb-core kmod-usb3 kmod-usb-dwc3 kmod-usb-dwc3-of-simple \
	kmod-usb-phy-qcom-ipq4019 kmod-usb-ledtrig-usbport \
	kmod-ath10k ath10k-firmware-qca4019 wpad-mini uboot-envtools block-mount

$(eval $(call BuildTarget))
