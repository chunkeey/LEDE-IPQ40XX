# Copyright (c) 2014 The Linux Foundation. All rights reserved.
#
include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

UBIFS_OPTS = -m 2048 -e 124KiB -c 4096 -U -F

DEVICE_VARS += DTB_SIZE

KERNEL_LOADADDR := 0x80208000

define Build/dtb
        $(call Image/BuildDTB,../dts/$(DEVICE_DTS).dts,$@.dtb,,--space $(DTB_SIZE))
endef

define Device/Default
	DEVICE_DTS :=
	DEVICE_DTS_DIR := ../dts
	PROFILES := Default
	KERNEL_SUFFIX := -fit-uImage.itb
	KERNEL_DEPENDS = $$(wildcard $(DTS_DIR)/$$(DEVICE_DTS).dts)
	KERNEL_INITRAMFS_PREFIX := $$(IMG_PREFIX)-$(1)-initramfs
	KERNEL_PREFIX := $$(IMAGE_PREFIX)
	KERNEL_LOADADDR := 0x80208000
	SUPPORTED_DEVICES = $$(DEVICE_DTS)
endef

define Build/ramdisk
        ( cd $(TARGET_DIR); find . | cpio -o -H newc | gzip -9n > $(KDIR)/root.cpiogz )
endef

define Build/export-dtb
        cp $(IMAGE_KERNEL).dtb $@
endef

# $(1): FIT used kernel compression
# $(2): FIT used ramdisk compression
# $(3): FIT used ramdisk file
define Build/fit-ramdisk
	# create FIT image with and without ramdisk
	./mkits.sh \
		-D $(DEVICE_NAME) -o $@.its -k $@ -d $@.dtb \
		-C $(word 1,$(1)) -a $(KERNEL_LOADADDR) -A $(ARCH) -v $(LINUX_VERSION) \
		-e $(if $(KERNEL_ENTRY),$(KERNEL_ENTRY),$(KERNEL_LOADADDR)) \
		$(if $(word 3,$(1)),-r $(KDIR)/$(word 3,$(1)) -z $(if $(word 2,$(1)),none))
	PATH=$(LINUX_DIR)/scripts/dtc:$(PATH) mkimage -f $@.its $@.new
	@mv $@.new $@
endef

$(eval $(call BuildImage))
