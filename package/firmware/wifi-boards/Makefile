include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/version.mk

PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
endef

define Build/Compile
endef

ALLWIFIBOARDS:=
ALLWIFIPACKAGES:=$(foreach BOARD,$(ALLWIFIBOARDS),wifi-board-$(BOARD))

define Package/wifi-board-default
  SUBMENU:=ath10k IPQ4019 Boarddata
  SECTION:=firmware
  CATEGORY:=Firmware
  DEPENDS:=+ath10k-firmware-qca4019
  TITLE:=Custom Board
endef

define generate-wifi-board-package
  define Package/wifi-board-$(1)
    $(call Package/wifi-board-default)
    TITLE:=Board for $(3)
    CONFLICTS:=$(PREV_BOARD)
#   CONFLICTS=$(filter-out wifi-board-$(1),$(ALLWIFIPACKAGES))
  endef

  define Package/wifi-board-$(1)/description
This device custom package board-2.bin overwrites the board-2.bin
file which is supplied by the ath10k-firmware-qca4019 package.

This is package is only necessary for the $(3).
Don't install it for any other device!
  endef

  define Package/wifi-board-$(1)/install-overlay
	$(INSTALL_DIR) $$(1)/lib/firmware/ath10k/QCA4019/hw1.0
	$(INSTALL_DATA) ./$(2) $$(1)/lib/firmware/ath10k/QCA4019/hw1.0/board-2.bin
  endef

  PREV_BOARD+=wifi-board-$(1)
endef

$(foreach PACKAGE,$(ALLWIFIPACKAGES),$(eval $(call BuildPackage,$(PACKAGE))))
