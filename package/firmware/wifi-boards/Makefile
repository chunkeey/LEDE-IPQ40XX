include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/version.mk

PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
endef

define Build/Compile
endef

ALLWIFIBOARDS:=rt-ac58u fritz4040 nbg6617
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

$(eval $(call generate-wifi-board-package,rt-ac58u,board-rt-ac58u.bin,ASUS RT-AC58U/RT-ACRH13))
$(eval $(call generate-wifi-board-package,fritz4040,board-fritz4040.bin,AVM FRITZBox 4040))
$(eval $(call generate-wifi-board-package,nbg6617,board-nbg6617.bin,ZyXEL NBG6617))

$(foreach PACKAGE,$(ALLWIFIPACKAGES),$(eval $(call BuildPackage,$(PACKAGE))))
