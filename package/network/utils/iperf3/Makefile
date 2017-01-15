#
# Copyright (C) 2007-2010 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=iperf
PKG_VERSION:=3.1.5
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/esnet/iperf.git
PKG_SOURCE_VERSION:=a46d5aec9726e196e86ab192c3f77dea6a3beb8e
PKG_MIRROR_HASH:=ebcbde749e02a437bc829163735ac0ce13c50594ad6e3728c0b5e006bb34f3f7
PKG_SOURCE:=$(PKG_NAME)-$(PKG_VERSION)-$(PKG_SOURCE_VERSION).tar.xz

PKG_MAINTAINER:=Felix Fietkau <nbd@nbd.name>
PKG_LICENSE:=BSD-3-Clause

PKG_BUILD_PARALLEL:=1
PKG_INSTALL:=1

include $(INCLUDE_DIR)/package.mk

define Package/iperf3
  SECTION:=net
  CATEGORY:=Network
  TITLE:=Internet Protocol bandwidth measuring tool
  URL:=https://github.com/esnet/iperf
endef

TARGET_CFLAGS += -D_GNU_SOURCE
CONFIGURE_ARGS += --disable-shared

MAKE_FLAGS += noinst_PROGRAMS=

define Package/iperf3/description
 Iperf is a modern alternative for measuring TCP and UDP bandwidth
 performance, allowing the tuning of various parameters and
 characteristics.
endef

define Package/iperf3/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_INSTALL_DIR)/usr/bin/iperf3 $(1)/usr/bin/
endef

$(eval $(call BuildPackage,iperf3))
