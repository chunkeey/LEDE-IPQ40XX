#!/bin/sh

do_ipq40xx() {
	. /lib/ipq40xx.sh

	ipq40xx_board_detect
}

boot_hook_add preinit_main do_ipq40xx
