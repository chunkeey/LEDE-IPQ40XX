. /lib/ipq40xx.sh

PART_NAME=firmware

platform_check_image() {
	return 0;
}

platform_pre_upgrade() {
	local board=$(ipq40xx_board_name)

	case "$board" in
	rt-ac58u)
		nand_do_upgrade "$1"
		;;
	esac
}

platform_do_upgrade() {
	local board=$(ipq40xx_board_name)
}

platform_nand_pre_upgrade() {
	local board=$(ipq40xx_board_name)
}

blink_led() {
	. /etc/diag.sh; set_state upgrade
}

append sysupgrade_pre_upgrade blink_led
