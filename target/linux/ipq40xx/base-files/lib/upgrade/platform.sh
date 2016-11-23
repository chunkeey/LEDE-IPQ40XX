. /lib/ipq40xx.sh

REQUIRE_IMAGE_METADATA=1

platform_check_image() {
	local board=$(ipq40xx_board_name)

	case "$board" in
	rt-ac58u)
		CI_UBIPART="UBI_DEV"
		local ubidev=$(nand_find_ubi $CI_UBIPART)
		local asus_root=$(nand_find_volume $ubidev jffs2)

		[ -n "$asus_root" ] || return 0

		cat << EOF
jffs2 partition is still present.
There's probably no space left
to install the filesystem.

You need to delete the jffs2 partition first:
# ubirmvol /dev/ubi0 --name=jffs2

Once this is done. Retry.
EOF
		return 1
		;;
	esac
	return 0
}

platform_pre_upgrade() {
	local board=$(ipq40xx_board_name)

	case "$board" in
	rt-ac58u)
		CI_UBIPART="UBI_DEV"
		CI_KERNPART="linux"
		nand_do_upgrade "$1"
		;;
	esac
}

platform_do_upgrade() {
	local board=$(ipq40xx_board_name)

	case "$board" in
	*)
		default_do_upgrade "$ARGV"
		;;
	esac
}

platform_nand_pre_upgrade() {
	local board=$(ipq40xx_board_name)

	case "$board" in
	rt-ac58u)
		CI_UBIPART="UBI_DEV"
		CI_KERNPART="linux"
		;;
	esac
}

blink_led() {
	. /etc/diag.sh; set_state upgrade
}

append sysupgrade_pre_upgrade blink_led
