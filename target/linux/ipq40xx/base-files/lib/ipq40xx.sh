#!/bin/sh

IPQ40XX_BOARD_NAME=
IPQ40XX_MODEL=

ipq40xx_board_detect() {
	local machine
	local name

	machine=$(cat /proc/device-tree/model)

	case "$machine" in
		;;
	esac

	[ -z "$name" ] && name="unknown"

	[ -z "$IPQ40XX_BOARD_NAME" ] && IPQ40XX_BOARD_NAME="$name"
	[ -z "$IPQ40XX_MODEL" ] && IPQ40XX_MODEL="$machine"

	[ -e "/tmp/sysinfo/" ] || mkdir -p "/tmp/sysinfo/"

	echo "$IPQ40XX_BOARD_NAME" > /tmp/sysinfo/board_name
	echo "$IPQ40XX_MODEL" > /tmp/sysinfo/model
}

ipq40xx_board_name() {
	local name

	[ -f /tmp/sysinfo/board_name ] && name=$(cat /tmp/sysinfo/board_name)
	[ -z "$name" ] && name="unknown"

	echo "$name"
}

ipq40xx_get_dt_led() {
	local label
	local ledpath
	local basepath="/sys/firmware/devicetree/base"
	local nodepath="$basepath/aliases/led-$1"

	[ -f "$nodepath" ] && ledpath=$(cat "$nodepath")
	[ -n "$ledpath" ] && label=$(cat "$basepath$ledpath/label")

	echo "$label"
}
