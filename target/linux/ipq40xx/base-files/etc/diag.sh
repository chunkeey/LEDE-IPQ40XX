#!/bin/sh

. /lib/functions/leds.sh
. /lib/ipq40xx.sh

boot="$(ipq40xx_get_dt_led boot)"
failsafe="$(ipq40xx_get_dt_led failsafe)"
running="$(ipq40xx_get_dt_led running)"
upgrade="$(ipq40xx_get_dt_led upgrade)"

set_state() {
	status_led="$boot"

	case "$1" in
	preinit)
		status_led_blink_preinit
		;;
	failsafe)
		status_led_off
		[ -n "$running" ] && {
			status_led="$running"
			status_led_off
		}
		status_led="$failsafe"
		status_led_blink_failsafe
		;;
	preinit_regular)
		status_led_blink_preinit_regular
		;;
	upgrade)
		[ -n "$running" ] && {
			status_led="$upgrade"
			status_led_blink_preinit_regular
		}
		;;
	done)
		status_led_off
		[ -n "$running" ] && {
			status_led="$running"
			status_led_on
		}
		;;
	esac
}
