#!/usr/bin/env bash
#
# This script generates a ubi installation script for either a
# given kernel- and rootfs-file or a existing sysupgrade.tar/bin
#
#
# Currently, the script supports three modes
# legacy	this will build a legacy script. The kernel and rootfs
#		files are simply appended to the file. no checksumming
#		is performed by the script itself on the appended kernel
#		and rootfs images. And neither can the script verify the
#		location on where it is stored in device memory.
#		hence if the --address does not match with the real memory
#		location on the device running u-boot then random
#		memory content will be written to the partitions.
#		(Use this option only if you have no other option)
#
#  legacy-itest this will build a legacy script. The kernel and rootfs
#		files are simply appended to the file. The kernel and
#		rootfs images are protected by a crc32 checksum.
#		(This requires that the u-boot has itest + crc32 commands)
#  usage on the target:
#
#  (IPQ40XX)# tftpboot 84000000 install.scr
#             ...
#  (IPQ40XX)# crc32 $fileaddr $filesize
#             CRC32 for 84000000 ... 84XXXXXX ==> DEADBEEF
#		!!! Verify this value before you continue !!!
#		!!! If the values don't match, then try to
#		    reupload the file and test it again!!!
#  (IPQ40XX)# source 84000000
#
#  fit 		this will build a fit script. The target's u-boot needs
#		to support fit-images and has to support the imxextract
#		command. This script is position independend and will
#		test the integrity of the kernel and rootfs images
#		before flashing.
#
#  usage on the target:
#
#  (IPQ40XX)# tftpboot 84000000 install.scr
#             ...
#  (IPQ40XX)# source 84000000:install
#
#
# Note:
#	The u-boot scripts will halt execution if a important step
#	failed. Unfortunately, there's only so much \"echo bla bla"
#	progress bars can do. So if this needs debugging please
#	always include the full logs.
#
#
#
#
ADDR=

ALIGN=4096

UBI_PART=ubi
KERNEL=kernel
KERNEL_ADDR=
KERNEL_SIZE=
KERNEL_FILE=""

ROOTFS=rootfs
ROOTFS_ADDR=
ROOTFS_SIZE=
ROOTFS_FILE=""

ROOTFS_DATA=rootfs_data

outfile=""
ARCH=

DEVICE=""
UBI_CMD=ubi
INIT_CMDS=
PRE_REMOVE_CMDS=
PRE_CREATE_CMDS=
PRE_ROOTFS_DATA_CMDS=
POST_CMDS=
FINAL_CMDS=
SYSUPGRADE=
DELETE_TMP=true

MODE=fit

tmpdir="$( mktemp -d 2> /dev/null )"
if [ -z "$tmpdir" ]; then
        # try OSX signature
        tmpdir="$( mktemp -t 'ubitmp' -d )"
fi

if [ -z "$tmpdir" ]; then
        exit 1
fi

cleanup_tmp_and_exit()
{
	if "$DELETE_TMP"; then
		rm -rf "$tmpdir"
	else
		echo "keeping temporary directory $tmpdir"
	fi
	exit $1
}

PARAMS="$@"
help()
{
	[ $# -ge 1 ] && (>&2 echo $@)
	cat << EOFSYN
syntax: $0 --device MyRouter [--address 0xdeadbeef] [--arch ARM] ( --kernel kernelimage --rootfs rootfsimage ) | (--sysupgrade sysupgrade.bin) [--kernel-volname kernel] [--rootfs-volname rootfs] [--ubi-partition ubi] [--mode fit|legacy] outfile
	--address			memory location on the target's u-boot
					(see $imgaddr uboot env or CONFIG_SYS_LOAD_ADDR)
	--device			User visible device name
	--arch				CPU/System Architecture
	--mode				fit | legacy (see $0 for details)

	--kernel			kernel image file
	--rootfs			rootfs image file
	or
	--sysupgrade			openwrt-compatible sysupgrade.tar/bin file

	--ubi-partition			ubi partition name (default "ubi")
	--kernel-volname		kernel volume name (default "kernel")
	--rootfs-volname		rootfs volume name (default "rootfs")


	--test				replaces all "ubi command ..." with "echo command ..." (generates a dryrun script)
	--init-command			adds additional initialization commands. make sure to add parentheses.
	--pre-remove-command		adds additional commands. make sure to add parentheses.
	--pre-create-command		adds additional commands. make sure to add parentheses.
	--pre-rootfs_data-command	adds additional commands. make sure to add parentheses.
	--post-command			adds additional commands. make sure to add parentheses.
	--final-command			adds additional commands. make sure to add parentheses.

EOFSYN

	echo "input parameters: $PARAMS"

	cleanup_tmp_and_exit 1
}
# getopt anyone?

while [ "$1" ]; do
	case "$1" in
	"--address")
                ADDR="$2"
		shift
		shift
		continue
	;;
	"--arch")
		ARCH="$2"
		shift
		shift
		continue
	;;
	"--device")
		DEVICE="$2"
		shift
		shift
		continue
		;;
	"--kernel")
                KERNEL_FILE="$2"
                shift
                shift
                continue
	;;
	"--kernel-volname")
                KERNEL="$2"
                shift
                shift
                continue
	;;
	"--rootfs")
                ROOTFS_FILE="$2"
                shift
                shift
                continue
	;;
	"--rootfs-volname")
                ROOTFS="$2"
                shift
                shift
                continue
	;;
	"--ubi-partition")
                UBI_PART="$2"
                shift
                shift
                continue
	;;
	"--test")
		UBI_CMD="echo"
		shift
		continue
	;;
	"--init-command")
		INIT_CMDS="$2"
		shift
		shift
		continue
	;;
	"--pre-remove-command")
		PRE_REMOVE_CMDS="$2"
		shift
		shift
		continue
	;;
	"--pre-create-command")
		PRE_CREATE_CMDS="$2"
		shift
		shift
		continue
	;;
	"--pre-rootfs_data-command")
		PRE_ROOTFS_DATA_CMDS="$2"
		shift
		shift
		continue
	;;
	"--post-command")
		POST_CMDS="$2"
		shift
		shift
		continue
	;;
	"--final-command")
		FINAL_CMDS="$2"
		shift
		shift
		continue
	;;
	"--sysupgrade")
		SYSUPGRADE="$2"
		shift
		shift
		continue
	;;
	"--mode")
		MODE="$2"
		shift
		shift
		continue
	;;
	"--dont-delete-temp")
		DELETE_TMP=false
		shift
		continue
	;;
	*)
		if [ ! "$outfile" ]; then
			outfile=$1
			shift
			continue
		else
			help "outfile already set to $outfile - but got new '$1'"
		fi
		;;
	esac
done

if [ -r "$SYSUPGRADE" ]; then
	FWTOOL=$(command -v fwtool)
	SYSUPGRADE_META="$tmpdir/sysupgrade.meta"
	if [ ! -x "$FWTOOL" ]; then
		help "fwtool not found."
	else
		if ! "$FWTOOL" -q -i "$SYSUPGRADE_META" "$SYSUPGRADE"; then
			help "Image metadata not found."
		fi
	fi

	board_dir=$(tar tf "$SYSUPGRADE" | grep -m 1 '^sysupgrade-.*/$')
	board_dir="${board_dir%/}"
	KERNEL_FILE="$tmpdir/kernel"
	ROOTFS_FILE="$tmpdir/rootfs"
	tar Oxf "$SYSUPGRADE" "${board_dir}/kernel" > "$KERNEL_FILE" || help "kernel extraction failed."
	tar Oxf "$SYSUPGRADE" "${board_dir}/root" > "$ROOTFS_FILE" || help "rootfs extraction failed."
else
	if [ ! -r "$KERNEL_FILE" -o ! -r "$ROOTFS_FILE" ]; then
		help "no kernel or rootfs image files set."
	fi

	cp "$KERNEL_FILE" "$tmpdir/kernel"
	cp "$ROOTFS_FILE" "$tmpdir/rootfs"
fi

if [ ! -n "$ADDR" -o ! -n "$ARCH" -o ! -n "$DEVICE" -o ! -n "$UBI_PART" -o ! -n "$KERNEL" -o  ! -n "$ROOTFS" -o ! "$outfile" ]; then
	help "Invalid parameters"
fi

KERNEL_SIZE_RAW=$(cat "$KERNEL_FILE" | wc -c)
if [ "$KERNEL_SIZE_RAW" -lt "1048576" -o "$KERNEL_SIZE_RAW" -gt "16777216" ]; then
	help "Kernel size out of range $KERNEL_SIZE_RAW"
fi

ROOTFS_SIZE_RAW=$(cat "$ROOTFS_FILE" | wc -c)
if [ "$ROOTFS_SIZE_RAW" -lt "1048576" -o "$ROOTFS_SIZE_RAW" -gt "67108864" ]; then
	help "Rootfs size out of range $KERNEL_SIZE_RAW"
fi

KERNEL_SIZE_HEX=$(printf "%8x" $KERNEL_SIZE_RAW)
ROOTFS_SIZE_HEX=$(printf "%8x" $ROOTFS_SIZE_RAW)
KERNEL_CRC32=$(crc32 "$KERNEL_FILE")
ROOTFS_CRC32=$(crc32 "$ROOTFS_FILE")
TESTVAL_ADDR=$(printf "%x" $(($ADDR + 4092)) )

make_script()
{
	local script="$1"

	cat <<- EOFMKS0 > "$script"
		echo "Installing $DEVICE Factory Image."
		setenv scrimgaddr
		scrimgaddr=\$fileaddr
	EOFMKS0

	case "${MODE}" in
	fit)
		cat <<- EOFMKS1 >> "$script"
			echo "Testing if FIT image script is located at @ fileaddr = \$fileaddr"
			imxtract \$scrimgaddr install || exit 1
			echo "Testing rootfs image integrity"
			imxtract \$scrimgaddr rootfs || exit 1
			if test \$filesize -eq $ROOTFS_SIZE_HEX; then
			echo "Rootfs image check passed."
			echo "Testing kernel image integrity"
			imxtract \$scrimgaddr kernel || exit 1
			if test \$filesize -eq $KERNEL_SIZE_HEX; then
			echo "kernel image check passed."
		EOFMKS1
		;;
	legacy-itest)
		cat <<- EOFMSKA >> "$script"
			echo "Testing rootfs image integrity"
			crc32 $ROOTFS_ADDR $ROOTFS_SIZE_HEX $TESTVAL_ADDR
			if itest *$TESTVAL_ADDR == $ROOTFS_CRC32; then
			echo "Rootfs image check passed."
			echo "Testing kernel image integrity"
			crc32 $KERNEL_ADDR $KERNEL_SIZE_HEX $TESTVAL_ADDR
			if itest *$TESTVAL_ADDR == $KERNEL_CRC32; then
			echo "kernel image check passed."
		EOFMSKA
		;;
	legacy)
		cat <<- EOFMSKB >> "$script"
			echo "Testing image location"
			if test $fileaddr = $ADDR; then
		EOFMSKB
	esac

	cat <<- EOFMKS2 >> "$script"
		$INIT_CMDS
		$UBI_CMD part $UBI_PART || exit 1
		$PRE_REMOVE_CMDS
		echo "Deleting kernel($KERNEL), rootfs($ROOTFS) and rootfs_data partitions."
		setenv tmp_owrt $UBI_CMD remove $ROOTFS_DATA
		run tmp_owrt
		setenv tmp_owrt $UBI_CMD remove $ROOTFS
		run tmp_owrt
		setenv tmp_owrt $UBI_CMD remove $KERNEL
		run tmp_owrt
		setenv tmp_owrt
		$PRE_CREATE_CMDS
		echo "Creating new kernel($KERNEL) partitions."
		$UBI_CMD create $KERNEL $KERNEL_SIZE_HEX s || exit 1
		echo "Creating new rootfs($ROOTFS) partitions."
		$UBI_CMD create $ROOTFS $ROOTFS_SIZE_HEX d || exit 1
		$PRE_ROOTFS_DATA_CMDS
		echo "Creating new rootfs_data partition."
		$UBI_CMD create $ROOTFS_DATA 0 d || exit 1
		$POST_CMDS
	EOFMKS2

	case "${MODE}" in
	legacy*)
		cat <<- EOFMKS3 >> "$script"
			echo "Writing $KERNEL and $ROOTFS data to partitions."
			$UBI_CMD write $KERNEL_ADDR $KERNEL $KERNEL_SIZE_HEX
			$UBI_CMD write $ROOTFS_ADDR $ROOTFS $ROOTFS_SIZE_HEX
		EOFMKS3
	;;
	fit)
		cat <<- EOFMKS4 >> "$script"
			echo "Extracting kernel($ROOTFS) image"
			imxtract \$scrimgaddr kernel || run die_fail
			echo "Writing kernel content from \$fileaddr - \$filesize to $KERNEL partition."
			$UBI_CMD write \$fileaddr $KERNEL \$filesize
			echo "Extracting rootfs($ROOTFS) image"
			imxtract $ADDR rootfs || run die_fail
			echo "Writing rootfs content from \$fileaddr - \$filesize to $ROOTFS partition."
			$UBI_CMD write \$fileaddr $ROOTFS \$filesize
		EOFMKS4
	;;
	esac

	cat <<- EOFMKS5 >> "$script"
		echo "done... Either directly boot or perform a reset"
		setenv fileaddr \$scrimgaddr
		$FINAL_CMDS
		exit 0
	EOFMKS5

	case "${MODE}" in
	legacy-itest |\
	fit)
		cat <<- EOFMKS6 >> "$script"
			else
				echo "Kernel image verification failed - either due to corruption or bad address"
			fi
			else
				echo "rootfs image verification failed - either due to corruption or bad address"

			fi
		EOFMKS6
	;;
	legacy)
		cat <<- EOFMKS7 >> "$script"
			else
				echo "Image is at a bad address"
			fi
		EOFMKS7
	;;
	esac

	cat <<- EOFMKS8 >> "$script"
		echo "Please reupload image to $ADDR"
		setenv fileaddr \$scrimgaddr
	EOFMKS8
}

case "${MODE}" in
"legacy"|\
"legacy-itest")
	KERNEL_SIZE_RAW_ALIGNED=$(printf "%d" $(( ( ( ($KERNEL_SIZE_RAW+($ALIGN-1)) / $ALIGN) * $ALIGN) )) )
	KERNEL_ADDR=$(printf "%x" $(($ADDR + $ALIGN)) )

	ROOTFS_ADDR=$(printf "%x" $(($ADDR + $ALIGN + $KERNEL_SIZE_RAW_ALIGNED)) )

	SCRIPT="$tmpdir/script.scr"
	make_script "$SCRIPT" "$MODE"

	RAW_IMAGE="$tmpdir/rawimage.bin"

	OPENWRT_SCRIPT="$tmpdir/uboot-script.bin"
	mkimage -e "$ADDR" -a "$ADDR" -A "$ARCH" -T script -O u-boot -C none -d "$SCRIPT" "$OPENWRT_SCRIPT" &> /dev/null || help "script image creation failed."

	SCRIPT_SIZE=$(cat "$OPENWRT_SCRIPT" | wc -c)
	if [ "$SCRIPT_SIZE" -ge "$((ALIGN - 4))" ]; then
		help "$SCRIPT_SIZE exceeds $((ALIGN - 4))"
	fi

	(							\
	  dd if="$OPENWRT_SCRIPT" bs="$ALIGN" conv=sync;	\
	  dd if="$KERNEL_FILE" bs="$ALIGN" conv=sync;		\
	  dd if="$ROOTFS_FILE" bs="$ALIGN" conv=sync;		\
	) > "$outfile" 2>/dev/null || help "$outfile creation failed"
	;;

"fit")
	SCRIPT="$tmpdir/script.scr"

	make_script "$SCRIPT" "$MODE"

	ITS="$tmpdir/install.its"
	ITB="$tmpdir/install.itb"

	cat << EOFITS > "$ITS"
/dts-v1/;

/ {
	description = "${DEVICE} (Flattened Image Tree)";
	#address-cells = <1>;

	images {
		install@1 {
			description = "${DEVICE} Install script";
			data = /incbin/("${SCRIPT}");
			type = "script";
			compression = "none";
			hash@1 {
				algo = "crc32";
			};
			hash@2 {
				algo = "sha1";
			};
		};
		kernel@2 {
			description = "${DEVICE} kernel";
			data = /incbin/("kernel");
			type = "firmware";
			compression = "none";
			arch = "${ARCH}";
			hash@1 {
				algo = "crc32";
			};
			hash@2 {
				algo = "sha1";
			};
		};
		rootfs@3 {
			description = "${DEVICE} Rootfs";
			data = /incbin/("rootfs");
			type = "filesystem";
			compression = "none";
			hash@1 {
				algo = "crc32";
			};
			hash@2 {
				algo = "sha1";
			};
		};
	};
};
EOFITS

	mkimage -f "$ITS" "$outfile" &> /dev/null || help "$outfile image creation failed"
	;;
*)
	help "Unsupported mode $MODE"
esac

cleanup_tmp_and_exit 0
