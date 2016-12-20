#!/usr/bin/env sh
#
# --- ZyXEL header format ---
# Original Version by Benjamin Berg <benjamin@sipsolutions.net>
#
# The firmware image prefixed with a header (which is written into the MTD device).
# The header is one erase block (~64KiB) in size, but the checksum only convers the
# first 2KiB. Padding is 0xff. All integers are in big-endian.
#
# The checksum is always a 16-Bit System V checksum (sum -s) stored in a 32-Bit integer.
#
#   4 bytes:  checksum of the rootfs image
#   4 bytes:  length of the contained rootfs image file (big endian)
#  32 bytes:  Firmware Version string (NUL terminated, 0xff padded)
#   4 bytes:  checksum over the header partition (big endian - see below)
#  32 bytes:  Model (e.g. "NBG6617", NUL termiated, 0xff padded)
#   4 bytes:  checksum of the kernel partition
#   4 bytes:  length of the contained kernel image file (big endian)
#      rest:  0xff padding
#
# The checksums are calculated by adding up all bytes and if a 16bit
# overflow occurs, one is added and the sum is masked to 16 bit:
#   csum = csum + databyte; if (csum > 0xffff) { csum += 1; csum &= 0xffff };
# Should the file have an odd number of bytes then the byte len-0x800 is
# used additionally.
#
# The checksum for the header is calculated over the first 2048 bytes with
# the rootfs image checksum as the placeholder during calculation.
#
# The header is padded with 0xff to the erase block size of the device.
#
# Please excuse some idiocies in this script. Some of it is due to the fact
# that it has been designed around busybox's ash. So it works with the
# default OpenWrt installation on the NBG6617 and other ZyXEL routers.
#
# Needs: [, basename, cat, command, dd, echo, exit, hexdump, mktemp, printf, rm, seq, wc
# 	coreutils-sum is optional (but advised)
#

set -e

sysvsum() {
	local file="$1"
	local size="$2"
	local sum=0
	local offset=0
	local slice=65536
	while [ $offset -lt $size ]; do
		sum=$(( $sum+0$(hexdump -v -s $offset -n $slice -e '/1 "+%u"' "$file") ))
		offset=$(($offset+$slice))
	done

	local r=$(( ($sum & 0x0000ffff) + (($sum & 0xffffffff) >> 16) ))
	echo $(( ($r & 0xffff) + ($r >> 16) ))
}

board=""
version=""
kernel=""
rootfs=""
rootfssize=""
outfile=""
err=""

die()
{
	(>&2 echo "$@")
	exit 1
}

help()
{
	die "syntax: $0 --board ras-boardname --version ras-version --kernel kernelimage --rootfs rootfs [--rootfssize size] out"
}

while [ "$1" ]; do
	case "$1" in
	"--board")
		board="$2"
		shift
		shift
		continue
		;;
	"--version")
		version="$2"
		shift
		shift
		continue
		;;
	"--kernel")
		kernel="$2"
		shift
		shift
		continue
		;;
	"--rootfs")
		rootfs="$2"
		shift
		shift
		continue
		;;
	"--rootfssize")
		rootfssize="$2"
		shift
		shift
		continue
		;;
	*)
		if [ ! "$outfile" ]; then
			outfile="$1"
			shift
			continue
		else
			help
		fi
		;;
	esac
done

[ ! -n "$board" -o ! -n "$version" -o ! -r "$kernel" -o ! -r "$rootfs" -o ! "$outfile" ] && help

if [ ${#version} -ge 28 ]; then
	die "version: '$version' is too long"
fi

if [ ${#board} -ge 28 ]; then
	die "board: '$board' is too long"
fi

tmpdir="$( mktemp -d 2> /dev/null )"
if [ -z "$tmpdir" ]; then
	# try OSX signature
	tmpdir="$( mktemp -t 'ubitmp' -d )"
fi

if [ -z "$tmpdir" ]; then
	die "failed to create unique name for tmpdir"
fi

checksum_file() {
	local file="$1"
	local size="$2"

	# ZyXEL uses the good old System V sum mode... Now this is 0ldsk00l.
	if command -v sum > /dev/null 2>&1; then
		echo $(sum -s "$file" | cut -f1 -d" ")
	else
		(>&2 echo "Calculating checksum of "$(basename "$file")" - Please be patient")
		sysvsum "$file" "$size"
	fi
}

append_be32() {
	local val="$1"
	local file="$2"

	# Appends the given integer value (val) as a full big-endian 32bit binary string
	# This has to be done byte by byte with printf and not with bash because
	# a "nul" - byte will ruin the string.
	printf \\$(printf %o $((  $val >> 24         )) )\
\\$(printf %o $(( ($val >> 16) & 0xff )))\
\\$(printf %o $(( ($val >> 8) & 0xff  )) )\
\\$(printf %o $((  $val & 0xff        )) ) >> "$file"

}

# pad string
#pad=$(printf '%0.1s' $(printf "\xff"){1..64})
pad=$(printf "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff")

# make a 65408 bytes ff pad-file
pad_file="$tmpdir/pad_file"
printf "%s%s" "$pad" "$pad" >> "$pad_file.new"
for i in $(seq 1 5); do
	mv "$pad_file.new" "$pad_file"
	cat "$pad_file" "$pad_file" "$pad_file" "$pad_file" >> "$pad_file.new"
done
dd if="$pad_file.new" conv=sync bs=65408 count=1 of="$pad_file" 2>/dev/null

# prepare rootfs
tmprootfs="$tmpdir/rootfs"
rootfs_len=$(wc -c < "$rootfs")

if [ -z "$rootfssize" ] || [ "$rootfs_len" -eq "$rootfssize" ]; then
	rootfssize=$rootfs_len
	cp "$rootfs" "$tmprootfs"
elif [ "$rootfs_len" -gt "$rootfssize" ]; then
	die "rootfs is too big."
else
	# In order to preserve the flash's write endurance a bit,
	# we have to pad the rootfs with FFFFFFF...
	cp "$rootfs" "$tmprootfs"
	fullpad=$(( ($rootfssize - $rootfs_len) / 65408))
	lastpad=$(( ($rootfssize - $rootfs_len) % 65408))
	for i in $(seq 1 $fullpad); do
		cat "$pad_file" >> "$tmprootfs"
	done
	dd if="$pad_file" count=1 bs=$lastpad 2>/dev/null >> "$tmprootfs"
fi

# construct rootfs header (Located from 0x00 - 0x40)
rootfs_header_file="$tmpdir/rootfs_header"
rootfs_chksum=$(checksum_file "$tmprootfs" "$rootfssize")

versionpadlen=$(( 32 - ( ${#version} + 1) ))

#  4 bytes:  checksum of the rootfs image
append_be32 "$rootfs_chksum" "$rootfs_header_file"
#  4 bytes:  length of the contained rootfs image file (big endian)
append_be32 "$rootfssize" "$rootfs_header_file"
#  32 bytes:  Firmware Version string (NUL terminated, 0xff padded)
printf "%s\x00%.*s" "$version" "$versionpadlen" "$pad" >> "$rootfs_header_file"

# construct kernel header.
# not every device is using this. But it doesn't seem to hurt when its there.
kernel_header_file="$tmpdir/kernel_header"
kernel_len=$(wc -c < "$kernel")
kernel_chksum=$(checksum_file "$kernel" "$kernel_len")

#   4 bytes:  checksum of the kernel image
append_be32 "$kernel_chksum" "$kernel_header_file"
#   4 bytes:  length of the contained kernel image file (big endian)
append_be32 "$kernel_len" "$kernel_header_file"

# Because of the checksum placeholder shenanigans, the board_header has to be
# done twice. The first time with the rootfs checksum in place of the
# board_header checksum (this is the tmp_board_header). And then a second time
# with the calculated checksum of the tmp_board_header-2k.
#
# As a result of this mumbo jumbo, the board_header_file serves as a template
# for the first tmp_board_header and later for the real board_header.

board_header_file="$tmpdir/board_header"
board_file="$tmpdir/board"
boardpadlen=$(( 64 - ( ${#board} + 1) ))
#  32 bytes:  Model (e.g. "NBG6617", NUL termiated, 0xff padded)
printf "%s\x00%.*s" "$board" "$boardpadlen" "$pad" > "$board_file"
cat "$kernel_header_file" >> "$board_file"
printf "%.12s" "$pad" >> "$board_file"
#      rest: 0xff padding
cat "$pad_file" >> "$board_file"

tmp_board_file="$tmpdir/tmp_board_file"
cat "$rootfs_header_file" > "$tmp_board_file"

# The checksum for the header is calculated over the first 2048 bytes with
# the rootfs image checksum as the placeholder during calculation.
append_be32 "$rootfs_chksum" "$tmp_board_file"
cat "$board_file" >> "$tmp_board_file"

#truncate -s 2048 $tmp_board_file
dd if="$tmp_board_file" bs=2048 conv=sync of="$tmp_board_file-2k" 2> /dev/null

# Finally calculate the board header checksum we have been after for so long
board_chksum=$(checksum_file "$tmp_board_file-2k" 2048)

# Now add the calculated checksum of the first 2048 bytes to the final
# board_header:
#   4 bytes:  checksum over the header partition (big endian)
append_be32 "$board_chksum" "$board_header_file"
cat "$board_file" >> "$board_header_file"

# Append all headers, rootfs and kernel into the "product" $outfile.
cat "$rootfs_header_file" "$board_header_file" "$tmprootfs" "$kernel" > "$outfile"

# Cleanup the mess
rm -rf "$tmpdir"
