#!/usr/bin/env bash

#
# Grow or shrink a Raspberry Pi image (MBR, p1 = FAT boot, p2 = ext4 root).
# Minimal replacement of the image-resize script of the original builder image.
#
#   image-resize.sh [--i-know] grow <IMAGE> <SIZE>   # e.g. 8G, root partition fills the new space
#   image-resize.sh [--i-know] shrink <IMAGE>        # minimal root filesystem and file size
#
# Part of the ROS 2 port of Clover (https://github.com/CopterExpress/clover).
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${SCRIPT_DIR}/common.sh"

usage() {
  echo "Usage: $0 [--i-know] grow <IMAGE> <SIZE> | shrink <IMAGE>" >&2
  exit 2
}

# Partition table label (dos for the Ubuntu Raspberry Pi images).
part_label() {
  sfdisk -d "$1" | sed -n 's/^label: *//p'
}

# TEMPLATE: part_field <IMAGE> <NUMBER> <start|size>; value in sectors
part_field() {
  sfdisk -d "$1" | grep ' : start=' | sed -n "${2}p" | sed -n "s/.*${3}= *\\([0-9]*\\).*/\\1/p"
}

sectors_for_bytes() {
  echo $((($1 + 511) / 512))
}

check_layout() {
  local label
  label="$(part_label "$1")"
  [[ $label == dos ]] || die "Unsupported partition table '$label' in $1 (expected dos)"
  [[ -n "$(part_field "$1" 2 start)" ]] || die "$1 has no second partition"
  [[ -z "$(part_field "$1" 3 start)" ]] || die "$1 has more than two partitions"
}

grow_image() {
  local img=$1 want cur
  want=$(numfmt --from=iec "$2")
  cur=$(stat -c %s "$img")
  check_layout "$img"
  if ((want <= cur)); then
    echo_stamp "$img is already ${cur} bytes, nothing to grow"
    return 0
  fi

  echo_stamp "Growing $img to $2"
  truncate -s "$want" "$img"
  echo ', +' | sfdisk -N 2 --no-reread --no-tell-kernel "$img" > /dev/null

  loop_attach "$img"
  e2fsck -fp "${LOOPDEV}p2"
  resize2fs "${LOOPDEV}p2"
  loop_detach
}

shrink_image() {
  local img=$1 block_count block_size fs_bytes start sectors rc=0
  check_layout "$img"

  echo_stamp "Shrinking $img"
  loop_attach "$img"
  e2fsck -fy "${LOOPDEV}p2" || rc=$?
  ((rc <= 1)) || die "e2fsck failed with code $rc"
  resize2fs -M "${LOOPDEV}p2"
  block_count=$(dumpe2fs -h "${LOOPDEV}p2" 2> /dev/null | awk -F: '/^Block count:/ {gsub(/ /, "", $2); print $2}')
  block_size=$(dumpe2fs -h "${LOOPDEV}p2" 2> /dev/null | awk -F: '/^Block size:/ {gsub(/ /, "", $2); print $2}')
  loop_detach
  [[ -n $block_count && -n $block_size ]] || die "Could not read the filesystem size"

  fs_bytes=$((block_count * block_size))
  start=$(part_field "$img" 2 start)
  sectors=$(sectors_for_bytes "$fs_bytes")
  echo ", ${sectors}" | sfdisk -N 2 --no-reread --no-tell-kernel "$img" > /dev/null
  truncate -s $(((start + sectors) * 512)) "$img"
  echo_stamp "$img is $(stat -c %s "$img") bytes" SUCCESS
}

cleanup() {
  local rc=$?
  loop_detach
  exit "$rc"
}

main() {
  guard_init "$@"
  set -- "${GUARD_ARGS[@]}"
  require_ci
  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  [[ $# -ge 2 ]] || usage
  local action=$1 img=$2
  [[ -f $img ]] || die "No such image: $img"
  case "$action" in
    grow)
      [[ $# -eq 3 ]] || usage
      grow_image "$img" "$3";;
    shrink)
      [[ $# -eq 2 ]] || usage
      shrink_image "$img";;
    *) usage;;
  esac
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
