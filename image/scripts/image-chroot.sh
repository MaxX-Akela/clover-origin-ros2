#!/usr/bin/env bash

#
# Mount a Raspberry Pi image and run a command in it (native chroot, no qemu, no Docker).
# Minimal replacement of the image-chroot script of the original builder image.
#
#   image-chroot.sh [--i-know] <IMAGE> exec <COMMAND> [ARGS...]    # command path is inside the image
#   image-chroot.sh [--i-know] <IMAGE> copy <SRC> <DST> [RSYNC_ARGS...]   # DST is inside the image
#
# One call is: attach loop device -> mount -> action -> unmount -> detach. Nothing stays
# mounted between calls. The whole script runs in a private mount namespace, and the
# cleanup trap unmounts and detaches even on errors and signals.
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

ROOT=
MOUNTED=0
POLICY_CREATED=0

usage() {
  echo "Usage: $0 [--i-know] <IMAGE> exec <COMMAND> [ARGS...] | copy <SRC> <DST> [RSYNC_ARGS...]" >&2
  exit 2
}

# Kill processes that run in the image root (gpg-agent, dirmngr and similar leftovers)
# Only processes whose root directory is exactly $ROOT are touched.
kill_chroot_procs() {
  local proc pid
  for proc in /proc/[0-9]*; do
    pid=${proc#/proc/}
    if [[ $(readlink "$proc/root" 2> /dev/null) == "$ROOT" ]]; then
      kill -9 "$pid" 2> /dev/null || true
    fi
  done
}

cleanup() {
  local rc=$?
  set +e
  if [[ $MOUNTED == 1 && -n $ROOT ]]; then
    rm -f "${ROOT}${CHROOT_MARKER}"
    if [[ $POLICY_CREATED == 1 ]]; then
      rm -f "${ROOT}/usr/sbin/policy-rc.d"
    fi
    kill_chroot_procs
    if ! umount -R "$ROOT" 2> /dev/null; then
      sleep 1
      kill_chroot_procs
      umount -R "$ROOT" || umount -R -l "$ROOT" || echo_stamp "Could not unmount $ROOT" ERROR >&2
    fi
    MOUNTED=0
  fi
  loop_detach
  if [[ -n $ROOT && -d $ROOT ]]; then
    rmdir "$ROOT" 2> /dev/null
  fi
  exit "$rc"
}

# Root (p2 of the Ubuntu Raspberry Pi image): /etc/os-release with ID=ubuntu and /etc/cloud.
# TEMPLATE: check_ubuntu_root <ROOT_DIR>; prints the reason to stdout and returns 1 on failure
check_ubuntu_root() {
  local root=$1 osr=
  if [[ -f $root/etc/os-release ]]; then
    osr=$root/etc/os-release
  elif [[ -f $root/usr/lib/os-release ]]; then
    osr=$root/usr/lib/os-release
  else
    echo "no /etc/os-release in the root filesystem"
    return 1
  fi
  if ! grep -Eq '^ID="?ubuntu"?[[:space:]]*$' "$osr"; then
    echo "$osr does not contain ID=ubuntu"
    return 1
  fi
  if [[ ! -d $root/etc/cloud ]]; then
    echo "no /etc/cloud in the root filesystem (cloud-init is expected)"
    return 1
  fi
}

# Boot partition (p1, FAT, system-boot): config.txt and cmdline.txt.
# TEMPLATE: check_pi_boot <BOOT_DIR>
check_pi_boot() {
  local boot=$1 f
  for f in config.txt cmdline.txt; do
    if [[ ! -f $boot/$f ]]; then
      echo "no $f in the boot partition"
      return 1
    fi
  done
}

# What is known about the image, to find out why a check failed. Never fails.
image_diag() {
  local img=$1 root=${2:-} boot=${3:-} f
  {
    echo "=== diagnostics for $img ==="
    echo "--- checked: root (p2): /etc/os-release (ID=ubuntu), /etc/cloud; boot (p1): config.txt, cmdline.txt"
    echo "--- root=${root:-<not mounted>} boot=${boot:-<not mounted>} loop=${LOOPDEV:-<none>}"
    echo "--- lsblk -f"
    lsblk -f || true
    echo "--- losetup -a"
    losetup -a || true
    if [[ -n $root && -d $root ]]; then
      for f in "$root/etc/os-release" "$root/usr/lib/os-release"; do
        if [[ -f $f ]]; then
          echo "--- $f"
          cat "$f" || true
          break
        fi
      done
      echo "--- ls -la $root"
      ls -la "$root" || true
      echo "--- ls -la $root/boot"
      ls -la "$root/boot" || true
    fi
    if [[ -n $boot && -d $boot ]]; then
      echo "--- ls -la $boot"
      ls -la "$boot" || true
    fi
    echo "=== end of diagnostics ==="
  } >&2
}

mount_image() {
  local img=$1 why
  loop_attach "$img"
  ROOT=$(mktemp -d "${CLOVER_MOUNT_BASE:-/mnt}/clover-root.XXXXXX")
  mount "${LOOPDEV}p2" "$ROOT" || { image_diag "$img"; die "Could not mount ${LOOPDEV}p2 of $img"; }
  MOUNTED=1
  if ! why=$(check_ubuntu_root "$ROOT"); then
    image_diag "$img" "$ROOT"
    die "$img does not look like an Ubuntu Raspberry Pi image: $why"
  fi
  # The mount point of the boot partition normally exists in the root filesystem (fstab: /boot/firmware)
  mkdir -p "$ROOT/boot/firmware"
  mount "${LOOPDEV}p1" "$ROOT/boot/firmware" || { image_diag "$img" "$ROOT"; die "Could not mount ${LOOPDEV}p1 of $img"; }
  if ! why=$(check_pi_boot "$ROOT/boot/firmware"); then
    image_diag "$img" "$ROOT" "$ROOT/boot/firmware"
    die "$img does not look like an Ubuntu Raspberry Pi image: $why"
  fi

  # rslave: mounts made inside never propagate back to the host
  mount --rbind /dev "$ROOT/dev"
  mount --make-rslave "$ROOT/dev"
  mount --rbind /sys "$ROOT/sys"
  mount --make-rslave "$ROOT/sys"
  mount -t proc proc "$ROOT/proc"
  # A private /run: the host systemd must never be reachable from the chroot
  mount -t tmpfs tmpfs "$ROOT/run"
  mkdir -p "$ROOT/run/systemd/resolve"
  cp -L /etc/resolv.conf "$ROOT/run/systemd/resolve/stub-resolv.conf"

  # No services are started while packages are installed
  if [[ ! -e $ROOT/usr/sbin/policy-rc.d ]]; then
    printf '#!/bin/sh\nexit 101\n' > "$ROOT/usr/sbin/policy-rc.d"
    chmod 755 "$ROOT/usr/sbin/policy-rc.d"
    POLICY_CREATED=1
  fi

  # The token proves to the scripts inside that they were started by this wrapper
  CLOVER_CHROOT_TOKEN=$(head -c 16 /dev/urandom | od -An -tx1 | tr -d ' \n')
  export CLOVER_CHROOT_TOKEN
  printf '%s' "$CLOVER_CHROOT_TOKEN" > "${ROOT}${CHROOT_MARKER}"
}

run_in_chroot() {
  local envs=(
    DEBIAN_FRONTEND=noninteractive
    LANG=C.UTF-8
    LC_ALL=C.UTF-8
    HOME=/root
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
    "CLOVER_CHROOT_TOKEN=${CLOVER_CHROOT_TOKEN}"
  )
  local var
  # Build parameters: CLOVER_*, CI and GITHUB_* are forwarded
  while IFS= read -r var; do
    envs+=("$var")
  done < <(env | grep -E '^(CLOVER_[A-Z_]+|CI|GITHUB_[A-Z_]+)=' | grep -v '^CLOVER_CHROOT_TOKEN=' || true)
  chroot "$ROOT" /usr/bin/env -i "${envs[@]}" "$@"
}

copy_into() {
  local src=$1 dst=$2
  shift 2
  [[ $dst == /* ]] || die "Destination must be an absolute path inside the image: $dst"
  if [[ -d $src ]]; then
    mkdir -p "${ROOT}${dst}"
    rsync -a --no-owner --no-group "$@" "${src%/}/" "${ROOT}${dst%/}/"
  else
    mkdir -p "$(dirname "${ROOT}${dst}")"
    rsync -a --no-owner --no-group "$@" "$src" "${ROOT}${dst}"
  fi
}

main() {
  local orig=("$@")
  guard_init "$@"
  set -- "${GUARD_ARGS[@]}"
  require_ci
  [[ $(uname -m) == aarch64 ]] || die "Native arm64 only (no qemu): this host is $(uname -m)"

  # Everything below runs in a private mount namespace
  if [[ -z ${CLOVER_UNSHARED:-} ]]; then
    CLOVER_UNSHARED=1 exec unshare --mount --propagation private -- "$0" "${orig[@]}"
  fi

  [[ $# -ge 2 ]] || usage
  local img=$1 action=$2
  shift 2
  [[ -f $img ]] || die "No such image: $img"
  case "$action" in
    exec) [[ $# -ge 1 ]] || usage;;
    copy) [[ $# -ge 2 ]] || usage;;
    *) usage;;
  esac

  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM

  mount_image "$img"
  case "$action" in
    exec) run_in_chroot "$@";;
    copy) copy_into "$@";;
  esac
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
