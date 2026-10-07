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

mount_image() {
  local img=$1
  loop_attach "$img"
  ROOT=$(mktemp -d "${CLOVER_MOUNT_BASE:-/mnt}/clover-root.XXXXXX")
  mount "${LOOPDEV}p2" "$ROOT"
  MOUNTED=1
  [[ -d $ROOT/boot/firmware && -d $ROOT/etc ]] || die "$img does not look like an Ubuntu Raspberry Pi image"
  mount "${LOOPDEV}p1" "$ROOT/boot/firmware"

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
