#!/usr/bin/env bash

#
# Dry-run of the pure functions of the image scripts on temporary files.
# No root, no loop devices, no mounts, no chroot: this does NOT test the image build itself.
#
# Part of the ROS 2 port of Clover (https://github.com/CopterExpress/clover).
#
# Distributed under MIT License (available at https://opensource.org/licenses/MIT).
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#

set -euo pipefail

TEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_DIR="$(cd "${TEST_DIR}/.." && pwd)"

if [[ $EUID -eq 0 ]]; then
  echo "Do not run this test as root" >&2
  exit 2
fi

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
failures=0

# TEMPLATE: expect <DESCRIPTION> <COMMAND...>
expect() {
  local description=$1
  shift
  if "$@" > /dev/null 2>&1; then
    echo "ok: ${description}"
  else
    echo "FAILED: ${description}"
    failures=$((failures + 1))
  fi
}

# TEMPLATE: expect_fail <DESCRIPTION> <COMMAND...>
expect_fail() {
  local description=$1
  shift
  if ! "$@" > /dev/null 2>&1; then
    echo "ok: ${description}"
  else
    echo "FAILED: ${description} (succeeded, expected a failure)"
    failures=$((failures + 1))
  fi
}

eq() {
  [[ $1 == "$2" ]]
}

# shellcheck source=../scripts/common.sh
source "${IMAGE_DIR}/scripts/common.sh"

# --- common.sh ---------------------------------------------------------------
guard_init a --i-know b
expect "guard_init strips --i-know" eq "${GUARD_ARGS[*]}-${I_KNOW}" "a b-1"
guard_init a b
expect "guard_init without the flag" eq "${GUARD_ARGS[*]}-${I_KNOW}" "a b-0"
expect "version_ge: 2.16.0-1noble >= 2.16.0" version_ge 2.16.0-1noble.20260927.165620 2.16.0
expect_fail "version_ge: 2.15.1 < 2.16.0" version_ge 2.15.1-1noble.20260903.022619 2.16.0

# --- image-resize.sh: partition table helpers on a regular file -------------------
# shellcheck source=../scripts/image-resize.sh
source "${IMAGE_DIR}/scripts/image-resize.sh"
img="${TMP}/test.img"
truncate -s 64M "$img"
printf 'label: dos\nstart=2048, size=4096, type=c\nstart=8192, size=20000, type=83\n' | sfdisk --quiet "$img"
expect "part_field start of p2" eq "$(part_field "$img" 2 start)" 8192
expect "part_field size of p2" eq "$(part_field "$img" 2 size)" 20000
expect "part_field start of p1" eq "$(part_field "$img" 1 start)" 2048
expect "check_layout accepts two dos partitions" check_layout "$img"
expect "sectors_for_bytes 1" eq "$(sectors_for_bytes 1)" 1
expect "sectors_for_bytes 512" eq "$(sectors_for_bytes 512)" 1
expect "sectors_for_bytes 513" eq "$(sectors_for_bytes 513)" 2
img3="${TMP}/three.img"
truncate -s 64M "$img3"
printf 'label: dos\nstart=2048, size=2048, type=c\nstart=8192, size=2048, type=83\nstart=16384, size=2048, type=83\n' | sfdisk --quiet "$img3"
expect_fail "check_layout rejects three partitions" bash -c "source '${IMAGE_DIR}/scripts/image-resize.sh'; check_layout '$img3'"
imgg="${TMP}/gpt.img"
truncate -s 64M "$imgg"
printf 'label: gpt\nstart=2048, size=2048\nstart=8192, size=2048\n' | sfdisk --quiet "$imgg"
expect_fail "check_layout rejects GPT" bash -c "source '${IMAGE_DIR}/scripts/image-resize.sh'; check_layout '$imgg'"

# --- image-chroot.sh: checks of the image layout on fake trees ----------------------------------
# shellcheck source=../scripts/image-chroot.sh
source "${IMAGE_DIR}/scripts/image-chroot.sh"
good="${TMP}/good-root"
mkdir -p "${good}/etc/cloud" "${good}/usr/lib" "${good}/boot/firmware"
printf 'PRETTY_NAME="Ubuntu 24.04.5 LTS"\nNAME="Ubuntu"\nID=ubuntu\nID_LIKE=debian\n' > "${good}/usr/lib/os-release"
ln -s ../usr/lib/os-release "${good}/etc/os-release"
expect "ubuntu root passes (relative os-release symlink)" check_ubuntu_root "$good"
# The boot mount point may be absent in p2: this must not matter for the root check
rm -rf "${good:?}/boot"
expect "ubuntu root passes without /boot/firmware" check_ubuntu_root "$good"
bad="${TMP}/debian-root"
mkdir -p "${bad}/etc/cloud"
printf 'ID=debian\n' > "${bad}/etc/os-release"
expect_fail "debian root is rejected" check_ubuntu_root "$bad"
nocloud="${TMP}/nocloud-root"
mkdir -p "${nocloud}/etc"
printf 'ID=ubuntu\n' > "${nocloud}/etc/os-release"
expect_fail "root without /etc/cloud is rejected" check_ubuntu_root "$nocloud"
expect_fail "empty root is rejected" check_ubuntu_root "${TMP}"
expect_fail "wrong ID that merely contains ubuntu is rejected" bash -c "mkdir -p '${TMP}/like/etc/cloud'; printf 'ID=notubuntu\nID_LIKE=ubuntu\n' > '${TMP}/like/etc/os-release'; source '${IMAGE_DIR}/scripts/image-chroot.sh'; check_ubuntu_root '${TMP}/like'"
goodboot="${TMP}/good-boot"
mkdir -p "$goodboot"
touch "${goodboot}/config.txt" "${goodboot}/cmdline.txt"
expect "pi boot partition passes" check_pi_boot "$goodboot"
mkdir -p "${TMP}/bad-boot"
touch "${TMP}/bad-boot/config.txt"
expect_fail "boot partition without cmdline.txt is rejected" check_pi_boot "${TMP}/bad-boot"
expect_fail "a root tree is not a boot partition" check_pi_boot "$good"
expect "image_diag never fails on fake trees" image_diag fake.img "$good" "$goodboot"
expect "the failure reason is reported" bash -c "source '${IMAGE_DIR}/scripts/image-chroot.sh'; [[ \$(check_ubuntu_root '$bad' || true) == *ID=ubuntu* ]]"

# --- image-validate.sh: unit files in a chroot (no systemd as PID 1) --------------------------------
# "systemctl cat" exits 0 in a chroot for any name, so the units are looked for as files
# shellcheck source=../image-validate.sh
source "${IMAGE_DIR}/image-validate.sh"
unit_with="${TMP}/unit-with"
mkdir -p "${unit_with}/usr/lib/systemd/system" "${unit_with}/etc/systemd/system/multi-user.target.wants"
touch "${unit_with}/usr/lib/systemd/system/roscore.service"
ln -s /usr/lib/systemd/system/roscore.service "${unit_with}/etc/systemd/system/multi-user.target.wants/roscore.service"
unit_without="${TMP}/unit-without"
mkdir -p "${unit_without}/usr/lib/systemd/system" "${unit_without}/etc/systemd/system"
touch "${unit_without}/usr/lib/systemd/system/clover.service"
expect_fail "check_no_unit fails when roscore.service exists" check_no_unit roscore.service "$unit_with"
expect "check_no_unit passes without roscore.service" check_no_unit roscore.service "$unit_without"
dangling="${TMP}/unit-dangling"
mkdir -p "${dangling}/etc/systemd/system/multi-user.target.wants"
ln -s /nowhere "${dangling}/etc/systemd/system/multi-user.target.wants/roscore.service"
expect_fail "check_no_unit catches a dangling enable symlink" check_no_unit roscore.service "$dangling"
etc_only="${TMP}/unit-etc"
mkdir -p "${etc_only}/etc/systemd/system"
touch "${etc_only}/etc/systemd/system/roscore.service"
expect_fail "check_no_unit looks in /etc/systemd/system" check_no_unit roscore.service "$etc_only"
expect "check_no_unit passes on an empty root" check_no_unit roscore.service "${TMP}"
expect "check_no_unit names the file it found" bash -c "source '${IMAGE_DIR}/image-validate.sh'; [[ \$(check_no_unit roscore.service '$unit_with' || true) == *usr/lib/systemd/system/roscore.service* ]]"
# check(): a failure is reported with the command and counted, the last step is the name of the check
expect "check reports the failing check and keeps its name as last step" bash -c "
  source '${IMAGE_DIR}/image-validate.sh'
  check 'sample failing check' bash -c 'echo some-output; exit 3' 2> '${TMP}/check.err' > /dev/null
  [[ \$FAILURES == 1 && \$CLOVER_LAST_STEP == 'check: sample failing check' ]]
  grep -q 'exit code: 3' '${TMP}/check.err' && grep -q 'some-output' '${TMP}/check.err'"
expect "check does not count a passing check" bash -c "
  source '${IMAGE_DIR}/image-validate.sh'
  check 'sample ok' true > /dev/null
  [[ \$FAILURES == 0 ]]"

# --- common.sh: apt sources of the image ------------------------------------------------------------
# The Ubuntu Raspberry Pi image as it was seen in CI: noble and noble-security, no noble-updates
src_d="${TMP}/sources.d"
mkdir -p "$src_d"
cat > "${src_d}/ubuntu.sources" << 'SRC'
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg

Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
SRC
: > "${TMP}/sources.list"
expect_fail "suite_configured: noble-updates is absent" suite_configured noble-updates "${src_d}/ubuntu.sources" "${TMP}/sources.list"
expect "suite_configured: noble-security is present" suite_configured noble-security "${src_d}/ubuntu.sources"
expect_fail "suite_configured: noble is not noble-security" suite_configured noble-sec "${src_d}/ubuntu.sources"
expect "ensure_ubuntu_suites runs" ensure_ubuntu_suites "$src_d" "${TMP}/sources.list" noble
expect "ensure_ubuntu_suites: noble-updates is configured" suite_configured noble-updates "${src_d}/ubuntu.sources" "${TMP}/sources.list"
expect "ensure_ubuntu_suites: noble-updates added to the noble stanza" grep -qx 'Suites: noble noble-updates' "${src_d}/ubuntu.sources"
expect "ensure_ubuntu_suites: security stanza untouched" grep -qx 'Suites: noble-security' "${src_d}/ubuntu.sources"
expect "ensure_ubuntu_suites: no extra file needed" test ! -e "${src_d}/clover-ubuntu-suites.sources"
cp "${src_d}/ubuntu.sources" "${TMP}/ubuntu.sources.1"
ensure_ubuntu_suites "$src_d" "${TMP}/sources.list" noble > /dev/null
expect "ensure_ubuntu_suites is idempotent" cmp "${src_d}/ubuntu.sources" "${TMP}/ubuntu.sources.1"
# Only the one-line list with noble: the missing suites go to a separate file with the ports mirror
src_d2="${TMP}/sources2.d"
mkdir -p "$src_d2"
printf 'deb http://ports.ubuntu.com/ubuntu-ports noble main restricted\n' > "${TMP}/sources2.list"
ensure_ubuntu_suites "$src_d2" "${TMP}/sources2.list" noble > /dev/null
expect "fallback file has noble-updates and noble-security" grep -qx 'Suites: noble-updates noble-security' "${src_d2}/clover-ubuntu-suites.sources"
expect "fallback file uses ports.ubuntu.com/ubuntu-ports" grep -qx 'URIs: http://ports.ubuntu.com/ubuntu-ports' "${src_d2}/clover-ubuntu-suites.sources"
src_d3="${TMP}/sources3.d"
mkdir -p "$src_d3"
: > "${TMP}/sources3.list"
expect_fail "ensure_ubuntu_suites dies without the base suite" bash -c "source '${IMAGE_DIR}/scripts/common.sh'; ensure_ubuntu_suites '$src_d3' '${TMP}/sources3.list' noble"

# --- image-hardware.sh: config.txt and cmdline.txt ---------------------------------
# shellcheck source=../image-hardware.sh
source "${IMAGE_DIR}/image-hardware.sh"
cat > "${TMP}/config.txt" << 'CFG'
[all]
kernel=vmlinuz
cmdline=cmdline.txt
initramfs initrd.img followkernel

[pi4]
max_framebuffers=2
arm_boost=1

[all]
dtparam=audio=on
dtparam=i2c_arm=on
dtparam=spi=on
enable_uart=1
CFG
config_apply < "${TMP}/config.txt" > "${TMP}/config1.txt"
config_apply < "${TMP}/config1.txt" > "${TMP}/config2.txt"
expect "config_apply is idempotent" cmp "${TMP}/config1.txt" "${TMP}/config2.txt"
expect "config.txt: one clover block" eq "$(grep -c '^# clover begin$' "${TMP}/config2.txt")" 1
expect "config.txt: disable-bt after [pi4]" bash -c "sed -n '/^# clover begin\$/,\$p' '${TMP}/config2.txt' | awk '/^\\[pi4\\]\$/{s=1;next} /^\\[/{s=0} s && /^dtoverlay=disable-bt\$/{f=1} END{exit !f}'"
expect "config.txt: uart0-pi5 after [pi5]" bash -c "sed -n '/^# clover begin\$/,\$p' '${TMP}/config2.txt' | awk '/^\\[pi5\\]\$/{s=1;next} /^\\[/{s=0} s && /^dtoverlay=uart0-pi5\$/{f=1} END{exit !f}'"
expect "config.txt: the original lines are kept" grep -q '^kernel=vmlinuz$' "${TMP}/config2.txt"
expect "config.txt: usb_max_current_enable is commented" grep -q '^#usb_max_current_enable=1$' "${TMP}/config2.txt"
expect_fail "config.txt: usb_max_current_enable is not active" grep -q '^usb_max_current_enable' "${TMP}/config2.txt"

printf 'console=serial0,115200 multipath=off dwc_otg.lpm_enable=0 console=tty1 root=LABEL=writable rootfstype=ext4 rootwait fixrtc\n' > "${TMP}/cmdline.txt"
cmdline_apply < "${TMP}/cmdline.txt" > "${TMP}/cmdline1.txt"
cmdline_apply < "${TMP}/cmdline1.txt" > "${TMP}/cmdline2.txt"
expect "cmdline_apply is idempotent" cmp "${TMP}/cmdline1.txt" "${TMP}/cmdline2.txt"
expect "cmdline.txt: one line" eq "$(wc -l < "${TMP}/cmdline2.txt")" 1
expect_fail "cmdline.txt: no console=serial0" grep -q 'console=serial0' "${TMP}/cmdline2.txt"
expect "cmdline.txt: console=tty1 and root are kept" grep -q 'console=tty1 root=LABEL=writable' "${TMP}/cmdline2.txt"
expect "cmdline.txt: regulatory domain" grep -q 'cfg80211.ieee80211_regdom=GB' "${TMP}/cmdline2.txt"

# --- clover-firstboot.sh -----------------------------------------------------------------
# shellcheck source=../assets/clover-firstboot.sh
source "${IMAGE_DIR}/assets/clover-firstboot.sh"
expect "model Pi 5" eq "$(hardware_model 'Raspberry Pi 5 Model B Rev 1.0')" pi5
expect "model Pi 4" eq "$(hardware_model 'Raspberry Pi 4 Model B Rev 1.5')" pi4
expect "model Pi 3 is unknown" eq "$(hardware_model 'Raspberry Pi 3 Model B Plus Rev 1.3')" unknown
expect "random suffix is 4 digits" bash -c "source '${IMAGE_DIR}/assets/clover-firstboot.sh'; [[ \$(random_suffix) =~ ^[0-9]{4}\$ ]]"
printf '127.0.0.1 localhost\n127.0.1.1 ubuntu\n' > "${TMP}/hosts"
set_hosts_name "${TMP}/hosts" clover-1234
expect "hosts: 127.0.1.1 line replaced" grep -qP '^127\.0\.1\.1\tclover-1234 clover-1234\.local$' "${TMP}/hosts"
expect "hosts: localhost kept" grep -q '^127.0.0.1 localhost$' "${TMP}/hosts"
printf '127.0.0.1 localhost\n' > "${TMP}/hosts2"
set_hosts_name "${TMP}/hosts2" clover-1234
expect "hosts: 127.0.1.1 line added" grep -qP '^127\.0\.1\.1\tclover-1234 clover-1234\.local$' "${TMP}/hosts2"
cp "${IMAGE_DIR}/assets/clover-ap.nmconnection" "${TMP}/ap.nmconnection"
set_ap_ssid "${TMP}/ap.nmconnection" clover-4321
expect "AP keyfile: SSID replaced" grep -q '^ssid=clover-4321$' "${TMP}/ap.nmconnection"
expect "AP keyfile: mode 600" eq "$(stat -c %a "${TMP}/ap.nmconnection")" 600

# --- image-validate.sh: access point profile ---------------------------------------------------
# shellcheck source=../image-validate.sh
source "${IMAGE_DIR}/image-validate.sh"
expect "AP keyfile passes the check" check_ap_profile "${TMP}/ap.nmconnection"

# --- image-ros.sh ----------------------------------------------------------------------------------
# shellcheck source=../image-ros.sh
source "${IMAGE_DIR}/image-ros.sh"
expect "mavros_source: apt requested" eq "$(CLOVER_MAVROS_SOURCE=apt mavros_source)" apt
expect "mavros_source: source requested" eq "$(CLOVER_MAVROS_SOURCE=source mavros_source)" source
expect_fail "mavros_source: wrong value dies" bash -c "source '${IMAGE_DIR}/image-ros.sh'; CLOVER_MAVROS_SOURCE=nope mavros_source"
expect "build_jobs: explicit value" eq "$(CLOVER_BUILD_JOBS=3 build_jobs)" 3
expect "build_jobs: at least 1" bash -c "source '${IMAGE_DIR}/image-ros.sh'; (( \$(CLOVER_BUILD_JOBS= build_jobs) >= 1 ))"

# --- common.sh: failure report of colcon build on a fake workspace ----------------------------------
ws="${TMP}/ws"
mkdir -p "${ws}/src" "${ws}/log/build_1/clover" "${ws}/log/build_1/led_msgs"
ln -s build_1 "${ws}/log/latest_build"
mkdir -p "${ws}/install/led_msgs/share/ament_index/resource_index/packages"
touch "${ws}/install/led_msgs/share/ament_index/resource_index/packages/led_msgs"
seq 1 100 | sed 's/^/line /' > "${ws}/log/build_1/clover/stdout_stderr.log"
echo 'CMake Error: could not find GeographicLib' >> "${ws}/log/build_1/clover/stdout_stderr.log"
printf 'Failed   <<< clover [2.1s, exited with code 2]\nSummary: 1 package finished\n  1 package failed: clover\n' > "${TMP}/console.log"
report="${TMP}/report.txt"
CLOVER_COLCON_CONSOLE="${TMP}/console.log" build_failure_report "$ws" nobody jazzy > "$report" 2>&1
expect "report: Failed <<< line" grep -q 'Failed   <<< clover' "$report"
expect "report: Summary line" grep -q 'Summary: 1 package finished' "$report"
expect "report: error from the log of the failed package" grep -q 'could not find GeographicLib' "$report"
expect "report: log is cut to 80 lines" bash -c "! grep -qx 'line 1' '$report' && grep -qx 'line 100' '$report'"
expect "report: installed packages" grep -q 'installed in .*: led_msgs$' "$report"
expect "report: missing packages" grep -q 'NOT installed.*: aruco_pose roswww_static clover$' "$report"
expect "report: versions and resources" grep -q -- '--- nproc' "$report"
rm -f "${ws}/log/latest_build" "${ws}/log/build_1/clover/stdout_stderr.log"
printf 'event\n' > "${ws}/log/build_1/events.log"
CLOVER_COLCON_CONSOLE="${TMP}/none.log" build_failure_report "$ws" nobody jazzy > "$report" 2>&1
expect "report: falls back to events.log and the newest build_*" grep -q 'tail -n 80 .*build_1/events.log' "$report"
expect "report: works without a workspace" bash -c "source '${IMAGE_DIR}/scripts/common.sh'; build_failure_report '${TMP}/nowhere' nobody jazzy"

# --- static checks of the assets -------------------------------------------------------------------------
expect "clover.service name and ExecStart" grep -q 'ros2 launch clover clover.launch.py' "${IMAGE_DIR}/assets/clover.service"
expect_fail "clover.service does not need roscore" grep -qi roscore "${IMAGE_DIR}/assets/clover.service"
expect_fail "no ROS_HOSTNAME / ROS_IP outside of comments in the assets" bash -c "grep -rhvE '^[[:space:]]*#' '${IMAGE_DIR}/assets' | grep -qE 'ROS_HOSTNAME|ROS_IP'"

if ((failures > 0)); then
  echo "${failures} case(s) failed"
  exit 1
fi
echo "All function cases passed"
