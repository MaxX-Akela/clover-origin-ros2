#!/usr/bin/env bash

#
# Prepares the files of a GitHub Release: an asset must be smaller than 2 GiB (2147483648 bytes).
# A .img.xz of LIMIT_MIB or more is split into parts of that size; the whole file is not uploaded then.
#
#   prepare-release-assets.sh <IMAGES_DIR> <OUT_DIR> [LIMIT_MIB]
#
# OUT_DIR gets the files to upload and release-notes.md (how to join the parts and to check the hashes).
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

# TEMPLATE: write_notes <NOTES_FILE> <NAME> <PART_NAMES...>; instructions to join the parts
write_notes() {
  local notes=$1 name=$2 parts win
  shift 2
  parts=$(printf '%s ' "$@")
  win=$(printf '%s+' "$@")
  cat > "$notes" << NOTES
The image is split into parts: a GitHub Release does not accept files of 2 GiB or more.

Download all \`${name}.part*\` files, \`${name}.sha256\` and \`${name}.parts.sha256\` into one directory.

Linux, macOS:

\`\`\`
sha256sum -c ${name}.parts.sha256
cat ${parts% } > ${name}
sha256sum -c ${name}.sha256
\`\`\`

Windows (cmd):

\`\`\`
copy /b ${win%+} ${name}
certutil -hashfile ${name} SHA256
\`\`\`

Compare the result of certutil with the content of \`${name}.sha256\`.
Then write \`${name}\` with Raspberry Pi Imager ("Use custom").
NOTES
}

main() {
  local images=${1:?images directory} out=${2:?output directory} limit_mib=${3:-1900}
  local file name size_mib parts=() part
  mkdir -p "$out"
  : > "${out}/release-notes.md"
  shopt -s nullglob
  for file in "${images}"/*.img.xz; do
    name=$(basename "$file")
    size_mib=$(($(stat -c %s "$file") / 1048576))
    if ((size_mib < limit_mib)); then
      echo_stamp "${name}: ${size_mib} MiB, uploaded whole"
      cp "$file" "${out}/${name}"
      (cd "$out" && sha256sum "$name" > "${name}.sha256")
      continue
    fi
    echo_stamp "${name}: ${size_mib} MiB >= ${limit_mib} MiB, splitting"
    # the hash of the whole file, then the parts
    (cd "$images" && sha256sum "$name") > "${out}/${name}.sha256"
    split -b "${limit_mib}M" -d --suffix-length=2 "$file" "${out}/${name}.part"
    parts=()
    for part in "${out}/${name}".part??; do
      parts+=("$(basename "$part")")
    done
    (cd "$out" && sha256sum "${parts[@]}" > "${name}.parts.sha256")
    write_notes "${out}/release-notes.md" "$name" "${parts[@]}"
  done
  ls -l "$out"
}

# Sourcing (for tests) defines the functions only
if [[ ${BASH_SOURCE[0]} == "$0" ]]; then
  main "$@"
fi
