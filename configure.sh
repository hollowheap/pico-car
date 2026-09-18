#!/usr/bin/env bash
set -euxo pipefail
# cd to directory containing this script
cd -- "$( dirname -- "${BASH_SOURCE[0]}" )"

# Extra arguments pass through to cmake
# e.g., ./configure.sh -DCMAKE_BUILD_TYPE=Release
cmake -DPICO_BOARD=pico_w -S . -B build --fresh $@
