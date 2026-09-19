#!/usr/bin/env bash
set -euxo pipefail
# cd to directory containing this script
DIR=$( dirname -- "${BASH_SOURCE[0]}" )
cd $DIR

export PICO_SDK_PATH="$DIR/lib/pico-sdk"
# Extra arguments pass through to cmake
# e.g., ./configure.sh -DCMAKE_BUILD_TYPE=Release
cmake -DPICO_BOARD=pico_w -S . -B build --fresh $@
