#!/usr/bin/env bash
set -euxo pipefail
# cd to directory containing this script
cd -- "$( dirname -- "${BASH_SOURCE[0]}" )"

rm -f main.uf2
cd build/
make -j$(nproc)
ln -f main.uf2 ../main.uf2
