#!/usr/bin/env bash
set -euxo pipefail
# cd to directory containing this script
DIR=$( dirname -- "${BASH_SOURCE[0]}" )
cd $DIR

# Build with C99 compiler
PICO_SDK_PATH="$DIR/lib/pico-sdk" CHECK_ONLY=1 ./configure.sh
cd build/
make -j$(nproc)
cd ..

# Get all C files in the src/ directory
FILES=$(git ls-files 'src/*.c' 'src/*.h' 2>/dev/null || find ./src -name '*.c' -o -name '*.h')
[ -z "$FILES" ] && { echo "no C sources found"; exit 0; }

clang-format --style=file --dry-run --Werror $FILES

if [ -f compile_commands.json ]; then
    clang-tidy $FILES
else
    clang-tidy $FILES -- -x c -std=c99 -Wall
fi
