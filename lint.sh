#!/usr/bin/env bash
# Run clang-tidy (BARR-C checks from the repository .clang-tidy) over the
# production firmware sources.
#
# Usage: ./lint.sh [PREFIX...]
#
#   No argument   Lint application entries of build/compile_commands.json,
#                 excluding standalone bench images.
#   PREFIX        Select an app_program/ or bench/ source file or directory. A header
#                 argument filters reported findings to that file, and reads
#                 the selected sources or all production sources by default.
#
# Needs build/compile_commands.json (run `nix develop -c ./configure.sh`)
# and nix (clang-tidy comes from `nix shell nixpkgs#clang-tools`).
#
# Environment:
#   LINT_LOG_DIR  Log directory, default build/lint. The full clang-tidy
#                 output goes to $LINT_LOG_DIR/clang-tidy-<slug>.log, where
#                 slug is "all" or the prefixes joined with "_".
#
# Exit status: 0 no findings, 1 findings, 2 setup error.

set -euo pipefail

readonly EXCLUDED_FILES=(
    "bench/physics_check.c"
    "bench/car_bench.c"
    "bench/wdt_check.c"
)

die() {
    printf 'lint: %s\n' "$1" >&2
    exit 2
}

# Escape a literal string for use inside an extended regular expression.
regex_escape() {
    printf '%s' "$1" | sed 's/[][\.*^$+?(){}|]/\\&/g'
}

# Join the arguments with "|".
join_alt() {
    local out="" item
    for item in "$@"; do
        out="${out:+${out}|}${item}"
    done
    printf '%s' "${out}"
}

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
cd "${ROOT}"

DB="build/compile_commands.json"
if [[ ! -f "${DB}" ]]; then
    printf 'lint: %s not found. Run: nix develop -c ./configure.sh\n' "${DB}" >&2
    exit 2
fi
command -v nix >/dev/null 2>&1 || die "nix is required to run clang-tidy"

# Source prefixes select compile entries. Headers filter reported findings.
src_args=()
inc_args=()
for arg in "$@"; do
    [[ -e "${arg}" ]] || die "no such path: ${arg}"
    arg="$(realpath -m --relative-to="${ROOT}" -- "${arg}")"
    case "${arg}" in
        app_program/*.h | bench/*.h) inc_args+=("${arg}") ;;
        app_program | app_program/* | bench | bench/*) src_args+=("${arg}") ;;
        *) die "prefix must be under app_program/ or bench/: ${arg}" ;;
    esac
done

# Files to lint: compile-database entries under ROOT/app_program and ROOT/bench,
# minus exclusions.
mapfile -t db_files < <(
    sed -n 's/^ *"file": "\([^"]*\)".*/\1/p' "${DB}" |
        grep -E "^$(regex_escape "${ROOT}")/(app_program|bench)/" | sort -u
)
files=()
for file in "${db_files[@]}"; do
    rel="${file#"${ROOT}"/}"
    skip=0
    for excluded in "${EXCLUDED_FILES[@]}"; do
        [[ "${rel}" == "${excluded}" ]] && skip=1
    done
    ((skip == 0)) || continue
    if ((${#src_args[@]} == 0)); then
        files+=("${file}")
        continue
    fi
    for prefix in "${src_args[@]}"; do
        if [[ "${rel}" == "${prefix}" || "${rel}" == "${prefix}/"* ]]; then
            files+=("${file}")
            break
        fi
    done
done
((${#files[@]} > 0)) || die "no compile-database files match the given prefixes"

# Report filter: which paths count as findings and pass --header-filter.
root_re="$(regex_escape "${ROOT}")"
alts=()
if ((${#inc_args[@]} > 0)); then
    for prefix in "${inc_args[@]}" "${src_args[@]}"; do
        alts+=("$(regex_escape "${prefix}")")
    done
else
    alts=("app_program" "bench")
fi
path_re="^${root_re}/($(join_alt "${alts[@]}"))"
header_re="${path_re}(/|\$)"
finding_re="${path_re}(/[^:]*)?:[0-9]+:[0-9]+: (warning|error|fatal error):"
other_re="${path_re}(/[^:]*)?:[0-9]+:[0-9]+: "

# GCC root and version from the compiler named in the compile database.
gcc_bin="$(sed -n '/"command": "[^ ]*arm-none-eabi-gcc /{s/^ *"command": "\([^ ]*\)arm-none-eabi-gcc .*/\1/p;q}' "${DB}")"
[[ -n "${gcc_bin}" ]] || die "cannot find arm-none-eabi-gcc in ${DB}"
GCC="$(cd "${gcc_bin}/.." && pwd -P)"
gcc_versions=("${GCC}"/lib/gcc/arm-none-eabi/*/)
[[ -d "${gcc_versions[0]}" ]] || die "no GCC version directory under ${GCC}/lib/gcc/arm-none-eabi"
((${#gcc_versions[@]} == 1)) || die "expected one GCC version under ${GCC}/lib/gcc/arm-none-eabi"
GCC_INC="${gcc_versions[0]%/}/include"

# Log file.
if ((${#src_args[@]} + ${#inc_args[@]} == 0)); then
    slug="all"
else
    slug="$(join_alt "${inc_args[@]}" "${src_args[@]}" | tr '|/' '__')"
fi
log_dir="${LINT_LOG_DIR:-build/lint}"
mkdir -p "${log_dir}"
log="${log_dir}/clang-tidy-${slug}.log"

printf 'lint: %d file(s), log %s\n' "${#files[@]}" "${log}"

tidy_rc=0
nix shell nixpkgs#clang-tools -c clang-tidy \
    -p build \
    --quiet \
    "--header-filter=${header_re}" \
    --extra-arg=--target=armv6m-none-eabi \
    "--extra-arg=--sysroot=${GCC}/arm-none-eabi" \
    --extra-arg=-nostdlibinc \
    "--extra-arg=-isystem${GCC}/arm-none-eabi/include" \
    "--extra-arg=-isystem${GCC_INC}" \
    --extra-arg=-Wno-unknown-warning-option \
    --extra-arg=-Wno-unused-command-line-argument \
    "${files[@]}" >"${log}" 2>&1 || tidy_rc=$?

# Findings: warnings/errors in reported project paths, plus compiler errors
# elsewhere (for example a missing header). Notes are not counted.
# grep exits 1 for no match and 2 when it cannot read the log; only 2 is a failure.
grep_failed() {
    printf 'lint: grep could not read %s\n' "${log}" >&2
    exit 2
}
project_count="$(grep -Ec "${finding_re}" "${log}")" || (($? == 1)) || grep_failed
error_lines="$(grep -E 'fatal error: |\[clang-diagnostic-error\]' "${log}")" || (($? == 1)) || grep_failed
other_errors="$(printf '%s' "${error_lines}" | grep -Evc "${other_re}")" || (($? == 1)) || grep_failed
count=$((project_count + other_errors))

printf 'lint: %d finding(s), log %s\n' "${count}" "${log}"

if ((count > 0)); then
    exit 1
fi
if ((tidy_rc != 0)); then
    printf 'lint: clang-tidy exited with status %d and no findings, see %s\n' \
        "${tidy_rc}" "${log}" >&2
    exit 2
fi
exit 0
