# Repo Conventions

## Style & Standards
- Enforce BARR-C:2018 in `src/` with `-Werror`.
- C standard: C99 for application code (`CMAKE_C_STANDARD 99`); the kernel, lwIP, and Pico SDK build as gnu11.
- Format: `.clang-format` and `.clang-tidy`.
- Keep μT-Kernel task attributes literal in `src/main.c`; do not add `PRIO_*` constants.
- Log with `LOG_*` from `platform/log.h`: `[<SEV>](<task>:<module>) message`. Never call `tm_printf` in `src/` (`python3 artifacts/scripts/check_firmware_policy.py . logging`). See `docs/debugging.md`.

## Architecture & Layout
- Dual Hierarchy: public interfaces in `include/<subsystem>/`, implementations in `src/<subsystem>/`.
- Ports & Adapters: domain logic is pure C (driver-free); `src/main.c` is sole composition root.
- Ground station CLI planned under `host/cli/`; host unit tests live under `tests/` on feature branches.

## Branching & Commits
- Isolated feature branches: `feature/<topic>` (never commit on `main`).
- Conventional Commits with linear history (rebase, `--ff-only`).
- Signed commits required (`-S`); never bypass signing or hooks.
- Never push unless explicitly asked. Squash before pushing.
- Treat `package-lock.json` separately in PR sizing; review lockfiles apart from code.

## Essential Commands
- Configure: `nix develop -c ./configure.sh`
- Build: `nix develop -c ./build.sh`
- Host tests on `feature/restructure`: `cmake -S tests -B build && cmake --build build && ctest --test-dir build`
- Artifact format: `python3 artifacts/scripts/check_artifact_format.py .`
- Trace links: `bash .config/opencode/skills/update/scripts/artifacts.sh verify-links` (resolve the active skill path first)
- Cross-spec tasks: `python3 artifacts/scripts/check_task_branches.py .`

## Build & Environment
- Nix devshell: `nix develop -c ./configure.sh` and `nix develop -c ./build.sh`.
- Board target: Raspberry Pi Pico W (`pico_w`).
- Submodules: Pinned lean forks (`lib/pico-sdk`, `lib/mtk3smp-rp2040`).

## Concurrency & Safety
- RTOS: μT-Kernel 3.0 SMP (`CNF_SMP=1`) on dual Cortex-M0+.
- Multi-core exclusion: hardware SIO spinlocks; never use `hw_set_bits` on SRAM.
- Core pinning: Core 0 for real-time control (Pri 1–5), Core 1 for network (Pri 6–7).
