# pico-car

Pico W line-following firmware and LAN ground station.

## Scope

- Firmware in `src` and `include`, host tests in `tests`, dashboard, bridge and CLI in `host`.
- See [README.md](README.md) for build and ground-station setup. Configure `cmake -S tests -B build-tests`, build with CMake, then run `ctest --test-dir build-tests --no-tests=error`.
- See the [build](skills/pico-firmware-build/SKILL.md) and [lint](skills/barr-c-lint/SKILL.md) runbooks. Run dashboard typecheck and dashboard and bridge tests with npm in the Nix shell.

## Conventions

- Build `pico_w` with pinned SDK and kernel forks. Application, PIO and SDK use C99. Kernel uses GNU11. Follow BARR-C:2018, clang-format and clang-tidy. Keep `-Werror`, lint and tests enabled.
- Match public `include/<subsystem>` to `src/<subsystem>`. Keep domain C driver-free. Create tasks only in `src/main.c` with literal attributes and no `PRIO_*` constants. With `CNF_SMP=1`, core 0 runs control at priorities 1–5 and CYW43/lwIP at 4. Core 1 runs MQTT at 6.
- Use SIO spinlocks, never `hw_set_bits` on SRAM. Use fixed buffers, packed fields and `STATIC_ASSERT`. Log with `LOG_*` as `[<SEV>](<task>:<module>)`, never `tm_printf`.
- Read source before editing. Fix causes with minimal changes. Reproduce bugs with failing tests and preserve assertions. Explain non-obvious reasons in comments, document behavior changes and measure tuning on the car. Keep artifact IDs out of code, comments and commits.
- Commit or push only when asked. Use isolated `feature/<topic>` branches, single-purpose signed Conventional Commits, hooks and `--ff-only` merges. Stage explicit paths. Back up before rewrites and force-push only with `--force-with-lease`.
- Keep secrets in the environment or local CMake cache. Review `package-lock.json` separately.
