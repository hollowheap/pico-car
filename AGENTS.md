# pico-car

Pico W line-following firmware and LAN ground station.

## Scope

- Firmware in `app_program`, host tests in `tests/<subsystem>`, dashboard, bridge and CLI in `host`.
- The repository follows the course template, [sirfonzie/INF2004_ProjectTemplate](https://github.com/sirfonzie/INF2004_ProjectTemplate). Where this file and the template disagree on layout or deliverables, the template wins.
- See [README.md](README.md) for build and ground-station setup. Configure `cmake -S tests -B build-tests`, build with CMake, then run `ctest --test-dir build-tests --no-tests=error`.
- See the [build](skills/pico-firmware-build/SKILL.md) and [lint](skills/barr-c-lint/SKILL.md) runbooks. Run dashboard typecheck and dashboard and bridge tests with npm in the Nix shell.

## Template deliverables

- Keep `TEAM.md` current with owners, pins, task priorities and the interfaces between subsystems.
- Record every test in `testing/` and list unlisted video links in `videos/`.
- Use GP0–GP22 and GP26–GP28 only. GP23, GP24, GP25 and GP29 belong to the Pico W radio.
- Keep the kernel, `device/` and template library code unmodified. Put every driver in its owning subsystem folder, never in `device/`.
- `make -j8` in `build_make/` builds the template image `mtk3pico_smp1_usb_cdc_wifi.uf2` through the CMake build. Describe the firmware in CMake only. Bench images live in `bench/`, outside `app_program/`.

## Conventions

- Build `pico_w` with the pinned SDK fork and the upstream `sirfonzie/mtk3smp-rp2040` kernel. Application, PIO and SDK use C99. Kernel uses GNU11. Follow BARR-C:2018, clang-format and clang-tidy. Keep `-Werror`, lint and tests enabled.
- Keep source and headers together in `app_program/<subsystem>`. Include the public `<subsystem>.h` headers across subsystem boundaries; each holds the object IDs and data formats other subsystems may use. Shared definitions live in `app_program/app.h`. Public headers may include a narrower public type header to avoid a dependency cycle. Bench targets may use private headers through target-local include paths. Keep domain C driver-free. Create tasks only in each subsystem's `<subsystem>_start()` with literal attributes and no `PRIO_*` constants; `app_program/app_main.c` `usermain()` calls every `<subsystem>_init()` and then every `<subsystem>_start()`. With `CNF_SMP=1`, core 0 runs control at priorities 1–5 and CYW43/lwIP at 4. Core 1 runs MQTT at 6.
- Name C host tests `<unit>_test.c`, keep their fakes in `fakes/`, and register them as `<subsystem>.<unit>.<case>`.
- Keep register, SDK, BSP, and radio hardware access inside the owning `drv_<name>.{c,h}` files. Call driver APIs from application code.
- Use SIO spinlocks, never `hw_set_bits` on SRAM. Use fixed buffers, packed fields and `STATIC_ASSERT`. Log with `LOG_*` as `[<SEV>](<task>:<module>)`, never `tm_printf`.
- Read source before editing. Fix causes with minimal changes. Reproduce bugs with failing tests and preserve assertions unless a documented behavior change replaces them. Explain non-obvious reasons in comments and document behavior changes. Mark every tuning value that was not measured on the car. Keep artifact IDs out of code, comments and commits.
- Keep each physical dimension in `app_program/platform/chassis.h` only.
- Commit or push only when asked. Use isolated `feature/<topic>` branches, single-purpose signed Conventional Commits, hooks and `--ff-only` merges. Stage explicit paths. Back up before rewrites and force-push only with `--force-with-lease`.
- Keep secrets in the environment or local CMake cache. Review `package-lock.json` separately.
