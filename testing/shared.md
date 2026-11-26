# shared test records

### 2026-11-26 — shared — Full host suite and lint

- Build: 3c299b5+dirty
- Kind: host test
- Setup: Nix shell, host CTest suite, clang-tidy lint, firmware build
- Steps: `ctest --test-dir build-tests --no-tests=error`, `bash lint.sh`, `./build.sh`
- Expected: every case passes, lint reports no findings, `main.uf2` builds
- Observed: 780 of 780 cases pass; lint reports `0 finding(s)`; `main.uf2` builds
- Result: pass
- Evidence: CTest and lint output
- Follow-up: n/a
