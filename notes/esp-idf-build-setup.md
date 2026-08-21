# Building OVMS v3 firmware — ESP-IDF setup

Verified on macOS 26 (Apple Silicon, arm64) in August 2026. Four things here are
non-obvious and cost real time; they are flagged **[gotcha]**.

Everything installs to `~/esp`, deliberately **outside the repo**, so nothing can
leak into a commit.

---

## 1. Toolchain

**[gotcha]** `.travis.yml` references `xtensa-esp32-elf-linux64-...`. The macOS
build is published under `-macos-`, not the `-osx-` you might guess — that URL
404s.

```bash
mkdir -p ~/esp && cd ~/esp
curl -fsSL https://dl.espressif.com/dl/xtensa-esp32-elf-macos-1.22.0-97-gc752ad5-5.2.0.tar.gz | tar -xzf -
```

The binaries are **x86_64 Mach-O** and run fine under Rosetta on Apple Silicon.
Verify:

```bash
~/esp/xtensa-esp32-elf/bin/xtensa-esp32-elf-gcc --version   # expect gcc 5.2.0
```

## 2. ESP-IDF

**[gotcha]** OVMS needs the **openvehicles fork**, not upstream Espressif. It is
an ESP-IDF v3.x era tree that uses `make`, not `idf.py`.

```bash
git clone --recurse-submodules --depth 1 https://github.com/openvehicles/esp-idf.git ~/esp/esp-idf
```

About 1.6 GB with submodules; the toolchain adds ~145 MB.

## 3. Python environment

**[gotcha]** Old IDF invokes `python`, not `python3`, and needs `pkg_resources`.
Modern setuptools (81+) **removed `pkg_resources`**, so it must be pinned.

**[gotcha]** Do **not** symlink a venv's python from outside the venv — venv
detection keys off the executable path and the symlink silently resolves to the
system interpreter with no site-packages. Put the venv's own `bin` on PATH
instead; it already provides `python`.

```bash
python3 -m venv ~/esp/venv
~/esp/venv/bin/pip install --upgrade pip "setuptools<81" pyserial cryptography pyparsing future
```

## 4. Repo submodules

The OVMS repo has its own submodules (mongoose, zlib, libzip, wolfssh, wolfssl).
A `--depth 1` clone will not have them:

```bash
git submodule update --init --recursive --depth 1
```

## 5. Configure and build

```bash
cd vehicle/OVMS.V3
cp support/sdkconfig.default.hw31 sdkconfig
export IDF_PATH="$HOME/esp/esp-idf"
export PATH="$HOME/esp/venv/bin:$HOME/esp/xtensa-esp32-elf/bin:$PATH"
```

`sdkconfig.default.hw31` enables both `CONFIG_OVMS_VEHICLE_FIAT500` and
`CONFIG_OVMS_VEHICLE_BOLTEV`.

### Per-component build — this is what you usually want

Compiles one component against the real headers with the real cross-compiler.
Fast, and enough to catch anything a native test cannot:

```bash
make component-vehicle_fiat500-build
```

Substitute any component name, e.g. `component-vehicle_boltev-build`.

### Full firmware image — currently blocked

```bash
make -j5
```

**[gotcha]** This fails in the `dbc` component: it runs `yacc`, and macOS
CommandLineTools does not ship bison.

```
xcode-select: error: tool 'bison' requires Xcode, but active developer
directory '/Library/Developer/CommandLineTools' is a command line tools instance
```

Fix with `brew install bison` and put it ahead of the system stub on PATH. This
was never run, so the full image is unverified — per-component builds are.

---

## Build artifacts

`make` writes `sdkconfig` and `build/` **inside the repo**, and the project
`.gitignore` covers neither. Rather than modify `.gitignore` (which would show
up in an upstream PR), add them to the local-only exclude file:

```bash
cat >> .git/info/exclude <<'EOF'
sdkconfig
sdkconfig.old
vehicle/OVMS.V3/sdkconfig
vehicle/OVMS.V3/sdkconfig.old
vehicle/OVMS.V3/build/
build/
vehicle/OVMS.V3/components/*/tests/test
EOF
```

`.git/info/exclude` is never committed, so it does not survive a fresh clone —
re-add it after cloning.

---

## Native tests — no ESP-IDF required

Decode and command logic can be exercised with plain `g++`, no toolchain and no
vehicle. See `vehicle/OVMS.V3/components/vehicle_fiat500/tests/`:

```bash
make -C vehicle/OVMS.V3/components/vehicle_fiat500/tests
```

The mock lives in `tests/mock/` and stubs only what the module touches. The
pattern was taken from `vehicle_vwegolf/tests/`.

`tests/` sits beside `src/`, and both `CMakeLists.txt` and `component.mk`
reference only `src/`, so adding tests cannot affect the firmware build.

One thing to preserve if you extend the mock: it counts metric **transitions**
separately from **writes**. The real framework only fires events when a value
changes, so tests about event-queue pressure must assert on transitions —
a write-count assertion cannot tell buggy code from fixed code.
