"""Compile + link the firmware for the ESP32 WITHOUT PlatformIO / Arduino IDE (used to verify the
code in a sandbox whose package registries were blocked).  Same core and compiler as
platformio.ini (espressif32@6.9.0 = arduino-esp32 2.0.17, xtensa-esp32-elf GCC 8.4.0
esp-2021r2-patch5).  Board = ESP32 Dev Module with default menu options.

  python3 tools/offline_build.py <arduino-esp32-2.0.17 dir> <xtensa-esp32-elf dir> [build dir]
  -> build/esp32_safety_controller.elf (+ .bin if esptool is installed)

Normal users: just run `pio run` (docs/11_software_setup.md)."""
import os, re, sys, glob, shlex, subprocess, pathlib

CORE, TC = map(os.path.abspath, sys.argv[1:3])
OUT = os.path.abspath(sys.argv[3] if len(sys.argv) > 3 else "build")
SKETCH = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
NAME = "esp32_safety_controller"
os.makedirs(OUT, exist_ok=True)


def read_props(path, prefix=""):
    d = {}
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        if k.startswith(prefix):
            d[k[len(prefix):]] = v
    return d


P = read_props(os.path.join(CORE, "platform.txt"))
board = read_props(os.path.join(CORE, "boards.txt"), "esp32.")
menus = {}
for k, v in board.items():                      # first option of each menu = IDE default
    if k.startswith("menu."):
        _, menu, opt, *rest = k.split(".")
        menus.setdefault(menu, opt)
for k, v in board.items():
    if k.startswith("menu."):
        _, menu, opt, *rest = k.split(".")
        if opt == menus[menu] and rest:
            P[".".join(rest)] = v
    else:
        P[k] = v
P.update({"runtime.platform.path": CORE, "runtime.tools.xtensa-esp32-elf-gcc.path": TC,
          "runtime.tools.esptool_py.path": "", "runtime.ide.version": "10819",
          "build.path": OUT, "build.project_name": NAME, "build.source.path": SKETCH,
          "build.arch": "ESP32", "build.core.path": os.path.join(CORE, "cores", "esp32"),
          "build.variant.path": os.path.join(CORE, "variants", P["build.variant"])})


def expand(s, extra=None, depth=0):
    def rep(m):
        k = m.group(1)
        if extra and k in extra:
            return extra[k]
        return P.get(k, m.group(0))
    for _ in range(20):
        n = re.sub(r"\{([^{}]+)\}", rep, s)
        if n == s:
            break
        s = n
    return s


def run(cmd):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if r.returncode:
        print(cmd[:300], "\n", r.stdout, r.stderr)
        sys.exit(r.returncode)
    return r.stdout + r.stderr


open(os.path.join(OUT, "build_opt.h"), "w").close()
open(os.path.join(OUT, "file_opts"), "w").close()
libs = [os.path.join(CORE, "libraries", "Preferences", "src")]
inc = [P["build.core.path"], P["build.variant.path"], os.path.join(SKETCH, "src")] + libs
includes = " ".join(f'"-I{d}"' for d in inc)


def compile_all(sources, objdir, core_build=False):
    os.makedirs(objdir, exist_ok=True)
    objs, warn = [], []
    for src in sources:
        ext = src.rsplit(".", 1)[1]
        key = {"c": "recipe.c.o.pattern", "cpp": "recipe.cpp.o.pattern", "S": "recipe.S.o.pattern"}[ext]
        obj = os.path.join(objdir, os.path.basename(src) + ".o")
        cmd = expand(P[key], {"includes": includes, "source_file": src, "object_file": obj})
        out = run(cmd)
        if out.strip():
            warn.append((src, out.strip()))
        objs.append(obj)
    return objs, warn


core_src = [f for e in ("c", "cpp", "S") for f in glob.glob(os.path.join(P["build.core.path"], "**", "*." + e), recursive=True)]
core_src += glob.glob(os.path.join(P["build.variant.path"], "*.c*"))
with open(os.path.join(OUT, "file_opts"), "w") as f:
    f.write("-DARDUINO_CORE_BUILD\n")
core_objs, _ = compile_all(core_src, os.path.join(OUT, "core"))
open(os.path.join(OUT, "file_opts"), "w").close()
ar = os.path.join(OUT, "core.a")
if os.path.exists(ar):
    os.remove(ar)
for o in core_objs:
    run(expand(P["recipe.ar.pattern"], {"archive_file_path": ar, "object_file": o}))
lib_objs, _ = compile_all(glob.glob(os.path.join(libs[0], "*.cpp")), os.path.join(OUT, "libs"))
P["compiler.warning_flags"] = P.get("compiler.warning_flags.all", "-Wall -Wextra")
sk_objs, warns = compile_all(sorted(glob.glob(os.path.join(SKETCH, "src", "*.cpp"))), os.path.join(OUT, "sketch"))
for src, w in warns:
    print("WARNINGS in", os.path.basename(src), "\n", w)
objs = " ".join(f'"{o}"' for o in sk_objs + lib_objs)
run(expand(P["recipe.c.combine.pattern"], {"object_files": objs, "archive_file_path": ar}))
elf = os.path.join(OUT, NAME + ".elf")
print(run(expand(P["recipe.size.pattern"])))
try:
    import esptool  # noqa: F401
    run(f'{sys.executable} -m esptool ' + expand(P["recipe.objcopy.bin.pattern_args"]))
    print("image:", os.path.join(OUT, NAME + ".bin"), os.path.getsize(os.path.join(OUT, NAME + ".bin")), "bytes")
except ImportError:
    print("esptool not installed: ELF only")
print("OK", elf)
