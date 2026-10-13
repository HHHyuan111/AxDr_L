"""按当前 Keil 工程参数直接调用 ARM 工具链，构建普通版离线候选。

不调用 UV4，不访问设备，不覆盖 MDK-ARM/AxDr 的正式产物。
参数映射依据本机 UV4.chm 的 armclang 编译选项说明；遇到未支持的
文件级/组级覆盖或目标选项时拒绝继续，避免静默猜测参数。
"""
from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
APP = REPO / "firmware/AxDr_App"
PROJECT = APP / "MDK-ARM/AxDr.uvprojx"
OUTPUT = APP / "build/observation-keil"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def checked_text(node: ET.Element, path: str, expected: str) -> None:
    value = node.findtext(path, "")
    if value != expected:
        raise RuntimeError(f"尚未支持的 Keil 参数 {path}={value!r}；本工具已核对值为 {expected!r}")


def resolve_project_path(value: str) -> Path:
    return (PROJECT.parent / value.replace("\\", "/")).resolve()


def project_recipe(toolchain: Path) -> tuple[list[tuple[Path, list[str]]], list[str], Path]:
    root = ET.parse(PROJECT).getroot()
    targets = root.findall("./Targets/Target")
    if len(targets) != 1 or targets[0].findtext("TargetName") != "AxDr":
        raise RuntimeError("只支持当前普通版唯一 AxDr 目标")
    target = targets[0]
    checked_text(target, "pCCUsed", "6230000::V6.23::.\\ARMCLANG")
    checked_text(target, "uAC6", "1")
    if root.findall(".//FileOption") or root.findall(".//GroupOption"):
        raise RuntimeError("工程存在文件/组覆盖，必须先明确解析，不能使用目标参数代替")
    options = target.find("TargetOption")
    assert options is not None
    checked_text(options, "CommonProperty/IncludeInBuild", "1")
    for stage in ("BeforeCompile", "BeforeMake", "AfterMake"):
        for suffix in ("1", "2"):
            checked_text(options, f"TargetCommonOption/{stage}/RunUserProg{suffix}", "0")
    cads = options.find("TargetArmAds/Cads")
    misc = options.find("TargetArmAds/ArmAdsMisc")
    assembler = options.find("TargetArmAds/Aads")
    linker = options.find("TargetArmAds/LDads")
    assert cads is not None and misc is not None and assembler is not None and linker is not None
    for key, value in {"Optim": "4", "v6Lang": "3", "vShortEn": "1", "vShortWch": "1",
                       "PlainCh": "0", "OneElfS": "1", "v6Lto": "0", "Ropi": "0", "Rwpi": "0",
                       "SplitLS": "0", "useXO": "0", "VariousControls/MiscControls": "",
                       "VariousControls/Undefine": ""}.items():
        checked_text(cads, key, value)
    for key, value in {"AdsCpuType": '"Cortex-M4"', "RvdsVP": "2", "BigEnd": "0", "useUlib": "0"}.items():
        checked_text(misc, key, value)
    for key in ("VariousControls/MiscControls", "VariousControls/Define", "VariousControls/Undefine"):
        checked_text(assembler, key, "")
    for key in ("IncludeLibs", "IncludeLibsPath", "Misc", "LinkerInputFile", "DisabledWarnings"):
        checked_text(linker, key, "")
    checked_text(linker, "noStLib", "0")
    checked_text(options, "TargetCommonOption/DebugInformation", "1")
    flags = ["--target=arm-arm-none-eabi", "-mcpu=cortex-m4", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard",
             "-O3", "-g", "-gdwarf-4", "-fshort-enums", "-fshort-wchar", "-funsigned-char",
             "-ffunction-sections", "-std=c99"]
    flags += ["-D" + item.strip() for item in cads.findtext("VariousControls/Define", "").split(",") if item.strip()]
    includes = [resolve_project_path(p) for p in cads.findtext("VariousControls/IncludePath", "").split(";") if p]
    # UV4 构建日志列出的 RTE/CMSIS 组件搜索路径；标准头仍优先使用 XML 中的本仓 CMSIS。
    includes.append(PROJECT.parent / "RTE/_AxDr")
    flags += ["-I" + str(path) for path in includes]
    recipes: list[tuple[Path, list[str]]] = []
    objects: list[Path] = []
    for file in target.findall("./Groups/Group/Files/File"):
        source = resolve_project_path(file.findtext("FilePath", ""))
        if not source.is_file():
            raise RuntimeError(f"源码不存在：{source}")
        obj = OUTPUT / (source.stem + ".o")
        if obj in objects:
            raise RuntimeError(f"对象名冲突：{obj.name}")
        objects.append(obj)
        kind = file.findtext("FileType")
        if kind == "1":
            args = [str(toolchain / "armclang.exe"), *flags, "-MMD", "-MF", str(obj.with_suffix(".d")),
                    "-c", str(source), "-o", str(obj)]
        elif kind == "2":
            args = [str(toolchain / "armasm.exe"), "--cpu=Cortex-M4.fp.sp", "-g", str(source), "-o", str(obj)]
        else:
            raise RuntimeError(f"不支持文件类型 {kind}：{source}")
        recipes.append((source, args))
    scatter = resolve_project_path(linker.findtext("ScatterFile", ""))
    args = [str(toolchain / "armlink.exe"), "--cpu=Cortex-M4.fp.sp", *map(str, objects), "--strict",
            "--scatter", str(scatter), "--summary_stderr", "--info", "summarysizes", "--map",
            "--load_addr_map_info", "--xref", "--callgraph", "--symbols", "--info", "sizes,totals,unused,veneers",
            "--list", str(OUTPUT / "AxDr.map"), "-o", str(OUTPUT / "AxDr.axf")]
    return recipes, args, scatter


def dependencies(source: Path, obj: Path) -> list[Path]:
    result = [source]
    dep = obj.with_suffix(".d")
    if source.suffix.lower() == ".c":
        text = dep.read_text(encoding="utf-8").replace("\\\n", " ")
        body = text.split(": ", 1)[1]
        for word in re.findall(r"(?:\\ |[^\s])+", body):
            path = resolve_project_path(word.replace("\\ ", " "))
            if path not in result:
                result.append(path)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", type=Path, required=True, help="Arm Compiler 6.23 的 bin 目录")
    parser.add_argument("--jobs", type=int, choices=(1, 2), default=2)
    parser.add_argument("--incremental", action="store_true", help="只复用本工具同目录、依赖哈希一致的对象")
    args = parser.parse_args()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    started = time.time()
    run_id = str(time.time_ns())
    recipes, link, scatter = project_recipe(args.toolchain)
    tool_hashes = {name: sha(args.toolchain / name) for name in ("armclang.exe", "armasm.exe", "armlink.exe", "fromelf.exe")}
    version = subprocess.run([str(args.toolchain / "armclang.exe"), "--version"], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=True).stdout.decode("utf-8", errors="replace")
    if "Arm Compiler for Embedded 6.23" not in version:
        raise RuntimeError("当前工具实际版本不是工程指定的 Arm Compiler 6.23")
    initial_paths = {PROJECT, scatter, Path(__file__).resolve()}
    for folder in ("Core", "Drivers", "Middlewares", "User", "USB_Device", "MDK-ARM/RTE"):
        base = APP / folder
        if base.exists():
            initial_paths.update(p.resolve() for p in base.rglob("*") if p.is_file() and p.suffix.lower() in (".c", ".h", ".s", ".inc"))
    initial_paths.update(source for source, _ in recipes)
    initial = {str(p): sha(p) for p in sorted(initial_paths)}
    previous_path = OUTPUT / "objects-manifest.json"
    previous = json.loads(previous_path.read_text(encoding="utf-8")) if args.incremental and previous_path.exists() else {}
    records: dict[str, dict] = {}
    commands: list[dict] = []
    errors: list[str] = []
    write_json(OUTPUT / "source-before.json", initial)
    write_json(OUTPUT / "build-recipe.json", {"kind": "direct-armclang-not-uv4", "project": str(PROJECT),
               "compile": [cmd for _, cmd in recipes], "link": link, "jobs": args.jobs})
    log_path = OUTPUT / "build.log"
    if log_path.exists():
        shutil.copy2(log_path, OUTPUT / ("build-before-" + run_id + ".log"))

    def compile_one(item: tuple[Path, list[str]]) -> tuple[str, dict, str]:
        source, command = item
        obj = OUTPUT / (source.stem + ".o")
        recipe_hash = hashlib.sha256(json.dumps(command).encode()).hexdigest()
        old = previous.get(str(source))
        compiler_sha = tool_hashes[Path(command[0]).name]
        if (old and old.get("recipe_hash") == recipe_hash and old.get("compiler_sha256") == compiler_sha
                and obj.is_file() and old.get("object_sha256") == sha(obj)
                and all(Path(p).is_file() and sha(Path(p)) == value for p, value in old.get("dependencies", {}).items())):
            return str(source), old, "REUSED " + source.name + "\n"
        before = {str(source): initial[str(source)]}
        process = subprocess.run(command, cwd=PROJECT.parent, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        output = process.stdout.decode("utf-8", errors="replace")
        command_record = {"command": command, "returncode": process.returncode}
        commands.append(command_record)
        if process.returncode:
            raise RuntimeError(source.name + "\n" + output)
        for path in dependencies(source, obj):
            # -MMD 排除编译器系统头；所有工程依赖应已在构建开始时冻结。
            if str(path) not in initial:
                raise RuntimeError("发现未冻结依赖：" + str(path))
            before[str(path)] = initial[str(path)]
        return str(source), {"recipe_hash": recipe_hash, "compiler_sha256": compiler_sha,
                             "object_sha256": sha(obj), "dependencies": before}, "COMPILED " + source.name + "\n" + output

    with log_path.open("w", encoding="utf-8") as log:
        log.write("直接 ARM 构建；未调用 UV4，未连接硬件。\n")
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as executor:
            for task in concurrent.futures.as_completed([executor.submit(compile_one, item) for item in recipes]):
                try:
                    source, record, output = task.result()
                    records[source] = record
                    log.write(output)
                except Exception as error:
                    errors.append(str(error))
                    log.write("FAILED " + str(error) + "\n")
                log.flush()
        write_json(previous_path, records)
        changed = [name for name, value in initial.items() if not Path(name).exists() or sha(Path(name)) != value]
        if changed:
            errors.append("构建期间源码变化：" + ", ".join(changed))
        if errors:
            write_json(OUTPUT / "build-result.json", {"status": "FAILED", "errors": errors, "compiled": len(commands), "inputs_changed": changed})
            write_json(OUTPUT / "commands.json", commands)
            print("ARM_BUILD_FAILED", errors)
            return 1

        def run(command: list[str]) -> None:
            process = subprocess.run(command, cwd=PROJECT.parent, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            output = process.stdout.decode("utf-8", errors="replace")
            log.write(output)
            log.flush()
            commands.append({"command": command, "returncode": process.returncode})
            if process.returncode:
                raise RuntimeError(output)

        try:
            run(link)
            for flag, name in (("--i32combined", "AxDr.hex"), ("--bincombined", "AxDr.bin")):
                run([str(args.toolchain / "fromelf.exe"), flag, "--output", str(OUTPUT / name), str(OUTPUT / "AxDr.axf")])
            run([str(args.toolchain / "fromelf.exe"), "--text", "-c", "--output", str(OUTPUT / "disassembly.txt"), str(OUTPUT / "AxDr.axf")])
            final = {name: sha(Path(name)) for name in initial}
            if final != initial:
                raise RuntimeError("链接期间输入变化，候选未冻结；请增量重建后重新核对")
            write_json(OUTPUT / "source-final.json", final)
            result = {"status": "PASSED_DIRECT_ARM_BUILD", "uv4_build": False, "hardware_access": False,
                      "run_id": run_id, "toolchain_version": version.strip(), "tool_sha256": tool_hashes,
                      "object_count": len(records), "compiled_this_run": len([c for c in commands if Path(c["command"][0]).name.lower() in ("armclang.exe", "armasm.exe")]),
                      "elapsed_seconds": round(time.time() - started, 3),
                      "artifacts": {name: {"bytes": (OUTPUT / name).stat().st_size, "sha256": sha(OUTPUT / name)} for name in ("AxDr.axf", "AxDr.hex", "AxDr.bin", "AxDr.map")}}
            write_json(OUTPUT / "build-result.json", result)
            write_json(OUTPUT / ("build-evidence-" + run_id + ".json"), {"result": result, "sources": final, "commands": commands})
            print(json.dumps(result, ensure_ascii=False))
        except Exception as error:
            write_json(OUTPUT / "build-result.json", {"status": "FAILED", "error": str(error)})
            print("ARM_BUILD_FAILED", str(error))
            return 1
        finally:
            write_json(OUTPUT / "commands.json", commands)
    return 0


if __name__ == "__main__":
    sys.exit(main())
