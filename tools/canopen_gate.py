#!/usr/bin/env python3
"""canopen_gate.py —— CANopen v5 一致性门禁（取代 v4 的 can_check.py）

检查项（任一失败退出码 1）：
  1. 生成的 Core/Src/OD.c 里 ODList **按索引升序**（CANopenNode 用二分查找，乱序=HardFault）
  2. OD.h 的 OD_ENTRY_H* 宏与 ODList 实际下标一致
  3. EDS 能从 docs/canopen/<node>.json 逐字重新生成（清单=单一来源）
  4. 未在清单里使用的 RPDO/TPDO 槽位，COB-ID 必须置 bit31（禁用位）
  5. 两 MCU 仓的 third_party/canopen 逐文件同哈希（伞形仓下运行时）

用法：
    python3 tools/canopen_gate.py            # 在伞形仓根：校验两个仓 + 第三方一致性
    python3 tools/canopen_gate.py --repo .   # 在某个 MCU 仓内：只校验自己
"""
import argparse
import hashlib
import importlib.util
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

NODES = {"vehical_rak_motion": "chassis", "vehical_rak_arm": "arm"}


def load_gen_eds(repo: Path):
    path = repo / "tools" / "canopen" / "gen_eds.py"
    spec = importlib.util.spec_from_file_location("gen_eds", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def check_repo(repo: Path) -> int:
    node = NODES[repo.name]
    bad = 0

    manifest = json.loads((repo / f"docs/canopen/{node}.json").read_text())
    od_c = (repo / "Core/Src/OD.c").read_text()
    od_h = (repo / "Core/Inc/OD.h").read_text()

    body = od_c.split("OD_entry_t ODList[] = {", 1)[1].split("};", 1)[0]
    idx = [int(x, 16) for x in re.findall(r"\{0x([0-9A-Fa-f]{4}),", body)]
    core = idx[:-1] if idx and idx[-1] == 0 else idx
    if core != sorted(core):
        print(f"[FAIL] {repo.name}: ODList 未升序（会 HardFault）")
        bad += 1
    else:
        print(f"[ ok ] {repo.name}: ODList 升序（{len(core)} 项）")

    mismatch = []
    for m in re.finditer(r"#define\s+OD_ENTRY_H([0-9A-Fa-f]{4})(?:_\w+)?\s+&OD->list\[(\d+)\]", od_h):
        want, n = int(m.group(1), 16), int(m.group(2))
        if n >= len(core) or core[n] != want:
            mismatch.append(m.group(1))
    if mismatch:
        print(f"[FAIL] {repo.name}: OD_ENTRY_H* 下标错位 {mismatch[:6]}")
        bad += 1
    else:
        print(f"[ ok ] {repo.name}: OD_ENTRY_H* 下标一致")

    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / f"{node}.eds"
        r = subprocess.run([sys.executable, str(repo / "tools/canopen/gen_eds.py"),
                            str(repo / f"docs/canopen/{node}.json"),
                            str(repo / "tools/canopen/template.eds"), str(out)],
                           capture_output=True, text=True)
        if r.returncode != 0:
            print(f"[FAIL] {repo.name}: gen_eds 运行失败\n{r.stderr}")
            bad += 1
        elif out.read_text() != (repo / f"docs/canopen/{node}.eds").read_text():
            print(f"[FAIL] {repo.name}: EDS 与清单不一致（改了 json 没重生成？）")
            bad += 1
        else:
            print(f"[ ok ] {repo.name}: EDS == gen_eds({node}.json)")

    # 4) PDO 启用/禁用位与清单一致（bit31=禁用；搞错会让节点回不到 Operational）
    for i in range(4):
        for kind, base in (("rpdo", 0x200 + i * 0x100), ("tpdo", 0x180 + i * 0x100)):
            blk = re.search(r"\.x14%02X_RPDO\w*Parameter = \{(.*?)\n    \}," % i
                            if kind == "rpdo" else
                            r"\.x18%02X_TPDO\w*Parameter = \{(.*?)\n    \}," % i,
                            od_c, re.S)
            if blk is None:
                print(f"[FAIL] {repo.name}: OD.c 里找不到 {kind}{i+1} 通信参数")
                bad += 1
                continue
            m = re.search(r"\.COB_IDUsed\w+ = 0x([0-9A-Fa-f]{8})", blk.group(1))
            if not m:
                print(f"[FAIL] {repo.name}: {kind}{i+1} 没有 COB-ID 字段")
                bad += 1
                continue
            cob = int(m.group(1), 16)
            disabled = bool(cob & 0x80000000)
            in_use = f"{kind}{i + 1}" in manifest.get("pdo", {})
            if in_use and disabled:
                print(f"[FAIL] {repo.name}: 清单在用 {kind}{i+1} 但 COB-ID 被禁用 0x{cob:08X}")
                bad += 1
            if (not in_use) and (not disabled or (cob & 0x7FF) != base):
                print(f"[FAIL] {repo.name}: 未使用的 {kind}{i+1} COB-ID=0x{cob:08X}，"
                      f"应为 0x{0x80000000 | base:08X}（bit31 必须置 1）")
                bad += 1
    if not bad:
        print(f"[ ok ] {repo.name}: PDO 启用/禁用位与清单一致")
    return bad


def check_third_party(root: Path) -> int:
    def digest(base: Path):
        out = {}
        for p in sorted(base.rglob("*")):
            if p.is_file():
                out[str(p.relative_to(base))] = hashlib.sha256(p.read_bytes()).hexdigest()
        return out
    a = digest(root / "vehical_rak_motion" / "third_party/canopen")
    b = digest(root / "vehical_rak_arm" / "third_party/canopen")
    if a == b:
        print(f"[ ok ] third_party/canopen 两仓一致（{len(a)} 文件）")
        return 0
    diff = [k for k in set(a) | set(b) if a.get(k) != b.get(k)]
    print(f"[FAIL] third_party/canopen 两仓不一致：{diff[:8]}")
    return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=None, help="只校验这一个仓（在其目录内用 --repo .）")
    args = ap.parse_args()

    bad = 0
    if args.repo:
        repo = Path(args.repo).resolve()
        if repo.name not in NODES:
            print(f"[FAIL] {repo.name} 不是 MCU 仓（{list(NODES)}）")
            return 1
        bad += check_repo(repo)
    else:
        root = Path.cwd()
        for name in NODES:
            if (root / name).is_dir():
                bad += check_repo(root / name)
            else:
                print(f"[FAIL] 找不到 {root/name}")
                bad += 1
        if all((root / n).is_dir() for n in NODES):
            bad += check_third_party(root)
    print("门禁：" + ("全部通过 ✅" if bad == 0 else f"{bad} 项失败 ❌"))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
