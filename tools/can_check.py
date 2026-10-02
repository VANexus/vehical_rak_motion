#!/usr/bin/env python3
"""CAN 协议 v3 校验器 + 参考编解码 + 一致性测试向量生成。

单一来源是 docs/protocol-signals.yaml。本脚本做三件事：
  1. 结构校验：ID/Data ID 唯一、每帧信号位宽合计 48 bit、信号名存在、reserved 必须为 0、
     原始值不溢出、物理值在 min/max 内。
  2. 用参考实现编码 vectors，逐字节打印，并把结果写回 YAML 的 expect 字段（--update），
     之后 CI 只要比对 expect 是否变化即可（三方 C/Rust 实现都拿这张表对答案）。
  3. 负例自检：任意字节被篡改后 CRC 必须失败。

用法：
    python3 tools/can_check.py            # 校验 + 打印向量表（markdown）
    python3 tools/can_check.py --update   # 追加把 expect 回填进 YAML
    python3 tools/can_check.py --c-header # 生成可给两个 MCU 工程用的 C 头
"""
from __future__ import annotations
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
DB = ROOT / "docs" / "protocol-signals.yaml"


def crc8(data: bytes, data_id: int) -> int:
    """CRC-8/ATM 风格：poly 0x07，无反射，无最终异或，初值 = 该帧 Data ID。"""
    crc = data_id & 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def encode_signal(raw: int, sig: dict) -> bytes:
    """单个 16 bit 信号，小端。"""
    if sig["len"] != 16:
        raise ValueError(f"本版本只支持 16 bit 信号，见 {sig}")
    lim = 0x7FFF if sig["signed"] else 0xFFFF
    if raw < -(0x8000 if sig["signed"] else 0) or raw > lim:
        raise OverflowError(f"原始值 {raw} 超出 16 bit 范围")
    return (raw & 0xFFFF).to_bytes(2, "little")


def decode_signal(b: bytes, sig: dict) -> int:
    v = int.from_bytes(b, "little")
    if sig["signed"] and v >= 0x8000:
        v -= 0x10000
    return v


def build_frame(msg: dict, sigdb: dict, values: dict, counter: int, flags: int) -> bytes:
    body = bytearray()
    for name in msg["signals"]:
        sig = sigdb[name]
        body += encode_signal(int(values.get(name, 0)), sig)
    assert len(body) == 6, f"{msg['name']} 信号总长 {len(body)} != 6 字节"
    body.append(((counter & 0xF) << 4) | (flags & 0xF))
    body.append(crc8(bytes(body), msg["data_id"]))
    return bytes(body)


def load():
    db = yaml.safe_load(DB.read_text(encoding="utf-8"))
    return db, db["signals"], {m["name"]: m for m in db["messages"]}


def check_structure(db, sigdb, msgs) -> list[str]:
    errs: list[str] = []
    ids = [m["id"] for m in db["messages"]]
    dids = [m["data_id"] for m in db["messages"]]
    if len(set(ids)) != len(ids):
        errs.append("存在重复的 CAN ID")
    if len(set(dids)) != len(dids):
        errs.append("存在重复的 Data ID（CRC 初值必须每帧唯一）")
    # 优先级：安全/故障/互锁的 ID 必须小于指令与反馈
    safe_max = max(m["id"] for m in db["messages"] if m["id"] < 0x040)
    cmd_min = min(m["id"] for m in db["messages"] if m["id"] >= 0x100)
    if safe_max >= cmd_min:
        errs.append("ID 分配违背优先级要求：安全类帧 ID 必须小于指令帧")
    for m in db["messages"]:
        total = 0
        for name in m["signals"]:
            if name not in sigdb:
                errs.append(f"{m['name']}: 未知信号 {name}")
                continue
            total += sigdb[name]["len"]
        if total != 48:
            errs.append(f"{m['name']}: 信号合计 {total} bit，应为 48")
    for m in db["messages"]:
        for name in m["signals"]:
            sig = sigdb.get(name, {})
            if name.startswith("RSVD") and sig.get("max", 0) != 0:
                errs.append(f"{name}: 保留信号 max 必须为 0")
    return errs


def check_vectors(db, sigdb, msgs) -> tuple[list[str], list[tuple[str, int, str, str]]]:
    errs: list[str] = []
    rows = []
    for v in db.get("vectors", []):
        msg = msgs.get(v["msg"])
        if msg is None:
            errs.append(f"向量 {v['name']}: 未知帧 {v['msg']}")
            continue
        # 保留信号不得给非零值
        for name, val in v["values"].items():
            if name.startswith("RSVD") and int(val) != 0:
                errs.append(f"{v['name']}: {name} 是保留位，必须为 0")
            sig = sigdb.get(name)
            if sig is None:
                errs.append(f"{v['name']}: 未知信号 {name}")
                continue
            phys = int(val) * sig["factor"]
            if not (sig["min"] - 1e-9 <= phys <= sig["max"] + 1e-9):
                errs.append(f"{v['name']}: {name} 物理值 {phys} 超出 [{sig['min']}, {sig['max']}]")
        frame = build_frame(msg, sigdb, v["values"], v["counter"], v["flags"])
        rows.append((v["name"], msg["id"], frame.hex(" ").upper(), msg["name"]))
        if v.get("expect") not in (None, ""):
            want = bytes.fromhex(str(v["expect"]).replace(" ", ""))
            if want != frame:
                errs.append(f"{v['name']}: 与 YAML 中 expect 不一致（协议被改动？want {want.hex(' ')} got {frame.hex(' ')}）")
        # 负例：篡改任一字节，CRC 必须失败
        for i in range(8):
            tampered = bytearray(frame)
            tampered[i] ^= 0x01
            if crc8(bytes(tampered[:7]), msg["data_id"]) == tampered[7] and i < 7:
                errs.append(f"{v['name']}: 字节 {i} 被篡改后 CRC 仍通过，校验强度不足")
    return errs, rows


def update_expect(rows) -> None:
    """把编码结果回填到 YAML 的 expect 字段（按 vectors 出现顺序替换）。

    向量行是 flow mapping（末尾带 `}`），必须原样保留那个右括号。
    """
    import re

    lines = DB.read_text(encoding="utf-8").splitlines()
    it = iter(rows)
    out = []
    pat = re.compile(r"expect:\s*[^}]*\}")
    for ln in lines:
        if "expect:" in ln:
            try:
                _, _, hexstr, _ = next(it)
                ln = pat.sub(f"expect: {hexstr}}}", ln)
            except StopIteration:
                pass
        out.append(ln)
    DB.write_text("\n".join(out) + "\n", encoding="utf-8")


def c_header(db, sigdb, msgs) -> str:
    lines = [
        "/* 由 tools/can_check.py 从 docs/protocol-signals.yaml 生成，勿手改。 */",
        "#ifndef CAN_PROTOCOL_H",
        "#define CAN_PROTOCOL_H",
        "#include <stdint.h>",
        "#ifdef __cplusplus",
        'extern "C" {',
        "#endif",
        f"#define CAN_PROTO_VER {db['version']}",
        "#define CAN_CRC_POLY 0x07u /* 初值 = 帧 Data ID，覆盖字节 0..6，无反射 */",
        "",
        "/* 帧 ID / Data ID / 周期(Hz) */",
    ]
    for m in db["messages"]:
        n = m["name"]
        lines.append(f"#define {n}_ID 0x{m['id']:03X}u")
        lines.append(f"#define {n}_DATA_ID 0x{m['data_id']:02X}u")
        lines.append(f"#define {n}_HZ {m['cycle_hz']}u")
    lines += [
        "",
        "static inline uint8_t can_crc8(const uint8_t *d, uint8_t n, uint8_t data_id)",
        "{",
        "    uint8_t crc = data_id;",
        "    for (uint8_t i = 0; i < n; i++) {",
        "        crc ^= d[i];",
        "        for (uint8_t b = 0; b < 8; b++)",
        "            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);",
        "    }",
        "    return crc;",
        "}",
        "",
        "/* 16 bit 小端读写 */",
        "static inline void can_put16(uint8_t *p, int16_t v)",
        "{",
        "    p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(((uint16_t)v >> 8) & 0xFF);",
        "}",
        "static inline int16_t can_get16(const uint8_t *p)",
        "{",
        "    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));",
        "}",
        "#ifdef __cplusplus",
        "}",
        "#endif",
        "#endif",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    db, sigdb, msgs = load()
    errs = check_structure(db, sigdb, msgs)
    e2, rows = check_vectors(db, sigdb, msgs)
    errs += e2

    print("## 测试向量（参考实现实际编码结果）\n")
    print("| # | 帧 | ID | 8 字节（hex） |")
    print("| --- | --- | --- | --- |")
    for name, mid, hexstr, mname in rows:
        print(f"| {name} | `{mname}` | 0x{mid:03X} | `{hexstr}` |")

    if "--update" in sys.argv:
        update_expect(rows)
        print("\n[can_check] expect 字段已回填进", DB.name)
    if "--c-header" in sys.argv:
        out = ROOT / "docs" / "gen" / "can_protocol.h"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(c_header(db, sigdb, msgs), encoding="utf-8")
        print("[can_check] 生成", out, flush=True)

    if errs:
        print("\n[can_check] 校验失败：", file=sys.stderr)
        for x in errs:
            print("  -", x, file=sys.stderr)
        return 1
    print("\n[can_check] 结构 + 向量 + CRC 负例 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
