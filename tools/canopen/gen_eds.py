#!/usr/bin/env python3
"""gen_eds.py — 从 manifest 生成 CANopen EDS（DSP306）

EDS 是 CANopen 的标准单一来源；OD.c/OD.h 由 CANopenEditor CLI（官方
CanOpenNodeV4 导出器）从 EDS 生成，不手写生成代码。

用法：
    python3 tools/canopen/gen_eds.py \
        docs/canopen/chassis.json \
        tools/canopen/template.eds \
        docs/canopen/chassis.eds
"""
import json
import sys
from pathlib import Path

DTYPE = {
    "int8": (0x0002, 8),
    "int16": (0x0003, 16),
    "int32": (0x0004, 32),
    "uint8": (0x0005, 8),
    "uint16": (0x0006, 16),
    "uint32": (0x0007, 32),
    # VISIBLE_STRING：位宽按 default 文本长度（导出器用 DefaultValue 定长）
    "string": (0x0009, 0),
}


def parse_eds(text):
    sections = []
    cur = {"name": None, "lines": []}
    for line in text.splitlines():
        if line.startswith("[") and line.endswith("]") and len(line) > 2:
            if cur["name"] is not None:
                sections.append(cur)
            cur = {"name": line[1:-1], "lines": []}
        else:
            cur["lines"].append(line)
    if cur["name"] is not None:
        sections.append(cur)
    return sections


def dump_eds(sections):
    out = []
    for i, sec in enumerate(sections):
        if i:
            out.append("")
        out.append(f"[{sec['name']}]")
        out.extend(sec["lines"])
    return "\n".join(out) + "\n"


def is_obj(name):
    return len(name) == 4 and all(c in "0123456789abcdefABCDEF" for c in name)


def sort_object_sections(sections):
    """CANopenNode 的 OD 查找是二分；导出器按 EDS 段落出现顺序生成 ODList，
    所以对象段必须整体按索引升序，否则上电直接 HardFault（实测踩过）。"""
    objs = sorted((s for s in sections if is_obj(s["name"])),
                  key=lambda s: int(s["name"], 16))
    out, done = [], False
    for sec in sections:
        if is_obj(sec["name"]):
            if not done:
                out.extend(objs)
                done = True
            continue
        out.append(sec)
    return out


def find(sections, name):
    for sec in sections:
        if sec["name"] == name:
            return sec
    return None


def set_value(sec, key, value):
    for i, line in enumerate(sec["lines"]):
        if line.startswith(key + "="):
            sec["lines"][i] = f"{key}={value}"
            return
    sec["lines"].append(f"{key}={value}")


def set_manufacturer_list(sections, indices):
    sec = find(sections, "ManufacturerObjects")
    if sec is None:
        sec = {"name": "ManufacturerObjects", "lines": []}
        sections.append(sec)
    set_value(sec, "SupportedObjects", str(len(indices)))
    # 去掉旧的编号行，重新编号
    sec["lines"] = [ln for ln in sec["lines"] if not (ln and ln[0].isdigit() and "=" in ln)]
    for i, idx in enumerate(indices, 1):
        sec["lines"].append(f"{i}=0x{idx:04X}")


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 2

    manifest_path = Path(sys.argv[1])
    template_path = Path(sys.argv[2])
    out_path = Path(sys.argv[3])

    manifest = json.loads(manifest_path.read_text())
    sections = parse_eds(template_path.read_text())

    # --- 文件/设备信息 ---
    fi = find(sections, "FileInfo")
    set_value(fi, "FileName", out_path.name)
    set_value(fi, "Description", f"RAK CANopen v5 {manifest['node']} node")
    set_value(fi, "ModificationDate", "10-29-2026")

    di = find(sections, "DeviceInfo")
    set_value(di, "VendorName", "RAK")
    set_value(di, "VendorNumber", f"0x{int(manifest.get('vendor_id', 0)):08X}")
    set_value(di, "ProductName", manifest["product"])
    set_value(di, "ProductNumber", str(manifest["product_number"]))
    set_value(di, "RevisionNumber", str(manifest["revision"]))
    set_value(di, "BaudRate_1000", "1")

    # --- 心跳 / 身份 / 通信对象 CountLabel（官方导出器要求）---
    set_value(find(sections, "1017"), "CountLabel", "HB_PROD")
    set_value(find(sections, "1200"), "CountLabel", "SDO_SRV")
    set_value(find(sections, "1280"), "CountLabel", "SDO_CLI")
    set_value(find(sections, "1017"), "DefaultValue", "1000")
    ident = find(sections, "1018")
    set_value(ident, "DefaultValue", "0") if ident else None
    set_value(find(sections, "1018sub1"), "DefaultValue",
              f"0x{int(manifest.get('vendor_id', 0)):08X}")
    set_value(find(sections, "1018sub2"), "DefaultValue",
              f"0x{int(manifest['product_number']):08X}")
    set_value(find(sections, "1018sub3"), "DefaultValue",
              f"0x{int(manifest['revision']):08X}")
    set_value(find(sections, "1018sub4"), "DefaultValue", "0x00000000")

    # --- 应用对象 ---
    objs = manifest["objects"]
    by_index = {}
    app_sections = []
    for obj in objs:
        idx = int(obj["index"], 0)
        if idx in by_index:
            raise SystemExit(f"duplicate index 0x{idx:04X}")
        by_index[idx] = obj
        code, _bits = DTYPE[obj["type"]]
        if obj["type"] == "string":
            lines = [
                f"ParameterName={obj['name']}",
                "ObjectType=0x7",
                f"DataType=0x{code:04X}",
                f"AccessType={obj['access']}",
                f"DefaultValue={obj.get('default', '')}",
                "PDOMapping=0",
            ]
        else:
            lines = [
                f"ParameterName={obj['name']}",
                "ObjectType=0x7",
                ";StorageLocation=RAM",
                f"DataType=0x{code:04X}",
                f"AccessType={obj['access']}",
                "DefaultValue=0x00000000" if code == 0x0007 else "DefaultValue=0",
                f"PDOMapping={1 if obj.get('pdo') else 0}",
            ]
        app_sections.append({"name": f"{idx:04X}", "lines": lines})
    # CANopenNode 的 OD_getEntry() 是**二分查找**，ODList 必须按索引升序；
    # 清单里把 0x1008 之类写在 0x2xxx 之后会直接 HardFault（实测踩过）。
    app_sections.sort(key=lambda sec: int(sec["name"], 16))
    sections.extend(app_sections)
    # 0x1000 段属 CiA 301 可选对象；0x2000+ 属厂商对象
    optional = sorted(int(o["index"], 0) for o in objs if int(o["index"], 0) < 0x2000)
    manufact = sorted(int(o["index"], 0) for o in objs if int(o["index"], 0) >= 0x2000)
    if optional:
        sec = find(sections, "OptionalObjects")
        existing = [ln for ln in sec["lines"] if ln[:1].isdigit() and "=" in ln]
        add = [f"0x{i:04X}" for i in optional]
        regs = [ln.split("=", 1)[1].strip().lower() for ln in existing]
        merged = regs + [a for a in add if a not in regs]
        # 升序！CANopenNode 的 OD 查找是二分，导出器按本列表顺序生成 ODList
        merged.sort(key=lambda v: int(v, 16))
        sec["lines"] = [ln for ln in sec["lines"] if not (ln[:1].isdigit() and "=" in ln)]
        set_value(sec, "SupportedObjects", str(len(merged)))
        for i, val in enumerate(merged, 1):
            sec["lines"].append(f"{i}={val}")
    set_manufacturer_list(sections, manufact)

    # --- PDO 映射与传输类型（4 路 RPDO/TPDO 全部设置 CountLabel）---
    pdo = manifest.get("pdo", {})
    tpdo_types = manifest.get("tpdo_types", {})
    for i in range(4):
        # 未使用的 PDO 必须把 COB-ID 的 bit31（transmission disabled）置 1：
        # 否则 CANopenNode 会把它当"有合法 COB-ID 但零映射"，报 PDO_WRONG_MAPPING
        # → 错误寄存器置位 → NMT 被踢回 Pre-operational（实测踩过）。
        r_base = 0x200 + i * 0x100
        t_base = 0x180 + i * 0x100
        r_name, t_name = f"rpdo{i + 1}", f"tpdo{i + 1}"
        set_value(find(sections, f"{0x1400 + i:04X}"), "CountLabel", "RPDO")
        set_value(find(sections, f"{0x1400 + i:04X}sub1"), "DefaultValue",
                  f"0x{r_base:03X}" if r_name in pdo else f"0x{0x80000000 | r_base:08X}")
        set_value(find(sections, f"{0x1800 + i:04X}"), "CountLabel", "TPDO")
        set_value(find(sections, f"{0x1800 + i:04X}sub1"), "DefaultValue",
                  f"0x{t_base:03X}" if t_name in pdo else f"0x{0x80000000 | t_base:08X}")
        set_value(find(sections, f"{0x1800 + i:04X}sub2"), "DefaultValue",
                  str(int(tpdo_types.get(t_name, 254))))

    for name, entries in pdo.items():
        if name.startswith("rpdo"):
            n = int(name[4:]) - 1
            map_base = 0x1600 + n
            com_base = 0x1400 + n
        elif name.startswith("tpdo"):
            n = int(name[4:]) - 1
            map_base = 0x1A00 + n
            com_base = 0x1800 + n
        else:
            raise SystemExit(f"bad pdo name {name}")

        total_bits = 0
        for j in range(1, 9):
            sec = find(sections, f"{map_base:04X}sub{j}")
            if sec is None:
                raise SystemExit(f"missing EDS section {map_base:04X}sub{j}")
            if j <= len(entries):
                obj = by_index[int(entries[j - 1], 0)]
                code, bits = DTYPE[obj["type"]]
                total_bits += bits
                val = (int(obj["index"], 0) << 16) | (bits)
                set_value(sec, "DefaultValue", f"0x{val:08X}")
            else:
                set_value(sec, "DefaultValue", "0x00000000")
        if total_bits > 64:
            raise SystemExit(f"{name} mapping is {total_bits} bits > 64")
        set_value(find(sections, f"{map_base:04X}sub0"), "DefaultValue", str(len(entries)))
        if name.startswith("rpdo"):
            set_value(find(sections, f"{com_base:04X}"), "CountLabel", "RPDO")
        else:
            set_value(find(sections, f"{com_base:04X}"), "CountLabel", "TPDO")
            set_value(find(sections, f"{com_base:04X}sub2"), "DefaultValue",
                      str(int(tpdo_types.get(name, 254))))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(dump_eds(sort_object_sections(sections)))
    print(f"[gen_eds] {manifest['node']}: {len(objs)} app objects -> {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
