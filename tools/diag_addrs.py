#!/usr/bin/env python3
"""从 .axf 的 DWARF 调试信息解析全局结构成员的绝对地址。

冒烟/验收用：J-Link Commander 只认地址，本工具把 "g_diag.param_ident.result"
这类符号路径翻成 RAM 地址。armclang(DWARF4/5) 均支持。

用法:
    python diag_addrs.py <axf路径> 符号1 符号2 ...
    python diag_addrs.py AxDr.axf g_diag.active g_diag.param_ident.state

输出: 每行 "<符号> = 0xADDR"（找不到打 NOT_FOUND）
"""
import sys

from elftools.elf.elffile import ELFFile


def die_name(die):
    attr = die.attributes.get("DW_AT_name")
    if attr is None:
        return None
    return attr.value.decode() if isinstance(attr.value, bytes) else attr.value


def _bytes(val):
    """pyelftools 的 exprloc 可能是 bytes 或 int 列表，统一成 bytes。"""
    if isinstance(val, bytes):
        return val
    if isinstance(val, list) and all(isinstance(x, int) for x in val):
        return bytes(val)
    return None


def member_offset(die):
    """成员在结构内的字节偏移。DWARF4 直接常量，DWARF5 常见 exprloc。"""
    attr = die.attributes.get("DW_AT_data_member_location")
    if attr is None:
        return 0
    val = attr.value
    if isinstance(val, int):
        return val
    val = _bytes(val)
    if val is not None:
        if len(val) and val[0] == 0x23:  # DW_OP_plus_uconst + ULEB128
            off = 0
            shift = 0
            for b in val[1:]:
                off |= (b & 0x7F) << shift
                shift += 7
                if not (b & 0x80):
                    break
            return off
        if len(val) >= 5 and val[0] == 0x03:  # DW_OP_addr（少见）
            return int.from_bytes(val[1:5], "little")
    return None


def deref_type(die):
    """沿 DW_AT_type 取类型 DIE；typedef/volatile/const 穿透，struct 到达即停。"""
    seen = 0
    while die is not None and seen < 8:
        if die.tag in ("DW_TAG_structure_type", "DW_TAG_union_type",
                       "DW_TAG_array_type"):
            return die
        if "DW_AT_type" not in die.attributes:
            return die
        die = die.get_DIE_from_attribute("DW_AT_type")
        seen += 1
    return die


def var_address(die):
    """全局变量的静态地址（DW_OP_addr 形式；location list 不支持）。"""
    attr = die.attributes.get("DW_AT_location")
    if attr is None:
        return None
    val = _bytes(attr.value)
    if val is not None and len(val) >= 5 and val[0] == 0x03:
        return int.from_bytes(val[1:5], "little")
    return None


def resolve(dwarfinfo, varname, path):
    for cu in dwarfinfo.iter_CUs():
        for die in cu.iter_DIEs():
            if die.tag != "DW_TAG_variable" or die_name(die) != varname:
                continue
            base = var_address(die)
            if base is None:
                continue
            off = 0
            cur = die
            for seg in path:
                struct = deref_type(cur)
                if struct is None or not hasattr(struct, "iter_children"):
                    return None
                target = None
                for child in struct.iter_children():
                    if child.tag == "DW_TAG_member" and die_name(child) == seg:
                        target = child
                        break
                if target is None:
                    return None
                step = member_offset(target)
                if step is None:
                    return None
                off += step
                cur = target
            return base + off
    return None


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    axf = sys.argv[1]
    with open(axf, "rb") as f:
        dwarf = ELFFile(f).get_dwarf_info()
        for sym in sys.argv[2:]:
            var, _, rest = sym.partition(".")
            path = rest.split(".") if rest else []
            addr = resolve(dwarf, var, path)
            print("%s = %s" % (sym, "0x%08X" % addr if addr is not None
                               else "NOT_FOUND"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
