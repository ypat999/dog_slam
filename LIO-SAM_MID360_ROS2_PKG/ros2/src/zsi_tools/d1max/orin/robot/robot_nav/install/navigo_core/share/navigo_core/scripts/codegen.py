#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
codegen.py

Generates:
  - C++ header:  include/navigo_core/error_codes.hpp
    * Enums (Domain/Category/Severity)
    * Packed 32-bit Code struct with `const char* msg`
    * constexpr factories for each CSV row
    * inline constexpr Code get(uint32_t v)
    * inline bool try_get(uint32_t v, Code* out) noexcept
    * inline bool try_get_by_name(std::string_view, Code* out) noexcept   <-- NEW
    * inline Code get_by_name(std::string_view)                           <-- NEW
  - Python helper: gen/python/navigo_error_codes.py (includes msg)
  - Locales: locale/en.yaml, locale/zh.yaml
  - Markdown: docs/error_code_table.md

Inputs:
  - config/nav_error_codes.json  (enum maps)
  - config/nav_error_codes.csv   (rows; should include `message_en`)
"""

import csv
import json
import re
import textwrap
from pathlib import Path


def load_enums(base: Path):
    """Load enumerations from config/nav_error_codes.json."""
    cfg_path = base / "config" / "nav_error_codes.json"
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    domain = cfg["domain"]
    category = cfg["category"]
    severity = cfg.get("severity", {"INFO": 0, "NOTICE": 1, "WARNING": 2, "ERROR": 3, "CRITICAL": 4})
    return domain, category, severity


def _parse_byte(s: str) -> int:
    """
    Parse a byte value from string supporting:
      - hex with 0x prefix (e.g., '0x1A')
      - plain hex (e.g., '1A', 'ff')
      - decimal digits (e.g., '26', '255')
    Returns int in range [0, 255].
    """
    s = (s or "").strip()
    if not s:
        return 0
    if s.lower().startswith("0x"):
        v = int(s, 16)
    else:
        if any(c in "abcdefABCDEF" for c in s):
            v = int(s, 16)
        else:
            v = int(s, 10)
    return v & 0xFF


def build_value(domain_map, category_map, severity_map,
                domain, category, subhex, severity, retry, transient, external, userfix, reserved):
    """
    Pack the 32-bit code by bit fields:
      [31..28] Domain(4) | [27..24] Category(4) | [23..16] Subcode(8) |
      [15..12] Severity(4) | [11] Retry | [10] Transient | [9] External | [8] UserFix | [7..0] Reserved(8)
    """
    d = domain_map[domain]
    c = category_map[category]
    s = _parse_byte(subhex)
    sev = severity_map[severity]
    rsv = _parse_byte(reserved)
    val = (d & 0x0F) << 28
    val |= (c & 0x0F) << 24
    val |= (s & 0xFF) << 16
    val |= (sev & 0x0F) << 12
    if int(retry):     val |= (1 << 11)
    if int(transient): val |= (1 << 10)
    if int(external):  val |= (1 << 9)
    if int(userfix):   val |= (1 << 8)
    val |= rsv
    return val


def sanitize_name(name: str) -> str:
    """Make a valid identifier by replacing non [A-Za-z0-9_] with '_'."""
    return re.sub(r'[^A-Za-z0-9_]', '_', name)


def esc_cpp(s: str) -> str:
    """Escape text for safe inclusion as a C++ string literal."""
    s = (s or "")
    s = s.replace("\\", "\\\\").replace('"', '\\"')
    s = s.replace("\n", "\\n").replace("\r", "\\r").replace("\t", "\\t")
    return s


def render_cpp_preamble(domain_map, category_map, severity_map) -> str:
    """Render enums and Code struct (with msg) plus header comments."""
    domain_items = sorted(domain_map.items(), key=lambda kv: kv[1])
    category_items = sorted(category_map.items(), key=lambda kv: kv[1])
    severity_items = sorted(severity_map.items(), key=lambda kv: kv[1])

    def enum_body(items, tweak_error=False):
        lines = []
        for k, v in items:
            name = "ERROR_" if (tweak_error and k == "ERROR") else k
            lines.append(f"  {name}={v}")
        return ",\n".join(lines)

    domain_enum = enum_body(domain_items)
    category_enum = enum_body(category_items)
    severity_enum = enum_body(severity_items, tweak_error=True)

    return f"""\
#pragma once
// ============================================================================
// This file is AUTO-GENERATED. DO NOT EDIT.
//
// Packed 32-bit navigation error code layout:
//   31           28 27           24 23                    16 15          12 11 10  9  8  7                 0
//  +--------------+---------------+------------------------+--------------+--+--+--+--+---------------------+
//  |  Domain (4)  | Category (4)  |       Subcode (8)      | Severity (4) |Ry|Tr|Ex|Uf|   Reserved (8)      |
//  +--------------+---------------+------------------------+--------------+--+--+--+--+---------------------+
//
//  Ry=Retryable, Tr=Transient, Ex=External, Uf=UserFix
//
// Notes:
//  * C++ enumerator for severity "ERROR" is emitted as "ERROR_" to avoid macro collisions.
//  * Use constexpr factories in `navigo_core::err::make` to construct known codes.
//  * `msg` points to a built-in English message literal parsed from CSV (message_en).
//  * `get(uint32_t)` returns a Code (with message if known), otherwise an empty-message Code.
//  * `try_get(uint32_t, Code*)` returns true if known; writes to *out if provided.
//  * `get_by_name(std::string_view)` / `try_get_by_name(...)` look up by CSV code_name.
// ============================================================================

#include <cstdint>
#include <string>
#include <string_view>   // NEW
#include <cstdio>

namespace navigo_core::err {{

enum class Domain : uint8_t {{
{domain_enum}
}};

enum class Category : uint8_t {{
{category_enum}
}};

enum class Severity : uint8_t {{
{severity_enum}
}};

// A compact error code with a pre-baked English message literal.
// The message pointer is intended for display/logging and should point to a static string.
// No heap allocations are performed by this struct.
struct Code {{
  uint32_t    value {{0}};     // packed 32-bit code
  const char* msg   {{""}};     // English message (from CSV: message_en)

  // Construct from individual fields (see bit layout above).
  // Flags: retry, transient, external, userfix. 'reserved' is an 8-bit extension.
  constexpr Code(Domain d, Category c, uint8_t sub, Severity sev,
                 bool retry=false, bool transient=false, bool external=false, bool userfix=false,
                 uint8_t reserved=0, const char* message="") : value(0), msg(message) {{
    value =  (static_cast<uint32_t>(d)   & 0x0F) << 28
           | (static_cast<uint32_t>(c)   & 0x0F) << 24
           | (static_cast<uint32_t>(sub) & 0xFF) << 16
           | (static_cast<uint32_t>(sev) & 0x0F) << 12
           | (retry     ? (1u<<11) : 0u)
           | (transient ? (1u<<10) : 0u)
           | (external  ? (1u<< 9) : 0u)
           | (userfix   ? (1u<< 8) : 0u)
           |  (static_cast<uint32_t>(reserved) & 0xFF);
  }}

  // Construct directly from a packed value; message can be provided (defaults to empty).
  constexpr explicit Code(uint32_t packed, const char* message="") : value(packed), msg(message) {{}}

  // Accessors (constexpr-friendly).
  constexpr Domain   domain()   const {{ return static_cast<Domain>((value>>28)&0x0F); }}
  constexpr Category category() const {{ return static_cast<Category>((value>>24)&0x0F); }}
  constexpr uint8_t  subcode()  const {{ return (value>>16)&0xFF; }}
  constexpr Severity severity() const {{ return static_cast<Severity>((value>>12)&0x0F); }}
  constexpr bool     retryable()const {{ return value & (1u<<11); }}
  constexpr bool     transient()const {{ return value & (1u<<10); }}
  constexpr bool     external() const {{ return value & (1u<<9); }}
  constexpr bool     userfix()  const {{ return value & (1u<<8); }}
  constexpr const char* message() const {{ return msg; }}
}};

// Hex string helper (e.g., "0x3A010800")
inline std::string ToString(const Code& c) {{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "0x%08X", c.value);
  return std::string(buf);
}}

// Factories generated from CSV.
// Each function returns a Code with fields and the English message filled.
namespace make {{
"""


def main(csv_path: Path, out_base: Path):
    script_root = Path(__file__).resolve().parent
    base_candidate = out_base if out_base.is_absolute() else (script_root / out_base)
    base = base_candidate if base_candidate is not None and base_candidate.is_dir() else script_root

    domain_map, category_map, severity_map = load_enums(base)

    # Read CSV
    with open(csv_path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    # Header preamble
    hpp = render_cpp_preamble(domain_map, category_map, severity_map)

    # Python helper (for tests/tools)
    py = "# Auto-generated from CSV. Do not edit by hand.\n"
    py += "from dataclasses import dataclass\n\n"
    py += "@dataclass(frozen=True)\nclass Code:\n"
    py += "    value: int\n"
    py += "    msg: str = ''\n"
    py += "    def domain(self):    return (self.value>>28) & 0x0F\n"
    py += "    def category(self):  return (self.value>>24) & 0x0F\n"
    py += "    def subcode(self):   return (self.value>>16) & 0xFF\n"
    py += "    def severity(self):  return (self.value>>12) & 0x0F\n"
    py += "    def retryable(self): return bool(self.value & (1<<11))\n"
    py += "    def transient(self): return bool(self.value & (1<<10))\n"
    py += "    def external(self):  return bool(self.value & (1<<9))\n"
    py += "    def userfix(self):   return bool(self.value & (1<<8))\n\n"

    # Locales and switch/name bodies
    en = {}
    zh = {}
    cases_return = []   # for get(uint32_t)
    cases_assign = []   # for try_get(uint32_t)
    name_ifs = []       # for try_get_by_name(std::string_view, Code*)
    seen_values = {}
    seen_names = set()

    # Helper to normalize for C++ literal emission (always "XX")
    def _norm_byte(s: str) -> str:
        v = _parse_byte(s)
        return f"{v:02X}"

    for r in rows:
        csv_name_raw = (r["code_name"] or "").strip()          # original CSV name for string lookup
        code_name = sanitize_name(csv_name_raw)                 # identifier for function name

        # Encode packed value
        val = build_value(domain_map, category_map, severity_map,
                          r["domain"], r["category"], r["subcode_hex"], r["severity"],
                          r["retryable"], r["transient"], r["external"], r["userfix"], r["reserved"])
        val_hex = f"0x{val:08X}"

        # Duplicate checks
        if val in seen_values and seen_values[val] != code_name:
            raise ValueError(f"Duplicate value 0x{val:08X} for codes: {seen_values[val]} and {code_name}")
        seen_values[val] = code_name
        if csv_name_raw in seen_names:
            raise ValueError(f"Duplicate code_name in CSV: '{csv_name_raw}'")
        seen_names.add(csv_name_raw)

        # Severity symbol in C++
        sev_cpp = "ERROR_" if r["severity"] == "ERROR" else r["severity"]

        # Normalize subcode/reserved for emission (support dec/hex inputs)
        sub_hex = _norm_byte(r.get("subcode_hex", "00"))
        rsv_hex = _norm_byte(r.get("reserved", "00"))

        # English message
        msg_en = (r.get("message_en", "") or "").strip()
        msg_cpp = esc_cpp(msg_en)

        # Factory
        hpp += textwrap.dedent(f"""
        /// {code_name}: Domain={r['domain']}, Category={r['category']}, Sub=0x{sub_hex}, Severity={r['severity']}
        constexpr Code {code_name}() {{
          return Code(Domain::{r['domain']}, Category::{r['category']}, 0x{sub_hex},
                      Severity::{sev_cpp},
                      {'true' if int(r['retryable']) else 'false'},
                      {'true' if int(r['transient']) else 'false'},
                      {'true' if int(r['external']) else 'false'},
                      {'true' if int(r['userfix']) else 'false'},
                      0x{rsv_hex},
                      "{msg_cpp}");
        }}
        """)

        # Switch arms for numeric lookup
        cases_return.append(f"    case 0x{val:08X}u: return make::{code_name}();")
        cases_assign.append(f"    case 0x{val:08X}u: if (out) *out = make::{code_name}(); return true;")

        # If-else arm for name lookup (compare with CSV original code_name)
        name_ifs.append(
            f'  if (name == std::string_view("{esc_cpp(csv_name_raw)}")) {{ if (out) *out = make::{code_name}(); return true; }}'
        )

        # Python constant & locales
        py += f"{code_name} = Code(value=0x{val:08X}, msg={json.dumps(msg_en)})\n"
        en[val_hex] = {"message": msg_en, "hint": (r.get("hint_en", "") or "").strip()}
        zh[val_hex] = {"message": (r.get("message_zh", "") or msg_en).strip(),
                       "hint": (r.get("hint_zh", "") or "").strip()}

    # Close factories
    hpp += "\n} // namespace make\n\n"

    # get()/try_get() by numeric value
    hpp += f"""\
inline constexpr Code get(uint32_t v) {{
  switch (v) {{
{chr(10).join(cases_return)}
    default:
      return Code(v, "");
  }}
}}

inline bool try_get(uint32_t v, Code* out) noexcept {{
  switch (v) {{
{chr(10).join(cases_assign)}
    default:
      (void)out;
      return false;
  }}
}}
"""

    # try_get_by_name() and get_by_name()
    hpp += """\n/**
 * Try to map a CSV `code_name` (string) to a known Code.
 * @param name CSV code_name (case-sensitive, exact match).
 * @param out  Optional output pointer; if non-null and found, '*out' is written.
 * @return     true if found; false otherwise.
 */
inline bool try_get_by_name(std::string_view name, Code* out) noexcept {
"""
    if name_ifs:
        hpp += "\n".join(name_ifs) + "\n  (void)out;\n  return false;\n}\n"
    else:
        hpp += "  (void)name; (void)out; return false;\n}\n"

    hpp += """
/**
 * Get a Code by CSV `code_name`. If not found, returns Code(0u, "").
 */
inline Code get_by_name(std::string_view name) {
  Code tmp(0u, "");
  if (try_get_by_name(name, &tmp)) return tmp;
  return Code(0u, "");
}
"""

    # Close namespace
    hpp += "\n} // namespace navigo_core::err\n"

    # Output dirs
    include_dir = base / "include" / "navigo_core"
    py_dir = base / "gen" / "python"
    locale_dir = base / "locale"
    # Keep your docs_dir choice from current script (repo root four levels up)
    docs_dir = base.parent.parent.parent.parent
    for d in [include_dir, py_dir, locale_dir, docs_dir]:
        d.mkdir(parents=True, exist_ok=True)

    # Write files
    (include_dir / "error_codes.hpp").write_text(hpp, encoding="utf-8")
    (py_dir / "navigo_error_codes.py").write_text(py, encoding="utf-8")
    (locale_dir / "en.yaml").write_text(json.dumps(en, indent=2, ensure_ascii=False), encoding="utf-8")
    (locale_dir / "zh.yaml").write_text(json.dumps(zh, indent=2, ensure_ascii=False), encoding="utf-8")

    # ---------- Markdown (same as your current script; includes code_hex & decimal code) ----------
    def _md_escape(v: str) -> str:
        s = str(v) if v is not None else ""
        s = s.replace("|", "/")
        s = s.replace("\r\n", "\n").replace("\r", "\n").replace("\t", "    ")
        s = s.replace("\n", "<br>")
        return s

    def _norm_byte(s: str) -> str:
        return f"{_parse_byte(s):02X}"

    csv_headers = list(rows[0].keys())
    headers = ["code_hex", "code"] + [h for h in csv_headers if h != "code_hex"]
    md_lines = ["# Error Code Table (from CSV)\n"]
    md_lines.append("| " + " | ".join(headers) + " |")
    md_lines.append("|" + "|".join(["---"] * len(headers)) + "|")

    for r in rows:
        val = build_value(domain_map, category_map, severity_map,
                          r["domain"], r["category"], _norm_byte(r["subcode_hex"]), r["severity"],
                          r["retryable"], r["transient"], r["external"], r["userfix"], _norm_byte(r["reserved"]))
        row = {
            "code_hex": f"0x{val:08X}",
            "code": str(val),
        }
        for h in headers[2:]:
            if h == "subcode_hex":
                row[h] = f"0x{_norm_byte(r.get(h, '00'))}"
            elif h == "reserved":
                row[h] = f"0x{_norm_byte(r.get(h, '00'))}"
            else:
                row[h] = r.get(h, "")
        md_lines.append("| " + " | ".join(_md_escape(row[h]) for h in headers) + " |")

    (docs_dir / "error_code_table.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    csv_path = Path('../config/nav_error_codes.csv')
    out_base = Path('../')
    main(csv_path, out_base)
