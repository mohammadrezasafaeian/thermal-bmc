#!/usr/bin/env python3
"""
twi_dump_parser.py — parse the AVR TWI debug stream captured in the
STM32 dbg_ring and produce a paste-ready diagnostic report.

Usage:
    python twi_dump_parser.py dump.txt
    python twi_dump_parser.py dump.txt --head 217
Input formats (auto-detected):
  A) The ASCII stream itself, e.g.:  RST 45 40
                                     60 80 80 A0 60 80 80 A0 !
  B) CubeIDE Memory Browser export: hex bytes OF that ASCII, e.g.:
     52 53 54 20 34 35 20 34 30 20 0D 0A 36 30 20 ...
"""

import sys
import re
from collections import Counter

STATUS_NAMES = {
    0x00: "BUS_ERROR",
    0x60: "SLA+W rx, ACKed        (write frame opens)",
    0x68: "SLA+W rx (arb lost)    (ABNORMAL for pure slave)",
    0x70: "GENCALL rx             (ABNORMAL: gen-call is off)",
    0x80: "data rx, ACKed",
    0x88: "data rx, NACKed        (over-length or TWEA dropped)",
    0xA0: "STOP/RESTART           (frame closes)",
    0xA8: "SLA+R rx, ACKed        (read frame opens)",
    0xB0: "SLA+R rx (arb lost)    (ABNORMAL for pure slave)",
    0xB8: "data tx, master ACKed  (wants more)",
    0xC0: "data tx, master NACKed (read complete)",
    0xC8: "last byte tx, ACKed    (master over-reading)",
    0xF8: "no relevant state      (should never be logged)",
}

# Healthy vocabularies
WRITE_OK = [0x60, 0x80, 0x80, 0xA0]          # 2-byte command write
READ_OK  = [0xA8, 0xB8, 0xB8, 0xB8, 0xC0]    # 4-byte telemetry read


def looks_like_memory_export(text: str) -> bool:
    """Export = hex pairs that themselves decode to the stream's ASCII."""
    tokens = re.findall(r"\b[0-9A-Fa-f]{2}\b", text)
    if len(tokens) < 8:
        return False
    decoded = bytes(int(t, 16) for t in tokens)
    printable = sum(1 for b in decoded
                    if chr(b) in "0123456789ABCDEFRST !\r\nF")
    # Stream chars decoded from export are ~all in that alphabet.
    # The stream ITSELF read as hex pairs would decode to garbage.
    return printable / len(decoded) > 0.85


def extract_stream(text: str) -> str:
    if looks_like_memory_export(text):
        tokens = re.findall(r"\b[0-9A-Fa-f]{2}\b", text)
        return "".join(chr(int(t, 16)) for t in tokens)
    return text


def tokenize(stream: str):
    """Yield ('RST',), ('OVF',), ('FAIL', twcr, twar), ('ST', code)."""
    events = []
    i = 0
    parts = stream.replace("\r", "\n").split("\n")
    for line in parts:
        line = line.strip()
        if not line:
            events.append(("GAP",))          # quiet-period newline
            continue
        toks = line.split()
        j = 0
        while j < len(toks):
            t = toks[j]
            if t == "RST":
                twcr = toks[j+1] if j+1 < len(toks) else "??"
                twar = toks[j+2] if j+2 < len(toks) else "??"
                events.append(("RST", twcr, twar))
                j += 3
            elif t == "F":
                twcr = toks[j+1] if j+1 < len(toks) else "??"
                twar = toks[j+2] if j+2 < len(toks) else "??"
                events.append(("FAIL", twcr, twar))
                j += 3
            elif t == "!":
                events.append(("OVF",))
                j += 1
            elif re.fullmatch(r"[0-9A-Fa-f]{2}", t):
                events.append(("ST", int(t, 16)))
                j += 1
            else:
                events.append(("JUNK", t))
                j += 1
    return events


def group_transactions(events):
    """Split status codes into frames on the opener codes 0x60/0xA8."""
    frames, cur = [], []
    for ev in events:
        if ev[0] != "ST":
            if cur:
                frames.append(cur)
                cur = []
            continue
        code = ev[1]
        if code in (0x60, 0xA8) and cur:
            frames.append(cur)
            cur = []
        cur.append(code)
        if code == 0xA0:
            frames.append(cur)
            cur = []
    if cur:
        frames.append(cur)
    return frames


def classify(frame):
    if frame == WRITE_OK:
        return "WRITE-OK"
    if frame == READ_OK:
        return "READ-OK"
    if all(c == 0x00 for c in frame):
        return "BUS-ERROR-STORM"
    if frame[0] == 0x60:
        return "WRITE-PARTIAL"
    if frame[0] == 0xA8:
        return "READ-PARTIAL"
    return "OTHER"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    with open(sys.argv[1], "r", errors="replace") as f:
        text = f.read()
    head = None
    if "--head" in sys.argv:
        head = int(sys.argv[sys.argv.index("--head") + 1])

    stream = extract_stream(text)
    events = tokenize(stream)

    statuses = [e[1] for e in events if e[0] == "ST"]
    rsts     = [e for e in events if e[0] == "RST"]
    fails    = [e for e in events if e[0] == "FAIL"]
    ovfs     = sum(1 for e in events if e[0] == "OVF")
    frames   = group_transactions(events)
    kinds    = Counter(classify(f) for f in frames)
    scount   = Counter(statuses)

    # ---------- paste-ready report ----------
    L = []
    L.append("=== TWI DEBUG DUMP REPORT (paste this whole block) ===")
    L.append(f"dbg_head: {head if head is not None else 'NOT PROVIDED'}")
    L.append(f"status bytes: {len(statuses)}   frames: {len(frames)}   "
             f"ring overflows('!'): {ovfs}")
    L.append(f"RST banners: {len(rsts)}  " +
             " ".join(f"[TWCR={r[1]} TWAR={r[2]}]" for r in rsts[:4]))
    if fails:
        L.append(f"FAIL dumps: {len(fails)}  " +
                 " ".join(f"[TWCR={f_[1]} TWAR={f_[2]}]" for f_ in fails[:4]))
    L.append("")
    L.append("-- status histogram --")
    for code, n in sorted(scount.items()):
        L.append(f"  0x{code:02X} x{n:<5} {STATUS_NAMES.get(code, 'UNKNOWN CODE')}")
    L.append("")
    L.append("-- frame classification --")
    for k, n in kinds.most_common():
        L.append(f"  {k:<16} x{n}")
    L.append("")
    L.append("-- first 12 frames verbatim --")
    for f_ in frames[:12]:
        tag = classify(f_)
        L.append("  [" + " ".join(f"{c:02X}" for c in f_) + f"]  {tag}")
    L.append("")
    # ---------- verdict hint ----------
    L.append("-- auto-verdict (heuristic, confirm with tutor) --")
    if not statuses:
        L.append("  NO status bytes: AVR never entered TWI ISR during capture.")
        L.append("  -> address never matching, or no traffic reached the node.")
    elif kinds.get("BUS-ERROR-STORM"):
        L.append("  Bus-error storm present -> fork (a): physical layer.")
    elif kinds.get("WRITE-OK") and kinds.get("READ-OK"):
        L.append("  Both legs complete -> AVR looks innocent;")
        L.append("  suspect STM32-side is_online/HAL return handling.")
    elif kinds.get("WRITE-OK") and not kinds.get("READ-OK"):
        L.append("  Write leg fine, read leg absent/partial -> fork (b),")
        L.append("  fault in SLA+R path.")
    else:
        L.append("  Mixed/partial pattern - send full report for analysis.")
    L.append("=== END REPORT ===")

    print("\n".join(L))
    with open("twi_report.txt", "w") as f:
        f.write("\n".join(L))
    print("\n(saved to twi_report.txt)")


if __name__ == "__main__":
    main()