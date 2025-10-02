#!/usr/bin/env python3
"""
bin2hex_mem.py

Read a binary (.bin) or bitfile and output a text file assigning each byte
to a memory address, starting from a given hex address.

Example:
  python3 bin2hex_mem.py input.bin output.txt --start-addr 16'h4000
Output:
  mem[16'h4000] = 8'hA2;
  mem[16'h4001] = 8'hC5;
  mem[16'h4002] = 8'hFF;

Options:
  --bitfile         Treat input as a text file of '0'/'1' bits.
  --start-addr HEX  Starting address (default: 16'h0000)
"""

import argparse
import sys
from pathlib import Path
from intelhex import IntelHex

def parse_start_addr(s):
    """Parse addresses like 16'h4000 or 0x4000."""
    s = s.strip().lower()
    if s.startswith("16'h"):
        return int(s[4:], 16)
    elif s.startswith("0x"):
        return int(s, 16)
    elif s.isdigit():
        return int(s)
    else:
        raise ValueError(f"Invalid start address format: {s}")

def bytes_mode(inpath, outpath, start_addr):
    addr = start_addr
    with open(inpath, "rb") as fin, open(outpath, "w", newline="\n") as fout:
        chunk = fin.read(4096)
        while chunk:
            for b in chunk:
                fout.write(f"mem[16'h{addr:04X}] = 8'h{b:02X};\n")
                addr += 1
            chunk = fin.read(4096)

def hex_mode(inpath, outpath, start_addr):
    addr = start_addr
    ih = IntelHex()

    with open(inpath, "rb") as fin:
        chunk = fin.read(4096)
        while chunk:
            for b in chunk:
                ih[addr] = b
                addr += 1
            chunk = fin.read(4096)
        ih.write_hex_file(outpath)

def bitfile_mode(inpath, outpath, start_addr):
    txt = Path(inpath).read_text()
    bits = "".join(ch for ch in txt if ch in "01")
    if len(bits) % 8 != 0:
        pad = 8 - (len(bits) % 8)
        sys.stderr.write(f"Warning: bit count {len(bits)} not multiple of 8 — padding with {pad} zeros.\n")
        bits += "0" * pad
    addr = start_addr
    with open(outpath, "w", newline="\n") as fout:
        for i in range(0, len(bits), 8):
            byte_bits = bits[i:i+8]
            val = int(byte_bits, 2)
            fout.write(f"mem[16'h{addr:04X}] = 8'h{val:02X};\n")
            addr += 1

def main():
    p = argparse.ArgumentParser(description="Convert a .bin file into SystemVerilog-style mem[] hex assignments.")
    p.add_argument("input", help="input file (.bin or text bitfile with --bitfile)")
    p.add_argument("output", help="output text file")
    p.add_argument("--bitfile", action="store_true", help="treat input as a bitfile ('0'/'1' text)")
    p.add_argument("--start-addr", type=str, default="16'h0000",
                   help="starting address (e.g. 16'h4000, 0x4000, or decimal)")
    args = p.parse_args()

    try:
        start_addr = parse_start_addr(args.start_addr)
    except ValueError as e:
        sys.exit(str(e))

    inpath = Path(args.input)
    if not inpath.exists():
        sys.exit(f"Error: input file {inpath} not found.")

    try:
        if args.bitfile:
            bitfile_mode(inpath, args.output, start_addr)
        elif (args.output[-3:] == 'hex'):
            hex_mode(inpath, args.output, start_addr)
        else:
            bytes_mode(inpath, args.output, start_addr)
    except Exception as e:
        sys.exit(f"Error: {e}")

if __name__ == "__main__":
    main()
