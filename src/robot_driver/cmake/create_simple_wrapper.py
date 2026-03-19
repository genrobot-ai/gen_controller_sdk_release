#!/usr/bin/env python3
"""
Minimal helper: compile selected scripts to bytecode.
"""
import sys
import os
import py_compile
from pathlib import Path

def compile_files(source_dir, output_dir):
    """Compile listed .py files to optimized bytecode."""
    source_dir = Path(source_dir)
    output_dir = Path(output_dir)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    files = ["camera_view_single.py", "databus_single.py", "pack.py", "das_protocol.py"]
    
    for filename in files:
        source_file = source_dir / filename
        if source_file.exists():
            py_compile.compile(
                str(source_file),
                cfile=str(output_dir / f"{filename}c"),
                doraise=True,
                optimize=2
            )
            print(f"Compiled: {filename}")
        else:
            print(f"Warning: missing {filename}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 create_simple_wrapper.py <source_dir> <output_dir>")
        sys.exit(1)
    
    compile_files(sys.argv[1], sys.argv[2])
