#!/usr/bin/env python3
"""
Create Python bytecode launcher wrappers.
Usage: python3 create_wrappers.py --source-dir <src> --output-dir <out>
"""
import argparse
import os
import sys
import marshal
import py_compile
import shutil
from pathlib import Path

def create_wrapper(module_name, output_dir):
    """Write a small script that loads <module_name>.pyc and runs main()."""
    wrapper_content = f'''#!/usr/bin/env python3
"""
{module_name} wrapper - load from bytecode
"""
import sys
import os
import marshal
import types

def load_pyc_module(pyc_path, module_name):
    """Load module from .pyc (Python 3.7+ header is 16 bytes)."""
    with open(pyc_path, 'rb') as f:
        magic = f.read(16)
        code = marshal.load(f)
    
    module = types.ModuleType(module_name)
    
    module.__file__ = pyc_path
    
    exec(code, module.__dict__)
    return module

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pyc_file = os.path.join(script_dir, '{module_name}.pyc')
    
    if not os.path.exists(pyc_file):
        print(f"Error: bytecode not found: {{pyc_file}}")
        sys.exit(1)
    
    try:
        module = load_pyc_module(pyc_file, '{module_name}')
        
        if hasattr(module, 'main'):
            sys.argv[0] = '{module_name}.py'
            module.main()
        else:
            print(f"Error: no main() in module '{module_name}'")
            sys.exit(1)
    except Exception as e:
        print(f"Runtime error: {{e}}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
'''
    
    wrapper_path = os.path.join(output_dir, f'{module_name}.py')
    with open(wrapper_path, 'w', encoding='utf-8') as f:
        f.write(wrapper_content)
    
    os.chmod(wrapper_path, 0o755)
    print(f"Created wrapper: {wrapper_path}")

def compile_python(source_dir, output_dir):
    """Compile *.py in source_dir to optimized .pyc in output_dir."""
    source_dir = Path(source_dir)
    output_dir = Path(output_dir)
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    for py_file in source_dir.glob('*.py'):
        pyc_file = output_dir / f'{py_file.stem}.pyc'
        
        try:
            py_compile.compile(
                str(py_file),
                cfile=str(pyc_file),
                doraise=True,
                optimize=2
            )
            print(f"Compiled: {py_file.name} -> {pyc_file.name}")
        except Exception as e:
            print(f"Compile failed {py_file.name}: {e}")
            sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description='Create Python bytecode wrappers')
    parser.add_argument('--source-dir', required=True, help='Python source directory')
    parser.add_argument('--output-dir', required=True, help='Output directory')
    
    args = parser.parse_args()
    
    print("Compiling .py to bytecode...")
    compile_python(args.source_dir, args.output_dir)
    
    print("Creating wrapper scripts...")
    create_wrapper('camera_view_single', args.output_dir)
    create_wrapper('databus_single', args.output_dir)
    
    print("Done.")
    
    output_path = Path(args.output_dir)
    print(f"\nOutput files:")
    for file in output_path.iterdir():
        print(f"  - {file.name}")

if __name__ == '__main__':
    main()
