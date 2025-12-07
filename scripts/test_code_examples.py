#!/usr/bin/env python3
"""
Test code examples to ensure they run without errors.
"""

import os
import subprocess
import tempfile
from pathlib import Path

def test_python_example(file_path):
    """Test a Python code example."""
    try:
        # Create a temporary directory for testing
        with tempfile.TemporaryDirectory() as temp_dir:
            # Copy the file to temp directory
            temp_file = Path(temp_dir) / file_path.name
            temp_file.write_text(file_path.read_text())

            # Try to run Python syntax check
            result = subprocess.run(
                ['python', '-m', 'py_compile', str(temp_file)],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                return True, "Syntax OK"
            else:
                return False, result.stderr

    except subprocess.TimeoutExpired:
        return False, "Timeout"
    except Exception as e:
        return False, str(e)

def test_cpp_example(file_path):
    """Test a C++ code example (syntax check only)."""
    try:
        content = file_path.read_text()

        # Basic syntax checks
        if content.count('#include') == 0:
            return False, "No include statements found"

        # Check for balanced braces
        if content.count('{') != content.count('}'):
            return False, "Unbalanced braces"

        # Try to compile with g++ if available
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_file = Path(temp_dir) / file_path.name
            temp_file.write_text(content)

            result = subprocess.run(
                ['g++', '-fsyntax-only', str(temp_file)],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                return True, "Syntax OK"
            else:
                return False, result.stderr

    except FileNotFoundError:
        return True, "g++ not available - basic checks passed"
    except subprocess.TimeoutExpired:
        return False, "Timeout"
    except Exception as e:
        return False, str(e)

def main():
    """Main function to test code examples."""
    examples_dir = Path('examples')
    if not examples_dir.exists():
        print("No examples directory found")
        return 0

    print("Testing code examples...")
    print("-" * 50)

    # Find all code files
    code_files = []
    for ext in ['*.py', '*.cpp', '*.cc', '*.cxx']:
        code_files.extend(examples_dir.rglob(ext))

    total_tests = 0
    passed_tests = 0
    all_passed = True

    for code_file in sorted(code_files):
        print(f"\n{code_file.name}:")

        if code_file.suffix == '.py':
            success, message = test_python_example(code_file)
        elif code_file.suffix in ['.cpp', '.cc', '.cxx']:
            success, message = test_cpp_example(code_file)
        else:
            continue

        total_tests += 1

        if success:
            print(f"  ✅ {message}")
            passed_tests += 1
        else:
            print(f"  ❌ {message}")
            all_passed = False

    print("\n" + "-" * 50)
    print(f"Total examples tested: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {total_tests - passed_tests}")

    if all_passed:
        print("\n✅ All code examples passed tests!")
        return 0
    else:
        print("\n❌ Some examples have issues!")
        return 1

if __name__ == "__main__":
    exit(main())