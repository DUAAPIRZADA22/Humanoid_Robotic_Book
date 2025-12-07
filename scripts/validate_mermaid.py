#!/usr/bin/env python3
"""
Validate Mermaid.js diagram syntax in markdown files.
"""

import os
import re
from pathlib import Path

def extract_mermaid_blocks(file_path):
    """Extract Mermaid code blocks from markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Find all Mermaid blocks
    mermaid_blocks = re.findall(r'```mermaid\n(.*?)\n```', content, flags=re.DOTALL)
    return mermaid_blocks

def validate_mermaid_syntax(mermaid_code):
    """Basic Mermaid syntax validation."""
    # Check for required diagram type
    diagram_types = [
        'flowchart', 'graph', 'sequenceDiagram', 'classDiagram',
        'stateDiagram', 'erDiagram', 'journey', 'gantt', 'pie',
        'gitgraph', 'C4Context', 'mindmap', 'timeline'
    ]

    # Remove comments and empty lines
    lines = [line.strip() for line in mermaid_code.split('\n') if line.strip() and not line.strip().startswith('%%')]

    if not lines:
        return False, "Empty diagram"

    # Check first line for diagram type
    first_line = lines[0].lower()
    if not any(dt.lower() in first_line for dt in diagram_types):
        return False, f"No valid diagram type found. First line: {first_line}"

    # Basic syntax checks
    for line in lines:
        # Check for unclosed quotes
        if line.count('"') % 2 != 0:
            return False, f"Unclosed quotes in: {line[:50]}"

    return True, "Valid"

def main():
    """Main function to validate Mermaid diagrams."""
    docs_dir = Path('docs')
    print("Validating Mermaid diagrams...")
    print("-" * 50)

    # Find all markdown files
    md_files = list(docs_dir.rglob('*.md'))

    total_diagrams = 0
    valid_diagrams = 0
    all_valid = True

    for md_file in sorted(md_files):
        mermaid_blocks = extract_mermaid_blocks(md_file)

        if mermaid_blocks:
            print(f"\n{md_file.name}:")

            for i, block in enumerate(mermaid_blocks, 1):
                total_diagrams += 1
                is_valid, message = validate_mermaid_syntax(block)

                if is_valid:
                    print(f"  Diagram {i}: ✅ Valid")
                    valid_diagrams += 1
                else:
                    print(f"  Diagram {i}: ❌ {message}")
                    all_valid = False

    print("\n" + "-" * 50)
    print(f"Total diagrams: {total_diagrams}")
    print(f"Valid diagrams: {valid_diagrams}")
    print(f"Invalid diagrams: {total_diagrams - valid_diagrams}")

    if all_valid:
        print("\n✅ All Mermaid diagrams are valid!")
        return 0
    else:
        print("\n❌ Some diagrams have issues!")
        return 1

if __name__ == "__main__":
    exit(main())