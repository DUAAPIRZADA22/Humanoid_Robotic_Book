#!/usr/bin/env python3
"""
Check word count for chapters to ensure they meet minimum requirements.
"""

import os
import re
from pathlib import Path

def count_words(file_path):
    """Count words in a markdown file."""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Remove code blocks and frontmatter
    content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.MULTILINE | re.DOTALL)

    # Count words
    words = content.split()
    return len(words)

def main():
    """Main function to check word counts."""
    docs_dir = Path('docs')
    min_words = 2000

    print("Checking chapter word counts...")
    print(f"Minimum words required: {min_words}")
    print("-" * 50)

    # Find all chapter files
    chapter_files = list(docs_dir.rglob('chapter-*.md'))

    all_passed = True
    for chapter_file in sorted(chapter_files):
        word_count = count_words(chapter_file)
        status = "✅ PASS" if word_count >= min_words else "❌ FAIL"
        print(f"{chapter_file.name:40} {word_count:6d} words {status}")

        if word_count < min_words:
            all_passed = False

    print("-" * 50)

    if all_passed:
        print("✅ All chapters meet word count requirements!")
        return 0
    else:
        print("❌ Some chapters are too short!")
        return 1

if __name__ == "__main__":
    exit(main())