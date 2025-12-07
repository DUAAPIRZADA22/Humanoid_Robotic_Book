#!/usr/bin/env python3
"""
Asset management script for optimizing and organizing images and diagrams.
"""

import os
import subprocess
from pathlib import Path

def optimize_image(input_path, output_path=None):
    """Optimize image size and format."""
    if output_path is None:
        output_path = input_path

    # Try with ImageMagick if available
    try:
        # Convert to webp for better compression
        if input_path.suffix.lower() in ['.png', '.jpg', '.jpeg']:
            subprocess.run([
                'convert', str(input_path),
                '-resize', '1200x800>',
                '-quality', '85',
                str(output_path.with_suffix('.webp'))
            ], check=True)
            print(f"  Converted to WebP: {output_path.with_suffix('.webp')}")
    except (subprocess.CalledProcessError, FileNotFoundError):
        print(f"  ImageMagick not available - keeping original: {input_path}")

def check_image_sizes():
    """Check image sizes and report large ones."""
    static_dir = Path('static')
    if not static_dir.exists():
        print("No static directory found")
        return

    print("Checking image sizes...")
    print("-" * 50)

    # Find all image files
    image_extensions = ['.png', '.jpg', '.jpeg', '.gif', '.svg', '.webp']
    large_images = []

    for ext in image_extensions:
        for img_file in static_dir.rglob(f'*{ext}'):
            size_kb = img_file.stat().st_size / 1024

            if size_kb > 500:  # Larger than 500KB
                large_images.append((img_file, size_kb))

    # Report large images
    if large_images:
        print("Large images found (>500KB):")
        for img_file, size in large_images:
            print(f"  {img_file}: {size:.1f}KB")
            optimize_image(img_file)
    else:
        print("✅ All images are properly sized!")

def generate_image_index():
    """Generate an index of all images for reference."""
    static_dir = Path('static')
    output_file = Path('docs/assets/image-index.md')

    if not static_dir.exists():
        return

    output_file.parent.mkdir(exist_ok=True)

    with open(output_file, 'w') as f:
        f.write("# Image Asset Index\n\n")
        f.write("This file is auto-generated. Do not edit manually.\n\n")

        # Group by chapter
        for chapter_dir in sorted(static_dir.glob('chapter-*')):
            f.write(f"## {chapter_dir.name}\n\n")

            for img_file in sorted(chapter_dir.rglob('*.*')):
                if img_file.is_file() and img_file.suffix.lower() in ['.png', '.jpg', '.jpeg', '.svg', '.webp']:
                    rel_path = img_file.relative_to(static_dir)
                    f.write(f"- [{img_file.name}]({rel_path})\n")

            f.write("\n")

    print(f"Generated image index: {output_file}")

def main():
    """Main asset management function."""
    import argparse

    parser = argparse.ArgumentParser(description='Manage assets')
    parser.add_argument('--check', action='store_true', help='Check image sizes')
    parser.add_argument('--optimize', action='store_true', help='Optimize images')
    parser.add_argument('--index', action='store_true', help='Generate image index')
    args = parser.parse_args()

    if args.check:
        check_image_sizes()
    if args.optimize:
        check_image_sizes()
    if args.index:
        generate_image_index()

if __name__ == "__main__":
    main()