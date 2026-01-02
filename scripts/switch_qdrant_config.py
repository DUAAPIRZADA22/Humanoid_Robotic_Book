#!/usr/bin/env python3
"""
Qdrant Configuration Switcher
Switches between local and Qdrant Cloud configurations
"""

import os
import sys
from pathlib import Path

def update_env_file(mode):
    """Update .env file with specified mode"""
    env_file = Path(".env")

    if not env_file.exists():
        print("Error: .env file not found!")
        print("Please copy .env.example to .env first")
        sys.exit(1)

    # Read current content
    with open(env_file, 'r') as f:
        lines = f.readlines()

    # Update based on mode
    new_lines = []
    in_qdrant_section = False

    for line in lines:
        if "Vector Database Configuration (Qdrant)" in line:
            in_qdrant_section = True
            new_lines.append(line)
            continue

        if in_qdrant_section and line.strip().startswith("#"):
            if "Application Configuration" in line:
                in_qdrant_section = False

        if in_qdrant_section:
            # Modify Qdrant settings
            if mode == "local":
                if line.startswith("QDRANT_HOST"):
                    line = "QDRANT_HOST=localhost\n"
                elif line.startswith("QDRANT_PORT"):
                    line = "QDRANT_PORT=6333\n"
                elif line.startswith("# QDRANT_URL="):
                    line = "# QDRANT_URL=\n"
                elif line.startswith("QDRANT_URL="):
                    line = "QDRANT_URL=\n"
                elif line.startswith("# QDRANT_API_KEY=") and "your_qdrant_cloud_api_key" in line:
                    line = "QDRANT_API_KEY=your_qdrant_api_key_if_cloud\n"
                elif line.startswith("QDRANT_API_KEY=") and "your_qdrant_cloud_api_key" in line:
                    line = "QDRANT_API_KEY=your_qdrant_api_key_if_cloud\n"

            elif mode == "cloud":
                if line.startswith("QDRANT_URL=") and not line.strip().startswith("#"):
                    # Keep existing URL if set
                    pass
                elif line.startswith("# QDRANT_URL="):
                    # Uncomment
                    line = line.lstrip("# ")
                elif line.startswith("QDRANT_HOST="):
                    line = "# QDRANT_HOST=localhost\n"
                elif line.startswith("QDRANT_PORT="):
                    line = "# QDRANT_PORT=6333\n"

        new_lines.append(line)

    # Write back
    with open(env_file, 'w') as f:
        f.writelines(new_lines)

    return True

def main():
    """Main function"""
    print("Qdrant Configuration Switcher")
    print("=" * 40)

    if len(sys.argv) != 2:
        print("Usage: python switch_qdrant_config.py [local|cloud]")
        print("\nModes:")
        print("  local - Use local Qdrant server (localhost:6333)")
        print("  cloud - Use Qdrant Cloud (requires QDRANT_URL)")
        sys.exit(1)

    mode = sys.argv[1].lower()

    if mode not in ["local", "cloud"]:
        print(f"Error: Invalid mode '{mode}'")
        print("Valid modes: local, cloud")
        sys.exit(1)

    print(f"\nSwitching to {mode} Qdrant configuration...")

    if update_env_file(mode):
        print(f"\n✓ Configuration updated to {mode} mode")

        if mode == "local":
            print("\nNext steps:")
            print("1. Start local Qdrant: docker run -p 6333:6333 qdrant/qdrant")
            print("2. Test connection: python backend/test_qdrant.py")

        elif mode == "cloud":
            print("\nNext steps:")
            print("1. Set your QDRANT_URL in .env")
            print("2. Set your QDRANT_API_KEY in .env")
            print("3. Test connection: python backend/test_qdrant.py")
            print("\nFor cloud setup, see: QDRANT_CLOUD_SETUP.md")
    else:
        print("\n✗ Failed to update configuration")
        sys.exit(1)

if __name__ == "__main__":
    main()