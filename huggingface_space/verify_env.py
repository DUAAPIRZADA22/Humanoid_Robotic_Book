#!/usr/bin/env python3
"""
Environment Variable Verification Script for Hugging Face Spaces

Run this script to check if required environment variables are set.
This helps diagnose "limited mode" startup issues.
"""

import os
import sys
from dotenv import load_dotenv

# Load from .env if it exists (for local testing)
load_dotenv()

def check_env_var(name: str, required: bool = False) -> tuple[bool, str]:
    """Check if an environment variable is set."""
    value = os.getenv(name)
    is_set = value is not None and value != "" and value != f"{{{{ {name} }}}}"

    if is_set:
        # Mask sensitive values
        if "KEY" in name or "SECRET" in name or "TOKEN" in name:
            display = f"{value[:8]}...{value[-4:]}" if len(value) > 12 else "***"
        else:
            display = value
        return True, f"[OK] {name}: {display}"
    elif required:
        return False, f"[MISSING] {name}: NOT SET (REQUIRED)"
    else:
        return False, f"[OPTIONAL] {name}: Not set"


def main():
    """Main verification function."""
    print("=" * 60)
    print("Hugging Face Spaces Environment Verification")
    print("=" * 60)
    print()

    # Required variables for full chat functionality
    required_vars = [
        "OPENROUTER_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY",
    ]

    # Optional variables
    optional_vars = [
        "OPENROUTER_BASE_URL",
        "QDRANT_HOST",
        "QDRANT_PORT",
        "CORS_ORIGINS",
        "COHERE_API_KEY",
        "OPENAI_API_KEY",
    ]

    all_good = True

    print("REQUIRED Variables (for full chat functionality):")
    print("-" * 60)
    for var in required_vars:
        is_set, msg = check_env_var(var, required=True)
        print(msg)
        if not is_set:
            all_good = False

    print()
    print("OPTIONAL Variables:")
    print("-" * 60)
    for var in optional_vars:
        is_set, msg = check_env_var(var, required=False)
        print(msg)

    print()
    print("=" * 60)

    if all_good:
        print("[OK] All required variables are set!")
        print("  Backend should start in FULL mode with all features enabled.")
        return 0
    else:
        print("[ERROR] Some required variables are missing!")
        print()
        print("  For Hugging Face Spaces deployment:")
        print("  1. Go to your Space Settings -> Variables and secrets")
        print("  2. Add the missing variables with your actual values")
        print("  3. Click 'Restart' to apply changes")
        print()
        print("  For local testing:")
        print("  1. Copy .env.template to .env")
        print("  2. Fill in your actual API keys")
        print("  3. Run: python verify_env.py")
        return 1


if __name__ == "__main__":
    sys.exit(main())
