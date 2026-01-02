"""
Clear all database tables and reset to fresh state.
Run this script to start fresh with an empty database.
"""

import os
import sys
from dotenv import load_dotenv
from sqlmodel import Session, create_engine, text

# Set UTF-8 encoding for Windows
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')

# Load environment variables
load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    print("ERROR: DATABASE_URL not found in environment variables")
    exit(1)

print(f"Connecting to database...")
print(f"DATABASE_URL: {DATABASE_URL.split('@')[1] if '@' in DATABASE_URL else 'local'}")  # Hide password

# Create engine
engine = create_engine(DATABASE_URL)

print("\nWARNING: This will DELETE ALL DATA in the database!")
print("Proceeding automatically...")

# Drop all tables
with Session(engine) as session:
    # Get all table names
    result = session.exec(text("""
        SELECT tablename FROM pg_tables
        WHERE schemaname = 'public'
    """))
    tables = [row[0] for row in result]

    if not tables:
        print("No tables found to drop.")
    else:
        print(f"Found {len(tables)} tables: {', '.join(tables)}")

        # Drop each table
        for table in tables:
            session.exec(text(f'DROP TABLE IF EXISTS "{table}" CASCADE'))
            print(f"  Dropped table: {table}")

        session.commit()

print("\nDatabase cleared successfully!")
print("Restart the backend to recreate tables automatically.")
