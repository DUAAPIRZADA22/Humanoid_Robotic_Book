/**
 * Database Migration Script
 * Run: node src/db/migrate.js
 */

import dotenv from 'dotenv';
import { neon, neonConfig } from '@neondatabase/serverless';
import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';

dotenv.config();

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

// Configure Neon
neonConfig.fetchConnectionCache = true;

/**
 * Run database migration
 */
async function migrate() {
  const databaseUrl = process.env.NEON_DATABASE_URL;

  if (!databaseUrl) {
    console.error('❌ NEON_DATABASE_URL not set in .env');
    process.exit(1);
  }

  console.log('🔄 Connecting to Neon PostgreSQL...');
  const sql = neon(databaseUrl);

  try {
    // Read schema SQL
    const schemaPath = join(__dirname, 'schema.sql');
    const schema = readFileSync(schemaPath, 'utf-8');

    console.log('📜 Applying schema...');

    // Split by semicolon and execute each statement
    const statements = schema
      .split(';')
      .map(s => s.trim())
      .filter(s => s.length > 0 && !s.startsWith('--'));

    for (const statement of statements) {
      if (statement.length > 50) {  // Only execute non-empty statements
        await sql.query(statement);
      }
    }

    console.log('✅ Migration completed successfully!');
    console.log('');

    // Verify table creation
    const result = await sql`
      SELECT table_name
      FROM information_schema.tables
      WHERE table_schema = 'public'
      AND table_name = 'translations'
    `;

    if (result.length > 0) {
      console.log('✅ Verified: "translations" table exists');

      // Get table info
      const columns = await sql`
        SELECT column_name, data_type, is_nullable
        FROM information_schema.columns
        WHERE table_name = 'translations'
        ORDER BY ordinal_position
      `;

      console.log('');
      console.log('📊 Table structure:');
      console.table(columns);
    } else {
      console.error('❌ Table "translations" was not created');
    }

  } catch (error) {
    console.error('❌ Migration failed:', error.message);
    process.exit(1);
  }
}

// Run migration
migrate();
