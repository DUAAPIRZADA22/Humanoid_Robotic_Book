/**
 * Neon PostgreSQL Client
 * Handles translation caching operations
 */

import { neon, neonConfig } from '@neondatabase/serverless';
import crypto from 'crypto';

neonConfig.fetchConnectionCache = true;

let sql = null;

/**
 * Initialize database connection
 */
export function initDatabase() {
  const databaseUrl = process.env.NEON_DATABASE_URL;

  if (!databaseUrl) {
    console.warn('⚠️  NEON_DATABASE_URL not set - cache disabled');
    return null;
  }

  try {
    sql = neon(databaseUrl);
    console.log('✅ Neon database connected');
    return sql;
  } catch (error) {
    console.error('❌ Failed to connect to Neon:', error.message);
    return null;
  }
}

/**
 * Generate SHA-256 hash of content
 */
function generateHash(text) {
  return crypto.createHash('sha256').update(text).digest('hex');
}

/**
 * Get cached translation from database
 * @param {string} pageSlug - Page slug/identifier
 * @param {string} language - Target language code (e.g., 'ur')
 * @returns {Promise<string|null>} Cached translation or null
 */
export async function getCachedTranslation(pageSlug, language) {
  if (!sql) return null;

  try {
    const result = await sql`
      SELECT translated_text
      FROM translations
      WHERE page_slug = ${pageSlug}
      AND language_code = ${language}
      LIMIT 1
    `;

    if (result.length > 0) {
      // Update last accessed time
      await sql`
        UPDATE translations
        SET last_accessed_at = CURRENT_TIMESTAMP
        WHERE page_slug = ${pageSlug}
        AND language_code = ${language}
      `;
      return result[0].translated_text;
    }

    return null;
  } catch (error) {
    console.error('❌ Cache get error:', error.message);
    return null;
  }
}

/**
 * Save translation to database cache
 * @param {string} pageSlug - Page slug/identifier
 * @param {string} language - Target language code
 * @param {string} originalText - Original text
 * @param {string} translatedText - Translated text
 * @param {string} model - Model used for translation
 */
export async function saveTranslation(pageSlug, language, originalText, translatedText, model) {
  if (!sql) return;

  try {
    const contentHash = generateHash(originalText);
    const characterCount = originalText.length;

    // Check if exists (for upsert)
    const existing = await sql`
      SELECT id FROM translations
      WHERE page_slug = ${pageSlug}
      AND language_code = ${language}
      LIMIT 1
    `;

    if (existing.length > 0) {
      // Update existing
      await sql`
        UPDATE translations
        SET
          original_text = ${originalText},
          translated_text = ${translatedText},
          content_hash = ${contentHash},
          model_used = ${model},
          character_count = ${characterCount},
          updated_at = CURRENT_TIMESTAMP,
          last_accessed_at = CURRENT_TIMESTAMP
        WHERE page_slug = ${pageSlug}
        AND language_code = ${language}
      `;
      console.log('📝 Cache updated for:', pageSlug);
    } else {
      // Insert new
      await sql`
        INSERT INTO translations (
          page_slug,
          language_code,
          original_text,
          translated_text,
          content_hash,
          model_used,
          character_count
        ) VALUES (
          ${pageSlug},
          ${language},
          ${originalText},
          ${translatedText},
          ${contentHash},
          ${model},
          ${characterCount}
        )
      `;
      console.log('💾 Cache saved for:', pageSlug);
    }
  } catch (error) {
    console.error('❌ Cache save error:', error.message);
  }
}

/**
 * Get cache statistics
 */
export async function getCacheStats() {
  if (!sql) return null;

  try {
    const stats = await sql`
      SELECT
        COUNT(*) as total_translations,
        COUNT(DISTINCT page_slug) as unique_pages,
        COUNT(DISTINCT language_code) as languages_supported,
        SUM(character_count) as total_characters,
        MAX(created_at) as latest_translation
      FROM translations
    `;
    return stats[0];
  } catch (error) {
    console.error('❌ Stats error:', error.message);
    return null;
  }
}

/**
 * Clear all cached translations (use with caution)
 */
export async function clearCache() {
  if (!sql) return;

  try {
    await sql`DELETE FROM translations`;
    console.log('🗑️  Cache cleared');
  } catch (error) {
    console.error('❌ Clear cache error:', error.message);
  }
}

export { sql };
