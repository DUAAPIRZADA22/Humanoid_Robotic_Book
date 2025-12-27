-- ========================================
-- Neon PostgreSQL Schema
-- Docusaurus Translation Cache
-- ========================================

-- Drop table if exists (for clean migration)
DROP TABLE IF EXISTS translations CASCADE;

-- Create translations table
CREATE TABLE IF NOT EXISTS translations (
  -- Primary key
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),

  -- Page identification
  page_slug VARCHAR(500) NOT NULL,
  language_code VARCHAR(10) NOT NULL,

  -- Content
  original_text TEXT NOT NULL,
  translated_text TEXT NOT NULL,

  -- Metadata
  content_hash VARCHAR(64) NOT NULL,  -- SHA-256 of original text
  model_used VARCHAR(100) NOT NULL,
  character_count INTEGER NOT NULL,

  -- Timestamps
  created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
  last_accessed_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,

  -- Constraints
  CONSTRAINT unique_page_language UNIQUE (page_slug, language_code)
);

-- Create indexes for fast lookups
CREATE INDEX idx_translations_page_slug ON translations(page_slug);
CREATE INDEX idx_translations_language_code ON translations(language_code);
CREATE INDEX idx_translations_content_hash ON translations(content_hash);
CREATE INDEX idx_translations_created_at ON translations(created_at DESC);

-- Create composite index for common query pattern
CREATE INDEX idx_translations_lookup ON translations(page_slug, language_code);

-- Function to update updated_at timestamp automatically
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
  NEW.updated_at = CURRENT_TIMESTAMP;
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Trigger to auto-update updated_at
CREATE TRIGGER update_translations_updated_at
  BEFORE UPDATE ON translations
  FOR EACH ROW
  EXECUTE FUNCTION update_updated_at_column();

-- Function to update last_accessed_at
CREATE OR REPLACE FUNCTION update_last_accessed_at()
RETURNS TRIGGER AS $$
BEGIN
  NEW.last_accessed_at = CURRENT_TIMESTAMP;
  RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Trigger to update last_accessed_at on SELECT (via a separate function)
-- Note: This requires a separate UPDATE call since SELECT triggers don't exist

-- ========================================
-- Sample Queries for Testing
-- ========================================

-- Insert a translation
-- INSERT INTO translations (page_slug, language_code, original_text, translated_text, content_hash, model_used, character_count)
-- VALUES (
--   'foundation-ai-integration',
--   'ur',
--   'Hello World',
--   'ہیلو ورلڈ',
--   'a591a6d40bf420404a011733cfb7b190d62c65bf0bcda32b57b277d9ad9f146e',
--   'google/gemma-3-27b-it:free',
--   11
-- );

-- Query by page slug and language
-- SELECT * FROM translations WHERE page_slug = 'foundation-ai-integration' AND language_code = 'ur';

-- Get cache statistics
-- SELECT
--   language_code,
--   COUNT(*) as total_translations,
--   SUM(character_count) as total_characters,
--   MAX(created_at) as latest_translation
-- FROM translations
-- GROUP BY language_code;
