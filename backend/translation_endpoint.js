/**
 * Translation API Endpoint using OpenAI Agents SDK
 * @fileoverview Backend endpoint for page/chapter translation
 */

import { runTranslationAgent } from './agents/translation_agent.js';

/**
 * Setup translation routes
 * @param {FastifyInstance} fastify - Fastify instance
 */
export async function setupTranslationRoutes(fastify) {
  /**
   * GET /api/translation/health
   * Health check for translation service
   */
  fastify.get('/api/translation/health', async (request, reply) => {
    return {
      status: 'healthy',
      service: 'translation-agent',
      version: '2.0.0',
      timestamp: new Date().toISOString(),
    };
  });

  /**
   * POST /api/translation/translate
   * Main translation endpoint using Agent
   */
  fastify.post('/api/translation/translate', {
    schema: {
      body: {
        type: 'object',
        required: ['content', 'pageSlug'],
        properties: {
          content: {
            type: 'string',
            minLength: 1,
            maxLength: 100000,
            description: 'Markdown content to translate',
          },
          pageSlug: {
            type: 'string',
            minLength: 1,
            maxLength: 500,
            description: 'Page identifier for caching',
          },
          targetLang: {
            type: 'string',
            pattern: '^[a-z]{2}$',
            default: 'ur',
            description: 'Target language code (ISO 639-1)',
          },
        },
      },
      response: {
        200: {
          type: 'object',
          properties: {
            success: { type: 'boolean' },
            translatedContent: { type: 'string' },
            originalContent: { type: 'string' },
            language: { type: 'string' },
            characterCount: { type: 'number' },
            message: { type: 'string' },
          },
        },
        400: {
          type: 'object',
          properties: {
            success: { type: 'boolean' },
            error: { type: 'string' },
          },
        },
        500: {
          type: 'object',
          properties: {
            success: { type: 'boolean' },
            error: { type: 'string' },
          },
        },
      },
    },
  }, async (request, reply) => {
    const { content, pageSlug, targetLang = 'ur' } = request.body;

    // Validate content
    if (!content || content.trim().length === 0) {
      return reply.code(400).send({
        success: false,
        error: 'Content cannot be empty',
      });
    }

    // Check cache first (Neon)
    const cached = await getCachedTranslation(pageSlug, targetLang);
    if (cached) {
      request.log.info(`Cache hit for: ${pageSlug} (${targetLang})`);
      return {
        success: true,
        translatedContent: cached.translated_text,
        originalContent: cached.original_text,
        language: targetLang,
        characterCount: cached.character_count,
        fromCache: true,
      };
    }

    try {
      request.log.info(`Translating: ${pageSlug} (${targetLang})`);

      // RUN THE AGENT
      const result = await runTranslationAgent({
        content,
        pageSlug,
        targetLang,
      });

      // Cache the result
      await saveTranslationToCache(
        pageSlug,
        targetLang,
        content,
        result.translatedContent,
        CONFIG.model
      );

      return {
        success: true,
        ...result,
        message: 'Translation complete',
      };

    } catch (error) {
      request.log.error(error);

      return reply.code(500).send({
        success: false,
        error: error.message || 'Translation failed',
      });
    }
  });

  /**
   * GET /api/translation/cache/stats
   * Get translation cache statistics
   */
  fastify.get('/api/translation/cache/stats', async (request, reply) => {
    const stats = await getCacheStats();

    if (!stats) {
      return reply.code(503).send({
        success: false,
        error: 'Cache not available',
      });
    }

    return {
      success: true,
      stats,
    };
  });
}

// ===================================================================
// CACHE FUNCTIONS (Neon PostgreSQL)
// ===================================================================

/**
 * Get cached translation from Neon
 */
async function getCachedTranslation(pageSlug: string, targetLang: string) {
  // TODO: Implement Neon cache lookup
  // For now, return null to always use agent
  return null;
}

/**
 * Save translation to Neon cache
 */
async function saveTranslationToCache(
  pageSlug: string,
  targetLang: string,
  originalText: string,
  translatedText: string,
  model: string
) {
  // TODO: Implement Neon cache save
  console.log(`[Cache] Would save translation for: ${pageSlug}`);
}

/**
 * Get cache statistics
 */
async function getCacheStats() {
  // TODO: Implement stats query
  return { total_translations: 0 };
}

const CONFIG = {
  model: 'google/gemma-3-27b-it:free',
};
