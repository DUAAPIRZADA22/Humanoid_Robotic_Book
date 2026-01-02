/**
 * /translate API Endpoint
 * Handles translation requests with caching
 */

import { getTranslator } from '../agents/TranslatorAgent.js';
import {
  getCachedTranslation,
  saveTranslation,
  getCacheStats,
} from '../db/client.js';
import { getLanguageInfo } from '../config/openrouter.js';

/**
 * Register translation routes
 * @param {FastifyInstance} fastify - Fastify instance
 */
export async function translateRoutes(fastify) {
  // Health check
  fastify.get('/health', async (request, reply) => {
    const translator = getTranslator();
    const health = await translator.healthCheck();
    const stats = await getCacheStats();

    return {
      service: 'docusaurus-translation',
      status: 'running',
      translator: health,
      cache: stats
        ? {
            enabled: true,
            total_translations: stats.total_translations,
            unique_pages: stats.unique_pages,
          }
        : { enabled: false },
      timestamp: new Date().toISOString(),
    };
  });

  // Main translate endpoint
  fastify.post('/translate', {
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
          skipCache: {
            type: 'boolean',
            default: false,
            description: 'Force re-translation, bypass cache',
          },
        },
      },
      response: {
        200: {
          type: 'object',
          properties: {
            success: { type: 'boolean' },
            translatedText: { type: 'string' },
            originalText: { type: 'string' },
            language: { type: 'string' },
            fromCache: { type: 'boolean' },
            characterCount: { type: 'number' },
            model: { type: 'string' },
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
    const { content, pageSlug, targetLang = 'ur', skipCache = false } = request.body;

    // Validate content
    if (!content || content.trim().length === 0) {
      return reply.code(400).send({
        success: false,
        error: 'Content cannot be empty',
      });
    }

    // Validate page slug
    if (!pageSlug || pageSlug.trim().length === 0) {
      return reply.code(400).send({
        success: false,
        error: 'pageSlug is required',
      });
    }

    // Get language info
    const langInfo = getLanguageInfo(targetLang);

    try {
      // Check cache first (unless skipped)
      if (!skipCache) {
        const cached = await getCachedTranslation(pageSlug, targetLang);
        if (cached) {
          fastify.log.info(`Cache hit for: ${pageSlug} (${targetLang})`);
          return {
            success: true,
            translatedText: cached,
            originalText: content,
            language: targetLang,
            languageName: langInfo.name,
            direction: langInfo.direction,
            fromCache: true,
            characterCount: content.length,
          };
        }
      }

      // Get translator and perform translation
      const translator = getTranslator();
      const model = translator.model;

      fastify.log.info(`Translating: ${pageSlug} (${targetLang}) using ${model}`);

      const translatedText = await translator.translate(content, targetLang);

      // Save to cache
      await saveTranslation(pageSlug, targetLang, content, translatedText, model);

      return {
        success: true,
        translatedText,
        originalText: content,
        language: targetLang,
        languageName: langInfo.name,
        direction: langInfo.direction,
        fromCache: false,
        characterCount: content.length,
        model,
      };

    } catch (error) {
      fastify.log.error(error);

      // Handle specific error types
      if (error.message.includes('timeout')) {
        return reply.code(504).send({
          success: false,
          error: 'Translation request timed out. Please try again.',
        });
      }

      if (error.message.includes('rate limit')) {
        return reply.code(429).send({
          success: false,
          error: 'Rate limit exceeded. Please try again later.',
        });
      }

      if (error.message.includes('API key')) {
        return reply.code(500).send({
          success: false,
          error: 'Translation service configuration error.',
        });
      }

      // Generic error
      return reply.code(500).send({
        success: false,
        error: error.message || 'Translation failed. Please try again.',
      });
    }
  });

  // Cache statistics endpoint
  fastify.get('/cache/stats', async (request, reply) => {
    const stats = await getCacheStats();

    if (!stats) {
      return reply.code(503).send({
        success: false,
        error: 'Database not configured',
      });
    }

    return {
      success: true,
      stats,
    };
  });

  // Clear cache endpoint (use with caution)
  fastify.delete('/cache', async (request, reply) => {
    const { clear } = request.query;

    if (clear !== 'all') {
      return reply.code(400).send({
        success: false,
        error: 'Add ?clear=all to confirm cache deletion',
      });
    }

    const { clearCache } = await import('../db/client.js');
    await clearCache();

    return {
      success: true,
      message: 'Cache cleared successfully',
    };
  });

  // Batch translate endpoint (for multiple pages)
  fastify.post('/translate/batch', {
    schema: {
      body: {
        type: 'object',
        required: ['pages'],
        properties: {
          pages: {
            type: 'array',
            items: {
              type: 'object',
              required: ['content', 'pageSlug'],
              properties: {
                content: { type: 'string' },
                pageSlug: { type: 'string' },
                targetLang: { type: 'string', default: 'ur' },
              },
            },
            maxItems: 10,
          },
        },
      },
    },
  }, async (request, reply) => {
    const { pages } = request.body;

    if (pages.length > 10) {
      return reply.code(400).send({
        success: false,
        error: 'Maximum 10 pages per batch request',
      });
    }

    const results = [];
    const translator = getTranslator();

    for (const page of pages) {
      const { content, pageSlug, targetLang = 'ur' } = page;

      try {
        // Check cache first
        const cached = await getCachedTranslation(pageSlug, targetLang);

        if (cached) {
          results.push({
            pageSlug,
            success: true,
            translatedText: cached,
            fromCache: true,
          });
          continue;
        }

        // Translate
        const translatedText = await translator.translate(content, targetLang);
        await saveTranslation(pageSlug, targetLang, content, translatedText, translator.model);

        results.push({
          pageSlug,
          success: true,
          translatedText,
          fromCache: false,
        });

      } catch (error) {
        results.push({
          pageSlug,
          success: false,
          error: error.message,
        });
      }
    }

    return {
      success: true,
      results,
      total: pages.length,
      successful: results.filter(r => r.success).length,
      failed: results.filter(r => !r.success).length,
    };
  });
}
