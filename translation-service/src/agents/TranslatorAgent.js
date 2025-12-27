/**
 * TranslatorAgent using OpenAI Agents SDK
 * Handles English → Urdu translation with OpenRouter
 */

import OpenAI from 'openai';

/**
 * TranslatorAgent Class
 * Manages translation operations using OpenRouter via OpenAI SDK
 */
class TranslatorAgent {
  /**
   * Initialize the translator agent
   * @param {Object} config - Configuration object
   * @param {string} config.apiKey - OpenRouter API key
   * @param {string} config.baseURL - OpenRouter base URL
   * @param {string} config.model - Model to use for translation
   * @param {number} config.timeout - Request timeout in ms
   */
  constructor(config = {}) {
    const {
      apiKey = process.env.OPENROUTER_API_KEY,
      baseURL = process.env.OPENROUTER_BASE_URL || 'https://openrouter.ai/api/v1',
      model = process.env.TRANSLATION_MODEL || 'google/gemma-3-27b-it:free',
      timeout = 30000,
    } = config;

    if (!apiKey) {
      throw new Error('OPENROUTER_API_KEY is required');
    }

    // Initialize OpenAI client with OpenRouter configuration
    this.client = new OpenAI({
      apiKey,
      baseURL,
      timeout,
      defaultHeaders: {
        'HTTP-Referer': process.env.SITE_URL || 'http://localhost:3000',
        'X-Title': 'Docusaurus Translation Service',
      },
    });

    this.model = model;
    this.timeout = timeout;
  }

  /**
   * System prompt for the translator agent
   * @returns {string} System prompt
   */
  getSystemPrompt() {
    return `You are a professional English → Urdu translator for a technical textbook.

CRITICAL RULES:
1. Translate ONLY the given text to Urdu
2. Preserve ALL Markdown formatting (headers, bold, italic, code blocks, links)
3. Keep technical terms in English if they don't have common Urdu equivalents
4. Keep code blocks, inline code, and URLs unchanged
5. Keep proper nouns, API names, and command names in English
6. Output ONLY the translated Urdu text - NO explanations, NO notes, NO conversational filler
7. Do NOT add any preamble like "Here is the translation:"
8. Do NOT add any suffix like "Let me know if you need adjustments"

FORMATTING RULES:
- Preserve heading levels (# ## ###)
- Preserve bold (** **) and italic (* *)
- Preserve code blocks (\`\`\`) and inline code (\`)
- Preserve links [text](url)
- Preserve lists (- and numbered)
- Preserve blockquotes (>)

OUTPUT FORMAT:
Clean Urdu text only. No additional commentary.

Example:
Input: "# Introduction to AI\n\nArtificial Intelligence is **transforming** the world."
Output: "# مصنوعی ذہانت کا تعارف\n\nمصنوعی ذہانت دنیا کو **بدل** رہی ہے۔"`;
  }

  /**
   * Translate text to target language
   * @param {string} content - Content to translate
   * @param {string} targetLang - Target language code (default: 'ur')
   * @returns {Promise<string>} Translated content
   */
  async translate(content, targetLang = 'ur') {
    // Validate input
    if (!content || typeof content !== 'string') {
      throw new Error('Content must be a non-empty string');
    }

    if (content.trim().length === 0) {
      throw new Error('Content cannot be empty or whitespace only');
    }

    // Check content length
    const maxLen = parseInt(process.env.MAX_CONTENT_LENGTH) || 100000;
    if (content.length > maxLen) {
      throw new Error(`Content exceeds maximum length of ${maxLen} characters`);
    }

    // Language-specific prompts
    const languageNames = {
      ur: 'Urdu',
      ar: 'Arabic',
      hi: 'Hindi',
      bn: 'Bengali',
      fa: 'Persian',
    };

    const targetLanguage = languageNames[targetLang] || 'Urdu';

    try {
      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), this.timeout);

      const response = await this.client.chat.completions.create({
        model: this.model,
        messages: [
          {
            role: 'system',
            content: this.getSystemPrompt().replace('English → Urdu', `English → ${targetLanguage}`),
          },
          {
            role: 'user',
            content: `Translate the following markdown content to ${targetLanguage}:\n\n${content}`,
          },
        ],
        temperature: 0.3, // Lower temperature for more consistent translations
        max_tokens: Math.min(content.length * 2, 8000), // Allow up to 2x input length
        stream: false,
      }, {
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      const translatedText = response.choices[0]?.message?.content?.trim();

      if (!translatedText) {
        throw new Error('Empty translation received from model');
      }

      return translatedText;

    } catch (error) {
      if (error.name === 'AbortError') {
        throw new Error('Translation request timed out');
      }

      // Handle OpenAI API errors
      if (error.response) {
        const status = error.response.status;
        const message = error.response.data?.error?.message || error.message;

        switch (status) {
          case 401:
            throw new Error('Invalid API key');
          case 429:
            throw new Error('Rate limit exceeded - please try again later');
          case 500:
          case 502:
          case 503:
            throw new Error('OpenRouter service error - please try again');
          default:
            throw new Error(`Translation failed: ${message}`);
        }
      }

      throw new Error(`Translation failed: ${error.message}`);
    }
  }

  /**
   * Translate in chunks for long content
   * @param {string} content - Content to translate
   * @param {string} targetLang - Target language code
   * @param {number} chunkSize - Size of each chunk
   * @returns {Promise<string>} Translated content
   */
  async translateInChunks(content, targetLang = 'ur', chunkSize = 3000) {
    if (content.length <= chunkSize) {
      return await this.translate(content, targetLang);
    }

    // Split by markdown headers to preserve structure
    const chunks = this.splitByHeaders(content, chunkSize);
    const translatedChunks = [];

    for (let i = 0; i < chunks.length; i++) {
      console.log(`🔄 Translating chunk ${i + 1}/${chunks.length}...`);
      const translated = await this.translate(chunks[i], targetLang);
      translatedChunks.push(translated);
    }

    return translatedChunks.join('\n\n');
  }

  /**
   * Split content by markdown headers
   * @param {string} content - Content to split
   * @param {number} maxSize - Maximum chunk size
   * @returns {string[]} Array of chunks
   */
  splitByHeaders(content, maxSize) {
    const lines = content.split('\n');
    const chunks = [];
    let currentChunk = [];

    for (const line of lines) {
      const testChunk = [...currentChunk, line].join('\n');

      if (testChunk.length > maxSize && currentChunk.length > 0) {
        chunks.push(currentChunk.join('\n'));
        currentChunk = [line];
      } else {
        currentChunk.push(line);
      }
    }

    if (currentChunk.length > 0) {
      chunks.push(currentChunk.join('\n'));
    }

    return chunks;
  }

  /**
   * Health check for the translator
   * @returns {Promise<Object>} Health status
   */
  async healthCheck() {
    try {
      const testContent = '# Test\n\nThis is a test.';
      const result = await this.translate(testContent, 'ur');

      return {
        status: 'healthy',
        model: this.model,
        testTranslation: result ? 'OK' : 'Failed',
      };
    } catch (error) {
      return {
        status: 'unhealthy',
        model: this.model,
        error: error.message,
      };
    }
  }
}

/**
 * Create a singleton translator instance
 */
let translatorInstance = null;

/**
 * Get or create translator instance
 * @returns {TranslatorAgent} Translator instance
 */
export function getTranslator() {
  if (!translatorInstance) {
    translatorInstance = new TranslatorAgent();
  }
  return translatorInstance;
}

export default TranslatorAgent;
