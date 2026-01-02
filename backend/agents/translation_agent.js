/**
 * Translation Agent using OpenAI Agents SDK
 * @fileoverview Agent-based translation with Markdown preservation
 */

import { Agent } from 'https://esm.sh/openai@4.28.0';
import { z } from 'https://esm.sh/zod@3.22.4';

// ===================================================================
// CONFIGURATION
// ===================================================================

const CONFIG = {
  openRouterApiKey: process.env.OPENROUTER_API_KEY,
  baseURL: 'https://openrouter.ai/api/v1',
  model: 'google/gemma-3-27b-it:free',
  defaultTargetLang: 'ur', // Urdu
};

// ===================================================================
// SCHEMAS
// ===================================================================

const TranslationRequestSchema = z.object({
  content: z.string().min(1),
  pageSlug: z.string(),
  targetLang: z.string().default('ur').optional(),
  preserveCode: z.boolean().default(true),
});

const TranslationResponseSchema = z.object({
  translatedContent: z.string(),
  originalContent: z.string(),
  language: z.string(),
  characterCount: z.number(),
});

// ===================================================================
// TOOLS
// ===================================================================

/**
 * Extract and preserve code blocks from markdown
 */
const extractCodeBlocksTool = {
  name: 'extract_code_blocks',
  description: 'Extract code blocks from markdown content for preservation during translation',
  parameters: {
    type: 'object',
    properties: {
      markdown: {
        type: 'string',
        description: 'The markdown content to process',
      },
    },
    required: ['markdown'],
  },
  execute: async ({ markdown }) => {
    const codeBlocks: string[] = [];
    let placeholderIndex = 0;

    // Replace code blocks with placeholders
    const processed = markdown.replace(/```(\w*)\n([\s\S]*?)```/g, (match, lang, code) => {
      const placeholder = `__CODE_BLOCK_${placeholderIndex}__`;
      codeBlocks.push({ placeholder, lang, code });
      placeholderIndex++;
      return placeholder;
    });

    // Replace inline code
    const inlineProcessed = processed.replace(/`([^`]+)`/g, (match, code) => {
      const placeholder = `__INLINE_CODE_${placeholderIndex}__`;
      codeBlocks.push({ placeholder, code, isInline: true });
      placeholderIndex++;
      return placeholder;
    });

    return {
      processedContent: inlineProcessed,
      codeBlocks,
      count: codeBlocks.length,
    };
  },
};

/**
 * Restore code blocks after translation
 */
const restoreCodeBlocksTool = {
  name: 'restore_code_blocks',
  description: 'Restore preserved code blocks into translated content',
  parameters: {
    type: 'object',
    properties: {
      translatedContent: {
        type: 'string',
        description: 'The translated content with placeholders',
      },
      codeBlocks: {
        type: 'array',
        description: 'Array of code blocks to restore',
        items: {
          type: 'object',
          properties: {
            placeholder: { type: 'string' },
            lang: { type: 'string' },
            code: { type: 'string' },
            isInline: { type: 'boolean' },
          },
        },
      },
    },
    required: ['translatedContent', 'codeBlocks'],
  },
  execute: async ({ translatedContent, codeBlocks }) => {
    let restored = translatedContent;

    // Restore inline code first (shorter placeholders)
    for (const block of codeBlocks.filter(b => b.isInline)) {
      restored = restored.replace(block.placeholder, `\`${block.code}\``);
    }

    // Restore code blocks
    for (const block of codeBlocks.filter(b => !b.isInline)) {
      restored = restored.replace(
        block.placeholder,
        `\`\`\`${block.lang || ''}\n${block.code}\`\`\``
      );
    }

    return { restoredContent: restored };
  },
};

// ===================================================================
// AGENT DEFINITION
// ===================================================================

/**
 * Create Translation Agent
 */
export function createTranslationAgent() {
  const agent = new Agent({
    name: 'TranslationAgent',
    instructions: getSystemPrompt(),
    tools: [extractCodeBlocksTool, restoreCodeBlocksTool],
    model: CONFIG.model,
  });

  return agent;
}

/**
 * System prompt for translation
 */
function getSystemPrompt() {
  const targetLangNames = {
    ur: 'Urdu',
    ar: 'Arabic',
    hi: 'Hindi',
    bn: 'Bengali',
    fa: 'Persian',
    es: 'Spanish',
    fr: 'French',
    de: 'German',
    zh: 'Chinese',
    ja: 'Japanese',
  };

  return `You are a professional technical document translator for a book on Physical AI and Humanoid Robotics.

# TRANSLATION RULES

1. **Preserve Markdown Structure**: Keep all headings (#, ##, ###), bold (**), italic (*), lists (-, 1.), and links ([text](url))

2. **DO NOT Translate Code Blocks**:
   - Keep \`\`\`code blocks\`\`\` unchanged
   - Keep \`inline code\` unchanged
   - Keep command names, function names, API endpoints

3. **Preserve Technical Terms**:
   - Keep terms like: ROS, OpenCV, PyBullet, Isaac Sim, NumPy
   - Keep commands: npm install, pip install, python, etc.
   - Keep file extensions: .py, .js, .tsx, .md

4. **Translation Quality**:
   - Use natural, flowing ${targetLangNames[CONFIG.defaultTargetLang] || 'Urdu'}
   - For technical terms without common translations, keep English term
   - Maintain professional, educational tone

5. **Output Format**:
   - Return ONLY the translated markdown
   - NO explanations, NO notes, NO conversational filler
   - Preserve exact line breaks for code compatibility

# WORKFLOW

1. Use extract_code_blocks tool to preserve code
2. Translate the remaining content
3. Use restore_code_blocks tool to put code back
4. Return the final translated markdown

Remember: The user will read this translated content. Make it clear and accurate.`;
}

// ===================================================================
// RUNNER
// ===================================================================

/**
 * Run translation agent
 */
export async function runTranslationAgent(input: {
  content: string;
  pageSlug: string;
  targetLang?: string;
}) {
  const { content, pageSlug, targetLang = CONFIG.defaultTargetLang } = input;

  console.log(`[TranslationAgent] Starting translation for: ${pageSlug} → ${targetLang}`);
  console.log(`[TranslationAgent] Content length: ${content.length} chars`);

  // Validate input
  if (!content || content.length === 0) {
    throw new Error('Content is empty');
  }

  if (content.length > 100000) {
    throw new Error('Content too large (max 100k chars)');
  }

  try {
    // Step 1: Extract code blocks
    console.log('[TranslationAgent] Step 1: Extracting code blocks...');
    const extractResult = await extractCodeBlocksTool.execute({ markdown: content });
    console.log(`[TranslationAgent] Found ${extractResult.count} code blocks`);

    // Step 2: Translate the content (without code)
    console.log('[TranslationAgent] Step 2: Translating content...');
    const translated = await translateWithLLM(extractResult.processedContent, targetLang);

    // Step 3: Restore code blocks
    console.log('[TranslationAgent] Step 3: Restoring code blocks...');
    const restoreResult = await restoreCodeBlocksTool.execute({
      translatedContent: translated,
      codeBlocks: extractResult.codeBlocks,
    });

    const finalResult = restoreResult.restoredContent;

    console.log(`[TranslationAgent] Translation complete: ${finalResult.length} chars`);

    return {
      translatedContent: finalResult,
      originalContent: content,
      language: targetLang,
      characterCount: content.length,
    };

  } catch (error) {
    console.error('[TranslationAgent] Error:', error);
    throw error;
  }
}

/**
 * Translate with LLM (direct API call for simplicity)
 */
async function translateWithLLM(content: string, targetLang: string): Promise<string> {
  const response = await fetch(`${CONFIG.baseURL}/chat/completions`, {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${CONFIG.openRouterApiKey}`,
      'Content-Type': 'application/json',
      'HTTP-Referer': 'http://localhost:3000',
      'X-Title': 'Docusaurus Translation',
    },
    body: JSON.stringify({
      model: CONFIG.model,
      messages: [
        {
          role: 'system',
          content: getSystemPrompt(),
        },
        {
          role: 'user',
          content: `Translate the following markdown content to ${targetLang}:\n\n${content}`,
        },
      ],
      temperature: 0.3,
      max_tokens: Math.min(content.length * 2, 8000),
      stream: false,
    }),
  });

  if (!response.ok) {
    throw new Error(`Translation failed: ${response.status} ${response.statusText}`);
  }

  const data = await response.json();
  const translated = data.choices[0]?.message?.content;

  if (!translated) {
    throw new Error('Empty translation received');
  }

  return translated;
}

// Export for use in backend
export { TranslationRequestSchema, TranslationResponseSchema };
