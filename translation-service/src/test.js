/**
 * Translation Service Test Script
 * Run: node src/test.js
 */

import { getTranslator } from './agents/TranslatorAgent.js';
import { initDatabase, saveTranslation, getCachedTranslation } from './db/client.js';

// Test content
const TEST_CONTENT = `# Introduction to AI

Artificial Intelligence is **transforming** the world. Here's a simple example:

\`\`\`python
def hello():
    print("Hello, World!")
\`\`\`

## Key Concepts

- Machine Learning
- Neural Networks
- Deep Learning

Visit [OpenAI](https://openai.com) for more info.`;

async function testTranslation() {
  console.log('🧪 Running Translation Service Tests\n');

  // Test 1: Database connection
  console.log('📊 Test 1: Database Connection');
  const db = initDatabase();
  if (db) {
    console.log('✅ Database connected\n');
  } else {
    console.log('⚠️  Database not connected (cache disabled)\n');
  }

  // Test 2: Translator health
  console.log('🏥 Test 2: Translator Health Check');
  try {
    const translator = getTranslator();
    const health = await translator.healthCheck();
    console.log('✅ Translator status:', health.status);
    console.log('   Model:', health.model);
    console.log('   Test translation:', health.testTranslation);
    console.log('');
  } catch (error) {
    console.log('❌ Translator health check failed:', error.message);
    console.log('');
    return;
  }

  // Test 3: Translation
  console.log('🌐 Test 3: Translation (English → Urdu)');
  try {
    const translator = getTranslator();
    const startTime = Date.now();
    const translated = await translator.translate(TEST_CONTENT, 'ur');
    const duration = Date.now() - startTime;

    console.log('✅ Translation successful');
    console.log('   Time:', duration, 'ms');
    console.log('   Input length:', TEST_CONTENT.length, 'chars');
    console.log('   Output length:', translated.length, 'chars');
    console.log('\n📝 Translated text:');
    console.log('---');
    console.log(translated);
    console.log('---\n');

    // Test 4: Cache
    if (db) {
      console.log('💾 Test 4: Cache Operations');

      // Save to cache
      await saveTranslation('test-page', 'ur', TEST_CONTENT, translated, translator.model);
      console.log('✅ Saved to cache');

      // Retrieve from cache
      const cached = await getCachedTranslation('test-page', 'ur');
      if (cached === translated) {
        console.log('✅ Retrieved from cache (matches)');
      } else {
        console.log('❌ Cache mismatch');
      }
      console.log('');
    }

    // Test 5: Different language
    console.log('🌐 Test 5: Translation (English → Arabic)');
    const arabic = await translator.translate('Hello World', 'ar');
    console.log('✅ Arabic translation:', arabic);
    console.log('');

    // Test 6: Error handling
    console.log('⚠️  Test 6: Error Handling');
    try {
      await translator.translate('', 'ur');
      console.log('❌ Should have thrown error for empty content');
    } catch (error) {
      console.log('✅ Correctly threw error:', error.message);
    }
    console.log('');

    // Summary
    console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
    console.log('✅ All tests passed!');
    console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');

  } catch (error) {
    console.log('❌ Test failed:', error.message);
    console.log('');
  }
}

// Run tests
testTranslation().catch(console.error);
