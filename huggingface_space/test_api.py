#!/usr/bin/env python3
"""
Simple test script for the chatbot API.
Tests both the /chat and /ingest endpoints.
"""

import asyncio
import json
import aiohttp
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def test_health(session, base_url):
    """Test the health endpoint."""
    logger.info("Testing health endpoint...")

    async with session.get(f"{base_url}/health") as response:
        if response.status == 200:
            data = await response.json()
            logger.info(f"✅ Health check passed: {json.dumps(data, indent=2)}")
            return True
        else:
            logger.error(f"❌ Health check failed: {response.status}")
            return False


async def test_chat(session, base_url):
    """Test the chat endpoint with streaming."""
    logger.info("\nTesting chat endpoint with SSE...")

    query = "What are the key components of humanoid robots?"

    async with session.post(
        f"{base_url}/chat",
        json={
            "query": query,
            "top_k": 3,
            "similarity_threshold": 0.5,
            "use_rerank": True
        }
    ) as response:
        if response.status != 200:
            logger.error(f"❌ Chat endpoint failed: {response.status}")
            return False

        # Process SSE stream
        event_count = 0
        content_chunks = []

        async for line in response.content:
            line = line.decode('utf-8').strip()
            if line.startswith('data: '):
                data = line[6:]  # Remove 'data: ' prefix
                if data:
                    try:
                        event = json.loads(data)
                        event_type = event.get('type')

                        if event_type == 'metadata':
                            logger.info(f"📋 Metadata: Query='{event['query']}', Top-K={event['top_k']}")
                        elif event_type == 'content':
                            content_chunks.append(event['content'])
                            logger.info(f"📄 Content chunk {event['index']}: Score={event['score']:.3f}")
                        elif event_type == 'complete':
                            logger.info(f"✅ Complete: {event['total_chunks']} chunks retrieved")
                        elif event_type == 'error':
                            logger.error(f"❌ Error: {event['message']}")
                            return False

                        event_count += 1
                    except json.JSONDecodeError:
                        logger.warning(f"Failed to parse SSE data: {data}")

        logger.info(f"✅ Received {event_count} events, {len(content_chunks)} content chunks")
        return event_count > 0


async def test_ingest(session, base_url):
    """Test the ingest endpoint."""
    logger.info("\nTesting ingest endpoint...")

    async with session.post(
        f"{base_url}/ingest",
        json={
            "content_dir": "book_content",
            "file_pattern": "*.md",
            "overwrite": False
        }
    ) as response:
        if response.status == 200:
            data = await response.json()
            logger.info(f"✅ Ingestion started: {json.dumps(data, indent=2)}")
            return True
        else:
            error = await response.text()
            logger.error(f"❌ Ingest endpoint failed: {response.status} - {error}")
            return False


async def main():
    """Run all tests."""
    base_url = "http://localhost:7860"

    print("=" * 60)
    print("Physical AI & Humanoid Robotics Book Chatbot API Test")
    print("=" * 60)
    print(f"Testing API at: {base_url}")
    print("=" * 60)

    async with aiohttp.ClientSession() as session:
        # Test health first
        health_ok = await test_health(session, base_url)

        if not health_ok:
            print("\n❌ Health check failed. Please ensure the application is running.")
            return

        # Test chat
        chat_ok = await test_chat(session, base_url)

        # Test ingest
        ingest_ok = await test_ingest(session, base_url)

        # Summary
        print("\n" + "=" * 60)
        print("Test Summary:")
        print(f"  Health Check: {'✅ PASSED' if health_ok else '❌ FAILED'}")
        print(f"  Chat Endpoint: {'✅ PASSED' if chat_ok else '❌ FAILED'}")
        print(f"  Ingest Endpoint: {'✅ PASSED' if ingest_ok else '❌ FAILED'}")

        if all([health_ok, chat_ok, ingest_ok]):
            print("\n🎉 All tests passed!")
        else:
            print("\n⚠️  Some tests failed. Please check the logs above.")
        print("=" * 60)


if __name__ == "__main__":
    asyncio.run(main())