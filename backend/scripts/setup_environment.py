#!/usr/bin/env python3
"""
Setup script for the chatbot backend environment.
Initializes Qdrant collection and performs health checks.
"""

import asyncio
import os
import sys
import logging
from pathlib import Path

# Add the parent directory to Python path
sys.path.insert(0, str(Path(__file__).parent.parent))

from rag.embeddings import CohereEmbeddingService
from rag.vector_store import QdrantVectorStore
from rag.retrieval import RetrievalPipeline
from rag.rerank import CohereReranker

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


async def setup_environment():
    """Set up the RAG environment."""
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    # Check required environment variables
    required_vars = ["COHERE_API_KEY"]
    missing = [var for var in required_vars if not os.getenv(var)]

    if missing:
        logger.error(f"Missing required environment variables: {missing}")
        logger.error("Please copy .env.example to .env and fill in the values")
        return False

    # Initialize components
    logger.info("Initializing RAG components...")

    try:
        # Embedding service
        embedding_service = CohereEmbeddingService(
            api_key=os.getenv("COHERE_API_KEY"),
            model=os.getenv("EMBEDDING_MODEL", "embed-english-v3.0"),
            batch_size=int(os.getenv("EMBEDDING_BATCH_SIZE", "100"))
        )

        # Vector store
        vector_store = QdrantVectorStore(
            collection_name="humanoid_robotics_book",
            embedding_dim=1024,
            host=os.getenv("QDRANT_HOST", "localhost"),
            port=int(os.getenv("QDRANT_PORT", "6333")),
            api_key=os.getenv("QDRANT_API_KEY")
        )

        # Reranker
        reranker = CohereReranker(
            api_key=os.getenv("COHERE_API_KEY"),
            model=os.getenv("RERANK_MODEL", "rerank-english-v3.0")
        )

        # Retrieval pipeline
        retrieval_pipeline = RetrievalPipeline(
            embedding_service=embedding_service,
            vector_store=vector_store,
            reranker=reranker
        )

        # Ensure collection exists
        logger.info("Setting up Qdrant collection...")
        collection_ready = await vector_store.ensure_collection()

        if not collection_ready:
            logger.error("Failed to set up Qdrant collection")
            return False

        # Health checks
        logger.info("Performing health checks...")

        # Check embedding service
        embedding_health = await embedding_service.health_check()
        logger.info(f"Embedding service health: {embedding_health}")

        # Check vector store
        vector_health = await vector_store.health_check()
        logger.info(f"Vector store health: {vector_health}")

        # Check reranker
        rerank_health = await reranker.health_check()
        logger.info(f"Reranker health: {rerank_health}")

        # Get pipeline statistics
        stats = await retrieval_pipeline.get_statistics()
        logger.info(f"Pipeline statistics: {stats}")

        logger.info("✅ Environment setup complete!")
        return True

    except Exception as e:
        logger.error(f"Setup failed: {str(e)}", exc_info=True)
        return False


async def test_ingest():
    """Test ingestion with sample content."""
    logger.info("\nTesting ingestion with sample content...")

    # Create sample content directory
    content_dir = Path("book_content")
    content_dir.mkdir(exist_ok=True)

    # Create a sample markdown file
    sample_file = content_dir / "sample.md"
    if not sample_file.exists():
        sample_content = """# Introduction to Humanoid Robotics

Humanoid robotics is a fascinating field that combines robotics, artificial intelligence, and biomechanics to create robots that resemble and mimic human behavior.

## Key Components

### Mechanical Structure
The mechanical structure of humanoid robots typically includes:
- joints and actuators
- sensors for perception
- end-effectors like hands

### Control Systems
Advanced control systems enable humanoid robots to:
- maintain balance
- walk on various terrains
- interact with objects

## Applications

Humanoid robots have applications in:
1. Healthcare assistance
2. Search and rescue
3. Education and entertainment

The future of humanoid robotics holds exciting possibilities as technology continues to advance.
"""
        with open(sample_file, 'w') as f:
            f.write(sample_content)
        logger.info(f"Created sample file: {sample_file}")


async def main():
    """Main setup function."""
    print("=" * 60)
    print("Physical AI & Humanoid Robotics Book Chatbot Setup")
    print("=" * 60)

    # Setup environment
    success = await setup_environment()

    if success:
        # Create sample content for testing
        await test_ingest()
        print("\n✅ Setup completed successfully!")
        print("\nNext steps:")
        print("1. Add your book content to the book_content/ directory")
        print("2. Run the application with: python -m uvicorn main:app --reload")
        print("3. Visit http://localhost:7860/docs to test the API")
    else:
        print("\n❌ Setup failed. Please check the logs above.")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())