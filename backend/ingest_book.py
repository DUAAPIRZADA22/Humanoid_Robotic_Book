"""
Script to ingest the Physical AI & Humanoid Robotics book content into RAG system.
"""

import os
import asyncio
import logging
from pathlib import Path
import sys

# Add backend to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rag.chunking import MarkdownChunker
from rag.embeddings import CohereEmbeddingService
from rag.vector_store import QdrantVectorStore
from rag.rerank import CohereReranker
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


async def ingest_book_content():
    """Ingest all markdown files from the docs directory."""

    logger.info("Starting book content ingestion...")

    # Initialize RAG components
    try:
        # Load API keys from environment
        cohere_api_key = os.getenv('COHERE_API_KEY')
        qdrant_url = os.getenv('QDRANT_URL')
        qdrant_api_key = os.getenv('QDRANT_API_KEY')

        if not cohere_api_key:
            logger.error("COHERE_API_KEY not found in environment variables")
            return

        if not qdrant_url:
            logger.error("QDRANT_URL not found in environment variables")
            return

        if not qdrant_api_key:
            logger.error("QDRANT_API_KEY not found in environment variables")
            return

        logger.info("Environment variables loaded successfully")

        chunker = MarkdownChunker()
        embedding_service = CohereEmbeddingService(api_key=cohere_api_key)
        vector_store = QdrantVectorStore(
            collection_name="humanoid_robotics_book",
            embedding_dim=1024,
            url=qdrant_url,
            api_key=qdrant_api_key
        )
        reranker = CohereReranker(api_key=cohere_api_key)

        logger.info("RAG components initialized successfully")

        # Ensure collection exists
        await vector_store.ensure_collection()
        logger.info("Qdrant collection ready")

    except Exception as e:
        logger.error(f"Failed to initialize RAG components: {e}")
        return

    # Find all markdown files in docs directory
    docs_path = Path("../docs")
    if not docs_path.exists():
        logger.error(f"Docs directory not found: {docs_path}")
        return

    markdown_files = list(docs_path.rglob("*.md"))
    logger.info(f"Found {len(markdown_files)} markdown files")

    total_chunks = 0
    total_files = 0

    # Process each file
    for file_path in markdown_files:
        try:
            logger.info(f"Processing: {file_path}")

            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Create metadata
            metadata = {
                "source": str(file_path.relative_to(docs_path)),
                "title": file_path.stem.replace('-', ' ').replace('_', ' ').title(),
                "file_type": "markdown",
                "file_size": len(content),
                "chunk_count": 0
            }

            # Chunk the content
            chunks = chunker.chunk_document(content, str(file_path.relative_to(docs_path)))
            logger.info(f"Generated {len(chunks)} chunks from {file_path.name}")

            # Generate embeddings for chunks
            texts = [chunk.text for chunk in chunks]
            embeddings = await embedding_service.get_embeddings(texts)
            logger.info(f"Generated {len(embeddings)} embeddings for {file_path.name}")

            # Store in vector database
            documents = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                doc = {
                    "id": f"{file_path.stem}_{i}_{hash(chunk.text[:100])}",
                    "text": chunk.text,
                    "vector": embedding,
                    "metadata": {
                        **metadata,
                        "chunk_index": i,
                        "chunk_text": chunk.text[:200] + "..." if len(chunk.text) > 200 else chunk.text
                    }
                }
                documents.append(doc)

            # Upsert to Qdrant
            await vector_store.upsert(documents)
            logger.info(f"Stored {len(documents)} documents in Qdrant for {file_path.name}")

            total_chunks += len(chunks)
            total_files += 1

            # Clear embeddings to free memory
            del embeddings
            del points

        except Exception as e:
            logger.error(f"Error processing {file_path}: {e}")
            continue

    logger.info(f"Ingestion completed! Processed {total_files} files with {total_chunks} total chunks")

    # Test the retrieval
    try:
        logger.info("Testing retrieval with a sample query...")
        test_query = "What is Physical AI?"

        # Generate embedding for test query
        query_embedding = await embedding_service.get_embeddings([test_query])

        # Search in Qdrant
        search_results = await vector_store.search(
            query_vector=query_embedding[0],
            top_k=5,
            score_threshold=0.5
        )

        logger.info(f"Test search returned {len(search_results)} results:")
        for i, result in enumerate(search_results):
            payload = result.payload
            logger.info(f"  {i+1}. Score: {result.score:.4f}")
            logger.info(f"     Source: {payload.get('metadata', {}).get('source', 'Unknown')}")
            logger.info(f"     Preview: {payload.get('text', '')[:100]}...")

    except Exception as e:
        logger.error(f"Error during retrieval test: {e}")

    logger.info("Book ingestion process completed successfully!")


if __name__ == "__main__":
    asyncio.run(ingest_book_content())