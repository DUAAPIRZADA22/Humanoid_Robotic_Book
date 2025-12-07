#!/usr/bin/env python3
"""
Script to ingest book content into the vector database.
Can be run manually or integrated into CI/CD pipelines.
"""

import asyncio
import argparse
import sys
import logging
from pathlib import Path
import glob

# Add the parent directory to Python path
sys.path.insert(0, str(Path(__file__).parent.parent))

from rag.chunking import MarkdownChunker
from rag.embeddings import CohereEmbeddingService
from rag.vector_store import QdrantVectorStore

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


async def ingest_directory(
    content_dir: str,
    file_pattern: str = "*.md",
    overwrite: bool = False,
    batch_size: int = 100
):
    """
    Ingest all markdown files from a directory.

    Args:
        content_dir: Directory containing markdown files
        file_pattern: Pattern for file matching
        overwrite: Whether to overwrite existing content
        batch_size: Number of documents to process in each batch
    """
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    # Initialize components
    logger.info("Initializing RAG pipeline...")

    chunker = MarkdownChunker(
        min_chunk_size=50,
        max_chunk_size=1000,
        overlap=100
    )

    embedding_service = CohereEmbeddingService(
        api_key=os.getenv("COHERE_API_KEY"),
        model=os.getenv("EMBEDDING_MODEL", "embed-english-v3.0"),
        batch_size=batch_size
    )

    vector_store = QdrantVectorStore(
        collection_name="humanoid_robotics_book",
        embedding_dim=1024,
        host=os.getenv("QDRANT_HOST", "localhost"),
        port=int(os.getenv("QDRANT_PORT", "6333")),
        api_key=os.getenv("QDRANT_API_KEY")
    )

    # Ensure collection exists
    await vector_store.ensure_collection()

    # Find all markdown files
    content_path = Path(content_dir)
    if not content_path.exists():
        logger.error(f"Content directory does not exist: {content_path}")
        return False

    file_pattern = content_path / file_pattern
    file_paths = glob.glob(str(file_pattern))

    if not file_paths:
        logger.warning(f"No files found matching pattern: {file_pattern}")
        return True

    logger.info(f"Found {len(file_paths)} files to process")

    # Process each file
    total_chunks = 0
    total_files = 0

    for file_path in file_paths:
        logger.info(f"\nProcessing: {file_path}")

        try:
            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Skip if file is too small
            if len(content.strip()) < 50:
                logger.warning(f"Skipping {file_path} - content too short")
                continue

            # Chunk content
            chunks = chunker.chunk_document(content, source=file_path)
            if not chunks:
                logger.warning(f"No chunks generated for {file_path}")
                continue

            # Create embeddings
            texts = [chunk.text for chunk in chunks]
            embeddings = await embedding_service.get_embeddings(texts)

            if not embeddings or len(embeddings) != len(chunks):
                logger.error(f"Embedding generation failed for {file_path}")
                continue

            # Prepare documents for vector store
            documents = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                documents.append({
                    "id": f"{Path(file_path).stem}_{i}_{chunk.chunk_id}",
                    "text": chunk.text,
                    "metadata": {
                        "source": file_path,
                        "filename": Path(file_path).name,
                        "chunk_index": i,
                        "char_count": chunk.metadata.get("char_count", 0),
                        "word_count": chunk.metadata.get("word_count", 0),
                        "headers": chunk.metadata.get("headers", []),
                        "page_numbers": chunk.metadata.get("page_numbers", [])
                    },
                    "vector": embedding
                })

            # Delete existing documents if overwrite is enabled
            if overwrite:
                await vector_store.delete_by_metadata("source", file_path)

            # Upsert documents in batches
            for i in range(0, len(documents), batch_size):
                batch = documents[i:i + batch_size]
                await vector_store.upsert(batch)
                logger.debug(f"Upserted batch {i//batch_size + 1}/{(len(documents)-1)//batch_size + 1}")

            total_chunks += len(documents)
            total_files += 1

            logger.info(f"✅ Processed {len(documents)} chunks from {file_path}")

        except Exception as e:
            logger.error(f"Failed to process {file_path}: {str(e)}", exc_info=True)
            continue

    # Print summary
    logger.info("\n" + "=" * 60)
    logger.info("Ingestion Summary")
    logger.info("=" * 60)
    logger.info(f"Files processed: {total_files}")
    logger.info(f"Total chunks: {total_chunks}")

    # Get collection stats
    doc_count = await vector_store.count_documents()
    logger.info(f"Total documents in collection: {doc_count}")

    return True


async def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Ingest book content into vector database")
    parser.add_argument(
        "--content-dir",
        default="book_content",
        help="Directory containing markdown files (default: book_content)"
    )
    parser.add_argument(
        "--pattern",
        default="*.md",
        help="File pattern to match (default: *.md)"
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing content from same sources"
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=100,
        help="Batch size for processing (default: 100)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging"
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    print("Physical AI & Humanoid Robotics Book Content Ingestion")
    print("=" * 60)
    print(f"Content directory: {args.content_dir}")
    print(f"File pattern: {args.pattern}")
    print(f"Overwrite: {args.overwrite}")
    print(f"Batch size: {args.batch_size}")
    print("=" * 60)

    success = await ingest_directory(
        content_dir=args.content_dir,
        file_pattern=args.pattern,
        overwrite=args.overwrite,
        batch_size=args.batch_size
    )

    if success:
        print("\n✅ Ingestion completed successfully!")
    else:
        print("\n❌ Ingestion failed. Please check the logs above.")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())