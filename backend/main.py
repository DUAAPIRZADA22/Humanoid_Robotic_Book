"""
FastAPI backend for Physical AI & Humanoid Robotics book chatbot.
Provides streaming chat and ingestion endpoints using RAG pipeline.
"""

import os
import asyncio
import json
from contextlib import asynccontextmanager
from typing import AsyncGenerator, Dict, Any, List
import logging

from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import uvicorn
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

from rag.chunking import MarkdownChunker
from rag.embeddings_new import CohereEmbeddingService
from rag.vector_store_new import QdrantVectorStore
from rag.retrieval import RetrievalPipeline
from rag.rerank_new import CohereReranker
from rag.llm_service import create_llm_service


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Reduce httpcore debug noise
logging.getLogger("httpcore.connection").setLevel(logging.WARNING)
logging.getLogger("httpcore.http11").setLevel(logging.WARNING)

# Global instances
chunker = None
embedding_service = None
vector_store = None
retrieval_pipeline = None
reranker = None
llm_service = None


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    query: str
    top_k: int = 5
    similarity_threshold: float = 0.7
    use_rerank: bool = True


class IngestRequest(BaseModel):
    """Request model for ingest endpoint."""
    content_dir: str = "book_content"
    file_pattern: str = "*.md"
    overwrite: bool = False


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize and cleanup resources."""
    global chunker, embedding_service, vector_store, retrieval_pipeline, reranker, llm_service

    # Initialize components
    logger.info("Initializing RAG pipeline components...")

    chunker = MarkdownChunker(min_chunk_size=50, max_chunk_size=1000)

    # Get API keys with proper error handling
    cohere_api_key = os.getenv("COHERE_API_KEY")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")

    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY environment variable is required")

    embedding_service = CohereEmbeddingService(
        api_key=cohere_api_key,
        model="embed-english-v3.0",
        batch_size=96  # Updated to Cohere's maximum
    )

    # Use Qdrant URL if provided, otherwise use host/port
    if qdrant_url:
        vector_store = QdrantVectorStore(
            collection_name="humanoid_robotics_book",
            embedding_dim=1024,  # Cohere embed-english-v3.0 dimension
            url=qdrant_url,
            api_key=qdrant_api_key
        )
    else:
        vector_store = QdrantVectorStore(
            collection_name="humanoid_robotics_book",
            embedding_dim=1024,
            host=os.getenv("QDRANT_HOST", "localhost"),
            port=int(os.getenv("QDRANT_PORT", "6333")),
            api_key=qdrant_api_key
        )

    reranker = CohereReranker(
        api_key=cohere_api_key,
        model="rerank-v3.5"  # Updated to latest model
    )
    retrieval_pipeline = RetrievalPipeline(
        embedding_service=embedding_service,
        vector_store=vector_store,
        reranker=reranker
    )

    # Initialize LLM service
    try:
        llm_service = create_llm_service({
            "provider": "openai",
            "model": "gpt-3.5-turbo",
            "api_key": os.getenv("OPENAI_API_KEY"),
            "base_url": os.getenv("OPENAI_BASE_URL"),  # Optional for custom endpoints
            "max_tokens": 1000,
            "temperature": 0.7
        })
        logger.info("LLM service initialized successfully")
    except Exception as e:
        logger.warning(f"Failed to initialize LLM service: {e}")
        logger.warning("Chat will return retrieved chunks without generated answers")
        llm_service = None

    # Ensure collection exists
    await vector_store.ensure_collection()
    logger.info("RAG pipeline initialized successfully")

    yield

    logger.info("Shutting down RAG pipeline...")


# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book Chatbot",
    description="RAG-powered chatbot backend for the Physical AI & Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://github.com/DUAAPIRZADA22",
        "http://localhost:3000",
        "http://localhost:3001",
        "http://localhost:3002",
        "http://localhost:7860",
        "https://humanoid-robotic-book.vercel.app",
        "http://localhost:3000/*",
        "http://localhost:3001/*",
        "http://localhost:3002/*"
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)


@app.post("/ingest")
async def ingest_content(request: IngestRequest, background_tasks: BackgroundTasks) -> Dict[str, Any]:
    """
    Ingest book content into vector store.
    """
    if not chunker or not embedding_service or not vector_store:
        raise HTTPException(status_code=503, detail="RAG pipeline not initialized")

    content_path = os.path.abspath(request.content_dir)
    if not os.path.exists(content_path):
        raise HTTPException(status_code=404, detail=f"Content directory not found: {content_path}")

    # Run ingestion in background
    background_tasks.add_task(
        run_ingestion,
        content_path,
        request.file_pattern,
        request.overwrite
    )

    return {
        "status": "started",
        "message": f"Ingestion started for directory: {content_path}",
        "pattern": request.file_pattern,
        "overwrite": request.overwrite
    }


async def run_ingestion(content_dir: str, file_pattern: str, overwrite: bool) -> None:
    """Run the ingestion process."""
    try:
        logger.info(f"Starting ingestion from {content_dir} with pattern {file_pattern}")

        # Get all markdown files
        import glob
        file_paths = glob.glob(os.path.join(content_dir, file_pattern))

        if not file_paths:
            logger.warning(f"No files found matching pattern {file_pattern} in {content_dir}")
            return

        # Process each file
        total_chunks = 0
        for file_path in file_paths:
            logger.info(f"Processing file: {file_path}")

            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Chunk content
            chunks = chunker.chunk_document(content, source=file_path)
            if not chunks:
                logger.warning(f"No chunks generated for file: {file_path}")
                continue

            # Create embeddings
            texts = [chunk.text for chunk in chunks]
            embeddings = await embedding_service.get_embeddings(texts)

            # Prepare for vector store
            documents = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                documents.append({
                    "id": f"{os.path.basename(file_path)}_{i}",
                    "text": chunk.text,
                    "metadata": {
                        "source": file_path,
                        "chunk_index": i,
                        "page_numbers": chunk.metadata.get("page_numbers", []),
                        "headers": chunk.metadata.get("headers", [])
                    },
                    "vector": embedding
                })

            # Upsert to vector store
            if overwrite:
                # Delete existing documents from this source
                await vector_store.delete_by_metadata("source", file_path)

            await vector_store.upsert(documents)
            total_chunks += len(documents)

            logger.info(f"Processed {len(documents)} chunks from {file_path}")

        logger.info(f"Ingestion complete. Total chunks processed: {total_chunks}")

    except Exception as e:
        logger.error(f"Error during ingestion: {str(e)}", exc_info=True)


@app.post("/chat")
async def chat_stream(request: ChatRequest) -> StreamingResponse:
    """
    Stream chat responses using Server-Sent Events (SSE).
    """
    if not retrieval_pipeline:
        raise HTTPException(status_code=503, detail="RAG pipeline not initialized")

    async def generate_response() -> AsyncGenerator[str, None]:
        """Generate streaming response."""
        try:
            # Send initial metadata
            metadata = {
                "type": "metadata",
                "query": request.query,
                "top_k": request.top_k,
                "similarity_threshold": request.similarity_threshold,
                "use_rerank": request.use_rerank
            }
            yield f"data: {json.dumps(metadata)}\n\n"

            # Retrieve relevant chunks
            logger.info(f"Retrieving for query: {request.query}")
            results = await retrieval_pipeline.retrieve(
                query=request.query,
                top_k=request.top_k * 2,  # Fetch 2x for deduplication
                similarity_threshold=request.similarity_threshold
            )

            # Apply deduplication
            unique_results = []
            seen_texts = set()
            for result in results:
                text_hash = hash(result["text"])
                if text_hash not in seen_texts:
                    seen_texts.add(text_hash)
                    unique_results.append(result)
                    if len(unique_results) >= request.top_k:
                        break

            # Apply reranking if requested
            if request.use_rerank and unique_results and reranker:
                unique_results = await reranker.rerank(
                    query=request.query,
                    documents=unique_results,
                    top_n=request.top_k
                )

            # Generate answer using LLM if available, otherwise return raw chunks
            if llm_service and unique_results:
                # Send context chunks first for transparency
                for i, result in enumerate(unique_results):
                    chunk_data = {
                        "type": "source",
                        "index": i,
                        "content": result["text"],
                        "score": result.get("relevance_score", result.get("score", 0.0)),
                        "source": result.get("metadata", {}).get("source", ""),
                        "headers": result.get("metadata", {}).get("headers", [])
                    }
                    yield f"data: {json.dumps(chunk_data)}\n\n"

                # Generate and stream the answer
                yield f"data: {json.dumps({'type': 'answer_start'})}\n\n"

                answer_content = ""
                async for chunk in llm_service.generate_answer(
                    query=request.query,
                    context=unique_results,
                    stream=True
                ):
                    if chunk and not chunk.startswith("Error"):
                        answer_content += chunk
                        chunk_data = {
                            "type": "answer_chunk",
                            "content": chunk
                        }
                        yield f"data: {json.dumps(chunk_data)}\n\n"

                # Send completion with answer summary
                completion = {
                    "type": "complete",
                    "total_sources": len(unique_results),
                    "has_answer": True,
                    "answer_length": len(answer_content),
                    "message": "Response generation complete"
                }
            else:
                # Fallback to raw chunks if no LLM service
                for i, result in enumerate(unique_results):
                    chunk_data = {
                        "type": "content",
                        "index": i,
                        "content": result["text"],
                        "score": result.get("relevance_score", result.get("score", 0.0)),
                        "source": result.get("metadata", {}).get("source", ""),
                        "headers": result.get("metadata", {}).get("headers", [])
                    }
                    yield f"data: {json.dumps(chunk_data)}\n\n"
                    await asyncio.sleep(0.1)

                completion = {
                    "type": "complete",
                    "total_chunks": len(unique_results),
                    "has_answer": False,
                    "message": "Retrieved chunks without LLM generation"
                }

            yield f"data: {json.dumps(completion)}\n\n"

        except Exception as e:
            error_data = {
                "type": "error",
                "message": str(e)
            }
            yield f"data: {json.dumps(error_data)}\n\n"

    return StreamingResponse(
        generate_response(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"  # Disable nginx buffering
        }
    )


@app.get("/health")
async def health_check() -> Dict[str, Any]:
    """Health check endpoint."""
    health_status = {
        "status": "healthy",
        "components": {
            "chunker": chunker is not None,
            "embedding_service": embedding_service is not None,
            "vector_store": vector_store is not None,
            "retrieval_pipeline": retrieval_pipeline is not None,
            "reranker": reranker is not None,
            "llm_service": llm_service is not None
        }
    }

    # Check LLM service health if available
    if llm_service:
        try:
            llm_health = await llm_service.health_check()
            health_status["llm_health"] = llm_health
        except Exception as e:
            health_status["llm_health"] = {
                "status": "unhealthy",
                "error": str(e)
            }

    return health_status


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "Physical AI & Humanoid Robotics Book Chatbot API",
        "version": "1.0.0",
        "docs": "/docs",
        "endpoints": {
            "chat": "/chat",
            "ingest": "/ingest",
            "health": "/health"
        }
    }


if __name__ == "__main__":
    # Run the app
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=7860,  # Hugging Face Spaces default port
        reload=False,
        access_log=True
    )