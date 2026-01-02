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

from fastapi import FastAPI, HTTPException, BackgroundTasks, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import uvicorn
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Reduce httpcore debug noise
logging.getLogger("httpcore.connection").setLevel(logging.WARNING)
logging.getLogger("httpcore.http11").setLevel(logging.WARNING)

# NOTE: Auth module temporarily disabled due to psycopg2 dependency issues
# Auth will be re-enabled once Railway caching is resolved or psycopg2 is installed
AUTH_AVAILABLE = False
get_current_user_optional = None
auth_router = None
init_db = None

# Create stub User type for type hints when auth is disabled
class User:
    """Stub User class for type hints when auth is disabled."""
    id: str = "anonymous"
    email: str = "anonymous@example.com"

# Create no-op dependency for when auth is disabled
async def _no_auth_user() -> User | None:
    return None
get_current_user_optional = _no_auth_user

logger.info("Auth module disabled - running without authentication")

from rag.chunking import MarkdownChunker
from rag.openai_embeddings import OpenAIEmbeddingService
from rag.openrouter_embeddings import OpenRouterEmbeddingService
from rag.vector_store_new import QdrantVectorStore
from rag.retrieval import RetrievalPipeline
from rag.openrouter_rerank import OpenRouterReranker
from rag.llm_service import create_llm_service

# Import translation agent
from agents.translation_agent import get_translation_service


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
    question: str  # Changed from 'query' to match frontend
    top_k: int = 3  # Reduced from 5 to 3 for speed
    similarity_threshold: float = 0.4  # Lowered because cosine scores are ~0.5
    use_rerank: bool = True
    stream: bool = True  # Added stream parameter from frontend


class IngestRequest(BaseModel):
    """Request model for ingest endpoint."""
    content_dir: str = "../docs"
    file_pattern: str = "*.md"
    overwrite: bool = False


class TranslateRequest(BaseModel):
    """Request model for translation endpoint."""
    text: str
    target_lang: str = "ur"  # Default to Urdu


class TranslateResponse(BaseModel):
    """Response model for translation endpoint."""
    success: bool
    translated_text: str | None = None
    original_text: str | None = None
    language: str | None = None
    error: str | None = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize and cleanup resources."""
    global chunker, embedding_service, vector_store, retrieval_pipeline, reranker, llm_service

    try:
        # Initialize database (only if auth is available)
        if AUTH_AVAILABLE and init_db:
            try:
                logger.info("Initializing auth database...")
                init_db()
                logger.info("Auth database initialized")
            except Exception as e:
                logger.warning(f"Failed to initialize auth database: {e}")
                logger.warning("Continuing without authentication")
        else:
            logger.info("Auth module not available - running without authentication")

        # Initialize components
        logger.info("Initializing RAG pipeline components...")

        chunker = MarkdownChunker(min_chunk_size=50, max_chunk_size=1000)

        # Get API keys with proper error handling
        openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        qdrant_url = os.getenv("QDRANT_URL")

        if not openrouter_api_key:
            logger.warning("OPENROUTER_API_KEY not set - RAG pipeline and LLM features will be disabled")
            logger.warning("Set OPENROUTER_API_KEY in Space settings to enable chat features")
            logger.info("Application started in limited mode - health and docs endpoints available")
        else:
            # Initialize embedding service using OpenRouter
            embedding_service = OpenRouterEmbeddingService(
                api_key=openrouter_api_key,
                model="openai/text-embedding-3-small",
                base_url=os.getenv("OPENROUTER_BASE_URL")
            )
            logger.info("Using OpenRouter for embeddings")

            # Initialize vector store with OpenRouter embedding dimension (1536)
            if qdrant_url:
                vector_store = QdrantVectorStore(
                    collection_name="humanoid_robotics_book_openrouter",
                    embedding_dim=1536,  # OpenRouter OpenAI text-embedding-3-small dimension
                    url=qdrant_url,
                    api_key=qdrant_api_key
                )
            else:
                vector_store = QdrantVectorStore(
                    collection_name="humanoid_robotics_book_openrouter",
                    embedding_dim=1536,  # OpenRouter OpenAI text-embedding-3-small dimension
                    host=os.getenv("QDRANT_HOST", "localhost"),
                    port=int(os.getenv("QDRANT_PORT", "6333")),
                    api_key=qdrant_api_key
                )

            # Initialize OpenRouter reranker
            try:
                reranker = OpenRouterReranker(
                    api_key=openrouter_api_key,
                    model="openai/text-embedding-3-small",
                    top_n=5,
                    timeout=60,
                    base_url=os.getenv("OPENROUTER_BASE_URL")
                )
                logger.info("OpenRouter reranker initialized successfully")
            except Exception as e:
                logger.warning(f"Failed to initialize OpenRouter reranker: {e}")
                reranker = None
            retrieval_pipeline = RetrievalPipeline(
                embedding_service=embedding_service,
                vector_store=vector_store,
                reranker=reranker
            )

            # Initialize LLM service using OpenRouter (optimized for fast, short responses)
            try:
                llm_service = create_llm_service({
                    "provider": "openrouter",
                    "model": "mistralai/devstral-2512:free",
                    "api_key": openrouter_api_key,
                    "base_url": os.getenv("OPENROUTER_BASE_URL"),
                    "max_tokens": 300,  # Short responses for speed
                    "temperature": 0.7
                })
                logger.info("LLM service initialized successfully with OpenRouter (Fast Mode)")
            except Exception as e:
                logger.warning(f"Failed to initialize LLM service: {e}")
                logger.warning("Chat will return retrieved chunks without generated answers")
                llm_service = None

            # Ensure collection exists
            await vector_store.ensure_collection()
            logger.info("RAG pipeline initialized successfully")

        # HuggingFace compatibility: explicit startup completion message
        port = os.getenv("PORT", "7860")
        logger.info(f"=== APPLICATION READY - SERVING ON PORT {port} ===")
        print(f"=== APPLICATION READY - SERVING ON PORT {port} ===")  # Also print for HF detection

        yield  # FastAPI takes control - app is now ready to serve requests

    except asyncio.CancelledError:
        # Python 3.10+ compatibility: properly handle task cancellation
        logger.info("Lifespan cancelled during startup/shutdown")
        raise
    finally:
        # Cleanup - always runs, even if CancelledError is raised
        logger.info("Shutting down RAG pipeline...")


# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book Chatbot",
    description="RAG-powered chatbot backend for the Physical AI & Humanoid Robotics book",
    version="1.0.0",
    lifespan=lifespan
)

# Include authentication routes (only if available)
if AUTH_AVAILABLE and auth_router:
    app.include_router(auth_router)
    logger.info("Authentication routes registered")
else:
    logger.info("Authentication routes not available")

# Configure CORS - Read from environment variables
def parse_cors_origins() -> List[str]:
    """Parse CORS_ORIGINS from environment variable."""
    cors_origins_str = os.getenv("CORS_ORIGINS")
    if cors_origins_str:
        try:
            import json
            origins = json.loads(cors_origins_str)
            if isinstance(origins, list):
                return origins
        except json.JSONDecodeError:
            logger.warning(f"Failed to parse CORS_ORIGINS: {cors_origins_str}")

    # Fallback to default origins
    return [
        "https://humanoid-robotic-book-livid.vercel.app",
        "http://localhost:3000",
        "http://localhost:7860",
        "http://localhost:8000",
    ]

def parse_cors_list(env_var: str, default: List[str]) -> List[str]:
    """Parse CORS list from environment variable."""
    cors_str = os.getenv(env_var)
    if cors_str:
        try:
            import json
            items = json.loads(cors_str)
            if isinstance(items, list):
                return items
        except json.JSONDecodeError:
            logger.warning(f"Failed to parse {env_var}: {cors_str}")
    return default

# Get CORS configuration from environment
cors_origins = parse_cors_origins()
cors_methods = parse_cors_list("CORS_METHODS", ["GET", "POST", "PUT", "DELETE", "OPTIONS"])
cors_headers = parse_cors_list("CORS_HEADERS", ["Content-Type", "Authorization", "X-API-Key"])

logger.info(f"CORS configured for origins: {cors_origins}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=cors_methods,
    allow_headers=cors_headers,
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
            embeddings = await embedding_service.embed_texts(texts)

            # Prepare for vector store
            import uuid
            documents = []
            for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                documents.append({
                    "id": str(uuid.uuid4()),
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
                # Skip deletion due to Qdrant index limitations - just overwrite
                logger.info(f"[INGEST DEBUG] Skipping deletion due to Qdrant index limitations, overwriting content")
                # TODO: Fix Qdrant metadata indexing for proper overwrite functionality

            await vector_store.upsert(documents)
            total_chunks += len(documents)

            logger.info(f"Processed {len(documents)} chunks from {file_path}")

        logger.info(f"Ingestion complete. Total chunks processed: {total_chunks}")

    except Exception as e:
        logger.error(f"Error during ingestion: {str(e)}", exc_info=True)


@app.post("/chat")
async def chat_stream(
    request: ChatRequest,
    current_user: User | None = Depends(get_current_user_optional)
) -> StreamingResponse:
    """
    Stream chat responses using Server-Sent Events (SSE).

    FIX: Improved SSE streaming with:
    - Proper event flushing using double newline
    - Better error handling and logging
    - Immediate sending without buffering
    - Per-chunk logging for debugging
    """
    # Log authenticated user (optional - works without auth)
    if current_user:
        logger.info(f"Chat request from user: {current_user.email} (ID: {current_user.id})")
    else:
        logger.info("Chat request from anonymous user (no auth)")

    if not retrieval_pipeline:
        raise HTTPException(status_code=503, detail="RAG pipeline not initialized")

    async def generate_response() -> AsyncGenerator[str, None]:
        """Generate streaming response - start immediately, do RAG in background."""
        try:
            # Send initial metadata IMMEDIATELY with flush
            metadata = {
                "type": "metadata",
                "query": request.question,
                "top_k": request.top_k,
                "similarity_threshold": request.similarity_threshold,
                "use_rerank": request.use_rerank
            }
            logger.info(f"[CHAT] Sending metadata: query='{request.question[:50]}...'")
            yield f"data: {json.dumps(metadata)}\n\n"
            await asyncio.sleep(0)  # Force flush

            # Start retrieval in background (non-blocking)
            retrieval_task = asyncio.create_task(
                retrieval_pipeline.retrieve(
                    query=request.question,
                    top_k=request.top_k,
                    similarity_threshold=request.similarity_threshold
                )
            )

            # Start LLM generation immediately WITHOUT context (for instant response)
            if llm_service:
                logger.info("[CHAT] Starting LLM generation immediately (no context)...")
                yield f"data: {json.dumps({'type': 'answer_start'})}\n\n"
                await asyncio.sleep(0)  # Force flush

                answer_content = ""
                chunk_count = 0
                error_occurred = False

                try:
                    # Generate answer without waiting for retrieval
                    async for chunk in llm_service.generate_answer(
                        query=request.question,
                        context=[],  # Empty context for immediate response
                        stream=True
                    ):
                        # Check for error chunks
                        if chunk and chunk.startswith("Error"):
                            logger.error(f"[CHAT] LLM error chunk: {chunk}")
                            error_occurred = True
                            # Send error to client
                            error_data = {
                                "type": "error",
                                "message": chunk
                            }
                            yield f"data: {json.dumps(error_data)}\n\n"
                            await asyncio.sleep(0)
                            break

                        # Process normal content chunks
                        if chunk and not chunk.startswith("Error"):
                            answer_content += chunk
                            chunk_count += 1
                            chunk_data = {
                                "type": "answer_chunk",
                                "content": chunk
                            }
                            yield f"data: {json.dumps(chunk_data)}\n\n"
                            await asyncio.sleep(0)  # Force flush after each chunk

                            # Log first few chunks for debugging
                            if chunk_count <= 3:
                                logger.info(f"[CHAT] LLM chunk {chunk_count}: '{chunk[:50]}...'")

                    if error_occurred:
                        logger.warning(f"[CHAT] LLM generation had errors after {chunk_count} chunks")
                    else:
                        logger.info(f"[CHAT] LLM completed successfully: {chunk_count} chunks, {len(answer_content)} chars")

                except Exception as e:
                    logger.error(f"[CHAT] LLM generation exception: {str(e)}", exc_info=True)
                    # Send error to client
                    error_data = {
                        "type": "error",
                        "message": f"Generation failed: {str(e)}"
                    }
                    yield f"data: {json.dumps(error_data)}\n\n"
                    await asyncio.sleep(0)

                # Send completion event
                completion = {
                    "type": "complete",
                    "has_answer": len(answer_content) > 0,
                    "answer_length": len(answer_content),
                    "chunk_count": chunk_count,
                    "message": "Response complete"
                }
                logger.info(f"[CHAT] Sending complete: chunks={chunk_count}, length={len(answer_content)}")
                yield f"data: {json.dumps(completion)}\n\n"
                await asyncio.sleep(0)  # Force flush

                # Wait for retrieval in background (fire and forget)
                try:
                    results = await asyncio.wait_for(retrieval_task, timeout=5.0)
                    logger.info(f"[CHAT] Background retrieval completed: {len(results)} results")
                except asyncio.TimeoutError:
                    logger.warning("[CHAT] Background retrieval timed out (response already sent)")
                except Exception as e:
                    logger.warning(f"[CHAT] Background retrieval error: {e}")

            else:
                # No LLM service - send error
                logger.error("[CHAT] LLM service not available")
                error_data = {
                    "type": "error",
                    "message": "LLM service not available"
                }
                yield f"data: {json.dumps(error_data)}\n\n"
                await asyncio.sleep(0)

        except Exception as e:
            logger.error(f"[CHAT] Stream generation error: {str(e)}", exc_info=True)
            error_data = {
                "type": "error",
                "message": f"Stream error: {str(e)}"
            }
            yield f"data: {json.dumps(error_data)}\n\n"
            await asyncio.sleep(0)

    return StreamingResponse(
        generate_response(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",  # Disable nginx buffering
            "X-Content-Type-Options": "nosniff",  # Prevent MIME sniffing
        }
    )


@app.post("/api/translate", response_model=TranslateResponse)
async def translate_page(request: TranslateRequest) -> TranslateResponse:
    """
    Translate page content using AI Translation Agent.

    This endpoint:
    - Preserves Markdown structure
    - Skips code blocks (```...```)
    - Skips inline code (`...`)
    - Translates to target language (default: Urdu)

    Args:
        request: Translation request with text and target language

    Returns:
        Translated content with preserved structure
    """
    try:
        logger.info(f"Translation request: {len(request.text)} chars to {request.target_lang}")

        # Get translation service
        translation_service = get_translation_service()

        # Perform translation
        result = await translation_service.translate_text(
            text=request.text,
            target_lang=request.target_lang
        )

        if result.get("success"):
            logger.info(f"Translation successful: {len(result.get('translated_text', ''))} chars")
            return TranslateResponse(
                success=True,
                translated_text=result.get("translated_text"),
                original_text=result.get("original_text"),
                language=result.get("language")
            )
        else:
            logger.error(f"Translation failed: {result.get('error')}")
            return TranslateResponse(
                success=False,
                error=result.get("error", "Unknown translation error")
            )

    except Exception as e:
        logger.error(f"Translation endpoint error: {str(e)}", exc_info=True)
        return TranslateResponse(
            success=False,
            error=f"Translation service error: {str(e)}"
        )


@app.get("/health")
async def health_check() -> Dict[str, Any]:
    """Health check endpoint - HuggingFace compatible."""
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

    # Log health check for debugging (HF visibility)
    logger.info(f"Health check called - status: {health_status['status']}")
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
            "translate": "/api/translate",
            "health": "/health"
        }
    }


if __name__ == "__main__":
    # Run the app - read PORT from environment (default 8000)
    port = int(os.getenv("PORT", 8000))
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=port,
        reload=False,
        access_log=True
    )