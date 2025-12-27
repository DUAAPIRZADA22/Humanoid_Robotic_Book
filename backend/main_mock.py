"""
Mock FastAPI backend for Physical AI & Humanoid Robotics book chatbot.
Provides simple endpoints without requiring actual RAG pipeline dependencies.
"""

import os
import json
from typing import Dict, Any
from dotenv import load_dotenv

# Load environment variables from .env
load_dotenv()

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
import uvicorn
import asyncio
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


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


# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book Chatbot (Mock)",
    description="Mock RAG-powered chatbot backend for testing",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://humanoid-robotic-book.vercel.app",
        "http://localhost:3000",
        "http://localhost:7860",
        "http://localhost:8000"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.post("/ingest")
async def ingest_content(request: IngestRequest) -> Dict[str, Any]:
    """
    Mock ingest endpoint - simulates content ingestion.
    """
    logger.info(f"Mock ingestion started for directory: {request.content_dir}")

    return {
        "status": "completed",
        "message": f"Mock ingestion completed for directory: {request.content_dir}",
        "pattern": request.file_pattern,
        "overwrite": request.overwrite,
        "mock": True,
        "documents_processed": 42  # Mock number
    }


@app.post("/chat")
async def chat_stream(request: ChatRequest) -> StreamingResponse:
    """
    Mock streaming chat endpoint - returns predefined responses.
    """
    async def generate_mock_response():
        """Generate mock streaming response."""
        try:
            # Send initial metadata
            metadata = {
                "type": "metadata",
                "query": request.query,
                "top_k": request.top_k,
                "similarity_threshold": request.similarity_threshold,
                "use_rerank": request.use_rerank,
                "mock": True
            }
            yield f"data: {json.dumps(metadata)}\n\n"

            # Mock response content based on common queries
            mock_responses = {
                "humanoid": {
                    "content": "Humanoid robots are bipedal machines designed to mimic human form and behavior. They typically feature two arms, two legs, a torso, and a head. Key challenges include balance, locomotion, and human-like interaction.",
                    "source": "Chapter 1: Introduction to Humanoid Robotics",
                    "headers": ["1.1 What are Humanoid Robots?"]
                },
                "locomotion": {
                    "content": "Robot locomotion encompasses various methods of movement including walking, rolling, crawling, and flying. For humanoid robots, bipedal walking presents unique challenges due to the inherent instability of standing on two legs.",
                    "source": "Chapter 2: Robot Locomotion",
                    "headers": ["2.1 Bipedal Walking"]
                },
                "default": {
                    "content": "The Physical AI & Humanoid Robotics book covers comprehensive topics including robot kinematics, dynamics, control systems, perception, and artificial intelligence integration. Each chapter provides both theoretical foundations and practical examples.",
                    "source": "Physical AI & Humanoid Robotics Book",
                    "headers": ["Preface"]
                }
            }

            # Select appropriate response
            query_lower = request.query.lower()
            selected_response = mock_responses["default"]

            for key in mock_responses:
                if key in query_lower:
                    selected_response = mock_responses[key]
                    break

            # Send content chunk
            chunk_data = {
                "type": "content",
                "index": 0,
                "content": selected_response["content"],
                "score": 0.95,
                "source": selected_response["source"],
                "headers": selected_response["headers"],
                "mock": True
            }
            yield f"data: {json.dumps(chunk_data)}\n\n"

            # Small delay to simulate streaming
            await asyncio.sleep(0.5)

            # Send completion event
            completion = {
                "type": "complete",
                "total_chunks": 1,
                "message": "Mock response generation complete",
                "mock": True
            }
            yield f"data: {json.dumps(completion)}\n\n"

        except Exception as e:
            error_data = {
                "type": "error",
                "message": str(e),
                "mock": True
            }
            yield f"data: {json.dumps(error_data)}\n\n"

    return StreamingResponse(
        generate_mock_response(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no"
        }
    )


@app.get("/health")
async def health_check() -> Dict[str, Any]:
    """Mock health check endpoint."""
    return {
        "status": "healthy",
        "mock": True,
        "components": {
            "rag_pipeline": False,
            "mock_mode": True,
            "server": "running"
        }
    }


@app.get("/")
async def root() -> Dict[str, str]:
    """Root endpoint."""
    return {
        "message": "Physical AI & Humanoid Robotics Book Chatbot API (Mock Mode)",
        "version": "1.0.0",
        "mock": True,
        "docs": "/docs",
        "endpoints": {
            "chat": "/chat",
            "ingest": "/ingest",
            "health": "/health"
        }
    }


if __name__ == "__main__":
    # Run the app
    port = int(os.getenv("PORT", 8000))
    host = os.getenv("HOST", "0.0.0.0")

    logger.info(f"Starting mock server on {host}:{port}")
    uvicorn.run(
        "main_mock:app",
        host=host,
        port=port,
        reload=False,
        access_log=True
    )